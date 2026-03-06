#include "pipette_client/pipette_client_node.hpp"

#include <sstream>
#include <chrono>
#include <cstring>
#include <regex>
#include <arpa/inet.h>
#include <netdb.h>

namespace pipette_client
{

using std::placeholders::_1;
using std::placeholders::_2;

// ==================== 节点实现 ====================

PipetteClientNode::PipetteClientNode(const rclcpp::NodeOptions & options)
: Node("pipette_client", options),
  udp_socket_(-1),
  udp_running_(false)
{
  // 声明参数
  this->declare_parameter<std::string>("local_port", "10000");
  this->declare_parameter<int>("discovery_interval", 10);
  this->declare_parameter<int>("offline_threshold", 3);  // ✅ 改为 3 次
  
  this->get_parameter("local_port", local_port_);
  discovery_interval_ = this->get_parameter("discovery_interval").as_int();
  offline_threshold_ = this->get_parameter("offline_threshold").as_int();
  
  // 验证 offline_threshold 参数
  if (offline_threshold_ <= 0) {
    RCLCPP_WARN(this->get_logger(), 
                "Invalid offline_threshold %d, using default value 30", 
                offline_threshold_);
    offline_threshold_ = 30;
  }
  
  // 初始化 last_scan_time_
  last_scan_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "Pipette Client Node starting...");
  RCLCPP_INFO(this->get_logger(), "Local UDP Port: %s", local_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Discovery Interval: %d seconds", discovery_interval_);
  RCLCPP_INFO(this->get_logger(), 
              "Offline detection: threshold=%d, interval=%ds, timeout=%.1fs",
              offline_threshold_, discovery_interval_, 
              static_cast<double>(offline_threshold_ * discovery_interval_));
  
  // 创建发布者
  device_list_pub_ = this->create_publisher<pipette_client::msg::DeviceList>("ws/pipette/device_list", 10);
  command_result_pub_ = this->create_publisher<pipette_client::msg::ATCommandResult>("ws/pipette/command_result", 10);
  
  // 创建服务
  get_device_list_srv_ = this->create_service<pipette_client::srv::GetDeviceList>(
    "ws/pipette/service/get_device_list",
    std::bind(&PipetteClientNode::handleGetDeviceListService, this, _1, _2));
  
  send_at_command_srv_ = this->create_service<pipette_client::srv::SendATCommand>(
    "ws/pipette/service/send_at_command",
    std::bind(&PipetteClientNode::handleSendATCommandService, this, _1, _2));
  
  // 创建动作服务器
  aspirate_action_server_ = rclcpp_action::create_server<pipette_client::action::Aspirate>(
    this,
    "ws/pipette/action/aspirate",
    std::bind(&PipetteClientNode::handleAspirateGoal, this, _1, _2),
    std::bind(&PipetteClientNode::handleAspirateCancel, this, _1),
    std::bind(&PipetteClientNode::handleAspirateAccepted, this, _1));
  
  dispense_action_server_ = rclcpp_action::create_server<pipette_client::action::Dispense>(
    this,
    "ws/pipette/action/dispense",
    std::bind(&PipetteClientNode::handleDispenseGoal, this, _1, _2),
    std::bind(&PipetteClientNode::handleDispenseCancel, this, _1),
    std::bind(&PipetteClientNode::handleDispenseAccepted, this, _1));
  
  mix_action_server_ = rclcpp_action::create_server<pipette_client::action::Mix>(
    this,
    "ws/pipette/action/mix",
    std::bind(&PipetteClientNode::handleMixGoal, this, _1, _2),
    std::bind(&PipetteClientNode::handleMixCancel, this, _1),
    std::bind(&PipetteClientNode::handleMixAccepted, this, _1));
  
  eject_tip_action_server_ = rclcpp_action::create_server<pipette_client::action::EjectTip>(
    this,
    "ws/pipette/action/eject_tip",
    std::bind(&PipetteClientNode::handleEjectTipGoal, this, _1, _2),
    std::bind(&PipetteClientNode::handleEjectTipCancel, this, _1),
    std::bind(&PipetteClientNode::handleEjectTipAccepted, this, _1));
  
  // 启动 mDNS 浏览器
  if (!startMDNSBrowser()) {
    RCLCPP_WARN(this->get_logger(), "Failed to start mDNS browser");
  }
  
  // 创建 UDP socket
  if (!createUDPSocket()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
  }
  
  // 创建定时器
  timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&PipetteClientNode::publishDeviceList, this));
  
  RCLCPP_INFO(this->get_logger(), "Pipette Client Node started successfully");
}

PipetteClientNode::~PipetteClientNode()
{
  udp_running_ = false;
  
  if (udp_thread_.joinable()) {
    udp_thread_.join();
  }
  
  closeUDPSocket();
  stopMDNSBrowser();
  
  RCLCPP_INFO(this->get_logger(), "Pipette Client Node shutdown complete");
}

// ==================== mDNS 相关 ====================

bool PipetteClientNode::startMDNSBrowser()
{
  RCLCPP_INFO(this->get_logger(), "Starting mDNS browser for _tcp._tcp.local (COEVOS devices)");
  
  mdns_browser_.running = true;
  mdns_browser_.ref = nullptr;
  
  DNSServiceErrorType err = DNSServiceBrowse(
    &mdns_browser_.ref,
    0,  // 接收所有服务（新添加和移除的）
    0,
    "_tcp._tcp",
    NULL,
    DNSServiceBrowseReply,
    this
  );
  
  if (err != kDNSServiceErr_NoError) {
    RCLCPP_ERROR(this->get_logger(), "mDNS browse failed: %d", err);
    return false;
  }
  
  if (!mdns_browser_.ref) {
    RCLCPP_ERROR(this->get_logger(), "mDNS ref is null after DNSServiceBrowse");
    return false;
  }
  
  int fd = DNSServiceRefSockFD(mdns_browser_.ref);
  RCLCPP_INFO(this->get_logger(), "mDNS socket fd: %d", fd);
  
  mdns_thread_ = std::thread(&PipetteClientNode::mdnsLoop, this);
  
  return true;
}

void PipetteClientNode::stopMDNSBrowser()
{
  mdns_browser_.running = false;
  
  if (mdns_browser_.ref) {
    DNSServiceRefDeallocate(mdns_browser_.ref);
    mdns_browser_.ref = nullptr;
  }
  
  for (DNSServiceRef resolve_ref : mdns_browser_.resolve_refs) {
    if (resolve_ref) {
      DNSServiceRefDeallocate(resolve_ref);
    }
  }
  mdns_browser_.resolve_refs.clear();
  
  if (mdns_thread_.joinable()) {
    mdns_thread_.join();
  }
  
  RCLCPP_INFO(this->get_logger(), "mDNS browser stopped");
}

void PipetteClientNode::DNSServiceBrowseReply(
  DNSServiceRef sdRef,
  DNSServiceFlags flags,
  uint32_t interfaceIndex,
  DNSServiceErrorType errorCode,
  const char *serviceName,
  const char *regtype,
  const char *replyDomain,
  void *context)
{
  (void)sdRef;
  (void)interfaceIndex;
  (void)regtype;
  (void)replyDomain;
  
  auto node = static_cast<PipetteClientNode*>(context);
  
  if (errorCode != kDNSServiceErr_NoError) {
    RCLCPP_ERROR(node->get_logger(), "mDNS browse error: %d", errorCode);
    return;
  }
  
  if (flags & kDNSServiceFlagsAdd) {
    RCLCPP_INFO(node->get_logger(), "[mDNS SCAN] ➕ Found new service: %s", serviceName);
    
    DNSServiceRef resolve_ref = nullptr;
    DNSServiceErrorType resolve_err = DNSServiceResolve(
      &resolve_ref,
      0,
      0,
      serviceName,
      regtype,
      replyDomain,
      DNSServiceResolveReply,
      context
    );
    
    if (resolve_err != kDNSServiceErr_NoError) {
      RCLCPP_WARN(node->get_logger(), "[mDNS SCAN] ⚠️ DNSServiceResolve failed for %s: %d", serviceName, resolve_err);
    } else if (resolve_ref) {
      std::lock_guard<std::mutex> lock(node->devices_mutex_);
      node->mdns_browser_.resolve_refs.push_back(resolve_ref);
      RCLCPP_DEBUG(node->get_logger(), "[mDNS SCAN] Added resolve_ref for %s", serviceName);
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "[mDNS SCAN] ➖ Service removed: %s", serviceName);
  }
}

void PipetteClientNode::DNSServiceResolveReply(
  DNSServiceRef sdRef,
  DNSServiceFlags flags,
  uint32_t interfaceIndex,
  DNSServiceErrorType errorCode,
  const char *fullname,
  const char *hosttarget,
  uint16_t port,
  uint16_t txtLen,
  const unsigned char *txtRecord,
  void *context)
{
  (void)sdRef;
  (void)flags;
  (void)interfaceIndex;
  
  auto node = static_cast<PipetteClientNode*>(context);
  
  if (errorCode != kDNSServiceErr_NoError) {
    RCLCPP_DEBUG(node->get_logger(), "[mDNS SCAN] ❌ Resolve error: %d", errorCode);
    return;
  }
  
  RCLCPP_INFO(node->get_logger(), "[mDNS SCAN] ✅ Resolved: %s -> %s:%d, txtLen=%d", 
              fullname, hosttarget, ntohs(port), txtLen);
  
  node->parseTxtRecord(reinterpret_cast<const char*>(txtRecord), txtLen, fullname, hosttarget, ntohs(port));
}

void PipetteClientNode::DNSServiceQueryRecordReply(
  DNSServiceRef sdRef,
  DNSServiceFlags flags,
  uint32_t interfaceIndex,
  DNSServiceErrorType errorCode,
  const char *fullname,
  uint16_t rrtype,
  uint16_t rrclass,
  uint16_t rdlen,
  const void *rdata,
  uint32_t ttl,
  void *context)
{
  (void)sdRef;
  (void)interfaceIndex;
  (void)rrtype;
  (void)rrclass;
  (void)rdlen;
  (void)rdata;
  
  auto node = static_cast<PipetteClientNode*>(context);
  
  if (errorCode != kDNSServiceErr_NoError) {
    // 查询失败，可能是设备离线了
    RCLCPP_DEBUG(node->get_logger(), "[mDNS QUERY] ❌ Query failed for %s: %d", fullname, errorCode);
    return;
  }
  
  // ✅ 关键修复：检查 flags，判断是否是缓存记录
  // kDNSServiceFlagsAdd 表示这是添加记录（正常响应）
  // kDNSServiceFlagsUnique 表示这是唯一记录（来自设备本身）
  // 如果没有这些 flags，可能是缓存的旧记录
  bool is_add = (flags & kDNSServiceFlagsAdd) != 0;
  bool is_unique = (flags & kDNSServiceFlagsUnique) != 0;
  
  // ✅ 检查 TTL，判断是否是缓存记录
  // TTL=0 表示记录已过期，不应该信任
  // TTL>0 且是新响应，可以更新 last_seen
  if (ttl == 0) {
    RCLCPP_DEBUG(node->get_logger(), 
                 "[mDNS QUERY] ⚠️ Received expired record for %s (ttl=0), ignoring", 
                 fullname);
    return;
  }
  
  // ✅ 如果是缓存记录（没有 Add 或 Unique 标志），忽略它
  if (!is_add && !is_unique) {
    RCLCPP_DEBUG(node->get_logger(), 
                 "[mDNS QUERY] ⚠️ Received cached record for %s (flags=0x%x, ttl=%d), ignoring", 
                 fullname, flags, ttl);
    return;
  }
  
  RCLCPP_INFO(node->get_logger(), 
              "[mDNS QUERY] ✅ Received live response for %s (flags=0x%x, ttl=%d)", 
              fullname, flags, ttl);
  
  // 从 fullname 中提取 SN（格式：SN._tcp._tcp.local.）
  std::string fqdn(fullname);
  size_t pos = fqdn.find("._tcp._tcp");
  std::string sn;
  if (pos != std::string::npos) {
    sn = fqdn.substr(0, pos);
  }
  
  if (!sn.empty()) {
    // 更新设备的 last_seen 时间戳
    auto now = node->now();
    auto device = node->getDevice(sn);
    if (device) {
      device->last_seen.sec = now.seconds();
      device->last_seen.nanosec = now.nanoseconds();
      device->miss_count = 0;  // 重置缺失计数
      RCLCPP_DEBUG(node->get_logger(), 
                   "[mDNS QUERY] ✓ Updated last_seen for device %s", sn.c_str());
    } else {
      RCLCPP_DEBUG(node->get_logger(), 
                   "[mDNS QUERY] ⚠️ Device %s not found in devices map", sn.c_str());
    }
  } else {
    RCLCPP_WARN(node->get_logger(), 
                "[mDNS QUERY] ⚠️ Failed to extract SN from %s", fullname);
  }
}

void PipetteClientNode::parseTxtRecord(const char *txtRecord, uint16_t txtLen, 
                                       const std::string & /*name*/, const std::string &host, 
                                       uint16_t /*port*/)
{
  const unsigned char *ptr = (const unsigned char *)txtRecord;
  const unsigned char *end = ptr + txtLen;
  
  std::string ip_address;
  
  RCLCPP_INFO(get_logger(), "[mDNS SCAN] 🔍 Parsing TXT record for host: %s", host.c_str());
  
  // 尝试将主机名解析为 IPv4 地址
  struct addrinfo hints, *res;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  
  if (getaddrinfo(host.c_str(), NULL, &hints, &res) == 0 && res != nullptr) {
    struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(addr->sin_addr), ip_str, INET_ADDRSTRLEN);
    ip_address = std::string(ip_str);
    freeaddrinfo(res);
    RCLCPP_INFO(get_logger(), "[mDNS SCAN] 🌐 Resolved IP: %s -> %s", host.c_str(), ip_address.c_str());
  } else {
    ip_address = host;
    RCLCPP_WARN(get_logger(), "[mDNS SCAN] ⚠️ Failed to resolve %s, using hostname as-is", host.c_str());
  }
  
  std::string sn;
  std::string model;
  std::string product;
  
  while (ptr < end) {
    uint8_t len = *ptr++;
    if (len == 0 || ptr + len > end) break;
    
    std::string txt((const char*)ptr, len);
    ptr += len;
    
    size_t pos = txt.find('=');
    if (pos != std::string::npos) {
      std::string key = txt.substr(0, pos);
      std::string value = txt.substr(pos + 1);
      
      if (key == "sn") {
        sn = value;
        RCLCPP_INFO(get_logger(), "[mDNS SCAN] 📝 Found SN: %s", sn.c_str());
      } else if (key == "model") {
        model = value;
        RCLCPP_INFO(get_logger(), "[mDNS SCAN] 📝 Found Model: %s", model.c_str());
      } else if (key == "product") {
        product = value;
        RCLCPP_INFO(get_logger(), "[mDNS SCAN] 📝 Found Product: %s", product.c_str());
      }
    }
  }
  
  RCLCPP_INFO(get_logger(), "[mDNS SCAN] 📋 TXT parsing result: sn='%s', product='%s', model='%s'", 
              sn.c_str(), product.c_str(), model.c_str());
  
  // 过滤：必须包含 sn 且 product 必须是 "pippet"
  if (sn.empty()) {
    RCLCPP_WARN(get_logger(), "[mDNS SCAN] ❌ No SN found in TXT record, skipping device");
    return;
  }
  
  if (product != "pippet") {
    RCLCPP_INFO(get_logger(), "[mDNS SCAN] ❌ Device product '%s' is not 'pippet', skipping (SN: %s)", 
                product.c_str(), sn.c_str());
    return;
  }
  
  RCLCPP_INFO(get_logger(), "[mDNS SCAN] ✅ Valid COEVOS pipette found: SN=%s, Model=%s", 
              sn.c_str(), model.c_str());
  
  PipetteDevice device;
  device.sn = sn;
  device.model = model;
  device.product = product;
  device.ip_address = ip_address;
  device.port = 10002;
  device.available = true;
  device.socket_fd = udp_socket_;
  
  auto now = this->now();
  device.last_seen.sec = now.seconds();
  device.last_seen.nanosec = now.nanoseconds();
  
  memset(&device.addr, 0, sizeof(device.addr));
  device.addr.sin_family = AF_INET;
  device.addr.sin_port = htons(device.port);
  inet_pton(AF_INET, ip_address.c_str(), &device.addr.sin_addr);
  
  addDevice(sn, device);
  
  RCLCPP_INFO(get_logger(), "[mDNS SCAN] 🎉 Device added: %s (%s) at %s:%d", 
              sn.c_str(), model.c_str(), ip_address.c_str(), device.port);
}

void PipetteClientNode::mdnsLoop()
{
  RCLCPP_INFO(this->get_logger(), "mDNS event loop started");
  
  // 初始化 last_scan_time_ 为当前时间
  last_scan_time_ = this->now();
  auto cycle_start_time = last_scan_time_;  // 记录每个扫描周期的开始时间
  
  while (mdns_browser_.running && rclcpp::ok()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    
    int browse_fd = DNSServiceRefSockFD(mdns_browser_.ref);
    if (browse_fd >= 0) {
      FD_SET(browse_fd, &readfds);
    }
    
    int max_fd = browse_fd;
    std::vector<DNSServiceRef> active_resolve_refs;
    {
      std::lock_guard<std::mutex> lock(devices_mutex_);
      for (DNSServiceRef resolve_ref : mdns_browser_.resolve_refs) {
        int fd = DNSServiceRefSockFD(resolve_ref);
        if (fd >= 0) {
          FD_SET(fd, &readfds);
          active_resolve_refs.push_back(resolve_ref);
          if (fd > max_fd) max_fd = fd;
        }
      }
    }
    
    // ✅ 加入 QueryRecord 查询的 sockets
    std::vector<DNSServiceRef> active_query_refs;
    {
      std::lock_guard<std::mutex> lock(mdns_query_mutex_);
      for (auto &query : mdns_queries_) {
        int fd = DNSServiceRefSockFD(query.ref);
        if (fd >= 0) {
          FD_SET(fd, &readfds);
          active_query_refs.push_back(query.ref);
          if (fd > max_fd) max_fd = fd;
        }
      }
    }
    
    if (max_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "No valid mDNS sockets");
      break;
    }
    
    struct timeval timeout = {1, 0};
    int ret = select(max_fd + 1, &readfds, NULL, NULL, &timeout);
    
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "select error: %s", strerror(errno));
      break;
    }
    
    if (ret > 0) {
      if (browse_fd >= 0 && FD_ISSET(browse_fd, &readfds)) {
        DNSServiceErrorType err = DNSServiceProcessResult(mdns_browser_.ref);
        if (err != kDNSServiceErr_NoError) {
          RCLCPP_DEBUG(this->get_logger(), "Browse DNSServiceProcessResult: %d", err);
        }
      }
      
      for (size_t i = 0; i < active_resolve_refs.size(); ++i) {
        int fd = DNSServiceRefSockFD(active_resolve_refs[i]);
        if (fd >= 0 && FD_ISSET(fd, &readfds)) {
          DNSServiceErrorType err = DNSServiceProcessResult(active_resolve_refs[i]);
          if (err != kDNSServiceErr_NoError && err != kDNSServiceErr_NoSuchRecord) {
            RCLCPP_DEBUG(this->get_logger(), "Resolve DNSServiceProcessResult: %d", err);
          }
        }
      }
      
      // ✅ 处理 QueryRecord 查询的响应
      for (size_t i = 0; i < active_query_refs.size(); ++i) {
        int fd = DNSServiceRefSockFD(active_query_refs[i]);
        if (fd >= 0 && FD_ISSET(fd, &readfds)) {
          DNSServiceErrorType err = DNSServiceProcessResult(active_query_refs[i]);
          if (err != kDNSServiceErr_NoError && err != kDNSServiceErr_NoSuchRecord) {
            RCLCPP_DEBUG(this->get_logger(), "Query DNSServiceProcessResult: %d", err);
          }
        }
      }
    }
    
    // 检查是否到达扫描周期结束时间
    auto current_time = this->now();
    auto elapsed = (current_time - cycle_start_time).seconds();
    
    // 添加调试日志，每次循环都输出经过的时间
    static int loop_count = 0;
    loop_count++;
    if (loop_count % 10 == 0) {  // 每 10 次循环输出一次
      RCLCPP_DEBUG(this->get_logger(), 
                   "mdnsLoop: elapsed=%.2fs, discovery_interval=%d", 
                   elapsed, discovery_interval_);
    }
    
    // 时间异常处理
    if (elapsed < 0 || elapsed > 2 * discovery_interval_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Abnormal elapsed time: %.2fs, resetting scan timer", 
                  elapsed);
      cycle_start_time = current_time;
    } else if (elapsed >= discovery_interval_) {
      // 扫描周期结束，执行离线检测
      RCLCPP_INFO(this->get_logger(), 
                  "Scan cycle completed: elapsed=%.2fs, calling checkOfflineDevices()", 
                  elapsed);
      
      // ✅ 重新发起 Browse 请求来发现新设备
      RCLCPP_INFO(this->get_logger(), "Re-browsing _tcp._tcp.local for periodic scan...");
      
      // 清理旧的 resolve refs
      {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        for (DNSServiceRef resolve_ref : mdns_browser_.resolve_refs) {
          if (resolve_ref) {
            DNSServiceRefDeallocate(resolve_ref);
          }
        }
        mdns_browser_.resolve_refs.clear();
      }
      
      // 释放旧的 browse ref
      if (mdns_browser_.ref) {
        DNSServiceRefDeallocate(mdns_browser_.ref);
        mdns_browser_.ref = nullptr;
      }
      
      // 重新发起 Browse 请求
      DNSServiceErrorType browse_err = DNSServiceBrowse(
        &mdns_browser_.ref,
        0,
        0,
        "_tcp._tcp",
        NULL,
        DNSServiceBrowseReply,
        this
      );
      
      if (browse_err != kDNSServiceErr_NoError) {
        RCLCPP_DEBUG(this->get_logger(), "Periodic browse failed: %d", browse_err);
      } else if (!mdns_browser_.ref) {
        RCLCPP_ERROR(this->get_logger(), "Re-browse returned null ref");
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully re-browsed, waiting for responses...");
      }
      
      // ✅ 对已知设备发送 QueryRecord 查询（异步模式）
      // DNS-SD 官方推荐：创建查询后，将 ref 加入 select 监听，在事件循环中处理响应
      {
        std::lock_guard<std::mutex> lock(mdns_query_mutex_);
        auto now = this->now();
        
        // 先清理超过 15 秒的旧查询
        auto it = mdns_queries_.begin();
        while (it != mdns_queries_.end()) {
          auto elapsed_time = (now - it->query_time).seconds();
          if (elapsed_time > 15) {
            RCLCPP_DEBUG(this->get_logger(), 
                         "Removing expired query for %s (%.2fs old)", 
                         it->sn.c_str(), elapsed_time);
            if (it->ref) {
              DNSServiceRefDeallocate(it->ref);
            }
            it = mdns_queries_.erase(it);
          } else {
            ++it;
          }
        }
        
        // 只对 miss_count >= 2 的设备发送查询（给新设备 2 个周期的宽限期）
        int query_count = 0;
        for (auto &pair : devices_) {
          auto &device = pair.second;
          if (device.miss_count >= 2) {
            DNSServiceRef query_ref = nullptr;
            std::string fqdn = device.sn + "._tcp._tcp.local.";
            DNSServiceErrorType query_err = DNSServiceQueryRecord(
              &query_ref,
              0,
              0,
              fqdn.c_str(),
              kDNSServiceType_SRV,
              kDNSServiceClass_IN,
              DNSServiceQueryRecordReply,
              this
            );
            
            if (query_err == kDNSServiceErr_NoError && query_ref) {
              // 将查询加入列表，稍后在 select 中监听
              MDNSQuery query;
              query.sn = device.sn;
              query.ref = query_ref;
              query.query_time = now;
              mdns_queries_.push_back(query);
              query_count++;
              
              RCLCPP_DEBUG(this->get_logger(), 
                           "Started QueryRecord for %s (miss_count=%d)", 
                           device.sn.c_str(), device.miss_count);
            } else {
              RCLCPP_DEBUG(this->get_logger(), 
                           "Failed to send QueryRecord for %s: %d", 
                           device.sn.c_str(), query_err);
            }
          }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "Sent %d QueryRecord queries, active queries: %zu", 
                    query_count, mdns_queries_.size());
        
        // 增加 miss_count
        for (auto &pair : devices_) {
          auto &device = pair.second;
          device.miss_count++;
        }
      }
      
      checkOfflineDevices();
      cycle_start_time = current_time;
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "mDNS event loop stopped");
}

// ==================== UDP 相关 ====================

bool PipetteClientNode::createUDPSocket()
{
  udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
    return false;
  }

  int broadcast = 1;
  if (setsockopt(udp_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set broadcast option: %s", strerror(errno));
    close(udp_socket_);
    return false;
  }

  int reuse = 1;
  if (setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set reuseaddr option: %s", strerror(errno));
    close(udp_socket_);
    return false;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(std::stoi(local_port_));

  if (bind(udp_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket: %s", strerror(errno));
    close(udp_socket_);
    return false;
  }

  udp_running_ = true;
  RCLCPP_INFO(this->get_logger(), "UDP socket created on port %s", local_port_.c_str());
  
  udp_thread_ = std::thread(&PipetteClientNode::udpReceiveLoop, this);
  
  return true;
}

void PipetteClientNode::closeUDPSocket()
{
  udp_running_ = false;
  
  if (udp_socket_ >= 0) {
    close(udp_socket_);
    udp_socket_ = -1;
    RCLCPP_INFO(this->get_logger(), "UDP socket closed");
  }
  
  if (udp_thread_.joinable()) {
    udp_thread_.join();
  }
}

bool PipetteClientNode::sendATCommand(int socket_fd, const struct sockaddr_in &addr, 
                                       const std::string &command)
{
  std::string cmd = command + "\r\n";
  ssize_t sent = sendto(socket_fd, cmd.c_str(), cmd.size(), 0,
                        (struct sockaddr*)&addr, sizeof(addr));
  
  if (sent < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", strerror(errno));
    return false;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Sent command: %s", command.c_str());
  return true;
}

std::string PipetteClientNode::receiveResponse(int socket_fd, struct sockaddr_in &addr, 
                                                int timeout_ms)
{
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(socket_fd, &readfds);
  
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  
  int ret = select(socket_fd + 1, &readfds, NULL, NULL, &timeout);
  
  if (ret <= 0 || !FD_ISSET(socket_fd, &readfds)) {
    return "";
  }
  
  char buffer[1024];
  socklen_t addr_len = sizeof(addr);
  ssize_t received = recvfrom(socket_fd, buffer, sizeof(buffer) - 1, 0,
                              (struct sockaddr*)&addr, &addr_len);
  
  if (received < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive response: %s", strerror(errno));
    return "";
  }
  
  buffer[received] = '\0';
  std::string response(buffer, received);
  
  while (!response.empty() && (response.back() == '\r' || response.back() == '\n')) {
    response.pop_back();
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Received response: %s", response.c_str());
  return response;
}

void PipetteClientNode::udpReceiveLoop()
{
  RCLCPP_INFO(this->get_logger(), "UDP receive loop started");
  
  while (udp_running_ && rclcpp::ok()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(udp_socket_, &readfds);
    
    struct timeval timeout = {1, 0};
    int ret = select(udp_socket_ + 1, &readfds, NULL, NULL, &timeout);
    
    if (ret > 0 && FD_ISSET(udp_socket_, &readfds)) {
      char buffer[1024];
      struct sockaddr_in addr;
      socklen_t addr_len = sizeof(addr);
      
      ssize_t received = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                                  (struct sockaddr*)&addr, &addr_len);
      
      if (received > 0) {
        buffer[received] = '\0';
        std::string response(buffer, received);
        
        while (!response.empty() && (response.back() == '\r' || response.back() == '\n')) {
          response.pop_back();
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Received unsolicited response: %s", response.c_str());
        
        publishCommandResult("UNSOLICITED", response, true, "");
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "UDP receive loop stopped");
}

// ==================== 设备管理 ====================

void PipetteClientNode::addDevice(const std::string &sn, const PipetteDevice &device)
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  auto it = devices_.find(sn);
  if (it != devices_.end()) {
    // 设备已存在，更新状态
    it->second.last_seen = device.last_seen;
    it->second.miss_count = 0;  // 重置缺失计数
    it->second.seen_in_current_cycle = true;  // 标记为本周期已发现
    
    if (!it->second.available) {
      it->second.available = true;
      RCLCPP_INFO(this->get_logger(), "Device %s (%s) back online", 
                  sn.c_str(), it->second.ip_address.c_str());
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Device %s refreshed, miss_count reset to 0", 
                 sn.c_str());
  } else {
    // 新设备，初始化离线检测字段
    PipetteDevice new_device = device;
    new_device.miss_count = 0;
    new_device.seen_in_current_cycle = true;
    
    devices_[sn] = new_device;
    
    RCLCPP_INFO(this->get_logger(), "New device discovered: %s (%s) at %s:%d", 
                sn.c_str(), device.model.c_str(), device.ip_address.c_str(), device.port);
  }
}

void PipetteClientNode::removeDevice(const std::string &sn)
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  auto it = devices_.find(sn);
  if (it != devices_.end()) {
    it->second.available = false;
    RCLCPP_INFO(this->get_logger(), "Device marked as unavailable: %s", sn.c_str());
  }
}

pipette_client::PipetteDevice* PipetteClientNode::getDevice(const std::string &sn)
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  auto it = devices_.find(sn);
  if (it != devices_.end()) {
    return &it->second;
  }
  return nullptr;
}

void PipetteClientNode::updateDeviceHeartbeat(const std::string &sn)
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  auto it = devices_.find(sn);
  if (it != devices_.end()) {
    auto now = this->now();
    it->second.last_seen.sec = now.seconds();
    it->second.last_seen.nanosec = now.nanoseconds();
  }
}

std::vector<std::string> PipetteClientNode::getAllDeviceSNs()
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  std::vector<std::string> sns;
  for (const auto &pair : devices_) {
    if (pair.second.available) {
      sns.push_back(pair.first);
    }
  }
  return sns;
}

void PipetteClientNode::checkOfflineDevices()
{
  std::lock_guard<std::mutex> lock(devices_mutex_);
  
  RCLCPP_INFO(this->get_logger(), 
              "checkOfflineDevices() called, total devices: %zu", 
              devices_.size());
  
  std::vector<std::string> devices_to_remove;
  
  // 遍历所有设备，基于连续未响应次数判断是否离线
  for (auto &pair : devices_) {
    auto &device = pair.second;
    
    // ✅ 关键修复：使用连续未响应次数，而不是累加的时间间隔
    // miss_count 在 DNSServiceQueryRecordReply 中收到响应时重置为 0
    // 在每个扫描周期，如果没有收到响应，miss_count 会增加
    RCLCPP_DEBUG(this->get_logger(), 
                 "Device %s: miss_count=%d/%d", 
                 device.sn.c_str(), device.miss_count, offline_threshold_);
    
    // 如果连续未响应次数超过阈值，判定为离线
    if (device.miss_count >= offline_threshold_) {
      RCLCPP_WARN(this->get_logger(), 
                  "Device %s (%s) offline after %d consecutive missed responses", 
                  device.sn.c_str(), device.ip_address.c_str(), 
                  device.miss_count);
      devices_to_remove.push_back(pair.first);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
              "Scan summary: to_remove=%zu", 
              devices_to_remove.size());
  
  // 移除离线设备
  for (const auto &sn : devices_to_remove) {
    auto it = devices_.find(sn);
    if (it != devices_.end()) {
      // 关闭 socket（如果是独立的 socket）
      if (it->second.socket_fd >= 0 && it->second.socket_fd != udp_socket_) {
        if (close(it->second.socket_fd) < 0) {
          RCLCPP_DEBUG(this->get_logger(), 
                       "Failed to close socket for device %s: %s", 
                       it->second.sn.c_str(), strerror(errno));
        }
      }
      devices_.erase(it);
    }
  }
  
  // 如果有设备被移除，发布更新的设备列表
  if (!devices_to_remove.empty()) {
    // 注意：publishDeviceList() 会尝试获取 devices_mutex_，
    // 但我们已经持有锁，所以需要在释放锁后调用
    // 为了避免死锁，我们在这里不调用 publishDeviceList()
    // 而是依赖定时器定期发布设备列表
  }
}

// ==================== 话题发布 ====================

void PipetteClientNode::publishDeviceList()
{
  auto sns = getAllDeviceSNs();
  
  auto msg = std::make_unique<pipette_client::msg::DeviceList>();
  msg->count = sns.size();
  
  for (const auto &sn : sns) {
    auto device = getDevice(sn);
    if (device) {
      pipette_client::msg::PipetteDevice dev_msg;
      dev_msg.sn = device->sn;
      dev_msg.model = device->model;
      dev_msg.ip_address = device->ip_address;
      dev_msg.port = device->port;
      dev_msg.product = device->product;
      dev_msg.last_seen = device->last_seen;
      msg->devices.push_back(dev_msg);
    }
  }
  
  auto now = this->now();
  msg->timestamp.sec = now.seconds();
  msg->timestamp.nanosec = now.nanoseconds();
  
  device_list_pub_->publish(std::move(msg));
  
  RCLCPP_DEBUG(this->get_logger(), "Published device list: %d devices", msg->count);
}

void PipetteClientNode::publishCommandResult(const std::string &command, 
                                             const std::string &response, 
                                             bool success, const std::string &sn)
{
  auto msg = std::make_unique<pipette_client::msg::ATCommandResult>();
  msg->command = command;
  msg->response = response;
  msg->success = success;
  msg->sn = sn;
  
  auto now = this->now();
  msg->timestamp.sec = now.seconds();
  msg->timestamp.nanosec = now.nanoseconds();
  
  command_result_pub_->publish(std::move(msg));
}

// ==================== 服务实现 ====================

void PipetteClientNode::handleGetDeviceListService(
  const std::shared_ptr<pipette_client::srv::GetDeviceList::Request> /*request*/,
  std::shared_ptr<pipette_client::srv::GetDeviceList::Response> response)
{
  auto sns = getAllDeviceSNs();
  
  response->count = sns.size();
  response->success = true;
  response->message = "Successfully retrieved device list";
  
  for (const auto &sn : sns) {
    auto device = getDevice(sn);
    if (device) {
      pipette_client::msg::PipetteDevice dev_msg;
      dev_msg.sn = device->sn;
      dev_msg.model = device->model;
      dev_msg.ip_address = device->ip_address;
      dev_msg.port = device->port;
      dev_msg.product = device->product;
      dev_msg.last_seen = device->last_seen;
      response->devices.push_back(dev_msg);
    }
  }
  
  auto now = this->now();
  response->timestamp.sec = now.seconds();
  response->timestamp.nanosec = now.nanoseconds();
  
  RCLCPP_INFO(this->get_logger(), "Get device list service called: %d devices", response->count);
}

void PipetteClientNode::handleSendATCommandService(
  const std::shared_ptr<pipette_client::srv::SendATCommand::Request> request,
  std::shared_ptr<pipette_client::srv::SendATCommand::Response> response)
{
  try {
    if (request->sn.empty()) {
      response->success = false;
      response->response = "Empty SN provided";
      return;
    }
    
    if (request->command.empty()) {
      response->success = false;
      response->response = "Empty command provided";
      return;
    }
    
    auto device = getDevice(request->sn);
    if (!device) {
      response->success = false;
      response->response = "Device not found: " + request->sn;
      return;
    }
    
    std::string result;
    bool success = executeATCommand(request->sn, request->command, result);
    
    response->success = success;
    response->response = result;
    response->sn = request->sn;
    
    auto now = this->now();
    response->timestamp.sec = now.seconds();
    response->timestamp.nanosec = now.nanoseconds();
    
    publishCommandResult(request->command, result, success, request->sn);
    
  } catch (const std::exception &e) {
    response->success = false;
    response->response = std::string("Error: ") + e.what();
    
    auto now = this->now();
    response->timestamp.sec = now.seconds();
    response->timestamp.nanosec = now.nanoseconds();
    
    RCLCPP_ERROR(this->get_logger(), "Error handling AT command: %s", e.what());
  }
}

bool PipetteClientNode::executeATCommand(const std::string &sn, const std::string &command,
                                         std::string &response, int timeout_ms)
{
  auto device = getDevice(sn);
  if (!device) {
    RCLCPP_ERROR(this->get_logger(), "Device not found: %s", sn.c_str());
    return false;
  }
  
  if (!sendATCommand(udp_socket_, device->addr, command)) {
    response = "SEND_FAILED";
    return false;
  }
  
  struct sockaddr_in from_addr;
  response = receiveResponse(udp_socket_, from_addr, timeout_ms);
  
  if (response.empty()) {
    response = "TIMEOUT";
    return false;
  }
  
  if (response == "OK" || response.find("OK") != std::string::npos ||
      response.find("BSY") != std::string::npos ||
      (response.length() > 0 && isdigit(response[0]))) {
    updateDeviceHeartbeat(sn);
    return true;
  }
  
  if (response == "ERR" || response.find("ERR") != std::string::npos) {
    RCLCPP_WARN(this->get_logger(), "Command failed: %s -> %s", command.c_str(), response.c_str());
    return false;
  }
  
  return true;
}

std::string PipetteClientNode::buildATCommand(const std::string &cmd, 
                                               int volume, int speed, int times)
{
  std::ostringstream oss;
  oss << "AT+" << cmd;
  
  if (volume > 0) {
    oss << " " << volume;
    if (speed > 0) {
      oss << " " << speed;
      if (times > 0) {
        oss << " " << times;
      }
    }
  }
  
  return oss.str();
}

// ==================== 动作实现 ====================

rclcpp_action::GoalResponse PipetteClientNode::handleAspirateGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipette_client::action::Aspirate::Goal> goal)
{
  (void)uuid;
  
  if (goal->sn.empty()) {
    RCLCPP_WARN(this->get_logger(), "Aspirate goal rejected: empty SN");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  auto device = getDevice(goal->sn);
  if (!device) {
    RCLCPP_WARN(this->get_logger(), "Aspirate goal rejected: device not found %s", goal->sn.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  RCLCPP_INFO(this->get_logger(), "Aspirate goal accepted: %s, volume=%d, speed=%d", 
              goal->sn.c_str(), goal->volume, goal->speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PipetteClientNode::handleAspirateCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Aspirate>> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Aspirate goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PipetteClientNode::handleAspirateAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Aspirate>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipette_client::action::Aspirate::Result>();
  
  std::string command = buildATCommand("ASP", goal->volume, goal->speed);
  std::string response;
  
  bool success = executeATCommand(goal->sn, command, response, 5000);
  
  result->success = success;
  result->response = response;
  result->result_sn = goal->sn;
  
  auto now = this->now();
  result->result_timestamp.sec = now.seconds();
  result->result_timestamp.nanosec = now.nanoseconds();
  
  goal_handle->succeed(result);
  
  RCLCPP_INFO(this->get_logger(), "Aspirate completed: %s, success=%d, response=%s", 
              goal->sn.c_str(), success, response.c_str());
}

rclcpp_action::GoalResponse PipetteClientNode::handleDispenseGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipette_client::action::Dispense::Goal> goal)
{
  (void)uuid;
  
  if (goal->sn.empty()) {
    RCLCPP_WARN(this->get_logger(), "Dispense goal rejected: empty SN");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  auto device = getDevice(goal->sn);
  if (!device) {
    RCLCPP_WARN(this->get_logger(), "Dispense goal rejected: device not found %s", goal->sn.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  RCLCPP_INFO(this->get_logger(), "Dispense goal accepted: %s, volume=%d, speed=%d", 
              goal->sn.c_str(), goal->volume, goal->speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PipetteClientNode::handleDispenseCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Dispense>> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Dispense goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PipetteClientNode::handleDispenseAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Dispense>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipette_client::action::Dispense::Result>();
  
  std::string command = buildATCommand("DIS", goal->volume, goal->speed);
  std::string response;
  
  bool success = executeATCommand(goal->sn, command, response, 5000);
  
  result->success = success;
  result->response = response;
  result->result_sn = goal->sn;
  
  auto now = this->now();
  result->result_timestamp.sec = now.seconds();
  result->result_timestamp.nanosec = now.nanoseconds();
  
  goal_handle->succeed(result);
  
  RCLCPP_INFO(this->get_logger(), "Dispense completed: %s, success=%d, response=%s", 
              goal->sn.c_str(), success, response.c_str());
}

rclcpp_action::GoalResponse PipetteClientNode::handleMixGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipette_client::action::Mix::Goal> goal)
{
  (void)uuid;
  
  if (goal->sn.empty()) {
    RCLCPP_WARN(this->get_logger(), "Mix goal rejected: empty SN");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  auto device = getDevice(goal->sn);
  if (!device) {
    RCLCPP_WARN(this->get_logger(), "Mix goal rejected: device not found %s", goal->sn.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  RCLCPP_INFO(this->get_logger(), "Mix goal accepted: %s, volume=%d, speed=%d, times=%d", 
              goal->sn.c_str(), goal->volume, goal->speed, goal->times);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PipetteClientNode::handleMixCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Mix>> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Mix goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PipetteClientNode::handleMixAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Mix>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipette_client::action::Mix::Result>();
  
  std::string command = buildATCommand("MIX", goal->volume, goal->speed, goal->times);
  std::string response;
  
  bool success = executeATCommand(goal->sn, command, response, 10000);
  
  result->success = success;
  result->response = response;
  result->result_sn = goal->sn;
  
  auto now = this->now();
  result->result_timestamp.sec = now.seconds();
  result->result_timestamp.nanosec = now.nanoseconds();
  
  goal_handle->succeed(result);
  
  RCLCPP_INFO(this->get_logger(), "Mix completed: %s, success=%d, response=%s", 
              goal->sn.c_str(), success, response.c_str());
}

rclcpp_action::GoalResponse PipetteClientNode::handleEjectTipGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipette_client::action::EjectTip::Goal> goal)
{
  (void)uuid;
  
  if (goal->sn.empty()) {
    RCLCPP_WARN(this->get_logger(), "EjectTip goal rejected: empty SN");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  auto device = getDevice(goal->sn);
  if (!device) {
    RCLCPP_WARN(this->get_logger(), "EjectTip goal rejected: device not found %s", goal->sn.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  RCLCPP_INFO(this->get_logger(), "EjectTip goal accepted: %s", goal->sn.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PipetteClientNode::handleEjectTipCancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::EjectTip>> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "EjectTip goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PipetteClientNode::handleEjectTipAccepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::EjectTip>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipette_client::action::EjectTip::Result>();
  
  std::string command = "AT+EJTIP";
  std::string response;
  
  bool success = executeATCommand(goal->sn, command, response, 3000);
  
  result->success = success;
  result->response = response;
  result->result_sn = goal->sn;
  
  auto now = this->now();
  result->result_timestamp.sec = now.seconds();
  result->result_timestamp.nanosec = now.nanoseconds();
  
  goal_handle->succeed(result);
  
  RCLCPP_INFO(this->get_logger(), "EjectTip completed: %s, success=%d, response=%s", 
              goal->sn.c_str(), success, response.c_str());
}

}  // namespace pipette_client
