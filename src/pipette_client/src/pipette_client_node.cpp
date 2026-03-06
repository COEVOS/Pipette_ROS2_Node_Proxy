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
  this->declare_parameter<std::string>("discovery_interval", "3");
  
  this->get_parameter("local_port", local_port_);
  std::string discovery_interval_str;
  this->get_parameter("discovery_interval", discovery_interval_str);
  discovery_interval_ = std::stoi(discovery_interval_str);
  
  RCLCPP_INFO(this->get_logger(), "Pipette Client Node starting...");
  RCLCPP_INFO(this->get_logger(), "Local UDP Port: %s", local_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Discovery Interval: %d seconds", discovery_interval_);
  
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
    0,
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
    RCLCPP_INFO(node->get_logger(), "Found COEVOS service: %s", serviceName);
    
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
      RCLCPP_WARN(node->get_logger(), "DNSServiceResolve failed for %s: %d", serviceName, resolve_err);
    } else if (resolve_ref) {
      std::lock_guard<std::mutex> lock(node->devices_mutex_);
      node->mdns_browser_.resolve_refs.push_back(resolve_ref);
    }
  } else {
    RCLCPP_INFO(node->get_logger(), "Service removed: %s", serviceName);
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
    RCLCPP_DEBUG(node->get_logger(), "mDNS resolve error: %d", errorCode);
    return;
  }
  
  RCLCPP_INFO(node->get_logger(), "Resolved service: %s -> %s:%d, txtLen=%d", 
              fullname, hosttarget, ntohs(port), txtLen);
  
  node->parseTxtRecord(reinterpret_cast<const char*>(txtRecord), txtLen, fullname, hosttarget, ntohs(port));
}

void PipetteClientNode::parseTxtRecord(const char *txtRecord, uint16_t txtLen, 
                                       const std::string & /*name*/, const std::string &host, 
                                       uint16_t /*port*/)
{
  const unsigned char *ptr = (const unsigned char *)txtRecord;
  const unsigned char *end = ptr + txtLen;
  
  std::string ip_address;
  
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
    RCLCPP_DEBUG(get_logger(), "Resolved %s to %s", host.c_str(), ip_address.c_str());
  } else {
    ip_address = host;
    RCLCPP_WARN(get_logger(), "Failed to resolve %s, using hostname as-is", host.c_str());
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
      } else if (key == "model") {
        model = value;
      } else if (key == "product") {
        product = value;
      }
    }
  }
  
  RCLCPP_INFO(get_logger(), "TXT parsing: sn='%s', product='%s', model='%s'", 
              sn.c_str(), product.c_str(), model.c_str());
  
  // 过滤：必须包含 sn 且 product 必须是 "pippet"
  if (sn.empty()) {
    RCLCPP_WARN(get_logger(), "No SN found in TXT record, skipping device");
    return;
  }
  
  if (product != "pippet") {
    RCLCPP_INFO(get_logger(), "Device product '%s' is not 'pippet', skipping (SN: %s)", 
                product.c_str(), sn.c_str());
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Valid COEVOS pipette found: SN=%s, Model=%s", 
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
  
  RCLCPP_INFO(get_logger(), "Device added: %s (%s) at %s:%d", 
              sn.c_str(), model.c_str(), ip_address.c_str(), device.port);
}

void PipetteClientNode::mdnsLoop()
{
  RCLCPP_INFO(this->get_logger(), "mDNS event loop started");
  
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
    it->second.last_seen = device.last_seen;
    if (!it->second.available) {
      it->second.available = true;
      RCLCPP_INFO(this->get_logger(), "Device reactivated: %s", sn.c_str());
    }
    RCLCPP_DEBUG(this->get_logger(), "Device refreshed: %s", sn.c_str());
  } else {
    devices_[sn] = device;
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
