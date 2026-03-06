#ifndef PIPETTE_CLIENT_NODE_HPP_
#define PIPETTE_CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// 自定义消息类型
#include "pipette_client/msg/pipette_device.hpp"
#include "pipette_client/msg/device_list.hpp"
#include "pipette_client/msg/at_command_result.hpp"

// 自定义服务类型
#include "pipette_client/srv/get_device_list.hpp"
#include "pipette_client/srv/send_at_command.hpp"

// 自定义动作类型
#include "pipette_client/action/aspirate.hpp"
#include "pipette_client/action/dispense.hpp"
#include "pipette_client/action/mix.hpp"
#include "pipette_client/action/eject_tip.hpp"

// Apple Bonjour/mDNS compatibility layer (via Avahi)
extern "C" {
#include <dns_sd.h>
}

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>
#include <queue>
#include <chrono>

// 网络相关
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

namespace pipette_client
{

// 移液器设备信息
struct PipetteDevice
{
  std::string sn;              // 设备 SN 号
  std::string model;           // 型号
  std::string ip_address;      // IP 地址
  int port;                    // 端口号
  std::string product;         // 产品类型
  builtin_interfaces::msg::Time last_seen;  // 最后发现时间
  int socket_fd;               // UDP socket
  struct sockaddr_in addr;     // 地址信息
  bool available;              // 是否可用
};

// mDNS 浏览器
struct MDNSBrowser
{
  DNSServiceRef ref;
  std::vector<DNSServiceRef> resolve_refs;
  std::atomic<bool> running;
};

class PipetteClientNode : public rclcpp::Node
{
public:
  explicit PipetteClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PipetteClientNode();

private:
  // mDNS 相关
  bool startMDNSBrowser();
  void stopMDNSBrowser();
  static void DNSServiceBrowseReply(
    DNSServiceRef sdRef,
    DNSServiceFlags flags,
    uint32_t interfaceIndex,
    DNSServiceErrorType errorCode,
    const char *serviceName,
    const char *regtype,
    const char *replyDomain,
    void *context);
  static void DNSServiceResolveReply(
    DNSServiceRef sdRef,
    DNSServiceFlags flags,
    uint32_t interfaceIndex,
    DNSServiceErrorType errorCode,
    const char *fullname,
    const char *hosttarget,
    uint16_t port,
    uint16_t txtLen,
    const unsigned char *txtRecord,
    void *context);
  void mdnsLoop();
  void parseTxtRecord(const char *txtRecord, uint16_t txtLen, 
                      const std::string &name, const std::string &host, uint16_t port);
  
  // UDP 相关
  bool createUDPSocket();
  void closeUDPSocket();
  bool sendATCommand(int socket_fd, const struct sockaddr_in &addr, const std::string &command);
  std::string receiveResponse(int socket_fd, struct sockaddr_in &addr, int timeout_ms = 1000);
  void udpReceiveLoop();
  
  // 设备管理
  void addDevice(const std::string &sn, const PipetteDevice &device);
  void removeDevice(const std::string &sn);
  PipetteDevice* getDevice(const std::string &sn);
  void updateDeviceHeartbeat(const std::string &sn);
  std::vector<std::string> getAllDeviceSNs();
  
  // 话题发布
  void publishDeviceList();
  void publishCommandResult(const std::string &command, const std::string &response, 
                           bool success, const std::string &sn);
  
  // 服务回调
  void handleGetDeviceListService(
    const std::shared_ptr<pipette_client::srv::GetDeviceList::Request> request,
    std::shared_ptr<pipette_client::srv::GetDeviceList::Response> response);
  
  void handleSendATCommandService(
    const std::shared_ptr<pipette_client::srv::SendATCommand::Request> request,
    std::shared_ptr<pipette_client::srv::SendATCommand::Response> response);
  
  // 动作回调
  rclcpp_action::GoalResponse handleAspirateGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipette_client::action::Aspirate::Goal> goal);
  
  rclcpp_action::CancelResponse handleAspirateCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Aspirate>> goal_handle);
  
  void handleAspirateAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Aspirate>> goal_handle);
  
  rclcpp_action::GoalResponse handleDispenseGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipette_client::action::Dispense::Goal> goal);
  
  rclcpp_action::CancelResponse handleDispenseCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Dispense>> goal_handle);
  
  void handleDispenseAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Dispense>> goal_handle);
  
  rclcpp_action::GoalResponse handleMixGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipette_client::action::Mix::Goal> goal);
  
  rclcpp_action::CancelResponse handleMixCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Mix>> goal_handle);
  
  void handleMixAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::Mix>> goal_handle);
  
  rclcpp_action::GoalResponse handleEjectTipGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipette_client::action::EjectTip::Goal> goal);
  
  rclcpp_action::CancelResponse handleEjectTipCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::EjectTip>> goal_handle);
  
  void handleEjectTipAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipette_client::action::EjectTip>> goal_handle);
  
  // 辅助函数
  std::string buildATCommand(const std::string &cmd, int volume = 0, int speed = 0, int times = 0);
  bool executeATCommand(const std::string &sn, const std::string &command, 
                       std::string &response, int timeout_ms = 1000);
  
  // ROS2 组件
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<pipette_client::msg::DeviceList>::SharedPtr device_list_pub_;
  rclcpp::Publisher<pipette_client::msg::ATCommandResult>::SharedPtr command_result_pub_;
  
  // 服务
  rclcpp::Service<pipette_client::srv::GetDeviceList>::SharedPtr get_device_list_srv_;
  rclcpp::Service<pipette_client::srv::SendATCommand>::SharedPtr send_at_command_srv_;
  
  // 动作服务器
  rclcpp_action::Server<pipette_client::action::Aspirate>::SharedPtr aspirate_action_server_;
  rclcpp_action::Server<pipette_client::action::Dispense>::SharedPtr dispense_action_server_;
  rclcpp_action::Server<pipette_client::action::Mix>::SharedPtr mix_action_server_;
  rclcpp_action::Server<pipette_client::action::EjectTip>::SharedPtr eject_tip_action_server_;
  
  // mDNS 组件
  MDNSBrowser mdns_browser_;
  std::thread mdns_thread_;
  
  // UDP 组件
  int udp_socket_;
  std::atomic<bool> udp_running_;
  std::thread udp_thread_;
  
  // 设备管理
  std::unordered_map<std::string, PipetteDevice> devices_;  // SN -> device
  std::mutex devices_mutex_;
  
  // 配置参数
  std::string local_port_;      // 本地 UDP 端口
  int discovery_interval_;      // 发现间隔（秒）
};

}  // namespace pipette_client

#endif  // PIPETTE_CLIENT_NODE_HPP_
