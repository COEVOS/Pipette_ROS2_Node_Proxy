# pipette_client - 移液器 UDP 客户端节点

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Version](https://img.shields.io/badge/version-0.0.1-orange.svg)]()

**pipette_client** 是一个 ROS2 节点，用于通过 **UDP 通信**和 **mDNS 服务发现**控制 COEVOS 品牌的电动移液器设备。

## 📖 功能特性

- ✅ **mDNS 自动发现** - 自动发现网络中的移液器设备
- ✅ **UDP 通信** - 高效的无线通信协议
- ✅ **AT 命令支持** - 直接发送 AT 指令控制设备
- ✅ **动作服务器** - 提供标准化的吸液、排液、混匀等动作接口
- ✅ **服务接口** - 提供设备列表查询和命令发送服务
- ✅ **实时状态发布** - 持续发布可用设备列表

## 🏗️ 系统架构

```
┌─────────────────┐      mDNS       ┌──────────────────┐
│  pipette_client │ ◄─────────────► │ 移液器设备 1     │
│   (ROS2 Node)   │      UDP        │ (COEVOS Pipette) │
└─────────────────┘                 └──────────────────┘
         │                                   ▲
         │ UDP                               │
         ▼                                   │
┌─────────────────┐                 ┌──────────────────┐
│ 移液器设备 2     │                 │ 移液器设备 N     │
└─────────────────┘                 └──────────────────┘
```

## 📦 安装

### 依赖项

确保已安装以下系统依赖：

```bash
# Avahi mDNS 库
sudo apt-get install libavahi-compat-libdnssd-dev

# ROS2 Humble 基础包
sudo apt-get install ros-humble-desktop
```

### 编译

```bash
cd ~/pipette_ros2_ws
colcon build --packages-select pipette_client
source install/setup.bash
```

## 🚀 快速开始

### 启动节点

#### 方法 1: 使用 launch 文件（推荐）

```bash
ros2 launch pipette_client pipette_client_launch.py
```

#### 方法 2: 直接运行节点

```bash
ros2 run pipette_client pipette_client_node
```

#### 方法 3: 自定义参数

```bash
ros2 launch pipette_client pipette_client_launch.py \
  local_port:=10000 \
  discovery_interval:=3
```

### 验证启动

成功启动后，您应该看到类似输出：

```
[INFO] [pipette_client]: Pipette Client Node starting...
[INFO] [pipette_client]: Local UDP Port: 10000
[INFO] [pipette_client]: Discovery Interval: 30 seconds
[INFO] [pipette_client]: Starting mDNS browser for _coevos-pip._udp.local
[INFO] [pipette_client]: UDP receiver thread started
[INFO] [pipette_client]: Successfully created UDP socket on port 10000
```

## 📡 话题（Topics）

### 发布的话题

#### `/ws/pipette/device_list` [pipette_client/msg/DeviceList]

定期发布可用的移液器设备列表。

**消息结构**:
```msg
PipetteDevice[] devices    # 设备列表
int32 count                # 设备数量
builtin_interfaces/Time timestamp  # 时间戳
```

**PipetteDevice 消息结构**:
```msg
string sn                  # 设备 SN 号
string model               # 型号
string ip_address          # IP 地址
int32 port                 # 端口号
string product             # 产品类型
builtin_interfaces/Time last_seen  # 最后发现时间
```

**示例数据**:
```yaml
devices:
  - sn: "YSGFD12506"
    model: "IDP S-1250"
    ip_address: "192.168.1.100"
    port: 10002
    product: "pippet"
    last_seen:
      sec: 1772705307
      nanosec: 3280772170
count: 1
timestamp:
  sec: 1772705357
  nanosec: 1728976279
```

**监听方法**:
```bash
ros2 topic echo /ws/pipette/device_list
```

---

#### `/ws/pipette/command_result` [pipette_client/msg/ATCommandResult]

发布 AT 命令的执行结果。

**消息结构**:
```msg
string command               # 执行的命令
string response              # 响应内容
bool success                 # 是否成功
string target_sn             # 目标设备 SN
builtin_interfaces/Time timestamp  # 时间戳
```

**监听方法**:
```bash
ros2 topic echo /ws/pipette/command_result
```

### 订阅的话题

该节点当前不订阅任何话题（主要使用服务和动作接口）。

## 🔧 服务（Services）

### `/ws/pipette/service/get_device_list` [pipette_client/srv/GetDeviceList]

获取当前可用的移液器设备列表。

**请求**: 无

**响应**:
```srv
PipetteDevice[] devices    # 设备列表
int32 count                # 设备数量
bool success               # 是否成功
string message             # 附加消息
builtin_interfaces/Time timestamp  # 时间戳
```

**调用示例**:
```bash
# 同步调用
ros2 service call /ws/pipette/service/get_device_list pipette_client/srv/GetDeviceList

# 查看输出
devices:
  - sn: "YSGFD12506"
    model: "IDP S-1250"
    ip_address: "192.168.1.100"
    port: 10002
    product: "pippet"
count: 1
success: true
message: "Successfully retrieved device list"
```

---

### `/ws/pipette/service/send_at_command` [pipette_client/srv/SendATCommand]

向指定设备发送原始 AT 命令。

**请求**:
```srv
string sn                  # 目标设备 SN 号
string command             # AT 命令（如 "AT+ACK"）
```

**响应**:
```srv
bool success               # 是否成功
string response            # 响应内容
string sn                  # 设备 SN 号
builtin_interfaces/Time timestamp  # 时间戳
```

**调用示例**:
```bash
# 查询设备状态
ros2 service call /ws/pipette/service/send_at_command pipette_client/srv/SendATCommand "{sn: 'YSGFD12506', command: 'AT+STATE?'}"

# 设置吸液速度
ros2 service call /ws/pipette/service/send_at_command pipette_client/srv/SendATCommand "{sn: 'YSGFD12506', command: 'AT+SPD=5'}"

# 发送 ACK 指令
ros2 service call /ws/pipette/service/send_at_command pipette_client/srv/SendATCommand "{sn: 'YSGFD12506', command: 'AT+ACK'}"
```

**常用 AT 命令**:
| 命令 | 说明 | 示例 | 应答 |
|------|------|------|------|
| `AT+ACK` | 搜索应答 | `AT+ACK` | OK |
| `AT+REBOOT` | 系统重启 | `AT+REBOOT` | OK |
| `AT+ASP V S` | 吸液（体积 V，速度 S） | `AT+ASP 100000 5` | OK/BSY |
| `AT+DIS V S` | 排液（体积 V，速度 S） | `AT+DIS 100000 5` | OK/BSY/ERR |
| `AT+EJTIP` | 退吸头 | `AT+EJTIP` | OK/ERR |
| `AT+BLOW` | 吹气 | `AT+BLOW` | OK/BSY/ERR |
| `AT+DRAIN` | 排空 | `AT+DRAIN` | OK/ERR |
| `AT+LQREM` | 查询枪头内剩余液体体积 | `AT+LQREM` | 12300\r\nOK |
| `AT+GABS D` | 吸头间距（0.1mm 单位） | `AT+GABS 50` | OK/ERR |
| `AT+MIX V S N` | 混匀（体积 V，速度 S，次数 N） | `AT+MIX 10000 5 3` | OK/ERR |

**注意**：体积单位是 0.01μL（例如 100000 = 1000μL），速度范围 1-10。

## 🎯 动作（Actions）

### `/ws/pipette/action/aspirate` [pipette_client/action/Aspirate]

控制移液器执行吸液操作。

**目标（Goal）**:
```action
string sn                  # 设备 SN 号
int32 volume               # 体积（0.01μL 单位，例如 100000 = 1000μL）
int32 speed                # 速度（1-10）
```

**结果（Result）**:
```action
bool success               # 是否成功
string response            # 响应内容
string result_sn           # 设备 SN 号
builtin_interfaces/Time result_timestamp  # 时间戳
```

**反馈（Feedback）**:
```action
int32 current_volume       # 当前已吸体积（0.01μL 单位）
```

**使用示例**:
```bash
# 吸液 1000μL，速度 5
ros2 action send_goal /ws/pipette/action/aspirate pipette_client/action/Aspirate \
  "{sn: 'YSGFD12506', volume: 100000, speed: 5}"

# 带反馈显示
ros2 action send_goal -f /ws/pipette/action/aspirate pipette_client/action/Aspirate \
  "{sn: 'YSGFD12506', volume: 100000, speed: 5}"
```

---

### `/ws/pipette/action/dispense` [pipette_client/action/Dispense]

控制移液器执行排液操作。

**目标（Goal）**:
```action
string sn                  # 设备 SN 号
int32 volume               # 体积（0.01μL 单位）
int32 speed                # 速度（1-10）
```

**结果（Result）**:
```action
bool success               # 是否成功
string response            # 响应内容
string result_sn           # 设备 SN 号
builtin_interfaces/Time result_timestamp  # 时间戳
```

**反馈（Feedback）**:
```action
int32 current_volume       # 当前已排体积（0.01μL 单位）
```

**使用示例**:
```bash
# 排液 1000μL，速度 10
ros2 action send_goal /ws/pipette/action/dispense pipette_client/action/Dispense \
  "{sn: 'YSGFD12506', volume: 100000, speed: 10}"

# 带反馈显示
ros2 action send_goal -f /ws/pipette/action/dispense pipette_client/action/Dispense \
  "{sn: 'YSGFD12506', volume: 100000, speed: 10}"
```

---

### `/ws/pipette/action/mix` [pipette_client/action/Mix]

控制移液器执行混匀操作。

**目标（Goal）**:
```action
string sn                  # 设备 SN 号
int32 volume               # 单次混匀体积（0.01μL 单位）
int32 speed                # 速度（1-10）
int32 times                # 循环次数
```

**结果（Result）**:
```action
bool success               # 是否成功
string response            # 响应内容
string result_sn           # 设备 SN 号
builtin_interfaces/Time result_timestamp  # 时间戳
```

**反馈（Feedback）**:
```action
int32 current_times        # 已完成的循环次数
```

**使用示例**:
```bash
# 混匀 3 次，每次 100μL，速度 6
ros2 action send_goal /ws/pipette/action/mix pipette_client/action/Mix \
  "{sn: 'YSGFD12506', volume: 10000, speed: 6, times: 3}"

# 带反馈显示
ros2 action send_goal -f /ws/pipette/action/mix pipette_client/action/Mix \
  "{sn: 'YSGFD12506', volume: 10000, speed: 6, times: 3}"
```

---

### `/ws/pipette/action/eject_tip` [pipette_client/action/EjectTip]

控制移液器弹出吸头。

**目标（Goal）**:
```action
string sn                  # 设备 SN 号
```

**结果（Result）**:
```action
bool success               # 是否成功
string response            # 响应内容
string result_sn           # 设备 SN 号
builtin_interfaces/Time result_timestamp  # 时间戳
```

**反馈（Feedback）**: 无

**使用示例**:
```bash
# 弹出吸头
ros2 action send_goal /ws/pipette/action/eject_tip pipette_client/action/EjectTip \
  "{sn: 'YSGFD12506'}"
```

## 📋 参数（Parameters）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `local_port` | string | `"10000"` | 本地 UDP 监听端口 |
| `discovery_interval` | string | `"30"` | mDNS 设备发现间隔（秒） |

## 🔍 监控和调试

### 查看节点信息

```bash
# 查看节点发布信息
ros2 node info /pipette_client

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 查看动作列表
ros2 action list
```

### 实时监控

```bash
# 监控设备列表发布频率
ros2 topic hz /ws/pipette/device_list

# 查看节点图
ros2 run rqt_graph rqt_graph
```

### 日志级别

```bash
# 设置 debug 级别日志
ros2 run pipette_client pipette_client_node --ros-args --log-level debug
```

## 💡 使用示例

### 示例 1: 完整的移液操作

```bash
# 1. 启动节点
ros2 launch pipette_client pipette_client_launch.py

# 2. 在另一个终端，查询设备列表
ros2 service call /ws/pipette/service/get_device_list pipette_client/srv/GetDeviceList

# 3. 吸液 200μL
ros2 action send_goal /ws/pipette/action/aspirate pipette_client/action/Aspirate \
  "{sn: 'YSGFD12506', volume: 20000, speed: 5}"

# 4. 等待吸液完成后，排液
ros2 action send_goal /ws/pipette/action/dispense pipette_client/action/Dispense \
  "{sn: 'YSGFD12506', volume: 20000, speed: 5}"

# 5. 弹出吸头
ros2 action send_goal /ws/pipette/action/eject_tip pipette_client/action/EjectTip \
  "{sn: 'YSGFD12506'}"
```

### 示例 2: 使用 Python 脚本

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from pipette_client.action import Aspirate, Dispense

class PipetteExample(Node):
    def __init__(self):
        super().__init__('pipette_example')
        self.aspirate_client = ActionClient(self, Aspirate, '/ws/pipette/action/aspirate')
        self.dispense_client = ActionClient(self, Dispense, '/ws/pipette/action/dispense')
    
    async def pipette_operation(self):
        # 等待动作服务器就绪
        await self.aspirate_client.wait_for_server()
        
        # 发送吸液目标
        goal_msg = Aspirate.Goal()
        goal_msg.sn = 'YSGFD12506'
        goal_msg.volume = 50000  # 500μL
        goal_msg.speed = 5
        
        future = self.aspirate_client.send_goal_async(goal_msg)
        await future
        
        if future.result().accepted:
            self.get_logger().info('吸液成功!')
        
        # TODO: 添加排液逻辑

def main():
    rclpy.init()
    node = PipetteExample()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

### 示例 3: 监听设备状态

```bash
# 持续监控设备列表
watch -n 1 'ros2 topic echo /ws/pipette/device_list --once'

# 或使用 rqt
rqt_topic -t /ws/pipette/device_list
```

## 🔍 监控和调试

### 查看节点信息

```bash
# 查看节点发布信息
ros2 node info /pipette_client

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 查看动作列表
ros2 action list
```

### 实时监控

```bash
# 监控设备列表发布频率
ros2 topic hz /ws/pipette/device_list

# 查看节点图
ros2 run rqt_graph rqt_graph
```

### 日志级别

```bash
# 设置 debug 级别日志
ros2 run pipette_client pipette_client_node --ros-args --log-level debug
```

## 🛠️ 故障排查

### 问题 1: 找不到设备

**可能原因**:
- 设备未连接到同一网络
- mDNS 服务未运行
- 防火墙阻止 UDP 通信

**解决方法**:
```bash
# 检查网络连接
ping <设备 IP>

# 检查 mDNS 服务
systemctl status avahi-daemon

# 查看发现的设备
avahi-browse -art | grep coevos-pip
```

### 问题 2: UDP 通信失败

**检查端口**:
```bash
# 查看端口是否被占用
netstat -ulnp | grep 10000

# 测试 UDP 连通性
nc -u <设备 IP> 5000
```

### 问题 3: 动作执行超时

**可能原因**:
- 设备电池电量低
- 网络延迟高
- 设备处于错误状态

**解决方法**:
```bash
# 查看设备状态
ros2 service call /ws/pipette/service/send_at_command pipette_client/srv/SendATCommand \
  "{sn: 'YSGFD12506', command: 'AT+STATE?'}"

# 查询电量
ros2 service call /ws/pipette/service/send_at_command pipette_client/srv/SendATCommand \
  "{sn: 'YSGFD12506', command: 'AT+BAT?'}"
```

## 📚 相关文件

- [`src/pipette_client_node.cpp`](src/pipette_client_node.cpp) - 主节点实现
- [`include/pipette_client/`](include/pipette_client/) - 头文件
- [`msg/`](msg/) - 自定义消息定义
- [`srv/`](srv/) - 自定义服务定义
- [`action/`](action/) - 自定义动作定义
- [`launch/`](launch/) - Launch 启动文件
- [`LIBRARY_ISOLATION.md`](LIBRARY_ISOLATION.md) - 库文件隔离配置说明

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

本项目采用 Apache-2.0 许可证。

## 👥 维护者

- franco <djc.bj2012@gmail.com>

## 🙏 致谢

---

**最后更新**: 2026-03-05  
**版本**: 0.0.1
