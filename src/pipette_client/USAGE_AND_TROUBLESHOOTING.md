# pipette_client 使用说明和故障排查

## ⚠️ 重要提示

### 消息类型错误解决

如果您遇到类似错误：
```
The message type 'pipette_client/msg/DeviceList' is invalid
```

**解决方法**：

1. **确保已正确编译和 sourced**：
```bash
cd /home/franco/pipette_ros2_ws

# 清理并重新编译
colcon build --packages-select pipette_client --cmake-clean-cache

# 必须 source setup.bash
source install/setup.bash
```

2. **验证消息类型是否可用**：
```bash
ros2 interface show pipette_client/msg/DeviceList
```

应该看到消息定义输出。

3. **检查话题是否发布**：
```bash
# 先启动节点
ros2 launch pipette_client pipette_client_launch.py

# 在另一个终端查看话题
ros2 topic list | grep device_list
```

## 🚀 正确使用流程

### 步骤 1: 启动节点

```bash
cd /home/franco/pipette_ros2_ws
source install/setup.bash
ros2 launch pipette_client pipette_client_launch.py
```

**期望输出**：
```
[INFO] [pipette_client]: Pipette Client Node starting...
[INFO] [pipette_client]: Local UDP Port: 10000
[INFO] [pipette_client]: Discovery Interval: 30 seconds
[INFO] [pipette_client]: Starting mDNS browser for _coevos-pip._udp.local
[INFO] [pipette_client]: UDP socket created on port 10000
[INFO] [pipette_client]: mDNS event loop started
[INFO] [pipette_client]: UDP receive loop started
[INFO] [pipette_client]: Pipette Client Node started successfully
```

### 步骤 2: 在新终端查看话题

打开**新的终端窗口**：

```bash
cd /home/franco/pipette_ros2_ws
source install/setup.bash

# 查看话题列表
ros2 topic list

# 应该能看到：
# /ws/pipette/device_list
# /ws/pipette/command_result
```

### 步骤 3: 监听设备列表

```bash
# 持续监听（会等待设备连接）
ros2 topic echo /ws/pipette/device_list

# 或只接收一次就退出
ros2 topic echo /ws/pipette/device_list --once
```

**如果没有设备连接**，您可能需要等待一段时间，因为节点需要通过 mDNS 发现设备。

## 🔍 诊断命令

### 检查节点状态

```bash
# 查看节点是否运行
ros2 node list

# 查看节点详细信息
ros2 node info /pipette_client
```

### 检查话题

```bash
# 查看所有话题
ros2 topic list

# 查看话题类型
ros2 topic type /ws/pipette/device_list

# 查看话题发布信息
ros2 topic info /ws/pipette/device_list --verbose
```

### 检查服务

```bash
# 列出所有服务
ros2 service list

# 测试获取设备列表服务
ros2 service call /ws/pipette/service/get_device_list pipette_client/srv/GetDeviceList
```

### 检查动作

```bash
# 列出所有动作
ros2 action list

# 查看动作类型
ros2 action type /ws/pipette/action/aspirate
```

## ❌ 常见问题

### 问题 1: "message type is invalid"

**原因**：ROS2 环境未正确加载

**解决**：
```bash
# 1. 确保在当前终端执行了 source
source /home/franco/pipette_ros2_ws/install/setup.bash

# 2. 如果仍然不行，尝试重新编译
cd /home/franco/pipette_ros2_ws
colcon build --packages-select pipette_client
source install/setup.bash
```

### 问题 2: "topic does not appear to be published yet"

**原因**：
- 节点未启动
- 节点正在启动但未完成初始化
- 没有设备被发现（某些话题可能需要设备连接后才发布）

**解决**：
```bash
# 1. 确认节点正在运行
ps aux | grep pipette_client_node

# 2. 重新启动节点
ros2 launch pipette_client pipette_client_launch.py

# 3. 等待几秒钟让节点完成初始化
sleep 5

# 4. 在新终端查看话题
ros2 topic list
```

### 问题 3: 找不到任何设备

**可能原因**：
- 移液器设备未连接到网络
- mDNS 服务未运行
- 网络配置问题

**诊断**：
```bash
# 检查 mDNS 服务状态
systemctl status avahi-daemon

# 搜索 COEVOS 设备
avahi-browse -art | grep coevos

# 检查网络连接
ping <设备 IP 地址>
```

**解决**：
```bash
# 重启 mDNS 服务（如果需要 root 权限）
sudo systemctl restart avahi-daemon
```

### 问题 4: 服务调用超时

```bash
# 测试服务调用
timeout 5 ros2 service call /ws/pipette/service/get_device_list pipette_client/srv/GetDeviceList
```

如果超时，说明：
- 节点未运行
- 服务名称错误
- 网络通信问题

## 📝 完整的测试脚本

创建一个测试脚本来验证所有功能：

```bash
#!/bin/bash
# test_pipette_client.sh

echo "=== pipette_client 测试脚本 ==="

# 1. 检查环境
echo "[1/6] 检查 ROS2 环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "错误：ROS2 环境未加载"
    exit 1
fi
echo "✓ ROS2 环境正常 ($ROS_DISTRO)"

# 2. 检查消息类型
echo "[2/6] 检查消息类型..."
ros2 interface show pipette_client/msg/DeviceList > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ 消息类型可用"
else
    echo "✗ 消息类型不可用，请运行：source install/setup.bash"
    exit 1
fi

# 3. 启动节点（后台）
echo "[3/6] 启动节点..."
ros2 launch pipette_client pipette_client_launch.py &
NODE_PID=$!
sleep 3

# 4. 检查节点状态
echo "[4/6] 检查节点状态..."
ros2 node list | grep pipette_client > /dev/null
if [ $? -eq 0 ]; then
    echo "✓ 节点运行正常"
else
    echo "✗ 节点未运行"
    kill $NODE_PID
    exit 1
fi

# 5. 检查话题
echo "[5/6] 检查话题..."
TOPICS=$(ros2 topic list)
if echo "$TOPICS" | grep "device_list" > /dev/null; then
    echo "✓ 话题已发布"
else
    echo "⚠ 话题尚未发布（可能需要等待设备连接）"
fi

# 6. 检查服务
echo "[6/6] 检查服务..."
SERVICES=$(ros2 service list)
if echo "$SERVICES" | grep "get_device_list" > /dev/null; then
    echo "✓ 服务可用"
else
    echo "⚠ 服务不可用"
fi

# 清理
echo ""
echo "=== 测试完成 ==="
echo "停止节点..."
kill $NODE_PID

echo ""
echo "提示："
echo "- 使用 'ros2 topic echo /ws/pipette/device_list' 监听设备列表"
echo "- 使用 'ros2 service call /ws/pipette/service/get_device_list ...' 调用服务"
echo "- 使用 'ros2 action send_goal ...' 执行动作"
```

使用方法：
```bash
chmod +x test_pipette_client.sh
./test_pipette_client.sh
```

## 🎯 快速参考卡片

### 启动
```bash
source install/setup.bash
ros2 launch pipette_client pipette_client_launch.py
```

### 监听设备
```bash
ros2 topic echo /ws/pipette/device_list
```

### 获取设备列表
```bash
ros2 service call /ws/pipette/service/get_device_list pipette_client/srv/GetDeviceList
```

### 发送 AT 命令
```bash
ros2 service call /ws/pipette/service/send_at_command \
  pipette_client/srv/SendATCommand \
  "{command: 'AT+STATE?', target_sn: 'PIP-2024-001'}"
```

### 执行吸液动作
```bash
ros2 action send_goal /ws/pipette/action/aspirate \
  pipette_client/action/Aspirate \
  "{sn: 'PIP-2024-001', volume: 50000, speed: 5}"
```

## 📚 相关文档

- [README.md](README.md) - 完整使用文档
- [LIBRARY_ISOLATION.md](LIBRARY_ISOLATION.md) - 库文件隔离配置

---

**更新日期**: 2026-03-05  
**状态**: ✅ 已验证
