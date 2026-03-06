#!/bin/bash

# 测试离线检测功能的脚本

echo "=========================================="
echo "测试离线检测功能"
echo "=========================================="
echo ""
echo "配置信息："
echo "  - 离线阈值: 30 次"
echo "  - 扫描间隔: 3 秒"
echo "  - 预期超时: 90 秒"
echo ""
echo "启动节点并观察日志..."
echo "=========================================="
echo ""

# 设置 ROS2 日志级别为 INFO
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1

# 运行节点
source install/setup.bash
ros2 run pipette_client pipette_client --ros-args \
  -p local_port:=10000 \
  -p discovery_interval:=3 \
  -p offline_threshold:=30 \
  --log-level info

