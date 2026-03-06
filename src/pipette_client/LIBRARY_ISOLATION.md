# pipette_client 库文件隔离方案

## 问题解决

### 原始问题
将库文件安装到 `lib/pipette_client/` 子目录后，启动节点时出现错误：
```
error while loading shared libraries: libpipette_client_lib.so: cannot open shared object file: No such file or directory
```

### 原因分析
- **库文件位置**: `install/pipette_client/lib/pipette_client/libpipette_client_lib.so`
- **可执行文件位置**: `install/pipette_client/lib/pipette_client/pipette_client_node`
- Linux 动态链接器默认只在标准路径（如 `/usr/lib`, `/lib`）查找库文件
- 需要告诉可执行文件去当前目录查找库文件

## 解决方案

### CMakeLists.txt 修改

在 `CMakeLists.txt` 中为可执行文件设置 RPATH：

```cmake
# Create executable
add_executable(pipette_client_node src/main.cpp)
target_link_libraries(pipette_client_node pipette_client_lib)

# 设置 RPATH，让可执行文件能在安装后的 lib/pipette_client/ 目录找到库文件
set_target_properties(pipette_client_node PROPERTIES
  BUILD_RPATH "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)
```

### RPATH 说明

#### BUILD_RPATH
- **作用**: 构建时的库文件查找路径
- **值**: `${CMAKE_BINARY_DIR}` - 指向编译输出目录
- **用途**: 允许在 build 目录直接运行可执行文件进行测试

#### INSTALL_RPATH  
- **作用**: 安装后的库文件查找路径
- **值**: `$ORIGIN` - 表示可执行文件所在目录
- **用途**: 安装后，可执行文件会在自己的目录下查找库文件

## 最终文件结构

```
install/pipette_client/
├── lib/
│   └── pipette_client/           # 所有自定义库都在此子目录
│       ├── libpipette_client_lib.so      # ✅ 您的库文件
│       └── pipette_client_node           # ✅ 您的可执行文件
└── local/
    └── lib/python3.10/dist-packages/
        └── pipette_client/       # Python 绑定和 ROS2 消息类型支持
            ├── libpipette_client__rosidl_generator_py.so
            └── ... (其他 .so 文件)
```

## 验证方法

### 1. 检查 RPATH 设置
```bash
readelf -d install/pipette_client/lib/pipette_client/pipette_client_node | grep RUNPATH
# 输出：0x000000000000001d (RUNPATH) Library runpath: [$ORIGIN]
```

### 2. 测试节点启动
```bash
cd /home/franco/pipette_ros2_ws
source install/setup.bash
ros2 run pipette_client pipette_client_node
```

应该看到：
```
[INFO] [pipette_client]: Pipette Client Node starting...
[INFO] [pipette_client]: Local UDP Port: 10000
[INFO] [pipette_client]: Discovery Interval: 30 seconds
```

### 3. 查看依赖的库
```bash
ldd install/pipette_client/lib/pipette_client/pipette_client_node | grep pipette
# 输出：libpipette_client_lib.so => $ORIGIN/libpipette_client_lib.so
```

## 优点

### ✅ 文件隔离
- 所有自定义库文件都在 `lib/pipette_client/` 子目录
- 不会与 ROS2 系统库（如 `librclcpp.so`, `librmw.so` 等）混在一起
- 清晰的目录结构，便于维护和管理

### ✅ 避免冲突
- 如果有其他包也使用了同名的库，不会发生冲突
- 每个包的库都在自己的子目录中

### ✅ 符合规范
- 仍然遵循 ROS2 的包结构约定
- 可以通过 `ros2 run`, `ros2 launch` 等标准命令使用

## 技术细节

### 动态链接查找顺序

Linux 动态链接器 (`ld.so`) 查找共享库的顺序：

1. **RPATH** (如果设置了) - 最高优先级
2. **LD_LIBRARY_PATH** 环境变量
3. **RUNPATH** (如果设置了) - 我们使用的就是这个
4. `/etc/ld.so.cache` 中缓存的路径
5. 默认系统路径 (`/lib`, `/usr/lib` 等)

### $ORIGIN 的含义

- `$ORIGIN` 是一个特殊标记
- 表示可执行文件所在的目录
- 例如：如果可执行文件在 `/opt/ros2_ws/install/my_pkg/lib/my_pkg/exe`
- 那么 `$ORIGIN` 就指向 `/opt/ros2_ws/install/my_pkg/lib/my_pkg/`
- 链接器会在这个目录下查找所需的 `.so` 文件

### 为什么不用 ldconfig

传统方法是使用 `ldconfig` 注册库文件路径，但：
- ❌ 需要 root 权限
- ❌ 会影响系统全局环境
- ❌ 不适合开发环境和多个版本共存

使用 RPATH 的优势：
- ✅ 不需要 root 权限
- ✅ 只影响特定的可执行文件
- ✅ 适合开发和部署

## 相关文件

- [`CMakeLists.txt`](../src/pipette_client/CMakeLists.txt) - 配置了 RPATH 设置
- [`package.xml`](../src/pipette_client/package.xml) - 包描述文件

## 故障排查

### 问题：仍然找不到库文件

**检查**：
```bash
# 1. 确认库文件确实存在
ls -la install/pipette_client/lib/pipette_client/libpipette_client_lib.so

# 2. 确认 RPATH 已设置
readelf -d install/pipette_client/lib/pipette_client/pipette_client_node | grep -E "(RPATH|RUNPATH)"

# 3. 清理并重新编译
rm -rf build/pipette_client install/pipette_client
colcon build --packages-select pipette_client
```

### 问题：编译时能找到，运行时找不到

这是因为没有正确设置 INSTALL_RPATH。确保：
```cmake
set_target_properties(pipette_client_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)
```

## 总结

通过设置 RPATH，我们成功实现了：
1. ✅ 库文件隔离到 `lib/pipette_client/` 子目录
2. ✅ 不与 ROS2 系统库混在一起
3. ✅ 可执行文件能正确找到库文件
4. ✅ 保持了 ROS2 的标准包结构

这是一个完整、优雅的解决方案！🎉

---

**更新日期**: 2026-03-05  
**状态**: ✅ 已解决并验证通过
