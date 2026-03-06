cmake_minimum_required(VERSION 3.8)
project(pipette_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find Avahi library using Apple Bonjour compatibility layer
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVAHI REQUIRED avahi-compat-libdns_sd)

if(NOT AVAHI_FOUND)
  find_library(AVAHI_COMPAT_LIB avahi-compat-libdns_sd)
  if(NOT AVAHI_COMPAT_LIB)
    message(FATAL_ERROR "Avahi compat library not found. Install with: sudo apt-get install libavahi-compat-libdnssd-dev")
  endif()
  set(AVAHI_LIB ${AVAHI_COMPAT_LIB})
else()
  set(AVAHI_LIB ${AVAHI_LIBRARIES})
endif()

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PipetteDevice.msg"
  "msg/DeviceList.msg"
  "msg/ATCommandResult.msg"
  "srv/GetDeviceList.srv"
  "srv/SendATCommand.srv"
  "action/Aspirate.action"
  "action/Dispense.action"
  "action/Mix.action"
  "action/EjectTip.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

add_library(pipette_client_lib src/pipette_client_node.cpp)
target_compile_features(pipette_client_lib PUBLIC c_std_99 cxx_std_17)
target_include_directories(pipette_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pipette_client_lib ${AVAHI_LIB})
ament_target_dependencies(
  pipette_client_lib
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "builtin_interfaces"
  "action_msgs"
)

# Add rosidl support
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pipette_client_lib "${cpp_typesupport_target}")

# Create executable
add_executable(pipette_client_node src/main.cpp)
target_link_libraries(pipette_client_node pipette_client_lib)

# 设置 RPATH，让可执行文件能在安装后的 lib/pipette_client/ 目录找到库文件
set_target_properties(pipette_client_node PROPERTIES
  BUILD_RPATH "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 安装 targets
install(TARGETS pipette_client_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS pipette_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Find GTest
  find_package(ament_cmake_gtest REQUIRED)
  
  # Property-based tests for checkOfflineDevices
  ament_add_gtest(test_check_offline_devices
    test/test_check_offline_devices.cpp
  )
  target_compile_definitions(test_check_offline_devices PRIVATE BUILD_TESTING)
  target_link_libraries(test_check_offline_devices
    pipette_client_lib
  )
  target_include_directories(test_check_offline_devices PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_check_offline_devices
    rclcpp
    rclcpp_action
    std_msgs
    builtin_interfaces
    action_msgs
  )
  rosidl_get_typesupport_target(cpp_typesupport_target_test
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(test_check_offline_devices "${cpp_typesupport_target_test}")
  
  # Property-based tests for addDevice
  ament_add_gtest(test_add_device
    test/test_add_device.cpp
  )
  target_compile_definitions(test_add_device PRIVATE BUILD_TESTING)
  target_link_libraries(test_add_device
    pipette_client_lib
  )
  target_include_directories(test_add_device PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_add_device
    rclcpp
    rclcpp_action
    std_msgs
    builtin_interfaces
    action_msgs
  )
  target_link_libraries(test_add_device "${cpp_typesupport_target_test}")
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  pipette_client_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
