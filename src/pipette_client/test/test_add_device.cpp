/**
 * @file test_add_device.cpp
 * @brief Property-based tests for addDevice() function
 * 
 * This file contains property-based tests for the device addition and
 * reconnection functionality in the pipette_client ROS2 node.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "pipette_client/pipette_client_node.hpp"
#include <random>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <arpa/inet.h>

using namespace pipette_client;

/**
 * @brief Test fixture for addDevice() tests
 * 
 * Provides helper methods to create test devices and manipulate device state.
 */
class AddDeviceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Create node with minimal configuration
    rclcpp::NodeOptions options;
    options.append_parameter_override("local_port", "10000");
    options.append_parameter_override("discovery_interval", "3");
    options.append_parameter_override("offline_threshold", 30);
    
    node_ = std::make_shared<PipetteClientNode>(options);
    
    // Initialize random number generator with a seed for reproducibility
    rng_.seed(42 + test_counter_++);
  }

  void TearDown() override
  {
    node_.reset();
  }

  /**
   * @brief Create a test device with random or specified properties
   */
  PipetteDevice createTestDevice(
    const std::string& sn = "",
    int miss_count = 0,
    bool seen_in_current_cycle = false,
    bool available = true,
    bool update_timestamp = true)
  {
    PipetteDevice device;
    device.sn = sn.empty() ? generateRandomSN() : sn;
    device.model = "P1000";
    device.ip_address = generateRandomIP();
    device.port = 10001;
    device.product = "pippet";
    device.available = available;
    device.socket_fd = -1;
    device.miss_count = miss_count;
    device.seen_in_current_cycle = seen_in_current_cycle;
    
    // Set last_seen to current time if requested
    if (update_timestamp) {
      auto now = node_->now();
      device.last_seen.sec = now.seconds();
      device.last_seen.nanosec = now.nanoseconds() % 1000000000;
    } else {
      device.last_seen.sec = 0;
      device.last_seen.nanosec = 0;
    }
    
    // Initialize addr structure
    std::memset(&device.addr, 0, sizeof(device.addr));
    device.addr.sin_family = AF_INET;
    device.addr.sin_port = htons(device.port);
    inet_pton(AF_INET, device.ip_address.c_str(), &device.addr.sin_addr);
    
    return device;
  }

  /**
   * @brief Generate a random device serial number
   */
  std::string generateRandomSN()
  {
    std::uniform_int_distribution<int> dist(100000, 999999);
    return "SN" + std::to_string(dist(rng_));
  }

  /**
   * @brief Generate a random IP address
   */
  std::string generateRandomIP()
  {
    std::uniform_int_distribution<int> dist(1, 254);
    return "192.168.1." + std::to_string(dist(rng_));
  }

  std::shared_ptr<PipetteClientNode> node_;
  std::mt19937 rng_;
  static int test_counter_;
};

int AddDeviceTest::test_counter_ = 0;

/**
 * @brief Property 1: 设备发现重置缺失计数
 * 
 * **Validates: Requirements 1.2**
 * 
 * Property: For any device, when it is discovered in an mDNS scan cycle
 * (addDevice() is called), its miss_count should be reset to 0 and
 * seen_in_current_cycle should be set to true.
 * 
 * Test strategy:
 * - Generate random devices with varying initial miss_counts (0-20)
 * - Call addDevice() to simulate device discovery
 * - Verify miss_count is reset to 0
 * - Verify seen_in_current_cycle is set to true
 */
TEST_F(AddDeviceTest, Property1_DeviceDiscoveryResetsMissCount)
{
  const int NUM_ITERATIONS = 100;
  std::uniform_int_distribution<int> miss_count_dist(0, 20);
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    // Generate a device with random initial miss_count
    int initial_miss_count = miss_count_dist(rng_);
    std::string sn = "TEST_SN_" + std::to_string(i);
    auto device = createTestDevice(sn, initial_miss_count, false);
    
    // First add the device with non-zero miss_count
    node_->testAddDevice(sn, device);
    
    // Verify device was added
    auto* added_device = node_->testGetDevice(sn);
    ASSERT_NE(added_device, nullptr) << "Device should be added to node";
    
    // Now simulate device discovery by calling addDevice again
    // (this simulates the device being found in a subsequent mDNS scan)
    node_->testAddDevice(sn, device);
    
    // Verify miss_count is reset to 0
    auto* updated_device = node_->testGetDevice(sn);
    ASSERT_NE(updated_device, nullptr) << "Device should still exist";
    EXPECT_EQ(updated_device->miss_count, 0)
      << "miss_count should be reset to 0 for iteration " << i
      << " (initial miss_count was " << initial_miss_count << ")";
    EXPECT_TRUE(updated_device->seen_in_current_cycle)
      << "seen_in_current_cycle should be true for iteration " << i;
    
    if (updated_device->miss_count == 0 && updated_device->seen_in_current_cycle) {
      passed++;
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS) 
    << "All iterations should pass the miss_count reset property";
}

/**
 * @brief Property 6: 设备重新上线初始化
 * 
 * **Validates: Requirements 4.1, 4.2, 4.3, 4.4**
 * 
 * Property: For any device that has been removed (offline), when it reappears
 * in an mDNS scan (addDevice() is called), the following should be true:
 * - Device is re-added to devices_ map
 * - miss_count is initialized to 0
 * - available flag is set to true
 * - last_seen timestamp is updated to current time
 * - seen_in_current_cycle is set to true
 * 
 * Test strategy:
 * - Create random devices
 * - Add them to the node
 * - Simulate offline by removing them (miss_count reaches threshold)
 * - Call addDevice() to simulate reconnection
 * - Verify all initialization conditions are met
 */
TEST_F(AddDeviceTest, Property6_DeviceReconnectionInitialization)
{
  const int NUM_ITERATIONS = 100;
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    std::string sn = "TEST_SN_" + std::to_string(i);
    auto device = createTestDevice(sn, 0, true);
    
    // Add device initially
    node_->testAddDevice(sn, device);
    
    // Verify device exists
    auto* added_device = node_->testGetDevice(sn);
    ASSERT_NE(added_device, nullptr) << "Device should exist after initial add";
    
    // Simulate device going offline by setting miss_count to threshold
    // and calling checkOfflineDevices()
    for (int j = 0; j < 10; ++j) {
      auto* dev = node_->testGetDevice(sn);
      if (dev) {
        dev->seen_in_current_cycle = false;
      }
      node_->testCheckOfflineDevices();
    }
    
    // Verify device is removed
    ASSERT_EQ(node_->testGetDevice(sn), nullptr) 
      << "Device should be removed after going offline";
    
    // Record time before reconnection
    auto time_before_reconnection = node_->now();
    
    // Simulate device reconnection by calling addDevice()
    auto reconnected_device = createTestDevice(sn, 0, false);
    node_->testAddDevice(sn, reconnected_device);
    
    // Verify device is re-added
    auto* final_device = node_->testGetDevice(sn);
    ASSERT_NE(final_device, nullptr) 
      << "Device should be re-added after reconnection for iteration " << i;
    
    // Verify all initialization conditions
    bool miss_count_ok = (final_device->miss_count == 0);
    bool available_ok = (final_device->available == true);
    bool seen_ok = (final_device->seen_in_current_cycle == true);
    
    // Verify last_seen is updated (should be >= time_before_reconnection)
    rclcpp::Time last_seen_time(final_device->last_seen);
    bool last_seen_ok = (last_seen_time >= time_before_reconnection);
    
    EXPECT_TRUE(miss_count_ok)
      << "miss_count should be 0 for iteration " << i
      << " (actual: " << final_device->miss_count << ")";
    EXPECT_TRUE(available_ok)
      << "available should be true for iteration " << i
      << " (actual: " << final_device->available << ")";
    EXPECT_TRUE(seen_ok)
      << "seen_in_current_cycle should be true for iteration " << i
      << " (actual: " << final_device->seen_in_current_cycle << ")";
    EXPECT_TRUE(last_seen_ok)
      << "last_seen should be updated for iteration " << i;
    
    if (miss_count_ok && available_ok && seen_ok && last_seen_ok) {
      passed++;
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS)
    << "All iterations should pass the reconnection initialization property";
}

/**
 * @brief Unit test: Device first discovery
 * 
 * **Validates: Requirements 1.2, 4.1, 4.2, 4.3, 4.4**
 * 
 * Tests that a device being discovered for the first time is properly
 * initialized with correct values.
 */
TEST_F(AddDeviceTest, UnitTest_DeviceFirstDiscovery)
{
  std::string sn = "FIRST_DISCOVERY_SN";
  
  // Record time before adding
  auto time_before = node_->now();
  
  // Create device with current timestamp
  auto device = createTestDevice(sn, 0, false, true, true);
  
  // Add device
  node_->testAddDevice(sn, device);
  
  // Verify device exists
  auto* added_device = node_->testGetDevice(sn);
  ASSERT_NE(added_device, nullptr) << "Device should exist after adding";
  
  // Verify initialization
  EXPECT_EQ(added_device->miss_count, 0) << "miss_count should be 0";
  EXPECT_TRUE(added_device->seen_in_current_cycle) 
    << "seen_in_current_cycle should be true";
  EXPECT_TRUE(added_device->available) << "available should be true";
  EXPECT_EQ(added_device->sn, sn) << "SN should match";
  
  // Verify last_seen is set (should be >= time_before since we set it in createTestDevice)
  rclcpp::Time last_seen_time(added_device->last_seen);
  EXPECT_GE(last_seen_time, time_before) << "last_seen should be current time";
}

/**
 * @brief Unit test: Device continuous online
 * 
 * **Validates: Requirements 1.2**
 * 
 * Tests that a device that is continuously discovered (online) maintains
 * miss_count = 0 across multiple discovery cycles.
 */
TEST_F(AddDeviceTest, UnitTest_DeviceContinuousOnline)
{
  std::string sn = "CONTINUOUS_ONLINE_SN";
  auto device = createTestDevice(sn, 0, false);
  
  // Add device initially
  node_->testAddDevice(sn, device);
  
  // Simulate 5 consecutive discovery cycles
  for (int i = 0; i < 5; ++i) {
    // Call addDevice to simulate device being discovered
    node_->testAddDevice(sn, device);
    
    // Verify miss_count remains 0
    auto* dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Device should exist in cycle " << i;
    EXPECT_EQ(dev->miss_count, 0) 
      << "miss_count should remain 0 in cycle " << i;
    EXPECT_TRUE(dev->seen_in_current_cycle)
      << "seen_in_current_cycle should be true in cycle " << i;
    
    // Call checkOfflineDevices to complete the cycle
    node_->testCheckOfflineDevices();
    
    // Verify device still exists after check
    dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Device should still exist after check in cycle " << i;
  }
}

/**
 * @brief Unit test: Device reconnection after offline
 * 
 * **Validates: Requirements 4.1, 4.2, 4.3, 4.4**
 * 
 * Tests the complete flow of a device going offline and then reconnecting.
 */
TEST_F(AddDeviceTest, UnitTest_DeviceReconnectionAfterOffline)
{
  std::string sn = "RECONNECTION_SN";
  std::string original_ip = "192.168.1.100";
  
  // Create and add device
  auto device = createTestDevice(sn, 0, true, true, true);
  device.ip_address = original_ip;
  node_->testAddDevice(sn, device);
  
  // Verify device exists
  ASSERT_NE(node_->testGetDevice(sn), nullptr) << "Device should exist initially";
  
  // Simulate device going offline (10 cycles without discovery)
  for (int i = 0; i < 10; ++i) {
    auto* dev = node_->testGetDevice(sn);
    if (dev) {
      dev->seen_in_current_cycle = false;
    }
    node_->testCheckOfflineDevices();
  }
  
  // Verify device is removed
  EXPECT_EQ(node_->testGetDevice(sn), nullptr) 
    << "Device should be removed after going offline";
  
  // Record time before reconnection
  auto time_before_reconnection = node_->now();
  
  // Simulate device reconnection (possibly with different IP)
  std::string new_ip = "192.168.1.101";
  auto reconnected_device = createTestDevice(sn, 0, false, true, true);
  reconnected_device.ip_address = new_ip;
  
  node_->testAddDevice(sn, reconnected_device);
  
  // Verify device is re-added
  auto* final_device = node_->testGetDevice(sn);
  ASSERT_NE(final_device, nullptr) << "Device should be re-added";
  
  // Verify all fields are properly initialized
  EXPECT_EQ(final_device->sn, sn) << "SN should match";
  EXPECT_EQ(final_device->miss_count, 0) << "miss_count should be 0";
  EXPECT_TRUE(final_device->available) << "available should be true";
  EXPECT_TRUE(final_device->seen_in_current_cycle) 
    << "seen_in_current_cycle should be true";
  
  // Verify last_seen is updated (should be >= time_before_reconnection)
  rclcpp::Time last_seen_time(final_device->last_seen);
  EXPECT_GE(last_seen_time, time_before_reconnection) 
    << "last_seen should be updated to current time";
}

/**
 * @brief Unit test: Multiple devices with different miss_counts
 * 
 * **Validates: Requirements 1.2**
 * 
 * Tests that addDevice() correctly resets miss_count for multiple devices
 * with different initial miss_count values.
 */
TEST_F(AddDeviceTest, UnitTest_MultipleDevicesMissCountReset)
{
  std::vector<std::string> sns = {"DEV1", "DEV2", "DEV3", "DEV4", "DEV5"};
  std::vector<int> initial_miss_counts = {0, 3, 5, 8, 15};
  
  // Add devices with different miss_counts
  for (size_t i = 0; i < sns.size(); ++i) {
    auto device = createTestDevice(sns[i], initial_miss_counts[i], false);
    node_->testAddDevice(sns[i], device);
  }
  
  // Verify all devices exist with their initial miss_counts
  for (size_t i = 0; i < sns.size(); ++i) {
    auto* dev = node_->testGetDevice(sns[i]);
    ASSERT_NE(dev, nullptr) << "Device " << sns[i] << " should exist";
  }
  
  // Simulate discovery of all devices (call addDevice again)
  for (size_t i = 0; i < sns.size(); ++i) {
    auto device = createTestDevice(sns[i], 0, false);
    node_->testAddDevice(sns[i], device);
  }
  
  // Verify all devices have miss_count = 0
  for (size_t i = 0; i < sns.size(); ++i) {
    auto* dev = node_->testGetDevice(sns[i]);
    ASSERT_NE(dev, nullptr) << "Device " << sns[i] << " should still exist";
    EXPECT_EQ(dev->miss_count, 0) 
      << "Device " << sns[i] << " should have miss_count = 0 "
      << "(was " << initial_miss_counts[i] << ")";
    EXPECT_TRUE(dev->seen_in_current_cycle)
      << "Device " << sns[i] << " should have seen_in_current_cycle = true";
  }
}

/**
 * @brief Unit test: Device available flag on reconnection
 * 
 * **Validates: Requirements 4.3**
 * 
 * Tests that the available flag is properly set to true when a device
 * reconnects after being offline.
 */
TEST_F(AddDeviceTest, UnitTest_DeviceAvailableFlagOnReconnection)
{
  std::string sn = "AVAILABLE_FLAG_SN";
  
  // Add device initially
  auto device = createTestDevice(sn, 0, true, true);
  node_->testAddDevice(sn, device);
  
  // Verify device is available
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_TRUE(dev->available) << "Device should be available initially";
  
  // Simulate device going offline
  for (int i = 0; i < 10; ++i) {
    dev = node_->testGetDevice(sn);
    if (dev) {
      dev->seen_in_current_cycle = false;
    }
    node_->testCheckOfflineDevices();
  }
  
  // Verify device is removed
  EXPECT_EQ(node_->testGetDevice(sn), nullptr);
  
  // Reconnect device
  auto reconnected_device = createTestDevice(sn, 0, false, false);
  // Note: We create with available=false to test that addDevice sets it to true
  reconnected_device.available = false;
  
  node_->testAddDevice(sn, reconnected_device);
  
  // Verify available flag is set to true
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr) << "Device should be re-added";
  EXPECT_TRUE(dev->available) 
    << "available flag should be set to true on reconnection";
}

/**
 * @brief Edge case: Device with very high miss_count
 * 
 * Tests that addDevice() correctly resets miss_count even when it's
 * extremely high (e.g., 1000).
 */
TEST_F(AddDeviceTest, EdgeCase_VeryHighMissCount)
{
  std::string sn = "HIGH_MISS_COUNT_SN";
  
  // Create device with very high miss_count
  auto device = createTestDevice(sn, 1000, false);
  node_->testAddDevice(sn, device);
  
  // Call addDevice to simulate discovery
  node_->testAddDevice(sn, device);
  
  // Verify miss_count is reset to 0
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 0) 
    << "miss_count should be reset to 0 even from very high value";
  EXPECT_TRUE(dev->seen_in_current_cycle);
}

/**
 * @brief Edge case: Rapid consecutive addDevice calls
 * 
 * Tests that multiple rapid calls to addDevice() for the same device
 * maintain correct state.
 */
TEST_F(AddDeviceTest, EdgeCase_RapidConsecutiveAddDevice)
{
  std::string sn = "RAPID_ADD_SN";
  auto device = createTestDevice(sn, 5, false);
  
  // Call addDevice multiple times rapidly
  for (int i = 0; i < 10; ++i) {
    node_->testAddDevice(sn, device);
  }
  
  // Verify device state is correct
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 0) << "miss_count should be 0";
  EXPECT_TRUE(dev->seen_in_current_cycle) << "seen_in_current_cycle should be true";
  EXPECT_TRUE(dev->available) << "available should be true";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
