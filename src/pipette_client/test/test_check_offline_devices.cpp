/**
 * @file test_check_offline_devices.cpp
 * @brief Property-based tests for checkOfflineDevices() function
 * 
 * This file contains property-based tests for the offline device detection
 * functionality in the pipette_client ROS2 node.
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
 * @brief Test fixture for offline device detection tests
 * 
 * Provides helper methods to manipulate device state for testing purposes.
 */
class OfflineDetectionTest : public ::testing::Test
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
    bool seen_in_current_cycle = false)
  {
    PipetteDevice device;
    device.sn = sn.empty() ? generateRandomSN() : sn;
    device.model = "P1000";
    device.ip_address = generateRandomIP();
    device.port = 10001;
    device.product = "pippet";
    device.available = true;
    device.socket_fd = -1;
    device.miss_count = miss_count;
    device.seen_in_current_cycle = seen_in_current_cycle;
    
    // Set last_seen to current time
    auto now = node_->now();
    device.last_seen.sec = now.seconds();
    device.last_seen.nanosec = now.nanoseconds() % 1000000000;
    
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

int OfflineDetectionTest::test_counter_ = 0;

/**
 * @brief Property 2: 未发现设备增加缺失计数
 * 
 * **Validates: Requirements 1.3**
 * 
 * Property: For any device, when it is not seen in the current scan cycle
 * (seen_in_current_cycle = false), its miss_count should be incremented by 1
 * when checkOfflineDevices() is called.
 * 
 * Test strategy:
 * - Generate random devices with varying initial miss_counts (0-9)
 * - Set seen_in_current_cycle to false
 * - Call checkOfflineDevices()
 * - Verify miss_count increased by exactly 1
 */
TEST_F(OfflineDetectionTest, Property2_MissingDeviceIncrementsMissCount)
{
  const int NUM_ITERATIONS = 100;
  std::uniform_int_distribution<int> miss_count_dist(0, 9);
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    // Generate a device with random initial miss_count < threshold
    int initial_miss_count = miss_count_dist(rng_);
    std::string sn = "TEST_SN_" + std::to_string(i);
    auto device = createTestDevice(sn, initial_miss_count, false);
    
    // Add device to node
    node_->testAddDevice(sn, device);
    
    // Verify device was added
    auto* added_device = node_->testGetDevice(sn);
    ASSERT_NE(added_device, nullptr) << "Device should be added to node";
    ASSERT_EQ(added_device->miss_count, initial_miss_count) 
      << "Initial miss_count should match";
    
    // Call checkOfflineDevices()
    node_->testCheckOfflineDevices();
    
    // Verify miss_count increased by 1
    auto* updated_device = node_->testGetDevice(sn);
    ASSERT_NE(updated_device, nullptr) << "Device should still exist";
    EXPECT_EQ(updated_device->miss_count, initial_miss_count + 1)
      << "miss_count should increment by 1 for iteration " << i;
    
    if (updated_device->miss_count == initial_miss_count + 1) {
      passed++;
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS) 
    << "All iterations should pass the miss_count increment property";
}

/**
 * @brief Property 3: 达到阈值判定离线
 * 
 * **Validates: Requirements 2.1, 3.1**
 * 
 * Property: For any device, when its miss_count reaches or exceeds the
 * offline_threshold, the device should be removed from the devices_ map
 * after checkOfflineDevices() is called.
 * 
 * Test strategy:
 * - Generate random thresholds (1-20)
 * - Create devices with miss_count = threshold - 1
 * - Set seen_in_current_cycle to false
 * - Call checkOfflineDevices()
 * - Verify device is removed from the list
 */
TEST_F(OfflineDetectionTest, Property3_ThresholdTriggersRemoval)
{
  const int NUM_ITERATIONS = 100;
  std::uniform_int_distribution<int> threshold_dist(1, 20);
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    int threshold = threshold_dist(rng_);
    std::string sn = "TEST_SN_" + std::to_string(i);
    
    // Set the threshold for this test
    node_->testSetOfflineThreshold(threshold);
    
    // Create a device at threshold - 1
    auto device = createTestDevice(sn, threshold - 1, false);
    
    // Add device to node
    node_->testAddDevice(sn, device);
    
    // Verify device exists
    ASSERT_NE(node_->testGetDevice(sn), nullptr) << "Device should exist before check";
    
    // Call checkOfflineDevices() - this should increment miss_count to threshold
    node_->testCheckOfflineDevices();
    
    // Verify device is removed
    auto* removed_device = node_->testGetDevice(sn);
    EXPECT_EQ(removed_device, nullptr)
      << "Device should be removed when miss_count reaches threshold " 
      << threshold << " for iteration " << i;
    
    if (removed_device == nullptr) {
      passed++;
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS)
    << "All iterations should pass the threshold removal property";
}

/**
 * @brief Property 4: 离线设备资源清理
 * 
 * **Validates: Requirements 3.2**
 * 
 * Property: For any device being removed, if it has an independent socket
 * (socket_fd >= 0 && socket_fd != udp_socket_), that socket should be closed.
 * 
 * Test strategy:
 * - Create devices with independent sockets
 * - Set miss_count to threshold
 * - Call checkOfflineDevices()
 * - Verify socket is closed (check file descriptor validity)
 * 
 * Note: We test socket closure by creating pipe file descriptors and
 * verifying they become invalid after device removal.
 */
TEST_F(OfflineDetectionTest, Property4_OfflineDeviceResourceCleanup)
{
  const int NUM_ITERATIONS = 50;
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    std::string sn = "TEST_SN_" + std::to_string(i);
    
    // Create a device with an independent socket
    auto device = createTestDevice(sn, 9, false);  // One below threshold
    
    // Create a dummy socket using a pipe for testing
    int fds[2];
    ASSERT_EQ(pipe(fds), 0) << "Failed to create pipe for testing";
    
    device.socket_fd = fds[0];
    close(fds[1]);  // Close write end immediately
    
    // Verify socket is valid before removal
    int flags = fcntl(device.socket_fd, F_GETFD);
    ASSERT_NE(flags, -1) << "Socket should be valid before removal";
    
    // Add device to node
    node_->testAddDevice(sn, device);
    
    // Call checkOfflineDevices() - this should remove the device and close socket
    node_->testCheckOfflineDevices();
    
    // Verify device is removed
    EXPECT_EQ(node_->testGetDevice(sn), nullptr) << "Device should be removed";
    
    // Verify socket is closed (fcntl should fail with EBADF)
    flags = fcntl(fds[0], F_GETFD);
    bool socket_closed = (flags == -1 && errno == EBADF);
    
    EXPECT_TRUE(socket_closed)
      << "Socket should be closed after device removal for iteration " << i;
    
    if (socket_closed) {
      passed++;
    } else {
      // Clean up if socket wasn't closed
      close(fds[0]);
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS)
    << "All iterations should pass the resource cleanup property";
}

/**
 * @brief Property 10: 离线检测幂等性
 * 
 * **Validates: Requirements 2.1, 3.1 (implicit)**
 * 
 * Property: For any device state, calling checkOfflineDevices() twice
 * consecutively (without new device discoveries in between) should produce
 * the same result. Devices already removed should not be processed again.
 * 
 * Test strategy:
 * - Create a random set of devices (0-10 devices)
 * - Set random miss_counts and seen_in_current_cycle flags
 * - Call checkOfflineDevices() twice
 * - Verify the device list is identical after both calls
 */
TEST_F(OfflineDetectionTest, Property10_OfflineDetectionIdempotence)
{
  const int NUM_ITERATIONS = 100;
  std::uniform_int_distribution<int> num_devices_dist(0, 10);
  std::uniform_int_distribution<int> miss_count_dist(0, 15);
  std::uniform_int_distribution<int> bool_dist(0, 1);
  
  int passed = 0;
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    int num_devices = num_devices_dist(rng_);
    
    // Create random devices
    for (int j = 0; j < num_devices; ++j) {
      std::string sn = "TEST_SN_" + std::to_string(i) + "_" + std::to_string(j);
      auto device = createTestDevice(sn,
                                     miss_count_dist(rng_),
                                     bool_dist(rng_) == 1);
      node_->testAddDevice(sn, device);
    }
    
    // Call checkOfflineDevices() first time
    node_->testCheckOfflineDevices();
    
    // Get device list after first call
    auto devices_after_first = node_->testGetAllDeviceSNs();
    std::sort(devices_after_first.begin(), devices_after_first.end());
    
    // Call checkOfflineDevices() second time (without new discoveries)
    node_->testCheckOfflineDevices();
    
    // Get device list after second call
    auto devices_after_second = node_->testGetAllDeviceSNs();
    std::sort(devices_after_second.begin(), devices_after_second.end());
    
    // Verify both lists are identical
    bool lists_match = (devices_after_first == devices_after_second);
    EXPECT_TRUE(lists_match)
      << "Device lists should be identical after consecutive checkOfflineDevices() calls "
      << "for iteration " << i;
    
    if (lists_match) {
      passed++;
    }
  }
  
  EXPECT_EQ(passed, NUM_ITERATIONS)
    << "All iterations should pass the idempotence property";
}

/**
 * @brief Integration test: Complete offline detection cycle
 * 
 * Tests the complete flow of device discovery, miss count tracking,
 * and offline detection over multiple scan cycles.
 */
TEST_F(OfflineDetectionTest, IntegrationTest_CompleteOfflineCycle)
{
  std::string sn = "INTEGRATION_TEST_SN";
  
  // Create a device and add it
  auto device = createTestDevice(sn, 0, true);
  node_->testAddDevice(sn, device);
  
  // Verify device exists
  ASSERT_NE(node_->testGetDevice(sn), nullptr) << "Device should exist initially";
  
  // Simulate 9 scan cycles without seeing the device
  for (int i = 0; i < 9; ++i) {
    // Manually set seen_in_current_cycle to false
    auto* dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr);
    dev->seen_in_current_cycle = false;
    
    node_->testCheckOfflineDevices();
    
    // Verify device still exists
    dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Device should still exist after " << (i + 1) << " cycles";
    EXPECT_EQ(dev->miss_count, i + 1) << "miss_count should be " << (i + 1);
  }
  
  // Verify device still exists with miss_count = 9
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 9);
  
  // Simulate 10th scan cycle
  dev->seen_in_current_cycle = false;
  node_->testCheckOfflineDevices();
  
  // Verify device is removed
  EXPECT_EQ(node_->testGetDevice(sn), nullptr) 
    << "Device should be removed after 10th cycle";
}

/**
 * @brief Edge case: Empty device list
 * 
 * Verifies that checkOfflineDevices() handles an empty device list gracefully.
 */
TEST_F(OfflineDetectionTest, EdgeCase_EmptyDeviceList)
{
  // Verify device list is empty
  EXPECT_EQ(node_->testGetDeviceCount(), 0) << "Device list should be empty initially";
  
  // Call checkOfflineDevices() on empty device list
  EXPECT_NO_THROW(node_->testCheckOfflineDevices()) 
    << "checkOfflineDevices() should handle empty list gracefully";
  
  // Verify list is still empty
  EXPECT_EQ(node_->testGetDeviceCount(), 0) << "Device list should still be empty";
}

/**
 * @brief Edge case: All devices at threshold boundary
 * 
 * Tests behavior when all devices are at miss_count = threshold - 1.
 */
TEST_F(OfflineDetectionTest, EdgeCase_AllDevicesAtBoundary)
{
  const int NUM_DEVICES = 5;
  std::vector<std::string> sns;
  
  // Create devices at threshold - 1
  for (int i = 0; i < NUM_DEVICES; ++i) {
    std::string sn = "BOUNDARY_SN_" + std::to_string(i);
    sns.push_back(sn);
    auto device = createTestDevice(sn, 9, false);  // threshold - 1
    node_->testAddDevice(sn, device);
  }
  
  // Verify all devices exist
  EXPECT_EQ(node_->testGetDeviceCount(), NUM_DEVICES);
  
  // Call checkOfflineDevices() - should increment all to threshold
  node_->testCheckOfflineDevices();
  
  // Verify all devices are removed
  EXPECT_EQ(node_->testGetDeviceCount(), 0) 
    << "All devices should be removed when reaching threshold";
  
  for (const auto& sn : sns) {
    EXPECT_EQ(node_->testGetDevice(sn), nullptr) 
      << "Device " << sn << " should be removed";
  }
}

/**
 * @brief Unit test: Device offline detection scenario
 * 
 * **Validates: Requirements 1.3, 2.1, 3.1**
 * 
 * Tests the basic offline detection flow:
 * - Device is added and initially online
 * - Device is not seen for multiple cycles
 * - Device is removed when threshold is reached
 */
TEST_F(OfflineDetectionTest, UnitTest_DeviceOfflineDetection)
{
  std::string sn = "OFFLINE_TEST_SN";
  
  // Add a device with miss_count = 0
  auto device = createTestDevice(sn, 0, true);
  node_->testAddDevice(sn, device);
  
  // Verify device exists
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr) << "Device should exist after adding";
  EXPECT_EQ(dev->miss_count, 0) << "Initial miss_count should be 0";
  
  // Simulate device not being seen for 9 cycles (below threshold)
  for (int i = 0; i < 9; ++i) {
    dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr);
    dev->seen_in_current_cycle = false;
    
    node_->testCheckOfflineDevices();
    
    // Device should still exist
    dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Device should exist after " << (i + 1) << " cycles";
    EXPECT_EQ(dev->miss_count, i + 1) << "miss_count should be " << (i + 1);
  }
  
  // Simulate 10th cycle - device should be removed
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  dev->seen_in_current_cycle = false;
  
  node_->testCheckOfflineDevices();
  
  // Verify device is removed
  EXPECT_EQ(node_->testGetDevice(sn), nullptr) 
    << "Device should be removed after reaching threshold";
}

/**
 * @brief Unit test: Boundary condition (miss_count = threshold - 1)
 * 
 * **Validates: Requirements 1.3, 2.1, 3.1**
 * 
 * Tests that a device at exactly threshold - 1 is NOT removed,
 * but is removed when it reaches the threshold.
 */
TEST_F(OfflineDetectionTest, UnitTest_BoundaryCondition)
{
  std::string sn = "BOUNDARY_TEST_SN";
  int threshold = 10;
  
  // Add a device at threshold - 1
  auto device = createTestDevice(sn, threshold - 1, false);
  node_->testAddDevice(sn, device);
  
  // Verify device exists with miss_count = 9
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, threshold - 1);
  
  // Call checkOfflineDevices() - should increment to threshold and remove
  node_->testCheckOfflineDevices();
  
  // Verify device is removed
  EXPECT_EQ(node_->testGetDevice(sn), nullptr)
    << "Device should be removed when miss_count reaches threshold";
  
  // Test the opposite: device at threshold - 2 should NOT be removed
  std::string sn2 = "BOUNDARY_TEST_SN_2";
  auto device2 = createTestDevice(sn2, threshold - 2, false);
  node_->testAddDevice(sn2, device2);
  
  node_->testCheckOfflineDevices();
  
  // Verify device still exists with miss_count = threshold - 1
  dev = node_->testGetDevice(sn2);
  ASSERT_NE(dev, nullptr) << "Device should NOT be removed at threshold - 1";
  EXPECT_EQ(dev->miss_count, threshold - 1);
}

/**
 * @brief Unit test: Multiple devices with mixed states
 * 
 * **Validates: Requirements 1.3, 2.1, 3.1**
 * 
 * Tests offline detection with multiple devices in different states:
 * - Some devices continuously online (seen_in_current_cycle = true)
 * - Some devices intermittently offline (miss_count < threshold)
 * - Some devices reaching threshold (should be removed)
 */
TEST_F(OfflineDetectionTest, UnitTest_MultipleDevicesMixedStates)
{
  // Create 5 devices with different states
  std::vector<std::string> online_devices = {"ONLINE_1", "ONLINE_2"};
  std::vector<std::string> intermittent_devices = {"INTERMITTENT_1", "INTERMITTENT_2"};
  std::string offline_device = "OFFLINE_1";
  
  // Add online devices (miss_count = 0, seen = true)
  for (const auto& sn : online_devices) {
    auto device = createTestDevice(sn, 0, true);
    node_->testAddDevice(sn, device);
  }
  
  // Add intermittent devices (miss_count = 5, seen = false)
  for (const auto& sn : intermittent_devices) {
    auto device = createTestDevice(sn, 5, false);
    node_->testAddDevice(sn, device);
  }
  
  // Add offline device (miss_count = 9, seen = false)
  auto device = createTestDevice(offline_device, 9, false);
  node_->testAddDevice(offline_device, device);
  
  // Verify all devices exist
  EXPECT_EQ(node_->testGetDeviceCount(), 5);
  
  // Call checkOfflineDevices()
  node_->testCheckOfflineDevices();
  
  // Verify online devices still have miss_count = 0
  for (const auto& sn : online_devices) {
    auto* dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Online device " << sn << " should still exist";
    EXPECT_EQ(dev->miss_count, 0) << "Online device should have miss_count = 0";
    EXPECT_FALSE(dev->seen_in_current_cycle) 
      << "seen_in_current_cycle should be reset to false";
  }
  
  // Verify intermittent devices have miss_count = 6
  for (const auto& sn : intermittent_devices) {
    auto* dev = node_->testGetDevice(sn);
    ASSERT_NE(dev, nullptr) << "Intermittent device " << sn << " should still exist";
    EXPECT_EQ(dev->miss_count, 6) << "Intermittent device should have miss_count = 6";
  }
  
  // Verify offline device is removed
  EXPECT_EQ(node_->testGetDevice(offline_device), nullptr)
    << "Offline device should be removed";
  
  // Verify total device count
  EXPECT_EQ(node_->testGetDeviceCount(), 4) << "Should have 4 devices remaining";
}

/**
 * @brief Unit test: Single device reaching threshold
 * 
 * **Validates: Requirements 2.1, 3.1**
 * 
 * Tests that a single device is correctly removed when it reaches
 * the offline threshold.
 */
TEST_F(OfflineDetectionTest, UnitTest_SingleDeviceThreshold)
{
  std::string sn = "SINGLE_DEVICE_SN";
  
  // Add device at threshold - 1
  auto device = createTestDevice(sn, 9, false);
  node_->testAddDevice(sn, device);
  
  // Verify device exists
  EXPECT_EQ(node_->testGetDeviceCount(), 1);
  
  // Call checkOfflineDevices()
  node_->testCheckOfflineDevices();
  
  // Verify device is removed and list is empty
  EXPECT_EQ(node_->testGetDevice(sn), nullptr) << "Device should be removed";
  EXPECT_EQ(node_->testGetDeviceCount(), 0) << "Device list should be empty";
}

/**
 * @brief Unit test: Device seen in current cycle is not incremented
 * 
 * **Validates: Requirements 1.2, 1.3**
 * 
 * Tests that devices marked as seen in the current cycle do not
 * have their miss_count incremented.
 */
TEST_F(OfflineDetectionTest, UnitTest_SeenDeviceNotIncremented)
{
  std::string sn = "SEEN_DEVICE_SN";
  
  // Add device with miss_count = 5, but seen in current cycle
  auto device = createTestDevice(sn, 5, true);
  node_->testAddDevice(sn, device);
  
  // Verify initial state
  auto* dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 5);
  EXPECT_TRUE(dev->seen_in_current_cycle);
  
  // Call checkOfflineDevices()
  node_->testCheckOfflineDevices();
  
  // Verify miss_count is NOT incremented
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr) << "Device should still exist";
  EXPECT_EQ(dev->miss_count, 5) << "miss_count should not change for seen devices";
  EXPECT_FALSE(dev->seen_in_current_cycle) 
    << "seen_in_current_cycle should be reset to false";
}

/**
 * @brief Unit test: Multiple cycles with alternating visibility
 * 
 * **Validates: Requirements 1.2, 1.3, 2.1**
 * 
 * Tests a device that alternates between being seen and not seen
 * across multiple cycles.
 */
TEST_F(OfflineDetectionTest, UnitTest_AlternatingVisibility)
{
  std::string sn = "ALTERNATING_SN";
  
  // Add device
  auto device = createTestDevice(sn, 0, true);
  node_->testAddDevice(sn, device);
  
  // Cycle 1: Not seen - miss_count should be 1
  auto* dev = node_->testGetDevice(sn);
  dev->seen_in_current_cycle = false;
  node_->testCheckOfflineDevices();
  
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 1);
  
  // Cycle 2: Seen - miss_count should reset to 0
  dev->seen_in_current_cycle = true;
  node_->testCheckOfflineDevices();
  
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 0) << "miss_count should reset when device is seen";
  
  // Cycle 3: Not seen - miss_count should be 1
  dev->seen_in_current_cycle = false;
  node_->testCheckOfflineDevices();
  
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 1);
  
  // Cycle 4: Seen - miss_count should reset to 0
  dev->seen_in_current_cycle = true;
  node_->testCheckOfflineDevices();
  
  dev = node_->testGetDevice(sn);
  ASSERT_NE(dev, nullptr);
  EXPECT_EQ(dev->miss_count, 0) << "miss_count should reset again";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
