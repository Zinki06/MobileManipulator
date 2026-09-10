#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>
#include "manipulation_hardware/opencr.hpp"
#include "manipulation_hardware/opencr_system.hpp"

using robotis::manipulation_hardware::DynamixelSDKWrapper;
using robotis::manipulation_hardware::OpenCR;

struct Traffic {
  std::vector<uint16_t> writes;
  std::array<uint8_t, 400> table{};
  int fail_address{-1};
  bool read_ok{true};
};

class FakeTransport : public DynamixelSDKWrapper {
public:
  explicit FakeTransport(std::shared_ptr<Traffic> traffic)
  : DynamixelSDKWrapper(200), traffic_(traffic) {}
  bool open_port(const std::string &) override {return true;}
  bool set_baud_rate(const uint32_t &) override {return true;}
  uint16_t ping() override {return 0x5000;}

  bool read(const uint16_t & address, const uint16_t & length, uint8_t * data) override {
    if (!traffic_->read_ok) {return false;}
    std::memcpy(data, traffic_->table.data() + address, length);
    return true;
  }

  bool write(const uint16_t & address, const uint16_t & length, uint8_t * data) override {
    traffic_->writes.push_back(address);
    if (traffic_->fail_address == address) {return false;}
    std::memcpy(traffic_->table.data() + address, data, length);
    return true;
  }
private:
  std::shared_ptr<Traffic> traffic_;
};

TEST(OpenCRTransport, GripperStagingFailureNeverCommits) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  traffic->fail_address = 216;
  EXPECT_FALSE(device.set_gripper_position(0.019));
  EXPECT_EQ(traffic->writes, std::vector<uint16_t>({216}));
}

TEST(OpenCRTransport, CommitFailureIsReturnedAndNextRequestRetries) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  traffic->fail_address = 221;
  EXPECT_FALSE(device.set_gripper_position(0.019));
  traffic->fail_address = -1;
  EXPECT_TRUE(device.set_gripper_position(0.019));
  EXPECT_EQ(traffic->writes, std::vector<uint16_t>({216, 221, 216, 221}));
}

TEST(OpenCRTransport, RepeatedGoalsDoNotRestartTheGripperButHeartbeatContinues) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  EXPECT_TRUE(device.set_gripper_position(-0.010));
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(device.set_gripper_position(-0.010));
    EXPECT_TRUE(device.send_heartbeat(i));
  }
  EXPECT_EQ(std::count(traffic->writes.begin(), traffic->writes.end(), 221), 1);
  EXPECT_EQ(std::count(traffic->writes.begin(), traffic->writes.end(), 19), 100);
  EXPECT_TRUE(device.set_gripper_position(0.019));
  EXPECT_EQ(std::count(traffic->writes.begin(), traffic->writes.end(), 221), 2);
}

TEST(OpenCRTransport, FailedReadInvalidatesCachedFeedback) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  EXPECT_FALSE(device.data_valid());
  EXPECT_TRUE(device.read_all());
  EXPECT_TRUE(device.data_valid());
  traffic->read_ok = false;
  EXPECT_FALSE(device.read_all());
  EXPECT_FALSE(device.data_valid());
}

TEST(OpenCRTransport, FingerVelocityUsesMetersAndDynamixelRawScale) {
  auto traffic = std::make_shared<Traffic>();
  int32_t raw = 10;
  std::memcpy(traffic->table.data() + 260, &raw, sizeof(raw));
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  ASSERT_TRUE(device.read_all());
  EXPECT_NEAR(device.get_gripper_velocity(), 10 * .229 * .104719755 * -.015, 1e-12);
}

TEST(OpenCRTransport, WheelFeedbackUsesCurrentSampleWithoutSharedHistory) {
  auto traffic = std::make_shared<Traffic>();
  const int32_t ticks[] = {1024, -2048};
  // OpenCR exposes raw, signed extended wheel encoder positions.
  std::memcpy(traffic->table.data() + 136, ticks, sizeof(ticks));
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  ASSERT_TRUE(device.read_all());
  auto positions = device.get_wheel_positions();
  EXPECT_NEAR(positions[0], M_PI / 2.0, 1e-12);
  EXPECT_NEAR(positions[1], -M_PI, 1e-12);
  EXPECT_EQ(device.get_wheel_positions(), positions);
  auto other_traffic = std::make_shared<Traffic>();
  OpenCR other(std::make_unique<FakeTransport>(other_traffic));
  ASSERT_TRUE(other.read_all());
  EXPECT_EQ(other.get_wheel_positions(), (std::array<double, 2>{0.0, 0.0}));
  EXPECT_EQ(device.get_wheel_positions(), positions);
}

TEST(OpenCRTransport, RevoluteJointVelocitiesUseRadiansPerSecond) {
  auto traffic = std::make_shared<Traffic>();
  const int32_t wheels[] = {10, -10};
  const int32_t joints[] = {20, -20, 0, 5};
  std::memcpy(traffic->table.data() + 128, wheels, sizeof(wheels));
  std::memcpy(traffic->table.data() + 244, joints, sizeof(joints));
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  ASSERT_TRUE(device.read_all());
  EXPECT_NEAR(device.get_wheel_velocities()[0], 10 * .229 * 2.0 * M_PI / 60.0, 1e-12);
  EXPECT_NEAR(device.get_wheel_velocities()[1], -10 * .229 * 2.0 * M_PI / 60.0, 1e-12);
  EXPECT_NEAR(device.get_joint_velocities()[0], 20 * .229 * .104719755, 1e-12);
  EXPECT_NEAR(device.get_joint_velocities()[1], -20 * .229 * .104719755, 1e-12);
}

TEST(OpenCRTransport, CurrentCommitIsCheckedAndNeverExceedsExistingLimit) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  EXPECT_FALSE(device.set_gripper_current(81));
  EXPECT_FALSE(device.set_gripper_current(0));
  EXPECT_TRUE(traffic->writes.empty());
  traffic->fail_address = 343;
  EXPECT_FALSE(device.set_gripper_current(80));
  EXPECT_EQ(traffic->writes, std::vector<uint16_t>({340, 343}));
}

TEST(OpenCRTransport, DestructionSendsNoPositionOrTorqueCommands) {
  auto traffic = std::make_shared<Traffic>();
  {OpenCR device(std::make_unique<FakeTransport>(traffic));}
  EXPECT_TRUE(traffic->writes.empty());
}

TEST(OpenCRTransport, InvalidFingerGoalsNeverReachTheTransport) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  EXPECT_FALSE(device.set_gripper_position(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(device.set_gripper_position(.020));
  EXPECT_FALSE(device.set_gripper_position(-.011));
  EXPECT_TRUE(traffic->writes.empty());
}

TEST(OpenCRTransport, GoalRefreshOnlyRequestsReadback) {
  auto traffic = std::make_shared<Traffic>();
  OpenCR device(std::make_unique<FakeTransport>(traffic));
  EXPECT_TRUE(device.refresh_goals());
  EXPECT_EQ(traffic->writes, std::vector<uint16_t>({222}));
}

class SystemTransportTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::InitOptions options;
    options.set_domain_id(205);
    rclcpp::init(0, nullptr, options);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  hardware_interface::HardwareInfo info() {
    hardware_interface::HardwareInfo result;
    result.name = "test_opencr";
    result.hardware_parameters = {{"opencr_id", "200"}, {"opencr_usb_port", "fake"},
      {"opencr_baud_rate", "1000000"}, {"dxl_joints_profile_acceleration", "20"},
      {"dxl_joints_profile_velocity", "200"}, {"dxl_gripper_profile_acceleration", "20"},
      {"dxl_gripper_profile_velocity", "200"}};
    for (const auto & name : {"wheel_left_joint", "wheel_right_joint", "joint1", "joint2",
      "joint3", "joint4", "gripper_left_joint", "gripper_right_joint"})
    {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      result.joints.push_back(joint);
    }
    result.sensors.resize(2);
    result.sensors[0].state_interfaces.resize(10);
    result.sensors[1].state_interfaces.resize(4);
    return result;
  }

  std::shared_ptr<Traffic> connected() {
    auto traffic = std::make_shared<Traffic>();
    traffic->table[15] = traffic->table[16] = traffic->table[148] = 1;
    const int32_t positions[] = {2048, 1800, 1800, 2600, 1223};
    std::memcpy(traffic->table.data() + 224, positions, sizeof(positions));
    return traffic;
  }
};

TEST_F(SystemTransportTest, ActivationKeepsHeartbeatAndSeedsTheMeasuredFingerPosition) {
  auto traffic = connected();
  robotis::manipulation_hardware::OpenCRSystemHardware hardware(
    std::make_unique<OpenCR>(std::make_unique<FakeTransport>(traffic)));
  ASSERT_EQ(hardware.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_GE(std::count(traffic->writes.begin(), traffic->writes.end(), 19), 30);
  int32_t goal;
  std::memcpy(&goal, traffic->table.data() + 216, sizeof(goal));
  EXPECT_NEAR(goal, 1223, 1);  // Never impose a zero/closed startup position.
  EXPECT_EQ(traffic->table[199], 1);
}

TEST_F(SystemTransportTest, FailedReadReturnsHardwareErrorAndInvalidatesExportedStates) {
  auto traffic = connected();
  robotis::manipulation_hardware::OpenCRSystemHardware hardware(
    std::make_unique<OpenCR>(std::make_unique<FakeTransport>(traffic)));
  ASSERT_EQ(hardware.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  auto states = hardware.export_state_interfaces();
  ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
  ASSERT_TRUE(std::isfinite(states[12].get_value()));
  traffic->read_ok = false;
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(std::isnan(states[12].get_value()));
}

TEST_F(SystemTransportTest, ShutdownNeverRepositionsAndReportsTorqueWriteFailure) {
  auto traffic = connected();
  robotis::manipulation_hardware::OpenCRSystemHardware hardware(
    std::make_unique<OpenCR>(std::make_unique<FakeTransport>(traffic)));
  ASSERT_EQ(hardware.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  traffic->writes.clear();
  traffic->fail_address = 199;
  EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(traffic->writes, std::vector<uint16_t>({150, 199, 149}));
}
