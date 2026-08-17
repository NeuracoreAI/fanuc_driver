// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>

#include "fanuc_client/fanuc_client.hpp"

class MockStreamMotionConnection : public stream_motion::StreamMotionInterface
{
public:
  explicit MockStreamMotionConnection(std::atomic<bool>& stream_connected) : stream_connected_{ stream_connected }
  {
    status_.status = 15;
    status_.joint_angle = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  bool streamConnected() const
  {
    return stream_connected_;
  }

  void sendStartPacket() const override
  {
    stream_connected_ = true;
  }

  void sendStopPacket() const override
  {
    stream_connected_ = false;
  }

  void sendCommand(const std::array<double, stream_motion::kMaxAxisNumber>& command_pos, bool is_last_command,
                   const std::array<uint8_t, 256>& io_command, const uint8_t do_motn_ctrl) const override
  {
    last_do_motn_ctrl_ = do_motn_ctrl;
    last_command_pos_ = command_pos;
    ++send_count_;
    if (echo_command_to_status_)
    {
      for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
      {
        status_.joint_angle[i] = static_cast<float>(command_pos[i]);
      }
    }
  }

  bool getStatusPacket(stream_motion::RobotStatusPacket& status) override
  {
    if (!stream_connected_)
    {
      return false;
    }
    status = status_;
    return true;
  }

  int sendCount() const
  {
    return send_count_.load();
  }

  uint8_t lastDoMotnCtrl() const
  {
    return last_do_motn_ctrl_.load();
  }

  std::array<double, stream_motion::kMaxAxisNumber> lastCommandPos() const
  {
    return last_command_pos_;
  }

  void setMeasuredJoints(const std::array<float, stream_motion::kMaxAxisNumber>& joints)
  {
    status_.joint_angle = joints;
  }

  void setEchoCommandToStatus(bool echo)
  {
    echo_command_to_status_ = echo;
  }

  bool getRobotLimits(const uint32_t axis_number, stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      stream_motion::RobotThresholdPacket& robot_threshold_jerk) const override
  {
    robot_threshold_velocity.axis_number = axis_number;
    robot_threshold_acceleration.axis_number = axis_number;
    robot_threshold_jerk.axis_number = axis_number;
    for (int i = 0; i < 20; ++i)
    {
      robot_threshold_velocity.full_payload[i] = 200.0f;
      robot_threshold_velocity.no_payload[i] = 200.0f;
      robot_threshold_acceleration.full_payload[i] = 2000.0f;
      robot_threshold_acceleration.no_payload[i] = 2000.0f;
      robot_threshold_jerk.full_payload[i] = 20000.0f;
      robot_threshold_jerk.no_payload[i] = 20000.0f;
    }
    return true;
  }

  bool configureGPIO(const stream_motion::GPIOConfiguration& config) const override
  {
    return true;
  }

  bool getControllerCapability(stream_motion::ControllerCapabilityResultPacket& controller_capability) override
  {
    if (!capability_ok_)
    {
      controller_capability = stream_motion::ControllerCapabilityResultPacket{};
      return false;
    }
    controller_capability.sampling_rate = 8;
    return true;
  }

  void setCapabilityOk(bool ok)
  {
    capability_ok_ = ok;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const override
  {
  }

private:
  mutable stream_motion::RobotStatusPacket status_;
  std::atomic<bool>& stream_connected_;
  mutable std::atomic<int> send_count_{ 0 };
  mutable std::atomic<uint8_t> last_do_motn_ctrl_{ 0 };
  mutable std::array<double, stream_motion::kMaxAxisNumber> last_command_pos_{};
  bool echo_command_to_status_{ true };
  bool capability_ok_{ true };
};

class MockRMIConnection : public rmi::RMIConnectionInterface
{
public:
  ~MockRMIConnection() override = default;

  MOCK_METHOD(rmi::ConnectROS2Packet::Response, connect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::DisconnectPacket::Response, disconnect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::InitializePacket::Response, initializeRemoteMotion, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::InitializePacket::Response, initializeRemoteMotion,
              (std::optional<double> timeout, const std::optional<uint8_t> groupmask,
               const std::optional<std::string>& rtsa, const std::optional<std::string>& pltzmode),
              (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Response, programCall,
              (const std::string& program_name, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Request, programCallNonBlocking, (const std::string& program_name), (override));
  MOCK_METHOD(rmi::StatusRequestPacket::Response, getStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetSpeedOverridePacket::Response, setSpeedOverride, (int value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::AbortPacket::Response, abort, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::PausePacket::Response, pause, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ContinuePacket::Response, resume, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ResetRobotPacket::Response, reset, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadErrorPacket::Response, readError, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WritePositionRegisterPacket::Response, writePositionRegister,
              (int register_number, const std::string& representation, const rmi::ConfigurationData& configuration,
               const rmi::PositionData& position, const rmi::JointAngleData& joint_angle, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadPositionRegisterPacket::Response, readPositionRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadNumericRegisterPacket::Response, readNumericRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteNumericRegisterPacket::Response, writeNumericRegister,
              (int register_number, (std::variant<int, float>)value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadDigitalInputPortPacket::Response, readDigitalInputPort,
              (uint16_t port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteDigitalOutputPacket::Response, writeDigitalOutputPort,
              (uint16_t port_number, bool port_value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadIOPortPacket::Response, readIOPort,
              (const std::string& port_type, int port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteIOPortPacket::Response, writeIOPort,
              (int port_number, const std::string& port_type, (std::variant<int, float>)port_value,
               std::optional<double> timeout),
              (override));
  rmi::ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                       std::optional<double> timeout) override
  {
    assert(variable_name == std::string("$STMO.$COM_INT"));
    rmi::ReadVariablePacket::Response response{};
    response.VariableValue = 8;
    return response;
  }
  MOCK_METHOD(rmi::WriteVariablePacket::Response, writeVariablePacket,
              (const std::string& variable_name, (std::variant<int, float>)value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::GetExtendedStatusPacket::Response, getExtendedStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadPacket::Response, setPayloadSchedule,
              (uint8_t payload_schedule_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadValuePacket::Response, setPayloadValue,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, bool use_in, float in_x,
               float in_y, float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::SetPayloadCompPacket::Response, setPayloadComp,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, float in_x, float in_y,
               float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::JointMotionJRepPacket::Response, sendJointMotion,
              (rmi::JointMotionJRepPacket::Request joint_motion_request, const std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadJointAnglesPacket::Response, readJointAngles,
              (const std::optional<uint8_t>& group, const std::optional<double> timeout), (override));
  MOCK_METHOD(std::optional<rmi::SystemFaultPacket>, checkSystemFault, (), (override));
  MOCK_METHOD(std::optional<rmi::TimeoutTerminatePacket>, checkTimeoutTerminate, (), (override));
  MOCK_METHOD(std::optional<rmi::CommunicationPacket>, checkCommunicationPacket, (), (override));
  MOCK_METHOD(std::optional<rmi::UnknownPacket>, checkUnknownPacket, (), (override));
};

using NiceMockStreamMotionConnection = testing::NiceMock<MockStreamMotionConnection>;

TEST(FanucClientTest, TestSuccessfulLifecycle)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected);
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));

  // Reading/Writing data before starting the stream should throw an error
  const Eigen::VectorXd initial_joint_targets = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  EXPECT_THROW(fanuc_client.writeJointTarget(initial_joint_targets), std::runtime_error);
  EXPECT_THROW(fanuc_client.readJointAngles(), std::runtime_error);

  // Start the real-time stream with a mock connection
  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(stream_connected);

  // After starting the stream, we the first read should return the initial joint angles
  Eigen::VectorXd joint_states = fanuc_client.readJointAngles();
  EXPECT_EQ(joint_states, initial_joint_targets);

  // Writing joint targets should update the joint states eventually
  fanuc_client.setDoMotnCtrl(true);
  Eigen::VectorXd joint_targets = joint_states.array() + 1.0;
  while (joint_states == initial_joint_targets)
  {
    fanuc_client.writeJointTarget(joint_targets);
    joint_states = fanuc_client.readJointAngles();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(joint_targets, joint_states);

  // Stop the real-time stream should disconnect the stream
  fanuc_client.stopRealtimeStream();
  while (stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(stream_connected);
}

TEST(FanucClientTest, CapabilityFailureDefaultsControlPeriod)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  stream_motion_interface->setCapabilityOk(false);
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));
  EXPECT_EQ(fanuc_client.getControlPeriod(), 8u);
}

TEST(FanucClientTest, SlewRateMultiplierStoresAndClamps)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected);
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));
  EXPECT_DOUBLE_EQ(fanuc_client.getSlewRateMultiplier(), 4.0);
  fanuc_client.setSlewRateMultiplier(6.0);
  EXPECT_DOUBLE_EQ(fanuc_client.getSlewRateMultiplier(), 6.0);
  fanuc_client.setSlewRateMultiplier(0.1);
  EXPECT_DOUBLE_EQ(fanuc_client.getSlewRateMultiplier(), 1.0);
  fanuc_client.setSlewRateMultiplier(100.0);
  EXPECT_DOUBLE_EQ(fanuc_client.getSlewRateMultiplier(), 16.0);
}

TEST(FanucClientTest, DisarmedTrackingKeepsSendingMeasured)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  auto* mock = stream_motion_interface.get();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));

  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const int before = mock->sendCount();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const int after = mock->sendCount();
  EXPECT_GT(after, before);
  EXPECT_EQ(mock->lastDoMotnCtrl(), 0);
  EXPECT_FALSE(fanuc_client.getDoMotnCtrl());

  fanuc_client.stopRealtimeStream();
}

TEST(FanucClientTest, SetDoMotnCtrlEdgeReseedsGoalFromMeasured)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  auto* mock = stream_motion_interface.get();
  mock->setEchoCommandToStatus(false);
  std::array<float, stream_motion::kMaxAxisNumber> measured_f{};
  measured_f[0] = 1.5f;
  measured_f[1] = -2.5f;
  mock->setMeasuredJoints(measured_f);

  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));

  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Drain status so last_joint_angles_ matches the configured measured pose.
  Eigen::VectorXd measured = fanuc_client.readJointAngles();
  EXPECT_NEAR(measured[0], 1.5, 1e-3);
  EXPECT_NEAR(measured[1], -2.5, 1e-3);

  Eigen::VectorXd divergent = Eigen::VectorXd::Ones(stream_motion::kMaxAxisNumber) * 5.0;
  fanuc_client.setJointGoal(divergent);
  EXPECT_EQ(fanuc_client.getJointGoal(), divergent);

  // Rising edge reseeds slewer/goal to measured.
  fanuc_client.setDoMotnCtrl(true);
  EXPECT_TRUE(fanuc_client.getDoMotnCtrl());
  Eigen::VectorXd goal = fanuc_client.getJointGoal();
  EXPECT_NEAR(goal[0], 1.5, 1e-3);
  EXPECT_NEAR(goal[1], -2.5, 1e-3);

  // Idempotent: no second edge.
  fanuc_client.setJointGoal(divergent);
  fanuc_client.setDoMotnCtrl(true);
  EXPECT_EQ(fanuc_client.getJointGoal(), divergent);

  // Falling edge also reseeds from measured.
  fanuc_client.setDoMotnCtrl(false);
  EXPECT_FALSE(fanuc_client.getDoMotnCtrl());
  goal = fanuc_client.getJointGoal();
  EXPECT_NEAR(goal[0], 1.5, 1e-3);
  EXPECT_NEAR(goal[1], -2.5, 1e-3);

  fanuc_client.stopRealtimeStream();
}

TEST(FanucClientTest, StartMotionControlRearmsDoMotnCtrl)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));

  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_FALSE(fanuc_client.getDoMotnCtrl());
  fanuc_client.stopMotionControl();
  EXPECT_FALSE(fanuc_client.getDoMotnCtrl());

  ASSERT_TRUE(fanuc_client.startMotionControl());
  EXPECT_TRUE(fanuc_client.getDoMotnCtrl());

  fanuc_client.stopMotionControl();
  EXPECT_FALSE(fanuc_client.getDoMotnCtrl());

  fanuc_client.stopRealtimeStream();
}

TEST(FanucClientTest, TestGetLimits)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, std::move(stream_motion_interface));
  const double v_peak = 1000.0;
  const double payload = 0.0;
  std::vector<double> vel_limit;
  std::vector<double> acc_limit;
  std::vector<double> jerk_limit;
  fanuc_client.getLimits(v_peak, payload, vel_limit, acc_limit, jerk_limit);
  for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
  {
    EXPECT_EQ(vel_limit[i], 200.0f);
    EXPECT_EQ(acc_limit[i], 2000.0f);
    EXPECT_EQ(jerk_limit[i], 20000.0f);
  }
}

TEST(RMISington, OnlyOneInstanceCreated)
{
  // Create the first MockRMIConnection instance and set it as the singleton
  auto rmi_inst = std::make_unique<MockRMIConnection>();
  fanuc_client::RMISingleton::setRMIInstance(std::move(rmi_inst));

  // Retrieve the singleton instance twice and check that both references point to the same object
  auto rmi_connection_inst1 = fanuc_client::RMISingleton::getRMIInstance();
  auto rmi_connection_inst2 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_EQ(rmi_connection_inst1.get(), rmi_connection_inst2.get());

  // Create a new MockRMIConnection instance and replace the singleton
  auto new_rmi_inst = std::make_unique<MockRMIConnection>();
  MockRMIConnection* new_rmi_inst_ptr = new_rmi_inst.get();
  fanuc_client::RMISingleton::setRMIInstance(std::move(new_rmi_inst));

  // Retrieve the singleton instance again and check that it has changed to the new object
  auto rmi_connection_inst3 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_NE(rmi_connection_inst2.get(), rmi_connection_inst3.get());
  EXPECT_EQ(rmi_connection_inst3.get(), new_rmi_inst_ptr);
}
