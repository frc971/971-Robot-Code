#ifndef AOS_CONTROLS_CONTROL_LOOP_TEST_H_
#define AOS_CONTROLS_CONTROL_LOOP_TEST_H_

#include <chrono>

#include "gtest/gtest.h"

#include "aos/events/simulated-event-loop.h"
#include "aos/logging/queue_logging.h"
#include "aos/robot_state/robot_state.q.h"
#include "aos/testing/test_shm.h"
#include "aos/time/time.h"

namespace aos {
namespace testing {

// Handles setting up the environment that all control loops need to actually
// run.
// This includes sending the queue messages and Clear()ing the queues when
// appropriate.
// It also includes dealing with ::aos::time.
template <typename TestBaseClass>
class ControlLoopTestTemplated : public TestBaseClass {
 public:
  ControlLoopTestTemplated(::std::chrono::nanoseconds dt = kTimeTick)
      : dt_(dt), robot_status_event_loop_(MakeEventLoop()) {
    robot_state_sender_ =
        robot_status_event_loop_->MakeSender<::aos::RobotState>(
            ".aos.robot_state");
    joystick_state_sender_ =
        robot_status_event_loop_->MakeSender<::aos::JoystickState>(
            ".aos.joystick_state");

    // Schedule the robot status send 1 nanosecond before the loop runs.
    send_robot_state_phased_loop_ = robot_status_event_loop_->AddPhasedLoop(
        [this](int) { SendRobotState(); }, dt_,
        dt - ::std::chrono::nanoseconds(1));

    send_joystick_state_timer_ =
        robot_status_event_loop_->AddTimer([this]() { SendJoystickState(); });

    robot_status_event_loop_->OnRun([this]() {
      send_joystick_state_timer_->Setup(
          robot_status_event_loop_->monotonic_now(), dt_);
    });
  }
  virtual ~ControlLoopTestTemplated() {}

  void set_team_id(uint16_t team_id) { team_id_ = team_id; }
  uint16_t team_id() const { return team_id_; }

  // Sets the enabled/disabled bit and (potentially) rebroadcasts the robot
  // state messages.
  void SetEnabled(bool enabled) {
    if (enabled_ != enabled) {
      enabled_ = enabled;
      SendJoystickState();
      SendRobotState();
      send_joystick_state_timer_->Setup(
          robot_status_event_loop_->monotonic_now(), dt_);
    }
  }

  // Simulate a reset of the process reading sensors, which tells loops that all
  // index counts etc will be reset.
  void SimulateSensorReset() {
    ++reader_pid_;
  }

  // Sets the battery voltage in robot_state.
  void set_battery_voltage(double battery_voltage) {
    battery_voltage_ = battery_voltage;
  }

  ::std::unique_ptr<::aos::EventLoop> MakeEventLoop() {
    return event_loop_factory_.MakeEventLoop();
  }

  void RunFor(monotonic_clock::duration duration) {
    event_loop_factory_.RunFor(duration);
  }

  ::aos::monotonic_clock::time_point monotonic_now() {
    return event_loop_factory_.monotonic_now();
  }

  ::std::chrono::nanoseconds dt() const { return dt_; }

 private:
  // Sends out all of the required queue messages.
  void SendJoystickState() {
    if (monotonic_now() >= kDSPacketTime + last_ds_time_ ||
        last_enabled_ != enabled_) {
      auto new_state = joystick_state_sender_.MakeMessage();
      new_state->fake = true;

      new_state->enabled = enabled_;
      new_state->autonomous = false;
      new_state->team_id = team_id_;

      AOS_LOG_STRUCT(INFO, "joystick_state", *new_state);
      new_state.Send();

      last_ds_time_ = monotonic_now();
      last_enabled_ = enabled_;
    }
  }

  bool last_enabled_ = false;

  void SendRobotState() {
    auto new_state = robot_state_sender_.MakeMessage();

    new_state->reader_pid = reader_pid_;
    new_state->outputs_enabled = enabled_;
    new_state->browned_out = false;

    new_state->is_3v3_active = true;
    new_state->is_5v_active = true;
    new_state->voltage_3v3 = 3.3;
    new_state->voltage_5v = 5.0;

    new_state->voltage_roborio_in = battery_voltage_;
    new_state->voltage_battery = battery_voltage_;

    AOS_LOG_STRUCT(INFO, "robot_state", *new_state);
    new_state.Send();
  }

  static constexpr ::std::chrono::microseconds kTimeTick{5000};
  static constexpr ::std::chrono::milliseconds kDSPacketTime{20};

  const ::std::chrono::nanoseconds dt_;

  SimulatedEventLoopFactory event_loop_factory_;

  uint16_t team_id_ = 971;
  int32_t reader_pid_ = 1;
  double battery_voltage_ = 12.4;

  ::aos::monotonic_clock::time_point last_ds_time_ =
      ::aos::monotonic_clock::min_time;

  bool enabled_ = false;

  ::std::unique_ptr<::aos::EventLoop> robot_status_event_loop_;

  ::aos::Sender<::aos::RobotState> robot_state_sender_;
  ::aos::Sender<::aos::JoystickState> joystick_state_sender_;

  ::aos::PhasedLoopHandler *send_robot_state_phased_loop_ = nullptr;
  ::aos::TimerHandler *send_joystick_state_timer_ = nullptr;
};

typedef ControlLoopTestTemplated<::testing::Test> ControlLoopTest;

template <typename TestBaseClass>
constexpr ::std::chrono::microseconds
    ControlLoopTestTemplated<TestBaseClass>::kTimeTick;

template <typename TestBaseClass>
constexpr ::std::chrono::milliseconds ControlLoopTestTemplated<TestBaseClass>::kDSPacketTime;

}  // namespace testing
}  // namespace aos

#endif  // AOS_CONTROLS_CONTROL_LOOP_TEST_H_
