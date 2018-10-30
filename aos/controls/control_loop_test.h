#ifndef AOS_CONTROLS_CONTROL_LOOP_TEST_H_
#define AOS_CONTROLS_CONTROL_LOOP_TEST_H_

#include <chrono>

#include "gtest/gtest.h"

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
  ControlLoopTestTemplated() {
    ::aos::joystick_state.Clear();
    ::aos::robot_state.Clear();

    ::aos::time::EnableMockTime(current_time_);

    SendMessages(false);
  }
  virtual ~ControlLoopTestTemplated() {
    ::aos::joystick_state.Clear();
    ::aos::robot_state.Clear();

    ::aos::time::DisableMockTime();
  }

  void set_team_id(uint16_t team_id) { team_id_ = team_id; }
  uint16_t team_id() const { return team_id_; }

  // Sends out all of the required queue messages.
  void SendMessages(bool enabled) {
    if (current_time_ >= kDSPacketTime + last_ds_time_ ||
        last_enabled_ != enabled) {
      last_ds_time_ = current_time_;
      auto new_state = ::aos::joystick_state.MakeMessage();
      new_state->fake = true;

      new_state->enabled = enabled;
      new_state->autonomous = false;
      new_state->team_id = team_id_;

      new_state.Send();
      last_enabled_ = enabled;
    }

    {
      auto new_state = ::aos::robot_state.MakeMessage();

      new_state->reader_pid = reader_pid_;
      new_state->outputs_enabled = enabled;
      new_state->browned_out = false;

      new_state->is_3v3_active = true;
      new_state->is_5v_active = true;
      new_state->voltage_3v3 = 3.3;
      new_state->voltage_5v = 5.0;

      new_state->voltage_roborio_in = battery_voltage_;
      new_state->voltage_battery = battery_voltage_;

      LOG_STRUCT(INFO, "robot_state", *new_state);
      new_state.Send();
    }
  }
  // Ticks time for a single control loop cycle.
  void TickTime(::std::chrono::nanoseconds dt = kTimeTick) {
    ::aos::time::SetMockTime(current_time_ += dt);
  }

  // Simulates everything that happens during 1 loop time step.
  void SimulateTimestep(bool enabled) {
    SendMessages(enabled);
    TickTime();
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

 private:
  static constexpr ::std::chrono::milliseconds kTimeTick{5};
  static constexpr ::std::chrono::milliseconds kDSPacketTime{20};

  uint16_t team_id_ = 971;
  int32_t reader_pid_ = 1;
  double battery_voltage_ = 12.4;

  ::aos::monotonic_clock::time_point last_ds_time_ =
      ::aos::monotonic_clock::epoch();
  ::aos::monotonic_clock::time_point current_time_ =
      ::aos::monotonic_clock::epoch();

  ::aos::testing::TestSharedMemory my_shm_;

  bool last_enabled_ = false;
};

typedef ControlLoopTestTemplated<::testing::Test> ControlLoopTest;

template <typename TestBaseClass>
constexpr ::std::chrono::milliseconds ControlLoopTestTemplated<TestBaseClass>::kTimeTick;

template <typename TestBaseClass>
constexpr ::std::chrono::milliseconds ControlLoopTestTemplated<TestBaseClass>::kDSPacketTime;

}  // namespace testing
}  // namespace aos

#endif  // AOS_CONTROLS_CONTROL_LOOP_TEST_H_
