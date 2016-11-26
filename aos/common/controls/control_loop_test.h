#ifndef AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_
#define AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_

#include "gtest/gtest.h"

#include "aos/testing/test_shm.h"
#include "aos/common/time.h"

namespace aos {
namespace testing {

// Handles setting up the environment that all control loops need to actually
// run.
// This includes sending the queue messages and Clear()ing the queues when
// appropriate.
// It also includes dealing with ::aos::time.
class ControlLoopTest : public ::testing::Test {
 public:
  ControlLoopTest();
  virtual ~ControlLoopTest();

  void set_team_id(uint16_t team_id) { team_id_ = team_id; }
  uint16_t team_id() const { return team_id_; }

  // Sends out all of the required queue messages.
  void SendMessages(bool enabled);
  // Ticks time for a single control loop cycle.
  void TickTime() { ::aos::time::SetMockTime(current_time_ += kTimeTick); }

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

}  // namespace testing
}  // namespace aos

#endif  // AOS_COMMON_CONTROLS_CONTROL_LOOP_TEST_H_
