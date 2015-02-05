#include "aos/common/controls/control_loop_test.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/controls/sensor_generation.q.h"
#include "aos/common/controls/output_check.q.h"

namespace aos {
namespace testing {

constexpr ::aos::time::Time ControlLoopTest::kTimeTick;
constexpr ::aos::time::Time ControlLoopTest::kDSPacketTime;

ControlLoopTest::ControlLoopTest() {
  ::aos::robot_state.Clear();
  ::aos::controls::sensor_generation.Clear();
  ::aos::controls::output_check_received.Clear();

  ::aos::controls::sensor_generation.MakeWithBuilder()
      .reader_pid(254)
      .cape_resets(5)
      .Send();
  ::aos::time::Time::EnableMockTime(current_time_);

  SimulateTimestep(false);
}

ControlLoopTest::~ControlLoopTest() {
  ::aos::robot_state.Clear();
  ::aos::controls::sensor_generation.Clear();
  ::aos::controls::output_check_received.Clear();

  ::aos::time::Time::DisableMockTime();
}

void ControlLoopTest::SendMessages(bool enabled) {
  if (current_time_ - last_ds_time_ >= kDSPacketTime) {
    ::aos::robot_state.MakeWithBuilder()
        .enabled(enabled)
        .autonomous(false)
        .fake(true)
        .team_id(971)
        .Send();
    last_ds_time_ = current_time_;
  }
  if (enabled) {
    // TODO(brians): Actually make this realistic once we figure out what that
    // means.
    ::aos::controls::output_check_received.MakeWithBuilder()
        .pwm_value(0)
        .pulse_length(0)
        .Send();
  }
}

}  // namespace testing
}  // namespace aos
