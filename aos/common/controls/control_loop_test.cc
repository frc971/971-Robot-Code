#include "aos/common/controls/control_loop_test.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/controls/sensor_generation.q.h"
#include "aos/common/controls/output_check.q.h"
#include "aos/common/time.h"

namespace aos {
namespace testing {

ControlLoopTest::ControlLoopTest() {
  ::aos::robot_state.Clear();
  ::aos::controls::sensor_generation.Clear();
  ::aos::controls::output_check_received.Clear();

  ::aos::controls::sensor_generation.MakeWithBuilder()
      .reader_pid(254)
      .cape_resets(5)
      .Send();
  ::aos::time::Time::EnableMockTime(::aos::time::Time::InSeconds(0.0));

  SimulateTimestep(false);
}

ControlLoopTest::~ControlLoopTest() {
  ::aos::robot_state.Clear();
  ::aos::controls::sensor_generation.Clear();
  ::aos::controls::output_check_received.Clear();

  ::aos::time::Time::DisableMockTime();
}

void ControlLoopTest::SimulateTimestep(bool enabled) {
  if (sent_robot_state_last_time_) {
    sent_robot_state_last_time_ = false;
  } else {
    ::aos::robot_state.MakeWithBuilder()
        .enabled(enabled)
        .autonomous(false)
        .fake(true)
        .team_id(971)
        .Send();
    sent_robot_state_last_time_ = true;
  }
  if (enabled) {
    // TODO(brians): Actually make this realistic once we figure out what that
    // means.
    ::aos::controls::output_check_received.MakeWithBuilder()
        .pwm_value(0)
        .pulse_length(0)
        .Send();
  }
  ::aos::time::Time::IncrementMockTime(::aos::time::Time::InMS(10.0));
}

}  // namespace testing
}  // namespace aos
