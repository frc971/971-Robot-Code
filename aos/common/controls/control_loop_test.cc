#include "aos/common/controls/control_loop_test.h"

#include "aos/common/messages/robot_state.q.h"

namespace aos {
namespace testing {

constexpr ::aos::time::Time ControlLoopTest::kTimeTick;
constexpr ::aos::time::Time ControlLoopTest::kDSPacketTime;

ControlLoopTest::ControlLoopTest() {
  ::aos::joystick_state.Clear();
  ::aos::robot_state.Clear();

  ::aos::time::Time::EnableMockTime(current_time_);

  SendMessages(false);
}

ControlLoopTest::~ControlLoopTest() {
  ::aos::joystick_state.Clear();
  ::aos::robot_state.Clear();

  ::aos::time::Time::DisableMockTime();
}

void ControlLoopTest::SendMessages(bool enabled) {
  if (current_time_ - last_ds_time_ >= kDSPacketTime) {
    last_ds_time_ = current_time_;
    auto new_state = ::aos::joystick_state.MakeMessage();
    new_state->fake = true;

    new_state->enabled = enabled;
    new_state->autonomous = false;
    new_state->team_id = team_id_;

    new_state.Send();
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

    new_state->voltage_roborio_in = 12.4;
    new_state->voltage_battery = 12.4;

    new_state.Send();
  }
}

}  // namespace testing
}  // namespace aos
