#include "aos/common/controls/control_loop_test.h"

#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace testing {

constexpr ::std::chrono::milliseconds ControlLoopTest::kTimeTick;
constexpr ::std::chrono::milliseconds ControlLoopTest::kDSPacketTime;

ControlLoopTest::ControlLoopTest() {
  ::aos::joystick_state.Clear();
  ::aos::robot_state.Clear();

  ::aos::time::EnableMockTime(current_time_);

  SendMessages(false);
}

ControlLoopTest::~ControlLoopTest() {
  ::aos::joystick_state.Clear();
  ::aos::robot_state.Clear();

  ::aos::time::DisableMockTime();
}

void ControlLoopTest::SendMessages(bool enabled) {
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

}  // namespace testing
}  // namespace aos
