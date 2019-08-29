#include "frc971/wpilib/joystick_sender.h"

#include "aos/input/driver_station_data.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "aos/robot_state/joystick_state_generated.h"

#include "frc971/wpilib/ahal/DriverStation.h"
#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {

using aos::Joystick;

JoystickSender::JoystickSender(::aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      joystick_state_sender_(
          event_loop_->MakeSender<::aos::JoystickState>("/aos")),
      team_id_(::aos::network::GetTeamNumber()) {
  event_loop_->SetRuntimeRealtimePriority(29);

  event_loop_->OnRun([this]() {
    frc::DriverStation *const ds = &frc::DriverStation::GetInstance();
    ::aos::SetCurrentThreadName("DSReader");

    // TODO(Brian): Fix the potential deadlock when stopping here (condition
    // variable / mutex needs to get exposed all the way out or something).
    while (event_loop_->is_running()) {
      ds->RunIteration([&]() {
        auto builder = joystick_state_sender_.MakeBuilder();

        HAL_MatchInfo match_info;
        auto status = HAL_GetMatchInfo(&match_info);

        std::array<flatbuffers::Offset<Joystick>,
                   aos::input::driver_station::JoystickFeature::kJoysticks>
            joysticks;

        for (size_t i = 0;
             i < aos::input::driver_station::JoystickFeature::kJoysticks; ++i) {
          std::array<double, aos::input::driver_station::JoystickAxis::kAxes>
              axis;
          for (int j = 0; j < aos::input::driver_station::JoystickAxis::kAxes;
               ++j) {
            axis[j] = ds->GetStickAxis(i, j);
          }

          flatbuffers::Offset<flatbuffers::Vector<double>> axis_offset =
              builder.fbb()->CreateVector(axis.begin(), axis.size());

          Joystick::Builder joystick_builder = builder.MakeBuilder<Joystick>();

          joystick_builder.add_buttons(ds->GetStickButtons(i));

          if (ds->GetStickPOVCount(i) > 0) {
            joystick_builder.add_pov(ds->GetStickPOV(i, 0));
          }

          joystick_builder.add_axis(axis_offset);

          joysticks[i] = joystick_builder.Finish();
        }

        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Joystick>>>
            joysticks_offset = builder.fbb()->CreateVector(joysticks.begin(),
                                                           joysticks.size());

        aos::JoystickState::Builder joystick_state_builder =
            builder.MakeBuilder<aos::JoystickState>();

        joystick_state_builder.add_joysticks(joysticks_offset);

        if (status == 0) {
          joystick_state_builder.add_switch_left(
              match_info.gameSpecificMessage[0] == 'L' ||
              match_info.gameSpecificMessage[0] == 'l');
          joystick_state_builder.add_scale_left(
              match_info.gameSpecificMessage[1] == 'L' ||
              match_info.gameSpecificMessage[1] == 'l');
        }

        joystick_state_builder.add_test_mode(ds->IsTestMode());
        joystick_state_builder.add_fms_attached(ds->IsFmsAttached());
        joystick_state_builder.add_enabled(ds->IsEnabled());
        joystick_state_builder.add_autonomous(ds->IsAutonomous());
        joystick_state_builder.add_team_id(team_id_);

        if (!builder.Send(joystick_state_builder.Finish())) {
          AOS_LOG(WARNING, "sending joystick_state failed\n");
        }
      });
    }
  });
}

}  // namespace wpilib
}  // namespace frc971
