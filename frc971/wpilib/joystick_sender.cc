#include "frc971/wpilib/joystick_sender.h"

#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "frc971/input/driver_station_data.h"
#include "frc971/input/joystick_state_generated.h"
#include "frc971/wpilib/ahal/DriverStation.h"
#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {

using aos::Joystick;

JoystickSender::JoystickSender(::aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      joystick_state_sender_(
          event_loop_->MakeSender<::aos::JoystickState>("/aos")),
      team_id_(::aos::network::GetTeamNumber()) {
  event_loop_->SetRuntimeRealtimePriority(28);
  event_loop->set_name("joystick_sender");

  event_loop_->OnRun([this]() {
    frc::DriverStation *const ds = &frc::DriverStation::GetInstance();

    // TODO(Brian): Fix the potential deadlock when stopping here (condition
    // variable / mutex needs to get exposed all the way out or something).
    while (event_loop_->is_running()) {
      ds->RunIteration([&]() {
        auto builder = joystick_state_sender_.MakeBuilder();

        std::array<flatbuffers::Offset<Joystick>,
                   frc971::input::driver_station::JoystickFeature::kJoysticks>
            joysticks;

        for (size_t i = 0;
             i < frc971::input::driver_station::JoystickFeature::kJoysticks;
             ++i) {
          std::array<double, frc971::input::driver_station::JoystickAxis::kAxes>
              axis;
          for (int j = 0;
               j < frc971::input::driver_station::JoystickAxis::kAxes; ++j) {
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

        flatbuffers::Offset<flatbuffers::String> game_data_offset =
            builder.fbb()->CreateString(ds->GetGameSpecificMessage());

        flatbuffers::Offset<flatbuffers::String> event_name_offset =
            builder.fbb()->CreateString(ds->GetEventName());

        aos::JoystickState::Builder joystick_state_builder =
            builder.MakeBuilder<aos::JoystickState>();

        joystick_state_builder.add_joysticks(joysticks_offset);

        if (ds->GetGameSpecificMessage().size() >= 2u) {
          joystick_state_builder.add_switch_left(
              ds->GetGameSpecificMessage()[0] == 'L' ||
              ds->GetGameSpecificMessage()[0] == 'l');
          joystick_state_builder.add_scale_left(
              ds->GetGameSpecificMessage()[1] == 'L' ||
              ds->GetGameSpecificMessage()[1] == 'l');
        }
        joystick_state_builder.add_game_data(game_data_offset);

        joystick_state_builder.add_test_mode(ds->IsTestMode());
        joystick_state_builder.add_fms_attached(ds->IsFmsAttached());
        joystick_state_builder.add_enabled(ds->IsEnabled());
        joystick_state_builder.add_autonomous(ds->IsAutonomous());
        switch (ds->GetAlliance()) {
          case frc::DriverStation::kRed:
            joystick_state_builder.add_alliance(aos::Alliance::kRed);
            break;
          case frc::DriverStation::kBlue:
            joystick_state_builder.add_alliance(aos::Alliance::kBlue);
            break;
          case frc::DriverStation::kInvalid:
            joystick_state_builder.add_alliance(aos::Alliance::kInvalid);
            break;
        }
        joystick_state_builder.add_location(ds->GetLocation());

        joystick_state_builder.add_team_id(team_id_);
        joystick_state_builder.add_match_number(ds->GetMatchNumber());
        joystick_state_builder.add_replay_number(ds->GetReplayNumber());

        switch (ds->GetMatchType()) {
          case frc::DriverStation::kNone:
            joystick_state_builder.add_match_type(aos::MatchType::kNone);
            break;
          case frc::DriverStation::kPractice:
            joystick_state_builder.add_match_type(aos::MatchType::kPractice);
            break;
          case frc::DriverStation::kQualification:
            joystick_state_builder.add_match_type(
                aos::MatchType::kQualification);
            break;
          case frc::DriverStation::kElimination:
            joystick_state_builder.add_match_type(aos::MatchType::kElimination);
            break;
        }
        joystick_state_builder.add_event_name(event_name_offset);

        if (builder.Send(joystick_state_builder.Finish()) !=
            aos::RawSender::Error::kOk) {
          AOS_LOG(WARNING, "sending joystick_state failed\n");
        }
      });
    }
  });
}

}  // namespace wpilib
}  // namespace frc971
