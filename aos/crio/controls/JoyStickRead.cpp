#include "WPILib/Task.h"
#include "WPILib/Timer.h"

#include "aos/aos_core.h"
#include "aos/common/logging/logging.h"
#include "aos/crio/controls/ControlsManager.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/messages/RobotState.q.h"

namespace aos {
namespace crio {

class JoystickRead {
 public:
  // Represents all of the states that FMS thinks of a robot as being in.
  enum class FMSState {
    kDisabled,
    kAutonomous,
    kTeleop,
    kTestMode,
  };

  DriverStation *ds;

  JoystickRead() {}

  void Run() {
    SendSocket sock(NetworkPort::kDS,
                    configuration::GetIPAddress(
                        configuration::NetworkDevice::kAtom));
    FRCCommonControlData data;

    ds = ControlsManager::GetInstance().GetDS();

    while (true) {
      // I checked, and this is done intelligently in WPILib.
      ds->WaitForData();

      robot_state.MakeWithBuilder().enabled(ds->IsEnabled())
          .autonomous(ds->IsAutonomous()).team_id(ds->GetTeamNumber()).Send();
      LOG(DEBUG, "sending joystick data\n");
      data.enabled = ds->IsEnabled();
      data.autonomous = ds->IsAutonomous();
      data.fmsAttached = ds->IsFMSAttached();
      SetStick(data.stick0Axes, 1);
      SetStick(data.stick1Axes, 2);
      SetStick(data.stick2Axes, 3);
      SetStick(data.stick3Axes, 4);
      data.stick0Buttons = ds->GetStickButtons(1);
      data.stick1Buttons = ds->GetStickButtons(2);
      data.stick2Buttons = ds->GetStickButtons(3);
      data.stick3Buttons = ds->GetStickButtons(4);
      data.teamID = ds->GetTeamNumber();
      sock.Send(&data, sizeof(data));

      // Echo the state back into the FMS system. It makes the correct lights
      // turn on so that field personnel don't get distracted when debugging
      // unrelated problems.
      FMSState state = GetCurrentState();
      ds->InDisabled(state == FMSState::kDisabled);
      ds->InAutonomous(state == FMSState::kAutonomous);
      ds->InOperatorControl(state == FMSState::kTeleop);
      ds->InTest(state == FMSState::kTestMode);
    }
  }

  void SetStick(int8_t axes[6], uint32_t stick) {
    for (int i = 0; i < 6; ++i) {
      double val = ds->GetStickAxis(stick, i + 1);
      if (val < 0) {
        axes[i] = (val * 128.0) + 0.5;
      } else {
        axes[i] = (val * 127.0) + 0.5;
      }
    }
  }

  FMSState GetCurrentState() {
    if (ds->IsDisabled()) return FMSState::kDisabled;
    if (ds->IsAutonomous()) return FMSState::kAutonomous;
    if (ds->IsOperatorControl()) return FMSState::kTeleop;
    if (ds->IsTest()) return FMSState::kTestMode;
    LOG(ERROR, "unknown fms state\n");
    return FMSState::kDisabled;
  }
};

// Designed to be called from the console.
extern "C" int battery_voltage() {
  printf("battery is currently at %fV\n",
         ControlsManager::GetInstance().GetDS()->GetBatteryVoltage());
  return 0;
}

}  // namespace crio
}  // namespace aos

AOS_RUN_FORK(aos::crio::JoystickRead, "JSR", 100)
