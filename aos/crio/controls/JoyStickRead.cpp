#include "WPILib/Task.h"
#include "WPILib/Timer.h"

#include "aos/aos_core.h"
#include "aos/crio/controls/ControlsManager.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/messages/RobotState.q.h"

namespace aos {
namespace crio {

class JoystickRead {
  /*virtual void Disabled () {
    int i = 0;
    while (IsDisabled()) {
    printf("Disabled! %d\n", i);
    Wait(0.1);
    i++;
    }
    printf("Done with disabled. %d\n", i);
    }
    virtual void Autonomous () {
    int j = 0;
    while (IsAutonomous()) {
    printf("Autonomous!  %d\n", j);
    Wait(0.1);
  //if (j > 5) {
  //i(0);
  //}
  j ++;
  }
  printf("Done with autonomous. %d\n", j);
  }
  virtual void OperatorControl () {
  int i = 0;
  while (IsOperatorControl()) {
  printf("Operator Control!  %d\n", i);
  Wait(0.1);
  i ++;
  }
  printf("Done with operator control. %d\n", i);
  }*/
 public:
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
};

}  // namespace crio
}  // namespace aos

AOS_RUN_FORK(aos::crio::JoystickRead, "JSR", 100)
