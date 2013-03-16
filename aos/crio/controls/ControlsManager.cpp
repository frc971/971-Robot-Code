#include "aos/crio/controls/ControlsManager.h"

#include <stdio.h>
#include <stdlib.h>

#include "WPILib/Compressor.h"

#include "aos/crio/logging/crio_logging.h"
#include "aos/common/Configuration.h"
#include "aos/crio/aos_ctdt.h"
#include "aos/crio/motor_server/MotorServer.h"

namespace aos {
namespace crio {

// Everything gets an explicit Start call here before calling all of the init
// functions because it means that all static variables will be initialized
// before anything actually starts running. It also means that everything will
// be initialized before any of the init functions start trying to register
// themselves etc.
void ControlsManager::StartCompetition() {
  printf("aos::ControlsManager::RobotMain\n");
  (new Compressor(14, 1))->Start();

  logging::crio::Register();
  LOG(INFO, "logging started\n");

  GetWatchdog().SetEnabled(false);
  LOG(INFO, "disabled watchdog\n");

  MotorServer::Start();
  LOG(INFO, "MotorServer started\n");

  LOG(INFO, "calling init functions\n");
  aos_call_init_functions();
  LOG(INFO, "initialized\n");

  RegisterControlLoops();
  LOG(INFO, "registered control loops\n");

  StartSensorBroadcasters();
  LOG(INFO, "started sensor broadcasters\n");

  // Wait forever so that this task doesn't end to avoid confusing any brittle
  // FIRST code that might be hiding somewhere.
  while (true) {
    select(0, NULL, NULL, NULL, NULL);
  }
}

}  // namespace crio
}  // namespace aos
