#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/drivetrain/localizer.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main() {
  ::aos::InitNRT();

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2020::control_loops::drivetrain::Localizer localizer(
      &event_loop, ::y2020::control_loops::drivetrain::GetDrivetrainConfig());
  DrivetrainLoop drivetrain(
      ::y2020::control_loops::drivetrain::GetDrivetrainConfig(), &event_loop,
      &localizer);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
