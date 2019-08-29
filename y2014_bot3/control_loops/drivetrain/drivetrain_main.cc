#include "aos/init.h"

#include "aos/events/shm_event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main() {
  ::aos::InitNRT(true);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::frc971::control_loops::drivetrain::DeadReckonEkf localizer(
      &event_loop,
      ::y2014_bot3::control_loops::drivetrain::GetDrivetrainConfig());
  DrivetrainLoop drivetrain(
      ::y2014_bot3::control_loops::drivetrain::GetDrivetrainConfig(),
      &event_loop, &localizer);

  event_loop.Run();

  ::aos::Cleanup();
  return 0;
}
