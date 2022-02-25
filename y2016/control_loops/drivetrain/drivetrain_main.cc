#include <memory>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());
  ::frc971::control_loops::drivetrain::DeadReckonEkf localizer(
      &event_loop, ::y2016::control_loops::drivetrain::GetDrivetrainConfig());
  std::unique_ptr<DrivetrainLoop> drivetrain = std::make_unique<DrivetrainLoop>(
      ::y2016::control_loops::drivetrain::GetDrivetrainConfig(), &event_loop,
      &localizer);

  event_loop.Run();

  return 0;
}
