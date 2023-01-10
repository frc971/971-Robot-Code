#include <memory>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2023/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain::DrivetrainLoop;

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  aos::ShmEventLoop event_loop(&config.message());
  std::unique_ptr<::frc971::control_loops::drivetrain::DeadReckonEkf>
      localizer =
          std::make_unique<::frc971::control_loops::drivetrain::DeadReckonEkf>(
              &event_loop,
              ::y2023::control_loops::drivetrain::GetDrivetrainConfig());

  event_loop.Run();

  return 0;
}
