#include <sys/resource.h>
#include <sys/time.h>

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/drivetrain/trajectory_generator.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"

using ::frc971::control_loops::drivetrain::TrajectoryGenerator;

DEFINE_bool(skip_renicing, false,
            "If true, skip renicing the trajectory generator.");

int main(int argc, char *argv[]) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  frc971::constants::WaitForConstants<y2024::Constants>(&config.message());

  ::aos::ShmEventLoop event_loop(&config.message());
  TrajectoryGenerator generator(
      &event_loop,
      ::y2024::control_loops::drivetrain::GetDrivetrainConfig(&event_loop));

  event_loop.OnRun([]() {
    if (FLAGS_skip_renicing) {
      LOG(WARNING) << "Ignoring request to renice to -20 due to "
                      "--skip_renicing.";
    } else {
      errno = 0;
      setpriority(PRIO_PROCESS, 0, -20);
      PCHECK(errno == 0)
          << ": Renicing to -20 failed, use --skip_renicing to skip renicing.";
    }
  });

  event_loop.Run();

  return 0;
}
