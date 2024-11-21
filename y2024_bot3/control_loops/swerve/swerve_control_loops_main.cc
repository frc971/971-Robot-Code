#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/swerve/swerve_control_loops.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/swerve/parameters.h"

using frc971::control_loops::swerve::SwerveControlLoops;
using y2024_bot3::control_loops::CreateWeights;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  frc971::constants::WaitForConstants<y2024_bot3::Constants>(&config.message());

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::ConstantsFetcher<y2024_bot3::Constants> constants(
      &event_loop);

  SwerveControlLoops swerve_control_loops(
      &event_loop, constants.constants().common()->rotation(),
      constants.constants().robot()->swerve_zeroing(),
      y2024_bot3::control_loops::MakeSwerveParameters<float>(),
      CreateWeights(constants.constants().common()->weights()), "/drivetrain");

  event_loop.Run();

  return 0;
}
