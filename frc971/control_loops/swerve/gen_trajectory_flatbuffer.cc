#include <iostream>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/event_loop.h"
#include "aos/init.h"
#include "frc971/control_loops/swerve/swerve_trajectory_static.h"

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  CHECK(argc == 3)
      << ": Expected input json and output bfbs files to be passed in.";

  std::string_view trajectory_path = argv[1];
  std::string_view output_path = argv[2];

  aos::FlatbufferDetachedBuffer<frc971::control_loops::swerve::SwerveTrajectory>
      trajectory_ = aos::JsonFileToFlatbuffer<
          frc971::control_loops::swerve::SwerveTrajectory>(trajectory_path);

  aos::WriteFlatbufferToFile(output_path, trajectory_);
  LOG(INFO) << "Created flatbuffer binary '" << output_path
            << "' from trajectory json '" << trajectory_path << "'";
}