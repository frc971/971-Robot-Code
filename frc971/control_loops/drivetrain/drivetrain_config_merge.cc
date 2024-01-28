#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/drivetrain/drivetrain_config_static.h"

// This takes in a bunch of JSON files, parses them as DrivetrainLoopConfig
// flatbuffers, and attempts to merge them all together. This allows us to take
// the outputs of the various drivetrain python codegen's and merge them all
// into a single JSON file that can be imported in the constants for a robot.
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<
      frc971::control_loops::drivetrain::fbs::DrivetrainLoopConfig>
      merged = aos::JsonToFlatbuffer<
          frc971::control_loops::drivetrain::fbs::DrivetrainLoopConfig>("{}");

  const std::string_view output_path = argv[argc - 1];
  for (int arg = 1; arg < argc - 1; ++arg) {
    VLOG(1) << "Reading " << argv[arg];
    merged = aos::MergeFlatBuffers(
        merged,
        aos::JsonFileToFlatbuffer<
            frc971::control_loops::drivetrain::fbs::DrivetrainLoopConfig>(
            argv[arg]));
  }
  aos::WriteFlatbufferToJson(output_path, merged);
  return EXIT_SUCCESS;
}
