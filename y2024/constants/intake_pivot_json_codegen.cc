#include "aos/util/file.h"
#include "y2024/constants.h"

using namespace y2024::constants;

// This file generates some JSON constants information for the intake that are
// currently dependent on values that are located in C++ headers and would be
// obnoxious/inappropriate to pull out. In the future the file may be used to
// generate constants information for other subsystems as well
int main(int argc, char *argv[]) {
  CHECK_EQ(argc, 2) << "Must supply file name to output to.";
  std::string output_file = argv[1];

  std::stringstream output;

  output << "\"average_filter_size\": " << Values::kZeroingSampleSize << ",\n";
  output << "\"one_revolution_distance\": "
         << M_PI * 2.0 * Values::kIntakePivotEncoderRatio() << ",\n";
  output << "\"zeroing_threshold\": 0.0005,\n";
  output << "\"moving_buffer_size\": 20,\n";
  output << "\"allowable_encoder_error\": 0.9\n";
  aos::util::WriteStringToFileOrDie(output_file, output.str());
  return 0;
}