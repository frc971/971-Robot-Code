#include "aos/util/file.h"
#include "y2023/constants.h"

using namespace y2023::constants;

// This file generates some JSON constants information that is currently
// dependent on values that are located in C++ headers and would be
// obnoxious/inappropriate to pull out.
int main(int argc, char *argv[]) {
  CHECK_EQ(argc, 2) << "Must supply file name to output to.";
  std::string output_file = argv[1];

  std::stringstream output;

  output << "\"average_filter_size\": " << Values::kZeroingSampleSize << ",\n";
  output << "\"one_revolution_distance\": "
         << M_PI * 2.0 * Values::kCompWristEncoderRatio() << ",\n";
  output << "\"zeroing_threshold\": 0.0005,\n";
  output << "\"moving_buffer_size\": 20,\n";
  output << "\"allowable_encoder_error\": 0.9,\n";
  output << "\"middle_position\": " << Values::kCompWristRange().middle()
         << "\n";
  aos::util::WriteStringToFileOrDie(output_file, output.str());
  return 0;
}
