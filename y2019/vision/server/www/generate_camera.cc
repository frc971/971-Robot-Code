#include "y2019/constants.h"
#include "y2019/vision/constants.h"

#include <fstream>
#include <iostream>

namespace y2019 {
namespace vision {
void DumpPose(std::basic_ostream<char> *o, const vision::CameraGeometry &pose) {
  *o << "{x: " << pose.location[0] << ", y: " << pose.location[1]
    << ", theta: " << pose.heading << "}";
}
void DumpTypescriptConstants(const char *fname) {
  ::std::ofstream out_file(fname);
  out_file << "export const CAMERA_POSES = [\n";
  for (size_t ii = 0; ii < constants::Values::kNumCameras; ++ii) {
    out_file << "    ";
    // TODO(james): Decide how to manage visualization for practice and code
    // bots.
    DumpPose(&out_file,
             GetCamera(CameraSerialNumbers(CompBotTeensyId())[ii])->geometry);
    out_file << ",\n";
  }
  out_file << "];\n";
}
}  // namespace constants
}  // namespace y2019

int main(int argc, char *argv[]) {
  if (argc != 2) {
    ::std::cout << "Must provide a filename for output as an argument\n";
    return 1;
  }
  ::y2019::vision::DumpTypescriptConstants(argv[1]);
}
