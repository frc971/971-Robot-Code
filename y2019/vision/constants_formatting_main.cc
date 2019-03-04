#include "y2019/vision/constants.h"
#include <iostream>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    ::std::cout << "Expected a command line argument specifying the file name "
                   "to write to.\n";
    return 1;
  }
  ::y2019::vision::DumpCameraConstants(argv[1], -1, {});
}
