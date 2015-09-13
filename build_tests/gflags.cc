#include "third_party/gflags/include/gflags/gflags.h"

DEFINE_int32(test_flag, 0, "Test command-line flag");

int main() {
  return FLAGS_test_flag;
}
