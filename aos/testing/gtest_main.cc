#include <getopt.h>

#include <iostream>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/init.h"
#include "aos/testing/tmpdir.h"

DEFINE_bool(print_logs, false,
            "Print the log messages as they are being generated.");
DEFINE_string(log_file, "",
              "Print all log messages to FILE instead of standard output.");

namespace aos {

namespace testing {

// Actually declared/defined in //aos/testing:test_logging.
void SetLogFileName(const char *filename) __attribute__((weak));
void ForcePrintLogsDuringTests() __attribute__((weak));

}  // namespace testing
}  // namespace aos

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = true;

  aos::InitGoogle(&argc, &argv);

  if (FLAGS_print_logs) {
    if (::aos::testing::ForcePrintLogsDuringTests) {
      ::aos::testing::ForcePrintLogsDuringTests();
    }
  }

  if (!FLAGS_log_file.empty()) {
    if (::aos::testing::ForcePrintLogsDuringTests) {
      ::aos::testing::ForcePrintLogsDuringTests();
    }
    if (::aos::testing::SetLogFileName) {
      ::aos::testing::SetLogFileName(FLAGS_log_file.c_str());
    }
  }

  // Point shared memory away from /dev/shm if we are testing.  We don't care
  // about RT in this case, so if it is backed by disk, we are fine.
  aos::testing::SetTestShmBase();

  return RUN_ALL_TESTS();
}
