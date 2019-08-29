#include <iostream>
#include <getopt.h>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

DEFINE_bool(print_logs, false,
            "Print the log messages as they are being generated.");
DEFINE_string(log_file, "",
              "Print all log messages to FILE instead of standard output.");

namespace aos {
namespace testing {

// Actually declared/defined in //aos/testing:test_logging.
void SetLogFileName(const char* filename) __attribute__((weak));
void ForcePrintLogsDuringTests() __attribute__((weak));

}  // namespace testing
}  // namespace aos

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  ::gflags::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

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

  return RUN_ALL_TESTS();
}
