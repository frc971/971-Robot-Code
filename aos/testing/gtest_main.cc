#include <iostream>
#include <getopt.h>

#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Actually declared/defined in //aos/testing:test_logging.
void SetLogFileName(const char* filename) __attribute__((weak));
void ForcePrintLogsDuringTests() __attribute__((weak));

}  // namespace testing
}  // namespace aos

GTEST_API_ int main(int argc, char **argv) {
  static const struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"print-logs", no_argument, 0, 'p'},
      {"log-file", required_argument, 0, 'o'},
      {0, 0, 0, 0},
  };

  testing::InitGoogleTest(&argc, argv);

  // The gtest library modifies argc and argv to remove all of its own command
  // line switches etc. So after calling InitGoogleTest() we can parse our own
  // command line options.
  while (true) {
    int c = getopt_long(argc, argv, "pho:", long_options, nullptr);

    if (c == -1) {
      break;
    }

    switch (c) {
      case 'h':
        printf(
            "\nFRC971 options:\n"
            "  -p, --print-logs\n"
            "      Print the log messages as they are being generated.\n"
            "  -o, --log-file=FILE\n"
            "      Print all log messages to FILE instead of standard output\n"
            );
        break;

      case 'p':
        if (::aos::testing::ForcePrintLogsDuringTests) {
          ::aos::testing::ForcePrintLogsDuringTests();
        }
        break;

      case 'o':
        if (::aos::testing::SetLogFileName) {
          ::aos::testing::SetLogFileName(optarg);
        }
        break;

      case '?':
        abort();
    }
  }

  return RUN_ALL_TESTS();
}
