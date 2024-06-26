#include <iostream>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"

#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/logfile_validator.h"
#include "aos/init.h"

ABSL_DECLARE_FLAG(bool, timestamps_to_csv);
ABSL_FLAG(bool, skip_order_validation, false,
          "If true, ignore any out of orderness in replay");

namespace aos::logger {

int Main(int argc, char **argv) {
  const LogFilesContainer log_files(SortParts(FindLogs(argc, argv)));
  CHECK(MultiNodeLogIsReadable(log_files,
                               absl::GetFlag(FLAGS_skip_order_validation)));
  return 0;
}

}  // namespace aos::logger

int main(int argc, char **argv) {
  absl::SetFlag(&FLAGS_timestamps_to_csv, true);
  absl::SetProgramUsageMessage(
      "Usage:\n"
      "  timestamp_extractor [args] logfile1 logfile2 ...\n\nThis program "
      "dumps out all the timestamps from a set of log files for plotting.  Use "
      "--skip_order_validation to skip any time estimation problems we find.");
  aos::InitGoogle(&argc, &argv);

  return aos::logger::Main(argc, argv);
}
