#include "aos/init.h"

#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/log/flags.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"

#include "aos/realtime.h"
#include "aos/uuid.h"

ABSL_FLAG(bool, coredump, false, "If true, write core dumps on failure.");
ABSL_FLAG(bool, backtrace, true, "If true, print backtraces out on crashes.");

namespace aos {
namespace {
std::atomic<bool> initialized{false};
}  // namespace

bool IsInitialized() { return initialized; }

void InitGoogle(int *argc, char ***argv) {
  CHECK(!IsInitialized()) << "Only initialize once.";
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  std::vector<char *> positional_arguments =
      absl::ParseCommandLine(*argc, *argv);

  {
    const std::vector<absl::UnrecognizedFlag> unrecognized_flags;
    absl::ReportUnrecognizedFlags(unrecognized_flags);
    if (!unrecognized_flags.empty()) {
      for (const absl::UnrecognizedFlag &flag : unrecognized_flags) {
        LOG(ERROR) << "Unrecognized flag " << flag.flag_name;
      }
      LOG(FATAL) << "Found unrecognized flags, aborting";
    }
  }

  CHECK_LE(positional_arguments.size(), static_cast<size_t>(*argc));
  for (size_t i = 0; i < positional_arguments.size(); ++i) {
    (*argv)[i] = positional_arguments[i];
  }
  *argc = positional_arguments.size();

  absl::InitializeLog();

  if (absl::GetFlag(FLAGS_backtrace)) {
    absl::InitializeSymbolizer((*argv)[0]);
    absl::FailureSignalHandlerOptions options;
    absl::InstallFailureSignalHandler(options);
  }

  if (absl::GetFlag(FLAGS_coredump)) {
    WriteCoreDumps();
  }

  RegisterMallocHook();
  // Ensure that the random number generator for the UUID code is initialized
  // (it does some potentially expensive random number generation).
  UUID::Random();

  initialized = true;
}

void MarkInitialized() { initialized = true; }

}  // namespace aos
