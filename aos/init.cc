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

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/realtime.h"
#include "aos/uuid.h"

DEFINE_bool(coredump, false, "If true, write core dumps on failure.");

namespace aos {
namespace {
bool initialized = false;
}  // namespace

bool IsInitialized() { return initialized; }

void InitGoogle(int *argc, char ***argv) {
  CHECK(!IsInitialized()) << "Only initialize once.";
  FLAGS_logtostderr = true;
  google::InitGoogleLogging((*argv)[0]);
  gflags::ParseCommandLineFlags(argc, argv, true);
  google::InstallFailureSignalHandler();

  if (FLAGS_coredump) {
    WriteCoreDumps();
  }

  RegisterMallocHook();
  // Ensure that the random number generator for the UUID code is initialized
  // (it does some potentially expensive random number generation).
  UUID::Random();

  initialized = true;
}

void InitFromRust(const char *argv0) {
  CHECK(!IsInitialized()) << "Only initialize once.";

  FLAGS_logtostderr = true;

  google::InitGoogleLogging(argv0);

  // TODO(Brian): Provide a way for Rust to configure C++ flags.
  char fake_argv0_val[] = "rust";
  char *fake_argv0 = fake_argv0_val;
  char **fake_argv = &fake_argv0;
  int fake_argc = 1;
  gflags::ParseCommandLineFlags(&fake_argc, &fake_argv, true);

  // TODO(Brian): Where should Rust binaries be configured to write coredumps?

  // TODO(Brian): Figure out what to do with allocator hooks for C++ and Rust.

  initialized = true;
}

}  // namespace aos
