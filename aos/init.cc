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

void MarkInitialized() { initialized = true; }

}  // namespace aos
