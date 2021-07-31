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

#include "aos/realtime.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

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
  initialized = true;
}

}  // namespace aos
