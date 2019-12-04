#include "aos/init.h"

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/prctl.h>
#include <malloc.h>

#include "aos/die.h"
#include "aos/ipc_lib/shared_mem.h"
#include "aos/logging/implementations.h"
#include "aos/realtime.h"

namespace FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead {
extern double FLAGS_tcmalloc_release_rate __attribute__((weak));
}
using FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead::
    FLAGS_tcmalloc_release_rate;

namespace aos {
namespace logging {
namespace internal {

// Implemented in aos/logging/context.cc.
void ReloadThreadName();

}  // namespace internal
}  // namespace logging
namespace {

// Common stuff that needs to happen at the beginning of both the realtime and
// non-realtime initialization sequences. May be called twice.
void InitStart() {
  ::aos::logging::Init();
  WriteCoreDumps();
  google::InstallFailureSignalHandler();
}

const char *const kNoRealtimeEnvironmentVariable = "AOS_NO_REALTIME";

bool ShouldBeRealtime() {
  return getenv(kNoRealtimeEnvironmentVariable) == nullptr;
}

}  // namespace

void InitGoogle(int *argc, char ***argv) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging((*argv)[0]);
  gflags::ParseCommandLineFlags(argc, argv, true);
  google::InstallFailureSignalHandler();
}

void InitNRT(bool for_realtime) {
  InitStart();
  aos_core_create_shared_mem(false, for_realtime && ShouldBeRealtime());
  logging::RegisterQueueImplementation();
  AOS_LOG(INFO, "%s initialized non-realtime\n", program_invocation_short_name);
}

void InitCreate() {
  InitStart();
  aos_core_create_shared_mem(true, false);
  logging::RegisterQueueImplementation();
  AOS_LOG(INFO, "%s created shm\n", program_invocation_short_name);
}

void Init(int relative_priority) {
  InitStart();
  aos_core_create_shared_mem(false, ShouldBeRealtime());
  logging::RegisterQueueImplementation();
  GoRT(relative_priority);
}

void GoRT(int relative_priority) {
  if (ShouldBeRealtime()) {
    InitRT();

    // Set our process to the appropriate priority.
    struct sched_param param;
    param.sched_priority = 30 + relative_priority;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      PDie("%s-init: setting SCHED_FIFO failed", program_invocation_short_name);
    }
  } else {
    fprintf(stderr,
            "%s not doing realtime initialization because environment"
            " variable %s is set\n",
            program_invocation_short_name, kNoRealtimeEnvironmentVariable);
    printf("no realtime for %s. see stderr\n", program_invocation_short_name);
  }

  AOS_LOG(INFO, "%s initialized realtime\n", program_invocation_short_name);
}

void Cleanup() {
  aos_core_free_shared_mem();
}

void PinCurrentThreadToCPU(int number) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(number, &cpuset);
  AOS_PRCHECK(pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset));
}

}  // namespace aos
