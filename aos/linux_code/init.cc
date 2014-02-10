#include "aos/linux_code/init.h"

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <sched.h>
#include <sys/resource.h>
#include <asm-generic/resource.h>  // for RLIMIT_RTTIME
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#include "aos/common/die.h"
#include "aos/linux_code/logging/linux_logging.h"
#include "aos/linux_code/ipc_lib/shared_mem.h"

namespace aos {

namespace {

void SetSoftRLimit(int resource, rlim64_t soft, bool set_for_root) {
  bool am_root = getuid() == 0;
  if (set_for_root || !am_root) {
    struct rlimit64 rlim;
    if (getrlimit64(resource, &rlim) == -1) {
      Die("%s-init: getrlimit64(%d) failed with %d (%s)\n",
          program_invocation_short_name, resource, errno, strerror(errno));
    }
    rlim.rlim_cur = soft;
    if (setrlimit64(resource, &rlim) == -1) {
      Die("%s-init: setrlimit64(%d, {cur=%ju,max=%ju})"
          " failed with %d (%s)\n", program_invocation_short_name,
          resource, (uintmax_t)rlim.rlim_cur, (uintmax_t)rlim.rlim_max,
          errno, strerror(errno));
    }
  }
}

// Common stuff that needs to happen at the beginning of both the realtime and
// non-realtime initialization sequences. May be called twice.
void InitStart() {
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, false);

  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, true);
}

int LockAllMemory() {
  InitStart();
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    Die("%s-init: mlockall failed with %d (%s)\n",
        program_invocation_short_name, errno, strerror(errno));
  }

  // Forces the memory pages for all the stack space that we're ever going to
  // use to be loaded into memory (so it can be locked there).
  uint8_t data[4096 * 8];
  // Not 0 because linux might optimize that to a 0-filled page.
  memset(data, 1, sizeof(data));

  return 0;
}

// Do the initialization code that is necessary for both realtime and
// non-realtime processes.
void DoInitNRT(aos_core_create create) {
  InitStart();
  if (aos_core_create_shared_mem(create)) {
    Die("%s-init: creating shared memory reference failed\n",
        program_invocation_short_name);
  }
  logging::linux_code::Register();
}

const char *const kNoRealtimeEnvironmentVariable = "AOS_NO_REALTIME";

}  // namespace

void InitNRT() { DoInitNRT(aos_core_create::reference); }
void InitCreate() { DoInitNRT(aos_core_create::create); }
void Init(int relative_priority) {
  if (getenv(kNoRealtimeEnvironmentVariable) == NULL) {  // if nobody set it
    LockAllMemory();
    // Only let rt processes run for 3 seconds straight.
    SetSoftRLimit(RLIMIT_RTTIME, 3000000, true);
    // Allow rt processes up to priority 40.
    SetSoftRLimit(RLIMIT_RTPRIO, 40, false);
    // Set our process to priority 40.
    struct sched_param param;
    param.sched_priority = 30 + relative_priority;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      Die("%s-init: setting SCHED_FIFO failed with %d (%s)\n",
          program_invocation_short_name, errno, strerror(errno));
    }
  } else {
    fprintf(stderr, "%s not doing realtime initialization because environment"
            " variable %s is set\n", program_invocation_short_name,
            kNoRealtimeEnvironmentVariable);
    printf("no realtime for %s. see stderr\n", program_invocation_short_name);
  }

  InitNRT();
}

void Cleanup() {
  if (aos_core_free_shared_mem()) {
    Die("%s-init: freeing shared mem failed\n",
        program_invocation_short_name);
  }
}

}  // namespace aos
