#include "aos/realtime.h"

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

#include "glog/logging.h"

namespace FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead {
extern double FLAGS_tcmalloc_release_rate __attribute__((weak));
}
using FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead::
    FLAGS_tcmalloc_release_rate;

namespace aos {
namespace logging {
namespace internal {

// Implemented in aos/logging/context.cc.
void ReloadThreadName() __attribute__((weak));

}  // namespace internal
}  // namespace logging

namespace {

enum class SetLimitForRoot {
  kYes,
  kNo
};

enum class AllowSoftLimitDecrease {
  kYes,
  kNo
};

void SetSoftRLimit(
    int resource, rlim64_t soft, SetLimitForRoot set_for_root,
    AllowSoftLimitDecrease allow_decrease = AllowSoftLimitDecrease::kYes) {
  bool am_root = getuid() == 0;
  if (set_for_root == SetLimitForRoot::kYes || !am_root) {
    struct rlimit64 rlim;
    PCHECK(getrlimit64(resource, &rlim) == 0)
        << ": getting limit for " << resource;

    if (allow_decrease == AllowSoftLimitDecrease::kYes) {
      rlim.rlim_cur = soft;
    } else {
      rlim.rlim_cur = std::max(rlim.rlim_cur, soft);
    }
    rlim.rlim_max = ::std::max(rlim.rlim_max, soft);

    PCHECK(setrlimit64(resource, &rlim) == 0)
        << ": changing limit for " << resource << " to " << rlim.rlim_cur
        << " with max of " << rlim.rlim_max;
  }
}

}  // namespace

void LockAllMemory() {
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, SetLimitForRoot::kNo);

  WriteCoreDumps();
  PCHECK(mlockall(MCL_CURRENT | MCL_FUTURE) == 0);

#if !__has_feature(address_sanitizer) && !__has_feature(memory_sanitizer)
  // Don't give freed memory back to the OS.
  CHECK_EQ(1, mallopt(M_TRIM_THRESHOLD, -1));
  // Don't use mmap for large malloc chunks.
  CHECK_EQ(1, mallopt(M_MMAP_MAX, 0));
#endif

  if (&FLAGS_tcmalloc_release_rate) {
    // Tell tcmalloc not to return memory.
    FLAGS_tcmalloc_release_rate = 0.0;
  }

  // Forces the memory pages for all the stack space that we're ever going to
  // use to be loaded into memory (so it can be locked there).
  uint8_t data[4096 * 8];
  // Not 0 because linux might optimize that to a 0-filled page.
  memset(data, 1, sizeof(data));
  __asm__ __volatile__("" :: "m" (data));

  static const size_t kHeapPreallocSize = 512 * 1024;
  char *const heap_data = static_cast<char *>(malloc(kHeapPreallocSize));
  memset(heap_data, 1, kHeapPreallocSize);
  __asm__ __volatile__("" :: "m" (heap_data));
  free(heap_data);
}

void InitRT() {
  LockAllMemory();

  // Only let rt processes run for 3 seconds straight.
  SetSoftRLimit(RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes);

  // Allow rt processes up to priority 40.
  SetSoftRLimit(RLIMIT_RTPRIO, 40, SetLimitForRoot::kNo);
}

void UnsetCurrentThreadRealtimePriority() {
  struct sched_param param;
  param.sched_priority = 0;
  PCHECK(sched_setscheduler(0, SCHED_OTHER, &param) == 0);
}

void SetCurrentThreadAffinity(const cpu_set_t &cpuset) {
  PCHECK(sched_setaffinity(0, sizeof(cpuset), &cpuset) == 0);
}

void SetCurrentThreadName(const std::string_view name) {
  CHECK_LE(name.size(), 16u) << ": thread name '" << name << "' too long";
  VLOG(1) << "This thread is changing to '" << name << "'";
  std::string string_name(name);
  PCHECK(prctl(PR_SET_NAME, string_name.c_str()) == 0)
      << ": changing name to " << string_name;
  if (&logging::internal::ReloadThreadName != nullptr) {
    logging::internal::ReloadThreadName();
  }
}

void SetCurrentThreadRealtimePriority(int priority) {
  // Make sure we will only be allowed to run for 3 seconds straight.
  SetSoftRLimit(RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes);

  // Raise our soft rlimit if necessary.
  SetSoftRLimit(RLIMIT_RTPRIO, priority, SetLimitForRoot::kNo,
                AllowSoftLimitDecrease::kNo);

  struct sched_param param;
  param.sched_priority = priority;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0)
      << ": changing to SCHED_FIFO with " << priority;
}

void WriteCoreDumps() {
  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, SetLimitForRoot::kYes);
}

void ExpandStackSize() {
  SetSoftRLimit(RLIMIT_STACK, 1000000, SetLimitForRoot::kYes,
                AllowSoftLimitDecrease::kNo);
}

}  // namespace aos
