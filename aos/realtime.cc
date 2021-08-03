#include "aos/realtime.h"

#include <malloc.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "aos/thread_local.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"

DEFINE_bool(
    die_on_malloc, false,
    "If true, die when the application allocates memory in a RT section.");
DEFINE_bool(skip_realtime_scheduler, false,
            "If true, skip changing the scheduler.  Pretend that we changed "
            "the scheduler instead.");
DEFINE_bool(skip_locking_memory, false,
            "If true, skip locking memory.  Pretend that we did it instead.");

extern "C" {
typedef void (*MallocHook_NewHook)(const void *ptr, size_t size);
int MallocHook_AddNewHook(MallocHook_NewHook hook) __attribute__((weak));
int MallocHook_RemoveNewHook(MallocHook_NewHook hook) __attribute__((weak));

typedef void (*MallocHook_DeleteHook)(const void *ptr);
int MallocHook_AddDeleteHook(MallocHook_DeleteHook hook) __attribute__((weak));
int MallocHook_RemoveDeleteHook(MallocHook_DeleteHook hook)
    __attribute__((weak));

// Declare tc_malloc weak so we can check if it exists.
void *tc_malloc(size_t size) __attribute__((weak));

void *__libc_malloc(size_t size);
void __libc_free(void *ptr);
}  // extern "C"

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

enum class SetLimitForRoot { kYes, kNo };

enum class AllowSoftLimitDecrease { kYes, kNo };

void SetSoftRLimit(
    int resource, rlim64_t soft, SetLimitForRoot set_for_root,
    std::string_view help_string,
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
        << " with max of " << rlim.rlim_max << help_string;
  }
}

}  // namespace

void LockAllMemory() {
  CheckNotRealtime();
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, SetLimitForRoot::kNo,
                "use --skip_locking_memory to not lock memory.");

  WriteCoreDumps();
  PCHECK(mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
      << ": Failed to lock memory, use --skip_locking_memory to bypass this.  "
         "Bypassing will impact RT performance.";

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
  __asm__ __volatile__("" ::"m"(data));

  static const size_t kHeapPreallocSize = 512 * 1024;
  char *const heap_data = static_cast<char *>(malloc(kHeapPreallocSize));
  memset(heap_data, 1, kHeapPreallocSize);
  __asm__ __volatile__("" ::"m"(heap_data));
  free(heap_data);
}

void InitRT() {
  if (FLAGS_skip_locking_memory) {
    LOG(WARNING) << "Ignoring request to lock all memory due to "
                    "--skip_locking_memory.";
    return;
  }

  CheckNotRealtime();
  LockAllMemory();

  if (FLAGS_skip_realtime_scheduler) {
    return;
  }
  // Only let rt processes run for 3 seconds straight.
  SetSoftRLimit(
      RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");

  // Allow rt processes up to priority 40.
  SetSoftRLimit(
      RLIMIT_RTPRIO, 40, SetLimitForRoot::kNo,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");
}

void UnsetCurrentThreadRealtimePriority() {
  struct sched_param param;
  param.sched_priority = 0;
  PCHECK(sched_setscheduler(0, SCHED_OTHER, &param) == 0);
  MarkRealtime(false);
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
  if (FLAGS_skip_realtime_scheduler) {
    LOG(WARNING) << "Ignoring request to switch to the RT scheduler due to "
                    "--skip_realtime_scheduler.";
    return;
  }
  // Make sure we will only be allowed to run for 3 seconds straight.
  SetSoftRLimit(
      RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");

  // Raise our soft rlimit if necessary.
  SetSoftRLimit(
      RLIMIT_RTPRIO, priority, SetLimitForRoot::kNo,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.",
      AllowSoftLimitDecrease::kNo);

  struct sched_param param;
  param.sched_priority = priority;
  MarkRealtime(true);
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0)
      << ": changing to SCHED_FIFO with " << priority
      << ", if you want to bypass this check for testing, use "
         "--skip_realtime_scheduler";
}

void WriteCoreDumps() {
  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, SetLimitForRoot::kYes, "");
}

void ExpandStackSize() {
  SetSoftRLimit(RLIMIT_STACK, 1000000, SetLimitForRoot::kYes, "",
                AllowSoftLimitDecrease::kNo);
}

namespace {
// Bool to track if malloc hooks have failed to be configured.
bool has_malloc_hook = true;
AOS_THREAD_LOCAL bool is_realtime = false;
}  // namespace

bool MarkRealtime(bool realtime) {
  if (realtime) {
    // For some applications (generally tools built for the host in Bazel), we
    // don't have malloc hooks available, but we also don't go realtime.  Delay
    // complaining in that case until we try to go RT and it matters.
    CHECK(has_malloc_hook)
        << ": Failed to register required malloc hooks before going realtime.  "
           "Disable --die_on_malloc to continue.";
  }
  const bool prior = is_realtime;
  is_realtime = realtime;
  return prior;
}

void CheckRealtime() { CHECK(is_realtime); }

void CheckNotRealtime() { CHECK(!is_realtime); }

ScopedRealtimeRestorer::ScopedRealtimeRestorer() : prior_(is_realtime) {}

void NewHook(const void *ptr, size_t size) {
  if (is_realtime) {
    is_realtime = false;
    RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }
}

void DeleteHook(const void *ptr) {
  // It is legal to call free(nullptr) unconditionally and assume that it won't
  // do anything.  Eigen does this.  So, if we are RT, ignore any of these
  // calls.
  if (is_realtime && ptr != nullptr) {
    is_realtime = false;
    RAW_LOG(FATAL, "Delete Hook %p", ptr);
  }
}

extern "C" {

// malloc hooks for libc. Tcmalloc will replace everything it finds (malloc,
// __libc_malloc, etc.), so we need its specific hook above as well.
void *aos_malloc_hook(size_t size) {
  if (FLAGS_die_on_malloc && aos::is_realtime) {
    aos::is_realtime = false;
    RAW_LOG(FATAL, "Malloced %zu bytes", size);
    return nullptr;
  } else {
    return __libc_malloc(size);
  }
}

void aos_free_hook(void *ptr) {
  if (FLAGS_die_on_malloc && aos::is_realtime && ptr != nullptr) {
    aos::is_realtime = false;
    RAW_LOG(FATAL, "Deleted %p", ptr);
  } else {
    __libc_free(ptr);
  }
}

void *malloc(size_t size) __attribute__((weak, alias("aos_malloc_hook")));
void free(void *ptr) __attribute__((weak, alias("aos_free_hook")));

}

void RegisterMallocHook() {
  if (FLAGS_die_on_malloc) {
    // tcmalloc redefines __libc_malloc, so use this as a feature test.
    if (&__libc_malloc == &tc_malloc) {
      RAW_LOG(INFO, "Hooking tcmalloc for die_on_malloc");
      if (&MallocHook_AddNewHook != nullptr) {
        CHECK(MallocHook_AddNewHook(&NewHook));
      } else {
        has_malloc_hook = false;
      }
      if (&MallocHook_AddDeleteHook != nullptr) {
        CHECK(MallocHook_AddDeleteHook(&DeleteHook));
      } else {
        has_malloc_hook = false;
      }
    } else {
      RAW_LOG(INFO, "Replacing glibc malloc");
      if (&malloc != &aos_malloc_hook) {
        has_malloc_hook = false;
      }
      if (&free != &aos_free_hook) {
        has_malloc_hook = false;
      }
    }
  }
}

}  // namespace aos
