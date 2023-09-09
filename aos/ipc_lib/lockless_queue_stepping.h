#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_STEPPING_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_STEPPING_H_

#include <cinttypes>
#include <functional>

#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/lockless_queue_memory.h"

namespace aos {
namespace ipc_lib {
namespace testing {

#if defined(__ARM_EABI__)
// There are various reasons why we might not actually be able to do this
// testing, but we still want to run the functions to test anything they test
// other than shm robustness.
//
// ARM doesn't have a nice way to do single-stepping, and I don't feel like
// dealing with editing the next instruction to be a breakpoint and then
// changing it back.
#else

#define SUPPORTS_SHM_ROBUSTNESS_TEST

// This code currently supports amd64 only, but it
// shouldn't be hard to port to i386 (it should just be using the name for
// only the smaller part of the flags register), but that's not tested, and
// porting it to any other architecture is more work.
// Currently, we skip doing anything exciting on arm (just run the code without
// any robustness testing) and refuse to compile anywhere else.

#define SIMPLE_ASSERT(condition, message)                                 \
  do {                                                                    \
    if (!(condition)) {                                                   \
      static const char kMessage[] = message "\n";                        \
      if (write(STDERR_FILENO, kMessage, sizeof(kMessage) - 1) !=         \
          (sizeof(kMessage) - 1)) {                                       \
        static const char kFailureMessage[] = "writing failed\n";         \
        __attribute__((unused)) int ignore = write(                       \
            STDERR_FILENO, kFailureMessage, sizeof(kFailureMessage) - 1); \
      }                                                                   \
      abort();                                                            \
    }                                                                     \
  } while (false)

// Array to track writes to memory, and make sure they happen in the right
// order.
class WritesArray {
 public:
  uintptr_t At(size_t location) const {
    SIMPLE_ASSERT(location < size_, "too far into writes array");
    return writes_[location];
  }
  void Add(uintptr_t pointer) {
    SIMPLE_ASSERT(size_ < kSize, "too many writes");
    writes_[size_++] = pointer;
  }

  size_t size() const { return size_; }

 private:
  static const size_t kSize = 20000;

  uintptr_t writes_[kSize];
  size_t size_ = 0;
};

enum class DieAtState {
  // No SEGVs should be happening.
  kDisabled,
  // SEGVs are fine.  Normal operation.
  kRunning,
  // We are manipulating a mutex.  No SEGVs should be happening.
  kWriting,
};

// What we exit with when we're exiting in the middle.
const int kExitEarlyValue = 123;

// We have to keep track of everything in a global variable because there's no
// other way for the signal handlers to find it.
struct GlobalState {
  // Constructs the singleton GlobalState.
  static std::tuple<GlobalState *, WritesArray *> MakeGlobalState();

  // Returns the global state.  Atomic and safe from signal handlers.
  static GlobalState *Get();

  // Pointer to the queue memory, and its size.
  void *lockless_queue_memory;
  size_t lockless_queue_memory_size;

  // Pointer to a second block of memory the same size.  This (on purpose) has
  // the same size as lockless_queue_memory so we can point the robust mutexes
  // here.
  void *lockless_queue_memory_lock_backup;

  // Expected writes.
  const WritesArray *writes_in;
  // Actual writes.
  WritesArray *writes_out;
  // Location to die at, and how far we have gotten.
  size_t die_at, current_location;
  // State.
  DieAtState state;

  // Returns true if the address is in the queue memory chunk.
  bool IsInLocklessQueueMemory(void *address);

  // Calls mprotect(2) for the entire shared memory region with the given prot.
  void ShmProtectOrDie(int prot);

  // Checks a write into the queue and conditionally dies.  Tracks the write.
  void HandleWrite(void *address);

  // Registers the handlers required to trap writes.
  void RegisterSegvAndTrapHandlers();
};

void TestShmRobustness(const LocklessQueueConfiguration &config,
                       ::std::function<void(void *)> prepare,
                       ::std::function<void(void *)> function,
                       ::std::function<void(void *)> check);

// Capture the tid in the child so we can tell if it died.  Uses mmap so it
// works across the process boundary.
class SharedTid {
 public:
  SharedTid();
  ~SharedTid();

  // Captures the tid.
  void Set();

  // Returns the captured tid.
  pid_t Get();

 private:
  pid_t *tid_;
};

// Sets FUTEX_OWNER_DIED if the owner was tid.  This fakes what the kernel does
// with a robust mutex.
bool PretendOwnerDied(aos_mutex *mutex, pid_t tid);

#endif

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos

#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_STEPPING_H_
