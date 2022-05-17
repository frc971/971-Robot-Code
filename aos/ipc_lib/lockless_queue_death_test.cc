#include <dlfcn.h>
#include <elf.h>
#include <linux/futex.h>
#include <sys/mman.h>
#include <sys/procfs.h>
#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <unistd.h>
#include <wait.h>

#include <chrono>
#include <cinttypes>
#include <functional>
#include <memory>
#include <thread>

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/libc/aos_strsignal.h"
#include "aos/realtime.h"
#include "aos/testing/prevent_exit.h"
#include "aos/testing/test_logging.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace ipc_lib {
namespace testing {

namespace chrono = ::std::chrono;

#if ((defined(AOS_SANITIZER_address) || defined(AOS_SANITIZER_thread)) && \
     !defined(__clang__) && __GNUC__ <= 4 && __GNUC_MINOR__ <= 8) ||      \
    defined(__ARM_EABI__)
// There are various reasons why we might not actually be able to do this
// testing, but we still want to run the functions to test anything they test
// other than shm robustness.
//
// GCC 4.8 has too old of a version of asan to allow SIGSEGV handler
// chaining.
//
// GCC 4.8's tsan doesn't work with the code for calling the real sigaction to
// work arounds its weirdness of the SIGTRAP handler.
//
// ARM doesn't have a nice way to do single-stepping, and I don't feel like
// dealing with editing the next instruction to be a breakpoint and then
// changing it back.
#else

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
};
::std::atomic<GlobalState *> global_state;

// Returns true if the address is in the queue memory chunk.
bool IsInLocklessQueueMemory(void *address) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  void *lockless_queue_memory = my_global_state->lockless_queue_memory;
  if (address < lockless_queue_memory) {
    return false;
    if (reinterpret_cast<uintptr_t>(address) >
        reinterpret_cast<uintptr_t>(lockless_queue_memory) +
            my_global_state->lockless_queue_memory_size)
      return false;
  }
  return true;
}

// Calls mprotect(2) for the entire shared memory region with the given prot.
void ShmProtectOrDie(int prot) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  PCHECK(mprotect(my_global_state->lockless_queue_memory,
                  my_global_state->lockless_queue_memory_size, prot) != -1)
      << ": mprotect(" << my_global_state->lockless_queue_memory << ", "
      << my_global_state->lockless_queue_memory_size << ", 0x" << std::hex
      << prot << ") failed";
}

// Checks a write into the queue and conditionally dies.  Tracks the write.
void HandleWrite(void *address) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  uintptr_t address_offset =
      reinterpret_cast<uintptr_t>(address) -
      reinterpret_cast<uintptr_t>(my_global_state->lockless_queue_memory);
  if (my_global_state->writes_in != nullptr) {
    SIMPLE_ASSERT(my_global_state->writes_in->At(
                      my_global_state->current_location) == address_offset,
                  "wrong write order");
  }
  if (my_global_state->writes_out != nullptr) {
    my_global_state->writes_out->Add(address_offset);
  }
  if (my_global_state->die_at != 0) {
    if (my_global_state->die_at == my_global_state->current_location) {
      _exit(kExitEarlyValue);
    }
  }
  ++my_global_state->current_location;
}

struct sigaction old_segv_handler, old_trap_handler;

// Calls the original signal handler.
bool CallChainedAction(const struct sigaction &action, int signal,
                       siginfo_t *siginfo, void *context) {
  if (action.sa_handler == SIG_IGN || action.sa_handler == SIG_DFL) {
    return false;
  }
  if (action.sa_flags & SA_SIGINFO) {
    action.sa_sigaction(signal, siginfo, context);
  } else {
    action.sa_handler(signal);
  }
  return true;
}

void segv_handler(int signal, siginfo_t *siginfo, void *context_void) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  const int saved_errno = errno;
  SIMPLE_ASSERT(signal == SIGSEGV, "wrong signal for SIGSEGV handler");

  // Only process memory addresses in our shared memory block.
  if (!IsInLocklessQueueMemory(siginfo->si_addr)) {
    if (CallChainedAction(old_segv_handler, signal, siginfo, context_void)) {
      errno = saved_errno;
      return;
    } else {
      SIMPLE_ASSERT(false, "actual SIGSEGV");
    }
  }
  SIMPLE_ASSERT(my_global_state->state == DieAtState::kRunning,
                "bad state for SIGSEGV");

  HandleWrite(siginfo->si_addr);

  ShmProtectOrDie(PROT_READ | PROT_WRITE);
  my_global_state->state = DieAtState::kWriting;
  errno = saved_errno;

#if defined(__x86_64__)
  __asm__ __volatile__("int $3" ::: "memory", "cc");
#elif defined(__aarch64__)
  __asm__ __volatile__("brk #0" ::: "memory", "cc");
#else
#error Unhandled architecture
#endif
}

// A mutex lock is about to happen.  Mark the memory rw, and check to see if we
// should die.
void futex_before(void *address, bool) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  if (IsInLocklessQueueMemory(address)) {
    assert(my_global_state->state == DieAtState::kRunning);
    HandleWrite(address);
    ShmProtectOrDie(PROT_READ | PROT_WRITE);
    my_global_state->state = DieAtState::kWriting;
  }
}

// The SEGV handler has set a breakpoint 1 instruction in the future.  This
// clears it, marks memory readonly, and continues.
void trap_handler(int signal, siginfo_t *, void * /*context*/) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  const int saved_errno = errno;
  SIMPLE_ASSERT(signal == SIGTRAP, "wrong signal for SIGTRAP handler");

  my_global_state->state = DieAtState::kWriting;
  SIMPLE_ASSERT(my_global_state->state == DieAtState::kWriting,
                "bad state for SIGTRAP");
  ShmProtectOrDie(PROT_READ);
  my_global_state->state = DieAtState::kRunning;
  errno = saved_errno;
}

// We have a manual trap for mutexes.  Check to see if we were supposed to die
// on this write (the compare/exchange for the mutex), and mark the memory ro
// again.
void futex_after(void *address, bool) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  if (IsInLocklessQueueMemory(address)) {
    assert(my_global_state->state == DieAtState::kWriting);
    ShmProtectOrDie(PROT_READ);
    my_global_state->state = DieAtState::kRunning;
  }
}

// Installs the signal handler.
void InstallHandler(int signal, void (*handler)(int, siginfo_t *, void *),
                    struct sigaction *old_action) {
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  action.sa_sigaction = handler;
  // We don't do a full normal signal handler exit with ptrace, so SA_NODEFER is
  // necessary to keep our signal handler active.
  action.sa_flags = SA_RESTART | SA_SIGINFO | SA_NODEFER;
#ifdef AOS_SANITIZER_thread
  // Tsan messes with signal handlers to check for race conditions, and it
  // causes problems, so we have to work around it for SIGTRAP.
  if (signal == SIGTRAP) {
    typedef int (*SigactionType)(int, const struct sigaction *,
                                 struct sigaction *);
    SigactionType real_sigaction =
        reinterpret_cast<SigactionType>(dlsym(RTLD_NEXT, "sigaction"));
    if (sigaction == real_sigaction) {
      LOG(WARNING) << "failed to work around tsan signal handling weirdness";
    }
    PCHECK(real_sigaction(signal, &action, old_action) == 0);
    return;
  }
#endif
  PCHECK(sigaction(signal, &action, old_action) == 0);
}

// gtest only allows creating fatal failures in functions returning void...
// status is from wait(2).
void DetectFatalFailures(int status) {
  if (WIFEXITED(status)) {
    FAIL() << " child returned status "
           << ::std::to_string(WEXITSTATUS(status));
  } else if (WIFSIGNALED(status)) {
    FAIL() << " child exited because of signal "
           << aos_strsignal(WTERMSIG(status));
  } else {
    FAIL() << " child exited with status " << ::std::hex << status;
  }
}

// Returns true if it runs all the way through.
bool RunFunctionDieAt(::std::function<void(void *)> prepare,
                      ::std::function<void(void *)> function,
                      bool *test_failure, size_t die_at, bool prepare_in_child,
                      uintptr_t writable_offset, const WritesArray *writes_in,
                      WritesArray *writes_out) {
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  my_global_state->writes_in = writes_in;
  my_global_state->writes_out = writes_out;
  my_global_state->die_at = die_at;
  my_global_state->current_location = 0;
  my_global_state->state = DieAtState::kDisabled;

  if (!prepare_in_child) prepare(my_global_state->lockless_queue_memory);

  const pid_t pid = fork();
  PCHECK(pid != -1) << ": fork() failed";
  if (pid == 0) {
    // Run the test.
    ::aos::testing::PreventExit();

    if (prepare_in_child) prepare(my_global_state->lockless_queue_memory);

    // Update the robust list offset.
    linux_code::ipc_lib::SetRobustListOffset(writable_offset);
    // Install a segv handler (to detect writes to the memory block), and a trap
    // handler so we can single step.
    InstallHandler(SIGSEGV, segv_handler, &old_segv_handler);
    InstallHandler(SIGTRAP, trap_handler, &old_trap_handler);
    CHECK_EQ(old_trap_handler.sa_handler, SIG_DFL);
    linux_code::ipc_lib::SetShmAccessorObservers(futex_before, futex_after);

    PCHECK(ptrace(PTRACE_TRACEME, 0, 0, 0) == 0);
    ShmProtectOrDie(PROT_READ);
    my_global_state->state = DieAtState::kRunning;

    function(my_global_state->lockless_queue_memory);
    my_global_state->state = DieAtState::kDisabled;
    ShmProtectOrDie(PROT_READ | PROT_WRITE);
    _exit(0);
  } else {
    // Annoying wrapper type because elf_gregset_t is an array, which C++
    // handles poorly.
    struct RestoreState {
      RestoreState(elf_gregset_t regs_in) {
        memcpy(regs, regs_in, sizeof(regs));
      }
      elf_gregset_t regs;
    };
    std::optional<RestoreState> restore_regs;
    bool pass_trap = false;
    // Wait until the child process dies.
    while (true) {
      int status;
      pid_t waited_on = waitpid(pid, &status, 0);
      if (waited_on == -1) {
        if (errno == EINTR) continue;
        PCHECK(false) << ": waitpid(" << pid << ", " << &status
                      << ", 0) failed";
      }
      CHECK_EQ(waited_on, pid)
          << ": waitpid got child " << waited_on << " instead of " << pid;
      if (WIFSTOPPED(status)) {
        // The child was stopped via ptrace.
        const int stop_signal = WSTOPSIG(status);
        elf_gregset_t regs;
        {
          struct iovec iov;
          iov.iov_base = &regs;
          iov.iov_len = sizeof(regs);
          PCHECK(ptrace(PTRACE_GETREGSET, pid, NT_PRSTATUS, &iov) == 0);
          CHECK_EQ(iov.iov_len, sizeof(regs))
              << ": ptrace regset is the wrong size";
        }
        if (stop_signal == SIGSEGV) {
          // It's a SEGV, hopefully due to writing to the shared memory which is
          // marked read-only. We record the instruction that faulted so we can
          // look for it while single-stepping, then deliver the signal so the
          // child can mark it read-write and then poke us to single-step that
          // instruction.

          CHECK(!restore_regs)
              << ": Traced child got a SEGV while single-stepping";
          // Save all the registers to resume execution at the current location
          // in the child.
          restore_regs = RestoreState(regs);
          PCHECK(ptrace(PTRACE_CONT, pid, nullptr, SIGSEGV) == 0);
          continue;
        }
        if (stop_signal == SIGTRAP) {
          if (pass_trap) {
            // This is the new SIGTRAP we generated, which we just want to pass
            // through so the child's signal handler can restore the memory to
            // read-only
            PCHECK(ptrace(PTRACE_CONT, pid, nullptr, SIGTRAP) == 0);
            pass_trap = false;
            continue;
          }
          if (restore_regs) {
            // Restore the state we saved before delivering the SEGV, and then
            // single-step that one instruction.
            struct iovec iov;
            iov.iov_base = &restore_regs->regs;
            iov.iov_len = sizeof(restore_regs->regs);
            PCHECK(ptrace(PTRACE_SETREGSET, pid, NT_PRSTATUS, &iov) == 0);
            restore_regs = std::nullopt;
            PCHECK(ptrace(PTRACE_SINGLESTEP, pid, nullptr, nullptr) == 0);
            continue;
          }
          // We executed the single instruction that originally faulted, so
          // now deliver a SIGTRAP to the child so it can mark the memory
          // read-only again.
          pass_trap = true;
          PCHECK(kill(pid, SIGTRAP) == 0);
          PCHECK(ptrace(PTRACE_CONT, pid, nullptr, nullptr) == 0);
          continue;
        }
        LOG(FATAL) << "Traced child was stopped with unexpected signal: "
                   << static_cast<int>(WSTOPSIG(status));
      }
      if (WIFEXITED(status)) {
        if (WEXITSTATUS(status) == 0) return true;
        if (WEXITSTATUS(status) == kExitEarlyValue) return false;
      }
      DetectFatalFailures(status);
      if (test_failure) *test_failure = true;
      return false;
    }
  }
}

bool RunFunctionDieAtAndCheck(const LocklessQueueConfiguration &config,
                              ::std::function<void(void *)> prepare,
                              ::std::function<void(void *)> function,
                              ::std::function<void(void *)> check,
                              bool *test_failure, size_t die_at,
                              bool prepare_in_child,
                              const WritesArray *writes_in,
                              WritesArray *writes_out) {
  // Allocate shared memory.
  GlobalState *my_global_state = global_state.load(::std::memory_order_relaxed);
  my_global_state->lockless_queue_memory_size = LocklessQueueMemorySize(config);
  my_global_state->lockless_queue_memory = static_cast<void *>(
      mmap(nullptr, my_global_state->lockless_queue_memory_size,
           PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, my_global_state->lockless_queue_memory);

  // And the backup used to point the robust list at.
  my_global_state->lockless_queue_memory_lock_backup = static_cast<void *>(
      mmap(nullptr, my_global_state->lockless_queue_memory_size,
           PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, my_global_state->lockless_queue_memory_lock_backup);

  // The writable offset tells us how to convert from a pointer in the queue to
  // a pointer that is safe to write.  This is so robust futexes don't spin the
  // kernel when the user maps a page PROT_READ, and the kernel tries to clear
  // the futex there.
  const uintptr_t writable_offset =
      reinterpret_cast<uintptr_t>(
          my_global_state->lockless_queue_memory_lock_backup) -
      reinterpret_cast<uintptr_t>(my_global_state->lockless_queue_memory);

  bool r;
  // Do the actual test in a new thread so any locked mutexes will be cleaned up
  // nicely with owner-died at the end.
  ::std::thread test_thread([&prepare, &function, &check, test_failure, die_at,
                             prepare_in_child, writes_in, writes_out,
                             writable_offset, &r]() {
    r = RunFunctionDieAt(prepare, function, test_failure, die_at,
                         prepare_in_child, writable_offset, writes_in,
                         writes_out);
    if (::testing::Test::HasFailure()) {
      r = false;
      if (test_failure) *test_failure = true;
      return;
    }

    check(
        global_state.load(::std::memory_order_relaxed)->lockless_queue_memory);
  });
  test_thread.join();
  return r;
}

// Tests function to make sure it handles dying after each store it makes to
// shared memory. check should make sure function behaved correctly.
// This will repeatedly create a new TestSharedMemory, run prepare, run
// function, and then
// run check, killing the process function is running in at various points. It
// will stop if anything reports a fatal gtest failure.
//
// prepare_in_child being true means the prepare function will be run in the
// child instead of the parent which doesn't die. This means that reference
// counts on any objects it allocates won't be duplicated, but it also means
// that any variables it sets will not be visible in check etc.
void TestShmRobustness(const LocklessQueueConfiguration &config,
                       ::std::function<void(void *)> prepare,
                       ::std::function<void(void *)> function,
                       ::std::function<void(void *)> check,
                       bool prepare_in_child) {
  // Map the global state and memory for the Writes array so it exists across
  // the process boundary.
  void *shared_allocations = static_cast<GlobalState *>(
      mmap(nullptr, sizeof(GlobalState) + sizeof(WritesArray),
           PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, shared_allocations);

  global_state.store(static_cast<GlobalState *>(shared_allocations));
  shared_allocations = static_cast<void *>(
      static_cast<uint8_t *>(shared_allocations) + sizeof(GlobalState));
  WritesArray *expected_writes = static_cast<WritesArray *>(shared_allocations);
  new (expected_writes) WritesArray();

  bool test_failed = false;
  ASSERT_TRUE(RunFunctionDieAtAndCheck(config, prepare, function, check,
                                       &test_failed, 0, prepare_in_child,
                                       nullptr, expected_writes));
  if (test_failed) {
    ADD_FAILURE();
    return;
  }

  size_t die_at = 1;
  while (true) {
    SCOPED_TRACE("dying at " + ::std::to_string(die_at) + "/" +
                 ::std::to_string(expected_writes->size()));
    if (RunFunctionDieAtAndCheck(config, prepare, function, check, &test_failed,
                                 die_at, prepare_in_child, expected_writes,
                                 nullptr)) {
      LOG(INFO) << "Tested " << die_at << " death points";
      return;
    }
    if (test_failed) {
      ADD_FAILURE();
    }
    if (::testing::Test::HasFailure()) return;
    ++die_at;
  }
}

namespace {
pid_t gettid() { return syscall(SYS_gettid); }

// Sets FUTEX_OWNER_DIED if the owner was tid.  This fakes what the kernel does
// with a robust mutex.
bool PretendOwnerDied(aos_mutex *mutex, pid_t tid) {
  if ((mutex->futex & FUTEX_TID_MASK) == tid) {
    mutex->futex = FUTEX_OWNER_DIED;
    return true;
  }
  return false;
}

static int kPinnedMessageIndex = 0;

constexpr monotonic_clock::duration kChannelStorageDuration =
    std::chrono::milliseconds(500);

}  // namespace

// Tests that death during sends is recovered from correctly.
TEST(LocklessQueueTest, Death) {
  ::aos::testing::EnableTestLogging();
  // Capture the tid in the child so we can tell if it died.  Use mmap so it
  // works across the process boundary.
  pid_t *tid =
      static_cast<pid_t *>(mmap(nullptr, sizeof(pid_t), PROT_READ | PROT_WRITE,
                                MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, tid);

  // Make a small queue so it is easier to debug.
  LocklessQueueConfiguration config;
  config.num_watchers = 2;
  config.num_senders = 2;
  config.num_pinners = 1;
  config.queue_size = 10;
  config.message_data_size = 32;

  TestShmRobustness(
      config,
      [config, tid](void *memory) {
        // Initialize the queue and grab the tid.
        LocklessQueue(
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            config)
            .Initialize();
        *tid = gettid();
      },
      [config](void *memory) {
        LocklessQueue queue(
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            config);
        // Now try to write some messages.  We will get killed a bunch as this
        // tries to happen.
        LocklessQueueSender sender =
            LocklessQueueSender::Make(queue, kChannelStorageDuration).value();
        LocklessQueuePinner pinner = LocklessQueuePinner::Make(queue).value();
        for (int i = 0; i < 5; ++i) {
          char data[100];
          size_t s = snprintf(data, sizeof(data), "foobar%d", i + 1);
          ASSERT_EQ(sender.Send(data, s + 1, monotonic_clock::min_time,
                                realtime_clock::min_time, 0xffffffffl,
                                UUID::Zero(), nullptr, nullptr, nullptr),
                    LocklessQueueSender::Result::GOOD);
          // Pin a message, so when we keep writing we will exercise the pinning
          // logic.
          if (i == 1) {
            CHECK_EQ(pinner.PinIndex(1), kPinnedMessageIndex);
          }
        }
      },
      [config, tid](void *raw_memory) {
        ::aos::ipc_lib::LocklessQueueMemory *const memory =
            reinterpret_cast<::aos::ipc_lib::LocklessQueueMemory *>(raw_memory);
        // Confirm that we can create 2 senders (the number in the queue), and
        // send a message.  And that all the messages in the queue are valid.
        LocklessQueue queue(memory, memory, config);

        bool print = false;

        // TestShmRobustness doesn't handle robust futexes.  It is happy to just
        // not crash with them.  We know what they are, and what the tid of the
        // holder is.  So go pretend to be the kernel and fix it for it.
        PretendOwnerDied(&memory->queue_setup_lock, *tid);

        for (size_t i = 0; i < config.num_senders; ++i) {
          if (PretendOwnerDied(&memory->GetSender(i)->tid, *tid)) {
            // Print out before and after results if a sender died.  That is the
            // more fun case.
            print = true;
          }
        }

        if (print) {
          LOG(INFO) << "Bad version:";
          PrintLocklessQueueMemory(memory);
        }

        // Building and destroying a sender will clean up the queue.
        LocklessQueueSender::Make(queue, kChannelStorageDuration).value();

        if (print) {
          LOG(INFO) << "Cleaned up version:";
          PrintLocklessQueueMemory(memory);
        }

        LocklessQueueReader reader(queue);

        // Verify that the pinned message still has its contents. Note that we
        // need to do this _before_ sending more messages, because the pinner
        // has been cleaned up.
        {
          const Message *const message =
              memory->GetMessage(Index(1, kPinnedMessageIndex));
          const auto queue_index =
              message->header.queue_index.Load(memory->queue_size());
          if (queue_index.valid()) {
            const char *const data = message->data(memory->message_data_size());
            EXPECT_EQ(data[LocklessQueueMessageDataSize(memory) -
                           message->header.length + 6],
                      '2');
          }
        }

        {
          LocklessQueueSender sender =
              LocklessQueueSender::Make(queue, kChannelStorageDuration).value();
          {
            // Make a second sender to confirm that the slot was freed.
            // If the sender doesn't get cleaned up, this will fail.
            LocklessQueueSender::Make(queue, kChannelStorageDuration).value();
          }

          // Send a message to make sure that the queue still works.
          char data[100];
          size_t s = snprintf(data, sizeof(data), "foobar%d", 971);
          ASSERT_EQ(sender.Send(data, s + 1, monotonic_clock::min_time,
                                realtime_clock::min_time, 0xffffffffl,
                                UUID::Zero(), nullptr, nullptr, nullptr),
                    LocklessQueueSender::Result::GOOD);
        }

        // Now loop through the queue and make sure the number in the snprintf
        // increments.
        char last_data = '0';
        int i = 0;
        while (true) {
          monotonic_clock::time_point monotonic_sent_time;
          realtime_clock::time_point realtime_sent_time;
          monotonic_clock::time_point monotonic_remote_time;
          realtime_clock::time_point realtime_remote_time;
          uint32_t remote_queue_index;
          UUID source_boot_uuid;
          char read_data[1024];
          size_t length;

          LocklessQueueReader::Result read_result = reader.Read(
              i, &monotonic_sent_time, &realtime_sent_time,
              &monotonic_remote_time, &realtime_remote_time,
              &remote_queue_index, &source_boot_uuid, &length, &(read_data[0]));

          if (read_result != LocklessQueueReader::Result::GOOD) {
            if (read_result == LocklessQueueReader::Result::TOO_OLD) {
              ++i;
              continue;
            }
            CHECK(read_result == LocklessQueueReader::Result::NOTHING_NEW)
                << ": " << static_cast<int>(read_result);
            break;
          }

          EXPECT_GT(
              read_data[LocklessQueueMessageDataSize(memory) - length + 6],
              last_data)
              << ": Got " << read_data;
          last_data =
              read_data[LocklessQueueMessageDataSize(memory) - length + 6];

          ++i;
        }

        // Confirm our message got through.
        EXPECT_EQ(last_data, '9') << ": Got through " << i;
      },
      /* prepare_in_child = true */ true);
}

#endif

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos
