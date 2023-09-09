#include "aos/ipc_lib/lockless_queue_stepping.h"

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

#include <memory>
#include <thread>

#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/libc/aos_strsignal.h"
#include "aos/testing/prevent_exit.h"

#ifdef SUPPORTS_SHM_ROBUSTNESS_TEST

namespace aos {
namespace ipc_lib {
namespace testing {

namespace {
pid_t gettid() { return syscall(SYS_gettid); }

::std::atomic<GlobalState *> global_state;

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
  GlobalState *my_global_state = GlobalState::Get();
  const int saved_errno = errno;
  SIMPLE_ASSERT(signal == SIGSEGV, "wrong signal for SIGSEGV handler");

  // Only process memory addresses in our shared memory block.
  if (!my_global_state->IsInLocklessQueueMemory(siginfo->si_addr)) {
    if (CallChainedAction(old_segv_handler, signal, siginfo, context_void)) {
      errno = saved_errno;
      return;
    } else {
      SIMPLE_ASSERT(false, "actual SIGSEGV");
    }
  }
  SIMPLE_ASSERT(my_global_state->state == DieAtState::kRunning,
                "bad state for SIGSEGV");

  my_global_state->HandleWrite(siginfo->si_addr);

  my_global_state->ShmProtectOrDie(PROT_READ | PROT_WRITE);
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

// The SEGV handler has set a breakpoint 1 instruction in the future.  This
// clears it, marks memory readonly, and continues.
void trap_handler(int signal, siginfo_t *, void * /*context*/) {
  GlobalState *my_global_state = GlobalState::Get();
  const int saved_errno = errno;
  SIMPLE_ASSERT(signal == SIGTRAP, "wrong signal for SIGTRAP handler");

  my_global_state->state = DieAtState::kWriting;
  SIMPLE_ASSERT(my_global_state->state == DieAtState::kWriting,
                "bad state for SIGTRAP");
  my_global_state->ShmProtectOrDie(PROT_READ);
  my_global_state->state = DieAtState::kRunning;
  errno = saved_errno;
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

// A mutex lock is about to happen.  Mark the memory rw, and check to see if we
// should die.
void futex_before(void *address, bool) {
  GlobalState *my_global_state = GlobalState::Get();
  if (my_global_state->IsInLocklessQueueMemory(address)) {
    assert(my_global_state->state == DieAtState::kRunning);
    my_global_state->HandleWrite(address);
    my_global_state->ShmProtectOrDie(PROT_READ | PROT_WRITE);
    my_global_state->state = DieAtState::kWriting;
  }
}

// We have a manual trap for mutexes.  Check to see if we were supposed to die
// on this write (the compare/exchange for the mutex), and mark the memory ro
// again.
void futex_after(void *address, bool) {
  GlobalState *my_global_state = GlobalState::Get();
  if (my_global_state->IsInLocklessQueueMemory(address)) {
    assert(my_global_state->state == DieAtState::kWriting);
    my_global_state->ShmProtectOrDie(PROT_READ);
    my_global_state->state = DieAtState::kRunning;
  }
}

}  // namespace

void GlobalState::HandleWrite(void *address) {
  uintptr_t address_offset = reinterpret_cast<uintptr_t>(address) -
                             reinterpret_cast<uintptr_t>(lockless_queue_memory);
  if (writes_in != nullptr) {
    SIMPLE_ASSERT(writes_in->At(current_location) == address_offset,
                  "wrong write order");
  }
  if (writes_out != nullptr) {
    writes_out->Add(address_offset);
  }
  if (die_at != 0) {
    if (die_at == current_location) {
      _exit(kExitEarlyValue);
    }
  }
  ++current_location;
}

GlobalState *GlobalState::Get() {
  return global_state.load(::std::memory_order_relaxed);
}

std::tuple<GlobalState *, WritesArray *> GlobalState::MakeGlobalState() {
  // Map the global state and memory for the Writes array so it exists across
  // the process boundary.
  void *shared_allocations = static_cast<GlobalState *>(
      mmap(nullptr, sizeof(GlobalState) + sizeof(WritesArray),
           PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, shared_allocations);

  global_state.store(static_cast<GlobalState *>(shared_allocations));
  void *expected_writes_shared_allocations = static_cast<void *>(
      static_cast<uint8_t *>(shared_allocations) + sizeof(GlobalState));
  WritesArray *expected_writes =
      static_cast<WritesArray *>(expected_writes_shared_allocations);
  new (expected_writes) WritesArray();
  return std::make_pair(static_cast<GlobalState *>(shared_allocations),
                        expected_writes);
}

bool GlobalState::IsInLocklessQueueMemory(void *address) {
  void *read_lockless_queue_memory = lockless_queue_memory;
  if (address < read_lockless_queue_memory) {
    return false;
    if (reinterpret_cast<uintptr_t>(address) >
        reinterpret_cast<uintptr_t>(read_lockless_queue_memory) +
            lockless_queue_memory_size)
      return false;
  }
  return true;
}

void GlobalState::ShmProtectOrDie(int prot) {
  PCHECK(mprotect(lockless_queue_memory, lockless_queue_memory_size, prot) !=
         -1)
      << ": mprotect(" << lockless_queue_memory << ", "
      << lockless_queue_memory_size << ", 0x" << std::hex << prot << ") failed";
}

void GlobalState::RegisterSegvAndTrapHandlers() {
  InstallHandler(SIGSEGV, segv_handler, &old_segv_handler);
  InstallHandler(SIGTRAP, trap_handler, &old_trap_handler);
  CHECK_EQ(old_trap_handler.sa_handler, SIG_DFL);
  linux_code::ipc_lib::SetShmAccessorObservers(futex_before, futex_after);
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
                      bool *test_failure, size_t die_at,
                      uintptr_t writable_offset, const WritesArray *writes_in,
                      WritesArray *writes_out) {
  GlobalState *my_global_state = GlobalState::Get();
  my_global_state->writes_in = writes_in;
  my_global_state->writes_out = writes_out;
  my_global_state->die_at = die_at;
  my_global_state->current_location = 0;
  my_global_state->state = DieAtState::kDisabled;

  const pid_t pid = fork();
  PCHECK(pid != -1) << ": fork() failed";
  if (pid == 0) {
    // Run the test.
    ::aos::testing::PreventExit();

    prepare(my_global_state->lockless_queue_memory);

    // Update the robust list offset.
    linux_code::ipc_lib::SetRobustListOffset(writable_offset);
    // Install a segv handler (to detect writes to the memory block), and a trap
    // handler so we can single step.
    my_global_state->RegisterSegvAndTrapHandlers();

    PCHECK(ptrace(PTRACE_TRACEME, 0, 0, 0) == 0);
    my_global_state->ShmProtectOrDie(PROT_READ);
    my_global_state->state = DieAtState::kRunning;

    function(my_global_state->lockless_queue_memory);
    my_global_state->state = DieAtState::kDisabled;
    my_global_state->ShmProtectOrDie(PROT_READ | PROT_WRITE);
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
                              const WritesArray *writes_in,
                              WritesArray *writes_out) {
  // Allocate shared memory.
  GlobalState *my_global_state = GlobalState::Get();
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
                             writes_in, writes_out, writable_offset, &r]() {
    r = RunFunctionDieAt(prepare, function, test_failure, die_at,
                         writable_offset, writes_in, writes_out);
    if (::testing::Test::HasFailure()) {
      r = false;
      if (test_failure) *test_failure = true;
      return;
    }

    check(GlobalState::Get()->lockless_queue_memory);
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
void TestShmRobustness(const LocklessQueueConfiguration &config,
                       ::std::function<void(void *)> prepare,
                       ::std::function<void(void *)> function,
                       ::std::function<void(void *)> check) {
  auto [my_global_state, expected_writes] = GlobalState::MakeGlobalState();

  bool test_failed = false;
  ASSERT_TRUE(RunFunctionDieAtAndCheck(config, prepare, function, check,
                                       &test_failed, 0, nullptr,
                                       expected_writes));
  if (test_failed) {
    ADD_FAILURE();
    return;
  }

  size_t die_at = 1;
  while (true) {
    SCOPED_TRACE("dying at " + ::std::to_string(die_at) + "/" +
                 ::std::to_string(expected_writes->size()));
    if (RunFunctionDieAtAndCheck(config, prepare, function, check, &test_failed,
                                 die_at, expected_writes, nullptr)) {
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

SharedTid::SharedTid() {
  // Capture the tid in the child so we can tell if it died.  Use mmap so it
  // works across the process boundary.
  tid_ =
      static_cast<pid_t *>(mmap(nullptr, sizeof(pid_t), PROT_READ | PROT_WRITE,
                                MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  CHECK_NE(MAP_FAILED, tid_);
}

SharedTid::~SharedTid() { CHECK_EQ(munmap(tid_, sizeof(pid_t)), 0); }

void SharedTid::Set() { *tid_ = gettid(); }

pid_t SharedTid::Get() { return *tid_; }

bool PretendOwnerDied(aos_mutex *mutex, pid_t tid) {
  if ((mutex->futex & FUTEX_TID_MASK) == tid) {
    mutex->futex = FUTEX_OWNER_DIED;
    return true;
  }
  return false;
}

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos

#endif  // SUPPORTS_SHM_ROBSTNESS_TEST
