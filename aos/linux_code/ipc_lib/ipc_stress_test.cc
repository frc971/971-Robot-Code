#include <errno.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <string>

#include "aos/common/die.h"
#include "aos/common/libc/aos_strsignal.h"
#include "aos/common/libc/dirname.h"
#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"
#include "aos/common/time.h"
#include "aos/common/type_traits.h"
#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/testing/test_shm.h"

// This runs all of the IPC-related tests in a bunch of parallel processes for a
// while and makes sure that they don't fail. It also captures the stdout and
// stderr output from each test run and only prints it out (not interleaved with
// the output from any other run) if the test fails.
//
// They have to be run in separate processes because (in addition to various
// parts of our code not being thread-safe...) gtest does not like multiple
// threads.
//
// It's written in C++ for performance. We need actual OS-level parallelism for
// this to work, which means that Ruby's out because it doesn't have good
// support for doing that. My Python implementation ended up pretty heavily disk
// IO-bound, which is a bad way to test CPU contention.

namespace aos {

namespace chrono = ::std::chrono;

// Each test is represented by the name of the test binary and then any
// arguments to pass to it.
// Using --gtest_filter is a bad idea because it seems to result in a lot of
// swapping which causes everything to be disk-bound (at least for me).
static const size_t kTestMaxArgs = 10;
static const char * kTests[][kTestMaxArgs] = {
  {"queue_test"},
  {"condition_test"},
  {"mutex_test"},
  {"raw_queue_test"},
};
static const size_t kTestsLength = sizeof(kTests) / sizeof(kTests[0]);
// These arguments get inserted before any per-test arguments.
static const char *kDefaultArgs[] = {
  "--gtest_repeat=30",
  "--gtest_shuffle",
};

// How many test processes to run at a time.
static const int kTesters = 100;
// How long to test for.
static constexpr monotonic_clock::duration kTestTime = chrono::seconds(30);

// The structure that gets put into shared memory and then referenced by all of
// the child processes.
struct Shared {
  Shared(const monotonic_clock::time_point stop_time)
    : stop_time(stop_time), total_iterations(0) {}

  // Synchronizes access to stdout/stderr to avoid interleaving failure
  // messages.
  Mutex output_mutex;

  // When to stop.
  monotonic_clock::time_point stop_time;

  // The total number of iterations. Updated by each child as it finishes.
  int total_iterations;
  // Sychronizes writes to total_iterations
  Mutex total_iterations_mutex;

  const char *path;
};
static_assert(shm_ok<Shared>::value,
              "it's going to get shared between forked processes");

// Gets called after each child forks to run a test.
void __attribute__((noreturn)) DoRunTest(
    Shared *shared, const char *(*test)[kTestMaxArgs], int pipes[2]) {
  if (close(pipes[0]) == -1) {
    PDie("close(%d) of read end of pipe failed", pipes[0]);
  }
  if (close(STDIN_FILENO) == -1) {
    PDie("close(STDIN_FILENO(=%d)) failed", STDIN_FILENO);
  }
  if (dup2(pipes[1], STDOUT_FILENO) == -1) {
    PDie("dup2(%d, STDOUT_FILENO(=%d)) failed", pipes[1], STDOUT_FILENO);
  }
  if (dup2(pipes[1], STDERR_FILENO) == -1) {
    PDie("dup2(%d, STDERR_FILENO(=%d)) failed", pipes[1], STDERR_FILENO);
  }

  size_t size = kTestMaxArgs;
  size_t default_size = sizeof(kDefaultArgs) / sizeof(kDefaultArgs[0]);
  // There's no chance to free this because we either exec or Die.
  const char **args = new const char *[size + default_size + 1];
  // The actual executable to run.
  ::std::string executable;
  int i = 0;
  for (size_t test_i = 0; test_i < size; ++test_i) {
    const char *c = (*test)[test_i];
    if (i == 0) {
      executable = ::std::string(shared->path) + "/" + c;
      args[0] = executable.c_str();
      for (const ::std::string &ci : kDefaultArgs) {
        args[++i] = ci.c_str();
      }
    } else {
      args[i] = c;
    }
    ++i;
  }
  args[size] = NULL;
  execv(executable.c_str(), const_cast<char *const *>(args));
  PDie("execv(%s, %p) failed", executable.c_str(), args);
}

void DoRun(Shared *shared) {
  int iterations = 0;
  // An iterator pointing to a random one of the tests.
  // We randomize based on PID because otherwise they all end up running the
  // same test at the same time for the whole test.
  const char *(*test)[kTestMaxArgs] = &kTests[getpid() % kTestsLength];
  int pipes[2];
  while (monotonic_clock::now() < shared->stop_time) {
    if (pipe(pipes) == -1) {
      PDie("pipe(%p) failed", &pipes);
    }
    switch (fork()) {
      case 0:  // in runner
        DoRunTest(shared, test, pipes);
      case -1:
        PDie("fork() failed");
    }

    if (close(pipes[1]) == -1) {
      PDie("close(%d) of write end of pipe failed", pipes[1]);
    }

    ::std::string output;
    char buffer[2048];
    while (true) {
      ssize_t ret = read(pipes[0], &buffer, sizeof(buffer));
      if (ret == 0) {  // EOF
        if (close(pipes[0]) == -1) {
          PDie("close(%d) of pipe at EOF failed", pipes[0]);
        }
        break;
      } else if (ret == -1) {
        PDie("read(%d, %p, %zd) failed", pipes[0], &buffer, sizeof(buffer));
      }
      output += ::std::string(buffer, ret);
    }

    int status;
    while (true) {
      if (wait(&status) == -1) {
        if (errno == EINTR) continue;
        PDie("wait(%p) in child failed", &status);
      } else {
        break;
      }
    }
    if (WIFEXITED(status)) {
      if (WEXITSTATUS(status) != 0) {
        MutexLocker sync(&shared->output_mutex);
        fprintf(stderr, "Test %s exited with status %d. output:\n",
                (*test)[0], WEXITSTATUS(status));
        fputs(output.c_str(), stderr);
      }
    } else if (WIFSIGNALED(status)) {
      MutexLocker sync(&shared->output_mutex);
      fprintf(stderr, "Test %s terminated by signal %d: %s.\n", (*test)[0],
              WTERMSIG(status), aos_strsignal(WTERMSIG(status)));
        fputs(output.c_str(), stderr);
    } else {
      CHECK(WIFSTOPPED(status));
      Die("Test %s was stopped.\n", (*test)[0]);
    }

    ++test;
    if (test == kTests + 1) test = kTests;
    ++iterations;
  }
  {
    MutexLocker sync(&shared->total_iterations_mutex);
    shared->total_iterations += iterations;
  }
}

void Run(Shared *shared) {
  switch (fork()) {
    case 0:  // in child
      DoRun(shared);
      _exit(EXIT_SUCCESS);
    case -1:
      PDie("fork() of child failed");
  }
}

int Main(int argc, char **argv) {
  if (argc < 1) {
    fputs("need an argument\n", stderr);
    return EXIT_FAILURE;
  }

  ::aos::testing::TestSharedMemory my_shm_;

  Shared *shared = static_cast<Shared *>(shm_malloc(sizeof(Shared)));
  new (shared) Shared(monotonic_clock::now() + kTestTime);

  if (asprintf(const_cast<char **>(&shared->path),
               "%s/../tests", ::aos::libc::Dirname(argv[0]).c_str()) == -1) {
    PDie("asprintf failed");
  }

  for (int i = 0; i < kTesters; ++i) {
    Run(shared);
  }

  bool error = false;
  for (int i = 0; i < kTesters; ++i) {
    int status;
    if (wait(&status) == -1) {
      if (errno == EINTR) {
        --i;
      } else {
        PDie("wait(%p) failed", &status);
      }
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      error = true;
    }
  }

  printf("Ran a total of %d tests.\n", shared->total_iterations);
  if (error) {
    printf("A child had a problem during the test.\n");
  }
  return error ? EXIT_FAILURE : EXIT_SUCCESS;
}

}  // namespace aos

int main(int argc, char **argv) {
  return ::aos::Main(argc, argv);
}
