#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#include <ostream>
#include <memory>
#include <map>

#include "gtest/gtest.h"

#include "aos/atom_code/ipc_lib/sharedmem_test_setup.h"
#include "aos/common/type_traits.h"

using testing::AssertionResult;
using testing::AssertionSuccess;
using testing::AssertionFailure;

// IMPORTANT: Some of the functions that do test predicate functions allocate
// shared memory (and don't free it).
class QueueTest : public SharedMemTestSetup {
 protected:
  static const size_t kFailureSize = 400;
  static char *fatal_failure;
 private:
  // This gets registered right after the fork, so it will get run before any
  // exit handlers that had already been registered.
  static void ExitExitHandler() {
    _exit(EXIT_SUCCESS);
  }
  enum class ResultType : uint8_t {
    NotCalled,
    Called,
    Returned,
  };
  const std::string ResultTypeString(volatile const ResultType &result) {
    switch (result) {
      case ResultType::Returned:
        return "Returned";
      case ResultType::Called:
        return "Called";
      case ResultType::NotCalled:
        return "NotCalled";
      default:
        return std::string("unknown(" + static_cast<uint8_t>(result)) + ")";
    }
  }
  static_assert(aos::shm_ok<ResultType>::value, "this will get put in shared memory");
  // Gets allocated in shared memory so it has to be volatile.
  template<typename T> struct FunctionToCall {
    ResultType result;
    bool expected;
    void (*function)(T*, char*);
    T *arg;
    volatile char failure[kFailureSize];
  };
  template<typename T> static void Hangs_(volatile FunctionToCall<T> *const to_call) {
    to_call->result = ResultType::Called;
    to_call->function(to_call->arg, const_cast<char *>(to_call->failure));
    to_call->result = ResultType::Returned;
  }

  static const long kMsToNs = 1000000;
  // The number of ms after which a function is considered to have hung.
  // Must be < 1000.
  static const long kHangTime = 10;
  static const unsigned int kForkSleep = 0; // how many seconds to sleep after forking

  // Represents a process that has been forked off. The destructor kills the
  // process and wait(2)s for it.
  class ForkedProcess {
    const pid_t pid_;
    mutex *const lock_;
   public:
    ForkedProcess(pid_t pid, mutex *lock) : pid_(pid), lock_(lock) {};
    ~ForkedProcess() {
      if (kill(pid_, SIGINT) == -1) {
        if (errno == ESRCH) {
          printf("process %jd was already dead\n", static_cast<intmax_t>(pid_));
        } else {
          fprintf(stderr, "kill(SIGKILL, %jd) failed with %d: %s\n",
                  static_cast<intmax_t>(pid_), errno, strerror(errno));
        }
        return;
      }
      const pid_t ret = wait(NULL);
      if (ret == -1) {
        fprintf(stderr, "wait(NULL) failed."
                " child %jd might still be alive\n",
                static_cast<intmax_t>(pid_));
      } else if (ret == 0) {
        fprintf(stderr, "child %jd wasn't waitable. it might still be alive\n",
                static_cast<intmax_t>(pid_));
      } else if (ret != pid_) {
        fprintf(stderr, "child %d is dead, but child %jd might still be alive\n",
               ret, static_cast<intmax_t>(pid_));
      }
    }

    enum class JoinResult {
      Finished, Hung, Error
    };
    JoinResult Join(long timeout = kHangTime) {
      timespec ts{kForkSleep, timeout * kMsToNs};
      switch (mutex_lock_timeout(lock_, &ts)) {
        case 2:
          return JoinResult::Hung;
        case 0:
          return JoinResult::Finished;
        default:
          return JoinResult::Error;
      }
    }
  } __attribute__((unused));

  // Member variables for HangsFork and HangsCheck.
  typedef uint8_t ChildID;
  static void ReapExitHandler() {
    for (auto it = children_.begin(); it != children_.end(); ++it) {
      delete it->second;
    }
  }
  static std::map<ChildID, ForkedProcess *> children_;
  std::map<ChildID, volatile FunctionToCall<void> *> to_calls_;

  void SetUp() {
    SharedMemTestSetup::SetUp();
    fatal_failure = reinterpret_cast<char *>(shm_malloc(sizeof(fatal_failure)));
    static bool registered = false;
    if (!registered) {
      atexit(ReapExitHandler);
      registered = true;
    }
  }

 protected:
  // Function gets called with arg in a forked process.
  // Leaks shared memory.
  // the attribute is in the middle to make gcc happy
  template<typename T> __attribute__((warn_unused_result))
      std::unique_ptr<ForkedProcess> ForkExecute(void (*function)(T*), T *arg) {
    mutex *lock = reinterpret_cast<mutex *>(shm_malloc_aligned(
            sizeof(*lock), sizeof(int)));
    *lock = 1;
    const pid_t pid = fork();
    switch (pid) {
      case 0: // child
        if (kForkSleep != 0) {
          printf("pid %jd sleeping for %u\n", static_cast<intmax_t>(getpid()),
                 kForkSleep);
          sleep(kForkSleep);
        }
        atexit(ExitExitHandler);
        function(arg);
        mutex_unlock(lock);
        exit(EXIT_SUCCESS);
      case -1: // parent failure
        printf("fork() failed with %d: %s\n", errno, strerror(errno));
        return std::unique_ptr<ForkedProcess>();
      default: // parent
        return std::unique_ptr<ForkedProcess>(new ForkedProcess(pid, lock));
    }
  }

  // Checks whether or not the given function hangs.
  // expected is whether to return success or failure if the function hangs
  // NOTE: There are other reasons for it to return a failure than the function
  // doing the wrong thing.
  // Leaks shared memory.
  template<typename T> AssertionResult Hangs(void (*function)(T*, char*), T *arg,
                                             bool expected) {
    AssertionResult fork_result(HangsFork<T>(function, arg, expected, 0));
    if (!fork_result) {
      return fork_result;
    }
    return HangsCheck(0);
  }
  // Starts the first part of Hangs.
  // Use HangsCheck to get the result.
  // Returns whether the fork succeeded or not, NOT whether or not the hang
  // check succeeded.
  template<typename T> AssertionResult HangsFork(void (*function)(T*, char *), T *arg,
                                                 bool expected, ChildID id) {
    static_assert(aos::shm_ok<FunctionToCall<T>>::value,
                  "this is going into shared memory");
    volatile FunctionToCall<T> *const to_call = reinterpret_cast<FunctionToCall<T> *>(
        shm_malloc_aligned(sizeof(*to_call), sizeof(int)));
    to_call->result = ResultType::NotCalled;
    to_call->function = function;
    to_call->arg = arg;
    to_call->expected = expected;
    to_call->failure[0] = '\0';
    static_cast<volatile char *>(fatal_failure)[0] = '\0';
    children_[id] = ForkExecute(Hangs_, to_call).release();
    if (!children_[id]) return AssertionFailure() << "ForkExecute failed";
    to_calls_[id] = reinterpret_cast<volatile FunctionToCall<void> *>(to_call);
    return AssertionSuccess();
  }
  // Checks whether or not a function hung like it was supposed to.
  // Use HangsFork first.
  // NOTE: calls to HangsFork and HangsCheck with the same id argument will
  // correspond, but they do not nest. Also, id 0 is used by Hangs.
  // Return value is the same as Hangs.
  AssertionResult HangsCheck(ChildID id) {
    std::unique_ptr<ForkedProcess> child(children_[id]);
    children_.erase(id);
    const ForkedProcess::JoinResult result = child->Join();
    if (to_calls_[id]->failure[0] != '\0') {
      return AssertionFailure() << "function says: "
          << const_cast<char *>(to_calls_[id]->failure);
    }
    if (result == ForkedProcess::JoinResult::Finished) {
      return !to_calls_[id]->expected ? AssertionSuccess() : (AssertionFailure()
          << "something happened and the the test only got to "
          << ResultTypeString(to_calls_[id]->result));
    } else {
      if (to_calls_[id]->result == ResultType::Called) {
        return to_calls_[id]->expected ? AssertionSuccess() : AssertionFailure();
      } else {
        return AssertionFailure() << "something weird happened";
      }
    }
  }
#define EXPECT_HANGS(function, arg) \
  EXPECT_HANGS_COND(function, arg, true, EXPECT_TRUE)
#define EXPECT_RETURNS(function, arg) \
  EXPECT_HANGS_COND(function, arg, false, EXPECT_TRUE)
#define EXPECT_RETURNS_FAILS(function, arg) \
  EXPECT_HANGS_COND(function, arg, false, EXPECT_FALSE)
#define EXPECT_HANGS_COND(function, arg, hangs, cond) do { \
  cond(Hangs(function, arg, hangs)); \
  if (fatal_failure[0] != '\0') { \
    FAIL() << fatal_failure; \
  } \
} while (false)

  struct TestMessage {
    int16_t data; // don't really want to test empty messages
  };
  struct MessageArgs {
    aos_queue *const queue;
    int flags;
    int16_t data; // -1 means NULL expected
  };
  static void WriteTestMessage(MessageArgs *args, char *failure) {
    TestMessage *msg = reinterpret_cast<TestMessage *>(aos_queue_get_msg(args->queue));
    if (msg == NULL) {
      snprintf(fatal_failure, kFailureSize, "couldn't get_msg from %p", args->queue);
      return;
    }
    msg->data = args->data;
    if (aos_queue_write_msg_free(args->queue, msg, args->flags) == -1) {
      snprintf(failure, kFailureSize, "write_msg_free(%p, %p, %d) failed",
               args->queue, msg, args->flags);
    }
  }
  static void ReadTestMessage(MessageArgs *args, char *failure) {
    const TestMessage *msg = reinterpret_cast<const TestMessage *>(
        aos_queue_read_msg(args->queue, args->flags));
    if (msg == NULL) {
      if (args->data != -1) {
        snprintf(failure, kFailureSize,
                 "expected data of %" PRId16 " but got NULL message",
                 args->data);
      }
    } else {
      if (args->data != msg->data) {
        snprintf(failure, kFailureSize,
                 "expected data of %" PRId16 " but got %" PRId16 " instead",
                 args->data, msg->data);
      }
      aos_queue_free_msg(args->queue, msg);
    }
  }
};
char *QueueTest::fatal_failure;
std::map<QueueTest::ChildID, QueueTest::ForkedProcess *> QueueTest::children_;

TEST_F(QueueTest, Reading) {
  static const aos_type_sig signature{sizeof(TestMessage), 1, 1};
  aos_queue *const queue = aos_fetch_queue("Queue", &signature);
  MessageArgs args{queue, 0, -1};

  EXPECT_EQ(BLOCK, 0);
  EXPECT_EQ(BLOCK | FROM_END, FROM_END);

  args.flags = NON_BLOCK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = NON_BLOCK | PEEK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = PEEK;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.data = 254;
  args.flags = BLOCK;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = PEEK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = PEEK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = PEEK | NON_BLOCK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  args.data = -1;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = NON_BLOCK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  args.data = 971;
  EXPECT_RETURNS_FAILS(ReadTestMessage, &args);
}
TEST_F(QueueTest, Writing) {
  static const aos_type_sig signature{sizeof(TestMessage), 1, 1};
  aos_queue *const queue = aos_fetch_queue("Queue", &signature);
  MessageArgs args{queue, 0, 973};

  args.flags = BLOCK;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = BLOCK;
  EXPECT_HANGS(WriteTestMessage, &args);
  args.flags = NON_BLOCK;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = NON_BLOCK;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = PEEK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.data = 971;
  args.flags = OVERRIDE;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = OVERRIDE;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = NON_BLOCK;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = OVERRIDE;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
}

TEST_F(QueueTest, MultiRead) {
  static const aos_type_sig signature{sizeof(TestMessage), 1, 1};
  aos_queue *const queue = aos_fetch_queue("Queue", &signature);
  MessageArgs args{queue, 0, 1323};

  args.flags = BLOCK;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = BLOCK;
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 1));
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 2));
  EXPECT_TRUE(HangsCheck(1) != HangsCheck(2));
  // TODO(brians) finish this
}

TEST_F(QueueTest, Recycle) {
  // TODO(brians) basic test of recycle queue
  // include all of the ways a message can get into the recycle queue
  static const aos_type_sig signature{sizeof(TestMessage), 1, 2},
               recycle_signature{sizeof(TestMessage), 2, 2};
  aos_queue *recycle_queue = reinterpret_cast<aos_queue *>(23);
  aos_queue *const queue = aos_fetch_queue_recycle("Queue", &signature,
                                                   &recycle_signature, &recycle_queue);
  ASSERT_NE(reinterpret_cast<aos_queue *>(23), recycle_queue);
  MessageArgs args{queue, 0, 973}, recycle{recycle_queue, 0, 973};

  args.flags = BLOCK;
  EXPECT_RETURNS(WriteTestMessage, &args);
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.data = 254;
  EXPECT_RETURNS(WriteTestMessage, &args);
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.data = 971;
  args.flags = OVERRIDE;
  EXPECT_RETURNS(WriteTestMessage, &args);
  recycle.flags = BLOCK;
  EXPECT_RETURNS(ReadTestMessage, &recycle);

  EXPECT_HANGS(ReadTestMessage, &recycle);

  TestMessage *msg = static_cast<TestMessage *>(aos_queue_get_msg(queue));
  ASSERT_TRUE(msg != NULL);
  msg->data = 341;
  aos_queue_free_msg(queue, msg);
  recycle.data = 341;
  EXPECT_RETURNS(ReadTestMessage, &recycle);

  EXPECT_HANGS(ReadTestMessage, &recycle);

  args.data = 254;
  args.flags = PEEK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  recycle.flags = BLOCK;
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.flags = BLOCK;
  EXPECT_RETURNS(ReadTestMessage, &args);
  recycle.data = 254;
  EXPECT_RETURNS(ReadTestMessage, &recycle);
}

