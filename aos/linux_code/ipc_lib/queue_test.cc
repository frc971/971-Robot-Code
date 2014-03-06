#include "aos/common/queue.h"

#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#include <ostream>
#include <memory>
#include <map>

#include "gtest/gtest.h"

#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/common/type_traits.h"
#include "aos/common/queue_testutils.h"
#include "aos/common/time.h"
#include "aos/common/logging/logging.h"
#include "aos/common/die.h"

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;
using ::aos::common::testing::GlobalCoreInstance;

namespace aos {
namespace testing {

class QueueTest : public ::testing::Test {
 protected:
  static const size_t kFailureSize = 400;
  static char *fatal_failure;
 private:
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
  static_assert(aos::shm_ok<ResultType>::value,
                "this will get put in shared memory");
  template<typename T>
  struct FunctionToCall {
    FunctionToCall() : result(ResultType::NotCalled) {
      started.Lock();
    }

    volatile ResultType result;
    bool expected;
    void (*function)(T*, char*);
    T *arg;
    volatile char failure[kFailureSize];
    Mutex started;
  };
  template<typename T>
  static void Hangs_(FunctionToCall<T> *const to_call) {
    to_call->started.Unlock();
    to_call->result = ResultType::Called;
    to_call->function(to_call->arg, const_cast<char *>(to_call->failure));
    to_call->result = ResultType::Returned;
  }

  // How long until a function is considered to have hung.
  static constexpr time::Time kHangTime = time::Time::InSeconds(0.035);
  // How long to sleep after forking (for debugging).
  static constexpr time::Time kForkSleep = time::Time::InSeconds(0);

  // Represents a process that has been forked off. The destructor kills the
  // process and wait(2)s for it.
  class ForkedProcess {
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
        LOG(WARNING, "wait(NULL) failed."
            " child %jd might still be alive\n",
            static_cast<intmax_t>(pid_));
      } else if (ret == 0) {
        LOG(WARNING, "child %jd wasn't waitable. it might still be alive\n",
            static_cast<intmax_t>(pid_));
      } else if (ret != pid_) {
        LOG(WARNING, "child %d is now confirmed dead"
            ", but child %jd might still be alive\n",
            ret, static_cast<intmax_t>(pid_));
      }
    }

    enum class JoinResult {
      Finished, Hung, Error
    };
    JoinResult Join(time::Time timeout = kHangTime) {
      timespec lock_timeout = (kForkSleep + timeout).ToTimespec();
      switch (mutex_lock_timeout(lock_, &lock_timeout)) {
        case 2:
          return JoinResult::Hung;
        case 0:
          return JoinResult::Finished;
        default:
          return JoinResult::Error;
      }
    }

   private:
    const pid_t pid_;
    mutex *const lock_;
  } __attribute__((unused));

  // State for HangsFork and HangsCheck.
  typedef uint8_t ChildID;
  static void ReapExitHandler() {
    for (auto it = children_.begin(); it != children_.end(); ++it) {
      delete it->second;
    }
  }
  static std::map<ChildID, ForkedProcess *> children_;
  std::map<ChildID, FunctionToCall<void> *> to_calls_;

  void SetUp() override {
    ::testing::Test::SetUp();

    SetDieTestMode(true);

    fatal_failure = static_cast<char *>(shm_malloc(sizeof(fatal_failure)));
    static bool registered = false;
    if (!registered) {
      atexit(ReapExitHandler);
      registered = true;
    }
  }

 protected:
  // function gets called with arg in a forked process.
  // Leaks shared memory.
  template<typename T> __attribute__((warn_unused_result))
  std::unique_ptr<ForkedProcess> ForkExecute(void (*function)(T*), T *arg) {
    mutex *lock = static_cast<mutex *>(shm_malloc_aligned(
            sizeof(*lock), sizeof(int)));
    assert(mutex_lock(lock) == 0);
    const pid_t pid = fork();
    switch (pid) {
      case 0:  // child
        if (kForkSleep != time::Time(0, 0)) {
          LOG(INFO, "pid %jd sleeping for %ds%dns\n",
              static_cast<intmax_t>(getpid()),
              kForkSleep.sec(), kForkSleep.nsec());
          time::SleepFor(kForkSleep);
        }
        ::aos::common::testing::PreventExit();
        function(arg);
        mutex_unlock(lock);
        exit(EXIT_SUCCESS);
      case -1:  // parent failure
        LOG(ERROR, "fork() failed with %d: %s\n", errno, strerror(errno));
        return std::unique_ptr<ForkedProcess>();
      default:  // parent
        return std::unique_ptr<ForkedProcess>(new ForkedProcess(pid, lock));
    }
  }

  // Checks whether or not the given function hangs.
  // expected is whether to return success or failure if the function hangs
  // NOTE: There are other reasons for it to return a failure than the function
  // doing the wrong thing.
  // Leaks shared memory.
  template<typename T>
  AssertionResult Hangs(void (*function)(T*, char*), T *arg, bool expected) {
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
  template<typename T>
  AssertionResult HangsFork(void (*function)(T*, char *), T *arg,
                            bool expected, ChildID id) {
    static_assert(aos::shm_ok<FunctionToCall<T>>::value,
                  "this is going into shared memory");
    FunctionToCall<T> *const to_call =
        static_cast<FunctionToCall<T> *>(
            shm_malloc_aligned(sizeof(*to_call), alignof(FunctionToCall<T>)));
    new (to_call) FunctionToCall<T>();
    to_call->function = function;
    to_call->arg = arg;
    to_call->expected = expected;
    to_call->failure[0] = '\0';
    static_cast<char *>(fatal_failure)[0] = '\0';
    children_[id] = ForkExecute(Hangs_, to_call).release();
    if (!children_[id]) return AssertionFailure() << "ForkExecute failed";
    to_calls_[id] = reinterpret_cast<FunctionToCall<void> *>(to_call);
    to_call->started.Lock();
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
        return to_calls_[id]->expected ? AssertionSuccess() :
            AssertionFailure();
      } else if (result == ForkedProcess::JoinResult::Error) {
        return AssertionFailure() << "error joining child";
      } else {
        abort();
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
    // Some contents because we don't really want to test empty messages.
    int16_t data;
  };
  struct MessageArgs {
    RawQueue *const queue;
    int flags;
    int16_t data;  // -1 means NULL expected
  };
  static void WriteTestMessage(MessageArgs *args, char *failure) {
    TestMessage *msg = static_cast<TestMessage *>(args->queue->GetMessage());
    if (msg == NULL) {
      snprintf(fatal_failure, kFailureSize,
               "couldn't get_msg from %p", args->queue);
      return;
    }
    msg->data = args->data;
    if (!args->queue->WriteMessage(msg, args->flags)) {
      snprintf(failure, kFailureSize, "write_msg_free(%p, %p, %d) failed",
               args->queue, msg, args->flags);
    }
  }
  static void ReadTestMessage(MessageArgs *args, char *failure) {
    const TestMessage *msg = static_cast<const TestMessage *>(
        args->queue->ReadMessage(args->flags));
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
      args->queue->FreeMessage(msg);
    }
  }

 private:
  GlobalCoreInstance my_core;
};
char *QueueTest::fatal_failure;
std::map<QueueTest::ChildID, QueueTest::ForkedProcess *> QueueTest::children_;
constexpr time::Time QueueTest::kHangTime;
constexpr time::Time QueueTest::kForkSleep;

TEST_F(QueueTest, Reading) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, 0, -1};

  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock | RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.data = 254;
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  args.data = -1;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = 0;
  args.data = 971;
  EXPECT_RETURNS_FAILS(ReadTestMessage, &args);
}
TEST_F(QueueTest, Writing) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, 0, 973};

  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_HANGS(WriteTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.data = 971;
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = 0;
  EXPECT_RETURNS(ReadTestMessage, &args);
}

TEST_F(QueueTest, MultiRead) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, 0, 1323};

  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 1));
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 2));
  EXPECT_TRUE(HangsCheck(1) != HangsCheck(2));
  // TODO(brians) finish this
}

// There used to be a bug where reading first without an index and then with an
// index would crash. This test makes sure that's fixed.
TEST_F(QueueTest, ReadIndexAndNot) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);

  // Write a message, read it (with ReadMessage), and then write another
  // message (before freeing the read one so the queue allocates a distinct
  // message to use for it).
  TestMessage *msg = static_cast<TestMessage *>(queue->GetMessage());
  ASSERT_NE(nullptr, msg);
  ASSERT_TRUE(queue->WriteMessage(msg, 0));
  const void *read_msg = queue->ReadMessage(0);
  EXPECT_NE(nullptr, read_msg);
  msg = static_cast<TestMessage *>(queue->GetMessage());
  queue->FreeMessage(read_msg);
  ASSERT_NE(nullptr, msg);
  ASSERT_TRUE(queue->WriteMessage(msg, 0));

  int index = 0;
  const void *second_read_msg = queue->ReadMessageIndex(0, &index);
  EXPECT_NE(nullptr, second_read_msg);
  EXPECT_NE(read_msg, second_read_msg)
      << "We already took that message out of the queue.";
}

TEST_F(QueueTest, Recycle) {
  // TODO(brians) basic test of recycle queue
  // include all of the ways a message can get into the recycle queue
  RawQueue *recycle_queue = reinterpret_cast<RawQueue *>(23);
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage),
                                          1, 2, 2, 2, &recycle_queue);
  ASSERT_NE(reinterpret_cast<RawQueue *>(23), recycle_queue);
  MessageArgs args{queue, 0, 973}, recycle{recycle_queue, 0, 973};

  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.data = 254;
  EXPECT_RETURNS(WriteTestMessage, &args);
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.data = 971;
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  recycle.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &recycle);

  EXPECT_HANGS(ReadTestMessage, &recycle);

  TestMessage *msg = static_cast<TestMessage *>(queue->GetMessage());
  ASSERT_TRUE(msg != NULL);
  msg->data = 341;
  queue->FreeMessage(msg);
  recycle.data = 341;
  EXPECT_RETURNS(ReadTestMessage, &recycle);

  EXPECT_HANGS(ReadTestMessage, &recycle);

  args.data = 254;
  args.flags = RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  recycle.flags = RawQueue::kBlock;
  EXPECT_HANGS(ReadTestMessage, &recycle);
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  recycle.data = 254;
  EXPECT_RETURNS(ReadTestMessage, &recycle);
}

// Makes sure that when a message doesn't get written with kNonBlock it does get
// freed.
TEST_F(QueueTest, NonBlockFailFree) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);

  void *message1 = queue->GetMessage();
  void *message2 = queue->GetMessage();
  ASSERT_TRUE(queue->WriteMessage(message1, RawQueue::kNonBlock));
  ASSERT_FALSE(queue->WriteMessage(message2, RawQueue::kNonBlock));
  EXPECT_EQ(message2, queue->GetMessage());
}

}  // namespace testing
}  // namespace aos
