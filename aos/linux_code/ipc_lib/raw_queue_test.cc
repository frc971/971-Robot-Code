#include "aos/linux_code/ipc_lib/queue.h"

#include <unistd.h>
#include <sys/mman.h>
#include <inttypes.h>

#include <ostream>
#include <memory>
#include <map>

#include "gtest/gtest.h"

#include "aos/linux_code/ipc_lib/core_lib.h"
#include "aos/common/type_traits.h"
#include "aos/testing/test_shm.h"
#include "aos/common/time.h"
#include "aos/common/logging/logging.h"
#include "aos/common/die.h"
#include "aos/common/util/thread.h"
#include "aos/common/util/options.h"
#include "aos/common/util/death_test_log_implementation.h"
#include "aos/testing/prevent_exit.h"

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;

namespace aos {
namespace testing {

// The same constant from queue.cc. This will have to be updated if that one is.
const int kExtraMessages = 20;

class RawQueueTest : public ::testing::Test {
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
        return std::string("unknown(") +
               ::std::to_string(static_cast<uint8_t>(result)) + ")";
    }
  }
  static_assert(aos::shm_ok<ResultType>::value,
                "this will get put in shared memory");
  template<typename T>
  struct FunctionToCall {
    FunctionToCall() : result(ResultType::NotCalled), started() {
    }

    volatile ResultType result;
    bool expected;
    void (*function)(T*, char*);
    T *arg;
    volatile char failure[kFailureSize];
    aos_futex started;
  };
  template<typename T>
  static void Hangs_(FunctionToCall<T> *const to_call) {
    time::SleepFor(time::Time::InSeconds(0.01));
    ASSERT_EQ(1, futex_set(&to_call->started));
    to_call->result = ResultType::Called;
    to_call->function(to_call->arg, const_cast<char *>(to_call->failure));
    to_call->result = ResultType::Returned;
  }

  // How long until a function is considered to have hung.
  static constexpr time::Time kHangTime = time::Time::InSeconds(0.09);
  // How long to sleep after forking (for debugging).
  static constexpr time::Time kForkSleep = time::Time::InSeconds(0);

  // Represents a process that has been forked off. The destructor kills the
  // process and wait(2)s for it.
  class ForkedProcess {
   public:
    ForkedProcess(pid_t pid, aos_futex *done)
        : pid_(pid), done_(done), exiting_(false) {};
    ~ForkedProcess() {
      if (!exiting_) {
        if (kill(pid_, SIGTERM) == -1) {
          if (errno == ESRCH) {
            printf("process %jd was already dead\n",
                   static_cast<intmax_t>(pid_));
          } else {
            PLOG(FATAL, "kill(SIGKILL, %jd) failed",
                 static_cast<intmax_t>(pid_));
          }
        }
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
      timespec done_timeout = (kForkSleep + timeout).ToTimespec();
      switch (futex_wait_timeout(done_, &done_timeout)) {
        case 2:
          return JoinResult::Hung;
        case 0:
          exiting_ = true;
          return JoinResult::Finished;
        default:
          return JoinResult::Error;
      }
    }

   private:
    const pid_t pid_;
    aos_futex *const done_;
    // True iff we know that the process is already exiting.
    bool exiting_;
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
    aos_futex *done = static_cast<aos_futex *>(shm_malloc_aligned(
            sizeof(*done), alignof(aos_futex)));
    *done = 0;
    const pid_t pid = fork();
    switch (pid) {
      case 0:  // child
        if (kForkSleep != time::Time(0, 0)) {
          LOG(INFO, "pid %jd sleeping for %ds%dns\n",
              static_cast<intmax_t>(getpid()),
              kForkSleep.sec(), kForkSleep.nsec());
          time::SleepFor(kForkSleep);
        }
        ::aos::testing::PreventExit();
        function(arg);
        CHECK_NE(-1, futex_set(done));
        exit(EXIT_SUCCESS);
      case -1:  // parent failure
        PLOG(ERROR, "fork() failed");
        return std::unique_ptr<ForkedProcess>();
      default:  // parent
        return std::unique_ptr<ForkedProcess>(new ForkedProcess(pid, done));
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
    CHECK_EQ(0, futex_wait(&to_call->started));
    to_calls_[id] = reinterpret_cast<FunctionToCall<void> *>(to_call);
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
        if (to_calls_[id]->result == ResultType::NotCalled) {
          return AssertionFailure() << "stuff took too long getting started";
        }
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
    Options<RawQueue> flags;
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
      snprintf(failure, kFailureSize, "%p->WriteMessage(%p, %x) failed",
               args->queue, msg, args->flags.printable());
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

  void PushMessage(RawQueue *queue, uint16_t data) {
    TestMessage *message = static_cast<TestMessage *>(queue->GetMessage());
    message->data = data;
    ASSERT_TRUE(queue->WriteMessage(message, RawQueue::kOverride));
  }

 private:
  ::aos::testing::TestSharedMemory my_shm_;
};

char *RawQueueTest::fatal_failure;
std::map<RawQueueTest::ChildID, RawQueueTest::ForkedProcess *>
    RawQueueTest::children_;
constexpr time::Time RawQueueTest::kHangTime;
constexpr time::Time RawQueueTest::kForkSleep;

typedef RawQueueTest RawQueueDeathTest;

TEST_F(RawQueueTest, Reading) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, RawQueue::kBlock, -1};

  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock | RawQueue::kPeek;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kBlock;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.data = 254;
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kBlock;
  args.data = -1;
  EXPECT_HANGS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kBlock;
  args.data = 971;
  EXPECT_RETURNS_FAILS(ReadTestMessage, &args);
}
TEST_F(RawQueueTest, Writing) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, RawQueue::kBlock, 973};

  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_HANGS(WriteTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS_FAILS(WriteTestMessage, &args);
  args.flags = RawQueue::kPeek | RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.data = 971;
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kNonBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
  args.flags = RawQueue::kOverride;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(ReadTestMessage, &args);
}

TEST_F(RawQueueTest, MultiRead) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  MessageArgs args{queue, RawQueue::kBlock, 1323};

  args.flags = RawQueue::kBlock;
  EXPECT_RETURNS(WriteTestMessage, &args);
  args.flags = RawQueue::kBlock;
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 1));
  ASSERT_TRUE(HangsFork(ReadTestMessage, &args, true, 2));
  AssertionResult one = HangsCheck(1);
  AssertionResult two = HangsCheck(2);
  EXPECT_TRUE(one != two) << "'" <<
      one.failure_message() << "' vs '" << two.failure_message() << "'";
  // TODO(brians) finish this
}

// There used to be a bug where reading first without an index and then with an
// index would crash. This test makes sure that's fixed.
TEST_F(RawQueueTest, ReadIndexAndNot) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);

  // Write a message, read it (with ReadMessage), and then write another
  // message (before freeing the read one so the queue allocates a distinct
  // message to use for it).
  TestMessage *msg = static_cast<TestMessage *>(queue->GetMessage());
  ASSERT_NE(nullptr, msg);
  ASSERT_TRUE(queue->WriteMessage(msg, RawQueue::kBlock));
  const void *read_msg = queue->ReadMessage(RawQueue::kBlock);
  EXPECT_NE(nullptr, read_msg);
  msg = static_cast<TestMessage *>(queue->GetMessage());
  queue->FreeMessage(read_msg);
  ASSERT_NE(nullptr, msg);
  ASSERT_TRUE(queue->WriteMessage(msg, RawQueue::kBlock));

  int index = 0;
  const void *second_read_msg =
      queue->ReadMessageIndex(RawQueue::kBlock, &index);
  EXPECT_NE(nullptr, second_read_msg);
  EXPECT_NE(read_msg, second_read_msg)
      << "We already took that message out of the queue.";
}

TEST_F(RawQueueTest, Recycle) {
  // TODO(brians) basic test of recycle queue
  // include all of the ways a message can get into the recycle queue
  RawQueue *recycle_queue = reinterpret_cast<RawQueue *>(23);
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage),
                                          1, 2, 2, 2, &recycle_queue);
  ASSERT_NE(reinterpret_cast<RawQueue *>(23), recycle_queue);
  MessageArgs args{queue, RawQueue::kBlock, 973},
      recycle{recycle_queue, RawQueue::kBlock, 973};

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
  args.flags = RawQueue::kPeek | RawQueue::kBlock;
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
TEST_F(RawQueueTest, NonBlockFailFree) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);

  void *message1 = queue->GetMessage();
  void *message2 = queue->GetMessage();
  ASSERT_TRUE(queue->WriteMessage(message1, RawQueue::kNonBlock));
  ASSERT_FALSE(queue->WriteMessage(message2, RawQueue::kNonBlock));
  EXPECT_EQ(message2, queue->GetMessage());
}

// All of the tests from here down are designed to test every branch to
// make sure it does what it's supposed to. They are generally pretty repetitive
// and boring, and some of them may duplicate other tests above, but these ones
// make it a lot easier to figure out what's wrong with bugs not related to race
// conditions.

TEST_F(RawQueueTest, ReadIndexNotFull) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  EXPECT_EQ(0, kExtraMessages + 2 - queue->FreeMessages());
  PushMessage(queue, 971);
  EXPECT_EQ(1, kExtraMessages + 2 - queue->FreeMessages());

  int index = 0;
  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(971, message->data);
  EXPECT_EQ(1, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  PushMessage(queue, 1768);
  EXPECT_EQ(2, kExtraMessages + 2 - queue->FreeMessages());
  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1768, message->data);
  EXPECT_EQ(2, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  PushMessage(queue, 254);
  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  EXPECT_EQ(2, kExtraMessages + 2 - queue->FreeMessages());
}

TEST_F(RawQueueTest, ReadIndexNotBehind) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  EXPECT_EQ(0, kExtraMessages + 2 - queue->FreeMessages());
  PushMessage(queue, 971);
  EXPECT_EQ(1, kExtraMessages + 2 - queue->FreeMessages());
  PushMessage(queue, 1768);
  EXPECT_EQ(2, kExtraMessages + 2 - queue->FreeMessages());

  int index = 0;
  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(971, message->data);
  EXPECT_EQ(1, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1768, message->data);
  EXPECT_EQ(2, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexLittleBehindNotFull) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1768, message->data);
  EXPECT_EQ(2, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1768, message->data);
  EXPECT_EQ(2, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexMoreBehind) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 254);

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1768, message->data);
  EXPECT_EQ(2, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexMoreBehindNotFull) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 254);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexLotBehind) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  {
    const void *message1, *message2;
    message1 = queue->ReadMessage(RawQueue::kNonBlock);
    ASSERT_NE(nullptr, message1);
    PushMessage(queue, 254);
    message2 = queue->ReadMessage(RawQueue::kNonBlock);
    ASSERT_NE(nullptr, message2);
    PushMessage(queue, 973);
    EXPECT_EQ(4, kExtraMessages + 2 - queue->FreeMessages());
    queue->FreeMessage(message1);
    EXPECT_EQ(3, kExtraMessages + 2 - queue->FreeMessages());
    queue->FreeMessage(message2);
    EXPECT_EQ(2, kExtraMessages + 2 - queue->FreeMessages());
  }

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(254, message->data);
  EXPECT_EQ(3, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(973, message->data);
  EXPECT_EQ(4, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(973, message->data);
  EXPECT_EQ(4, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexLotBehindNotFull) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 254);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 973);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(973, message->data);
  EXPECT_EQ(4, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(973, message->data);
  EXPECT_EQ(4, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexEvenMoreBehind) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 254);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 973);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 1114);

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(973, message->data);
  EXPECT_EQ(4, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1114, message->data);
  EXPECT_EQ(5, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1114, message->data);
  EXPECT_EQ(5, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, ReadIndexEvenMoreBehindNotFull) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const TestMessage *message, *peek_message;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 254);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 973);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));
  PushMessage(queue, 1114);
  ASSERT_NE(nullptr, queue->ReadMessage(RawQueue::kNonBlock));

  int index = 0;

  peek_message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock | RawQueue::kPeek, &index));
  message = static_cast<const TestMessage *>(
      queue->ReadMessageIndex(RawQueue::kNonBlock, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1114, message->data);
  EXPECT_EQ(5, index);
  queue->FreeMessage(message);
  queue->FreeMessage(peek_message);

  index = 0;
  peek_message = static_cast<const TestMessage *>(queue->ReadMessage(
      RawQueue::kNonBlock | RawQueue::kPeek | RawQueue::kFromEnd));
  message = static_cast<const TestMessage *>(queue->ReadMessageIndex(
      RawQueue::kNonBlock | RawQueue::kFromEnd, &index));
  ASSERT_NE(nullptr, message);
  EXPECT_EQ(message, peek_message);
  EXPECT_EQ(1114, message->data);
  EXPECT_EQ(5, index);
  queue->FreeMessage(message);
}

TEST_F(RawQueueTest, MessageReferenceCounts) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  const void *message1, *message2;

  EXPECT_EQ(0, kExtraMessages + 2 - queue->FreeMessages());
  message1 = queue->GetMessage();
  EXPECT_NE(nullptr, message1);
  EXPECT_EQ(1, kExtraMessages + 2 - queue->FreeMessages());
  message2 = queue->GetMessage();
  EXPECT_NE(nullptr, message2);
  EXPECT_EQ(2, kExtraMessages + 2 - queue->FreeMessages());
  queue->FreeMessage(message1);
  EXPECT_EQ(1, kExtraMessages + 2 - queue->FreeMessages());
  queue->FreeMessage(message2);
  EXPECT_EQ(0, kExtraMessages + 2 - queue->FreeMessages());
}

// Tests that writing with kNonBlock fails and frees the message.
TEST_F(RawQueueTest, WriteDontBlock) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  void *message;

  PushMessage(queue, 971);
  int free_before = queue->FreeMessages();
  message = queue->GetMessage();
  ASSERT_NE(nullptr, message);
  EXPECT_NE(free_before, queue->FreeMessages());
  EXPECT_FALSE(queue->WriteMessage(message, RawQueue::kNonBlock));
  EXPECT_EQ(free_before, queue->FreeMessages());
}

// Tests that writing with kOverride pushes the last message out of the queue.
TEST_F(RawQueueTest, WriteOverride) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 2);
  TestMessage *message1;

  PushMessage(queue, 971);
  PushMessage(queue, 1768);
  int free_before = queue->FreeMessages();
  message1 = static_cast<TestMessage *>(queue->GetMessage());
  ASSERT_NE(nullptr, message1);
  EXPECT_NE(free_before, queue->FreeMessages());
  message1->data = 254;
  EXPECT_TRUE(queue->WriteMessage(message1, RawQueue::kOverride));
  EXPECT_EQ(free_before, queue->FreeMessages());

  const TestMessage *message2;
  message2 =
      static_cast<const TestMessage *>(queue->ReadMessage(RawQueue::kNonBlock));
  EXPECT_EQ(1768, message2->data);
  queue->FreeMessage(message2);
  EXPECT_EQ(free_before + 1, queue->FreeMessages());
  message2 =
      static_cast<const TestMessage *>(queue->ReadMessage(RawQueue::kNonBlock));
  EXPECT_EQ(254, message2->data);
  queue->FreeMessage(message2);
  EXPECT_EQ(free_before + 2, queue->FreeMessages());
}

// Makes sure that ThreadSanitizer doesn't catch any issues freeing from
// multiple threads at once.
TEST_F(RawQueueTest, MultiThreadedFree) {
  RawQueue *const queue = RawQueue::Fetch("Queue", sizeof(TestMessage), 1, 1);
  PushMessage(queue, 971);
  int free_before = queue->FreeMessages();

  const void *const message1 =
      queue->ReadMessage(RawQueue::kPeek | RawQueue::kNonBlock);
  const void *const message2 =
      queue->ReadMessage(RawQueue::kPeek | RawQueue::kNonBlock);
  ASSERT_NE(nullptr, message1);
  ASSERT_NE(nullptr, message2);
  EXPECT_EQ(free_before, queue->FreeMessages());
  util::FunctionThread t1([message1, queue](util::Thread *) {
    queue->FreeMessage(message1);
  });
  util::FunctionThread t2([message2, queue](util::Thread *) {
    queue->FreeMessage(message2);
  });
  t1.Start();
  t2.Start();
  t1.WaitUntilDone();
  t2.WaitUntilDone();
  EXPECT_EQ(free_before, queue->FreeMessages());
}

TEST_F(RawQueueDeathTest, OptionsValidation) {
  RawQueue *const queue = RawQueue::Fetch("Queue", 1, 1, 1);

  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->WriteMessage(nullptr, RawQueue::kPeek);
      },
      ".*illegal write option.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->WriteMessage(nullptr, RawQueue::kFromEnd);
      },
      ".*illegal write option.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->WriteMessage(nullptr, RawQueue::kPeek | RawQueue::kFromEnd);
      },
      ".*illegal write option.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->WriteMessage(nullptr, RawQueue::kNonBlock | RawQueue::kBlock);
      },
      ".*invalid write option.*");

  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->ReadMessageIndex(
            RawQueue::kBlock | RawQueue::kFromEnd | RawQueue::kPeek, nullptr);
      },
      ".*ReadMessageIndex.*is not allowed.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->ReadMessageIndex(RawQueue::kOverride, nullptr);
      },
      ".*illegal read option.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->ReadMessageIndex(RawQueue::kOverride | RawQueue::kBlock,
                                nullptr);
      },
      ".*illegal read option.*");
  EXPECT_DEATH(
      {
        logging::AddImplementation(new util::DeathTestLogImplementation());
        queue->ReadMessage(RawQueue::kNonBlock | RawQueue::kBlock);
      },
      ".*invalid read option.*");
}

}  // namespace testing
}  // namespace aos
