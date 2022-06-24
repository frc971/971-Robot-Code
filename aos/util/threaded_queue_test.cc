#include "aos/util/threaded_queue.h"

#include "gtest/gtest.h"

namespace aos::util {

TEST(ThreadedQueueTest, BasicFunction) {
  std::atomic<int> counter{10000};
  int state = 0;
  std::atomic<int> observed_state{0};
  ThreadedQueue<int, int> queue(
      [&counter, &observed_state](const int state) {
        // Because this handler always returns more_to_push = false, it will
        // only get called when the queue is popped from.
        observed_state = state;
        int count = --counter;
        return ThreadedQueue<int, int>::PushResult{count, false, count == 0};
      },
      state);
  while (true) {
    std::optional<int> peek_result = queue.Peek();
    std::optional<int> pop_result = queue.Pop();
    ASSERT_EQ(peek_result.has_value(), pop_result.has_value());
    if (peek_result.has_value()) {
      ASSERT_EQ(peek_result.value(), pop_result.value());
    } else {
      break;
    }
    state++;
    queue.SetState(state);
  }
  ASSERT_EQ(counter, 0);
  // Our API doesn't make any guarantee about the push/pop cycle being kept in
  // lock-step, so just check that the observed state got incremente at all.
  ASSERT_LT(1, observed_state);
  ASSERT_EQ(state, 10000);
}

// Test running a queue where the consumer wants to have X entries pre-loaded
// and so communicates its current state back to the pusher.
TEST(ThreadedQueueTest, StatefulQueue) {
  std::atomic<int> counter{10000};
  int state = counter;
  constexpr int kMaxLookahead = 10;
  std::atomic<int> observed_state{0};
  ThreadedQueue<int, int> queue(
      [&counter, &observed_state](const int state) {
        observed_state = state;
        if (counter + kMaxLookahead < state) {
          return ThreadedQueue<int, int>::PushResult{std::nullopt, false,
                                                     counter == 0};
        } else {
          int count = --counter;
          return ThreadedQueue<int, int>::PushResult{count, true, count == 0};
        }
      },
      state);
  while (true) {
    std::optional<int> peek_result = queue.Peek();
    std::optional<int> pop_result = queue.Pop();
    ASSERT_EQ(peek_result.has_value(), pop_result.has_value());
    if (peek_result.has_value()) {
      ASSERT_EQ(peek_result.value(), pop_result.value());
    } else {
      break;
    }
    for (int ii = 0; ii < 2 * kMaxLookahead; ++ii) {
      // Trigger the internal condition variable a bunch of times to cause
      // trouble.
      queue.Peek();
    }
    // The pusher should never be more than the permissible distance ahead.
    ASSERT_GE(counter + kMaxLookahead + 1, state);
    ASSERT_GE(observed_state, state);
    state--;
    queue.SetState(state);
    // Periodically pause, ensure that the pusher has enough time to catch up,
    // and check that it has indeed pre-queued kMaxLookahead items.
    if (state % 1000 == 0 && state > 0) {
      queue.WaitForNoMoreWork();
      ASSERT_EQ(observed_state, state);
      ASSERT_EQ(counter + kMaxLookahead + 1, state);
    }
  }
  ASSERT_EQ(counter, 0);
  ASSERT_EQ(state, 0);
}


// Tests that we can exit early without any issues.
TEST(ThreadedQueueTest, ExitEarly) {
  // There used to exist a deadlock in this case where StopPushing would
  // improperly synchronize things internally, but required very (un)lucky
  // timing to hit.
  for (int ii = 0; ii < 10000; ++ii) {
    ThreadedQueue<int, int> queue(
        [](int) {
          return ThreadedQueue<int, int>::PushResult{971, false, false};
        },
        0);
    queue.StopPushing();
  }
}

}  // namespace aos::util
