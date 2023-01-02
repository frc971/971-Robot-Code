#include "aos/util/threaded_consumer.h"

#include "gtest/gtest.h"

namespace aos {
namespace util {

// We expect it to be able to pass through everything we submit and recieves it
// in the order that we submitted it. It should also be able to take in more
// tasks than the size of the ring buffer as long as the worker doesn't get
// behind.
TEST(ThreadedConsumerTest, BasicFunction) {
  std::atomic<int> counter{0};

  ThreadedConsumer<int, 4> threaded_consumer(
      [&counter](int task) {
        LOG(INFO) << "task:" << task << " counter: " << counter;
        counter = task;
      },
      0);

  for (int number : {9, 7, 1, 3, 100, 300, 42}) {
    EXPECT_TRUE(threaded_consumer.Push(number));

    // wait
    while (counter != number) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    EXPECT_EQ(counter, number);
  }
}

// We should be able to raise the realtime priority of the worker thread, and
// everything should work the same. It should also reset back to lower priority
// when shutting down the worker thread.
TEST(ThreadedConsumerTest, ElevatedPriority) {
  std::atomic<int> counter{0};

  {
    ThreadedConsumer<int, 4> threaded_consumer(
        [&counter](int task) {
          CheckRealtime();
          LOG(INFO) << "task:" << task << " counter: " << counter;
          counter = task;
        },
        20);

    for (int number : {9, 7, 1, 3, 100, 300, 42}) {
      EXPECT_TRUE(threaded_consumer.Push(number));

      // wait
      while (counter != number) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      EXPECT_EQ(counter, number);
    }
  }
  // TODO: Check that the worker thread's priority actually gets reset before
  // the thread is destroyed.

  CheckNotRealtime();
}

// If the worker gets behind, we shouldn't silently take in more tasks and
// destroy old ones.
TEST(ThreadedConsumerTest, OverflowRingBuffer) {
  std::atomic<int> counter{0};
  std::atomic<int> should_block{true};

  ThreadedConsumer<int, 4> threaded_consumer(
      [&counter, &should_block](int task) {
        LOG(INFO) << "task:" << task << " counter: " << counter;

        counter = task;

        // prevent it from making any progress to simulate it getting behind
        while (should_block) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      },
      20);

  // It consumes the 5 and then our worker blocks.
  EXPECT_TRUE(threaded_consumer.Push(5));

  // Wait for it to consume 5
  while (counter != 5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  };

  // 4 more fills up the queue.
  for (int number : {8, 9, 7, 1}) {
    EXPECT_TRUE(threaded_consumer.Push(number));
  }

  // this one should overflow the buffer.
  EXPECT_FALSE(threaded_consumer.Push(101));

  // clean up, so we don't join() an infinite loop
  should_block = false;
}

// The class should destruct gracefully and finish all of its work before
// dissapearing.
TEST(ThreadedConsumerTest, FinishesTasksOnQuit) {
  std::atomic<int> counter{0};
  std::atomic<int> should_block{true};

  {
    ThreadedConsumer<int, 4> threaded_consumer(
        [&counter, &should_block](int task) {
          LOG(INFO) << "task:" << task << " counter: " << counter;

          counter = task;

          // prevent it from making any progress to simulate it getting behind
          while (should_block) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        },
        20);

    // Give it some work to do
    for (int number : {8, 9, 7, 1}) {
      EXPECT_TRUE(threaded_consumer.Push(number));
    }

    // Wait for it to consume the first number
    while (counter != 8) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // allow it to continue
    should_block = false;
  }

  // It should have finished all the work and gotten to the last number.
  EXPECT_EQ(counter, 1);
}

}  // namespace util
}  // namespace aos
