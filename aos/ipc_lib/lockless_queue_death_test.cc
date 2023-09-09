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

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/ipc_lib/lockless_queue_stepping.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/realtime.h"
#include "aos/testing/test_logging.h"

namespace aos {
namespace ipc_lib {
namespace testing {

namespace chrono = ::std::chrono;

#ifdef SUPPORTS_SHM_ROBUSTNESS_TEST

namespace {
static int kPinnedMessageIndex = 0;

constexpr monotonic_clock::duration kChannelStorageDuration =
    std::chrono::milliseconds(500);

}  // namespace

// Tests that death during sends is recovered from correctly.
TEST(LocklessQueueTest, Death) {
  ::aos::testing::EnableTestLogging();

  SharedTid tid;

  // Make a small queue so it is easier to debug.
  LocklessQueueConfiguration config;
  config.num_watchers = 2;
  config.num_senders = 2;
  config.num_pinners = 1;
  config.queue_size = 10;
  config.message_data_size = 32;

  TestShmRobustness(
      config,
      [config, &tid](void *memory) {
        // Initialize the queue and grab the tid.
        LocklessQueue(
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            reinterpret_cast<aos::ipc_lib::LocklessQueueMemory *>(memory),
            config)
            .Initialize();
        tid.Set();
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
      [config, &tid](void *raw_memory) {
        ::aos::ipc_lib::LocklessQueueMemory *const memory =
            reinterpret_cast<::aos::ipc_lib::LocklessQueueMemory *>(raw_memory);
        // Confirm that we can create 2 senders (the number in the queue), and
        // send a message.  And that all the messages in the queue are valid.
        LocklessQueue queue(memory, memory, config);

        bool print = false;

        // TestShmRobustness doesn't handle robust futexes.  It is happy to just
        // not crash with them.  We know what they are, and what the tid of the
        // holder is.  So go pretend to be the kernel and fix it for it.
        PretendOwnerDied(&memory->queue_setup_lock, tid.Get());

        for (size_t i = 0; i < config.num_senders; ++i) {
          if (PretendOwnerDied(&memory->GetSender(i)->tid, tid.Get())) {
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

        std::function<bool(const Context &)> should_read = [](const Context &) {
          return true;
        };

        while (true) {
          monotonic_clock::time_point monotonic_sent_time;
          realtime_clock::time_point realtime_sent_time;
          monotonic_clock::time_point monotonic_remote_time;
          realtime_clock::time_point realtime_remote_time;
          uint32_t remote_queue_index;
          UUID source_boot_uuid;
          char read_data[1024];
          size_t length;

          LocklessQueueReader::Result read_result =
              reader.Read(i, &monotonic_sent_time, &realtime_sent_time,
                          &monotonic_remote_time, &realtime_remote_time,
                          &remote_queue_index, &source_boot_uuid, &length,
                          &(read_data[0]), std::ref(should_read));

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
      });
}

#endif

}  // namespace testing
}  // namespace ipc_lib
}  // namespace aos
