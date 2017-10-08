#include "motors/usb/queue.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace teensy {
namespace testing {

TEST(QueueTest, Basic) {
  Queue queue(64);
  ASSERT_EQ(0u, queue.data_queued());
  ASSERT_TRUE(queue.empty());
  ASSERT_EQ(5u, queue.Write("abcde", 5));
  ASSERT_EQ(5u, queue.data_queued());
  ASSERT_FALSE(queue.empty());
  char buffer[5];
  ASSERT_EQ(5u, queue.Read(buffer, 5));
  ASSERT_EQ("abcde", ::std::string(buffer, 5));
  ASSERT_TRUE(queue.empty());
}

TEST(QueueTest, Fill) {
  Queue queue(8);
  ASSERT_EQ(0u, queue.data_queued());
  ASSERT_EQ(7u, queue.Write("abcdefgh", 8));
  ASSERT_EQ(7u, queue.data_queued());
  char buffer[7];
  ASSERT_EQ(7u, queue.Read(buffer, 100));
  ASSERT_EQ("abcdefg", ::std::string(buffer, 7));
  ASSERT_TRUE(queue.empty());

  ASSERT_EQ(3u, queue.Write("xyz", 3));
  ASSERT_EQ(3u, queue.Read(buffer, 100));
  ASSERT_EQ(0u, queue.data_queued());

  ASSERT_EQ(7u, queue.Write("abcdefgh", 8));
  ASSERT_EQ(7u, queue.data_queued());
  ASSERT_EQ(7u, queue.Read(buffer, 100));
  ASSERT_EQ("abcdefg", ::std::string(buffer, 7));
  ASSERT_TRUE(queue.empty());
}

}  // namespace testing
}  // namespace teensy
}  // namespace frc971
