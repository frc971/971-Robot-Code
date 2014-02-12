#include "aos/common/queue_types.h"

#include <memory>

#include "gtest/gtest.h"

namespace aos {

typedef MessageType::Field Field;

static const MessageType kTestType1(0x1234, "TestType1",
                                    {new Field{0, "field1"},
                                     new Field{0, "field2"},
                                     new Field{0, "field3"}});

TEST(QueueTypesTest, Serialization) {
  char buffer[512];
  ssize_t size;

  size = kTestType1.Serialize(buffer, sizeof(buffer));
  EXPECT_GT(size, 1);
  size_t out_size = size;
  ::std::unique_ptr<MessageType> deserialized(MessageType::Deserialize(buffer, &out_size));
  EXPECT_EQ(0u, out_size);
}

}  // namespace aos
