#include "aos/common/queue_types.h"

#include <memory>

#include "gtest/gtest.h"

#include "aos/common/test_queue.q.h"

namespace aos {

typedef MessageType::Field Field;

static const MessageType kTestType1(0x1234, "TestType1",
                                    {new Field{0, "field1"},
                                     new Field{0, "field2"},
                                     new Field{0, "field3"}});

class QueueTypesTest : public ::testing::Test {
 public:
  ::testing::AssertionResult Equal(const MessageType &l, const MessageType &r) {
    using ::testing::AssertionFailure;
    if (l.id != r.id) {
      return AssertionFailure() << "id " << l.id << " != " << r.id;
    }
    if (strcmp(l.name, r.name) != 0) {
      return AssertionFailure() << "name '" << l.name << "' != '" << r.name
                                << "'";
    }
    if (l.number_fields != r.number_fields) {
      return AssertionFailure() << "number_fields " << l.number_fields
                                << " != " << r.number_fields;
    }
    for (int i = 0; i < l.number_fields; ++i) {
      SCOPED_TRACE("field " + ::std::to_string(i));
      if (l.fields[i]->type != r.fields[i]->type) {
        return AssertionFailure() << "type " << l.fields[i]->type
                                  << " != " << r.fields[i]->type;
      }
      if (strcmp(l.fields[i]->name, r.fields[i]->name) != 0) {
        return AssertionFailure() << "name '" << l.fields[i]->name << "' != '"
                                  << r.fields[i]->name << "'";
      }
    }
    return ::testing::AssertionSuccess();
  }
};

TEST_F(QueueTypesTest, Serialization) {
  char buffer[512];
  ssize_t size;
  size_t out_size;
  ::std::unique_ptr<MessageType> deserialized;

  size = kTestType1.Serialize(buffer, sizeof(buffer));
  EXPECT_GT(size, 1);

  out_size = size;
  deserialized.reset(MessageType::Deserialize(buffer, &out_size));
  EXPECT_EQ(0u, out_size);
  EXPECT_TRUE(Equal(kTestType1, *deserialized));

  out_size = size - 1;
  deserialized.reset(MessageType::Deserialize(buffer, &out_size));
  EXPECT_EQ(nullptr, deserialized.get());

  out_size = size + 1;
  ASSERT_LE(out_size, sizeof(buffer));
  deserialized.reset(MessageType::Deserialize(buffer, &out_size));
  EXPECT_EQ(1u, out_size);
  EXPECT_TRUE(Equal(kTestType1, *deserialized));
}

}  // namespace aos
