#include "aos/common/queue_types.h"

#include <memory>

#include "gtest/gtest.h"

#include "aos/common/test_queue.q.h"
#include "aos/common/byteorder.h"

using ::aos::common::testing::Structure;
using ::aos::common::testing::MessageWithStructure;

namespace aos {
namespace testing {

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

class PrintFieldTest : public ::testing::Test {
 public:
  char input[128], output[128];
  size_t input_bytes, output_bytes;
};

TEST_F(PrintFieldTest, Basic) {
  static const uint16_t kData = 971;
  input_bytes = sizeof(kData);
  to_network(&kData, input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintField(output, &output_bytes, input, &input_bytes,
                         Structure::GetType()->fields[1]->type));
  EXPECT_EQ(0u, input_bytes);
  EXPECT_EQ(sizeof(output) - 4, output_bytes);
  EXPECT_EQ(::std::string("971\0", 4),
            ::std::string(output, sizeof(output) - output_bytes));
}

// Tests PrintField with trailing input bytes and no extra output bytes.
TEST_F(PrintFieldTest, OtherSizes) {
  static const float kData = 16.78;
  static const ::std::string kString("16.780001");
  static const size_t kExtraInputBytes = 4;
  input_bytes = sizeof(kData) + kExtraInputBytes;
  to_network(&kData, input);
  output_bytes = kString.size() + 1;
  assert(output_bytes <= sizeof(output));
  ASSERT_TRUE(PrintField(output, &output_bytes, input, &input_bytes,
                         Structure::GetType()->fields[2]->type));
  EXPECT_EQ(kExtraInputBytes, input_bytes);
  EXPECT_EQ(0u, output_bytes);
  EXPECT_EQ(kString, ::std::string(output));
}

TEST_F(PrintFieldTest, InputTooSmall) {
  static const float kData = 0;
  input_bytes = sizeof(kData) - 1;
  output_bytes = sizeof(output);
  EXPECT_FALSE(PrintField(output, &output_bytes, input, &input_bytes,
                          Structure::GetType()->fields[2]->type));
}

TEST_F(PrintFieldTest, OutputTooSmall) {
  static const uint16_t kData = 12345;
  input_bytes = sizeof(input);
  to_network(&kData, input);
  output_bytes = 5;
  EXPECT_FALSE(PrintField(output, &output_bytes, input, &input_bytes,
                          Structure::GetType()->fields[1]->type));
}

}  // namespace testing
}  // namespace aos
