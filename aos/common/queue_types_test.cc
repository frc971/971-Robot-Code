#include "aos/common/queue_types.h"

#include <memory>

#include "gtest/gtest.h"

#include "aos/common/test_queue.q.h"
#include "aos/common/byteorder.h"

using ::aos::common::testing::Structure;
using ::aos::common::testing::MessageWithStructure;
using ::aos::common::testing::OtherTestingMessage;

namespace aos {
namespace testing {

typedef MessageType::Field Field;

static const MessageType kTestType1(5, 0x1234, "TestType1",
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
    if (l.name != r.name) {
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
      if (l.fields[i]->name != r.fields[i]->name) {
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
  ASSERT_GT(size, 1);
  ASSERT_LE(static_cast<size_t>(size), sizeof(buffer));

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
typedef PrintFieldTest PrintMessageTest;

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

static const OtherTestingMessage kTestMessage1(true, 8971, 3.2);
static const ::std::string kTestMessage1String =
    ".aos.common.testing.OtherTestingMessage{test_bool:T, test_int:8971"
    ", test_double:3.200000}";
static const Structure kTestStructure1(false, 973, 8.56);
static const ::std::string kTestStructure1String =
    ".aos.common.testing.Structure{struct_bool:f, struct_int:973"
    ", struct_float:8.560000}";

TEST_F(PrintMessageTest, Basic) {
  assert(sizeof(input) >= kTestMessage1.Size());
  input_bytes = kTestMessage1.Serialize(input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestMessage1.GetType()));
  EXPECT_EQ(kTestMessage1String, ::std::string(output));
  EXPECT_EQ(kTestMessage1String.size() + 1, sizeof(output) - output_bytes);
}

TEST_F(PrintMessageTest, OutputTooSmall) {
  assert(sizeof(input) >= kTestMessage1.Size());
  input_bytes = kTestMessage1.Serialize(input);
  output_bytes = kTestMessage1String.size();
  EXPECT_FALSE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestMessage1.GetType()));
}

TEST_F(PrintMessageTest, InputTooSmall) {
  input_bytes = kTestMessage1.Size() - 1;
  output_bytes = sizeof(output);
  EXPECT_FALSE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestMessage1.GetType()));
}

TEST_F(PrintMessageTest, Structure) {
  assert(sizeof(input) >= kTestStructure1.Size());
  input_bytes = kTestStructure1.Serialize(input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestStructure1.GetType()));
  EXPECT_EQ(kTestStructure1String, ::std::string(output));
  EXPECT_EQ(kTestStructure1String.size() + 1, sizeof(output) - output_bytes);
}

}  // namespace testing
}  // namespace aos
