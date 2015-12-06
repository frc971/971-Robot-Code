#include "aos/common/queue_types.h"

#include <memory>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "aos/common/test_queue.q.h"
#include "aos/common/byteorder.h"
#include "aos/common/queue_primitives.h"
#include "aos/common/logging/logging.h"
#include "aos/testing/test_logging.h"

using ::aos::common::testing::Structure;
using ::aos::common::testing::MessageWithStructure;
using ::aos::common::testing::OtherTestingMessage;
using ::aos::common::testing::MessageWithArrays;

namespace aos {
namespace testing {

typedef MessageType::Field Field;

static const MessageType kTestType1(5, 0x1234, "TestType1",
                                    {new Field{0, 0, "field1"},
                                     new Field{0, 0, "field2"},
                                     new Field{0, 0, "field3"}});

class QueueTypesTest : public ::testing::Test {
 public:
  QueueTypesTest() {
    ::aos::testing::EnableTestLogging();
  }

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

class PrintMessageTest : public ::testing::Test {
 public:
  char input[128], output[256];
  size_t input_bytes, output_bytes;
};

class PrintFieldTest : public PrintMessageTest {
 public:
  template <typename T>
  void TestInteger(T value, ::std::string result) {
    input_bytes = sizeof(value);
    to_network(&value, input);
    output_bytes = sizeof(output);
    ASSERT_TRUE(
        PrintField(output, &output_bytes, input, &input_bytes, TypeID<T>::id));
    EXPECT_EQ(0u, input_bytes);
    EXPECT_EQ(sizeof(output) - result.size(), output_bytes);
    EXPECT_EQ(result, ::std::string(output, sizeof(output) - output_bytes));
  }

  template <typename T>
  void TestAllIntegers(T multiple) {
    for (T i = ::std::numeric_limits<T>::min();
         i < ::std::numeric_limits<T>::max() - multiple; i += multiple) {
      TestInteger<T>(i, ::std::to_string(i));
    }
  }
};

TEST_F(PrintFieldTest, Basic) {
  TestInteger<uint16_t>(971, "971");
  TestInteger<uint8_t>(8, "8");
  TestInteger<uint8_t>(254, "254");
  TestInteger<uint8_t>(0, "0");
  TestInteger<int8_t>(254, "-2");
  TestInteger<int8_t>(67, "67");
  TestInteger<int8_t>(0, "0");

  input_bytes = 1;
  input[0] = 1;
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintField(output, &output_bytes, input, &input_bytes,
                         queue_primitive_types::bool_p));
  EXPECT_EQ(0u, input_bytes);
  EXPECT_EQ(sizeof(output) - 1, output_bytes);
  EXPECT_EQ(::std::string("T"),
            ::std::string(output, sizeof(output) - output_bytes));

  input_bytes = 1;
  input[0] = 0;
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintField(output, &output_bytes, input, &input_bytes,
                         queue_primitive_types::bool_p));
  EXPECT_EQ(0u, input_bytes);
  EXPECT_EQ(sizeof(output) - 1, output_bytes);
  EXPECT_EQ(::std::string("f"),
            ::std::string(output, sizeof(output) - output_bytes));
}

// Runs through lots of integers and makes sure PrintField gives the same result
// as ::std::to_string.
TEST_F(PrintFieldTest, Integers) {
  TestAllIntegers<uint8_t>(1);
  TestAllIntegers<int8_t>(1);
  TestAllIntegers<uint16_t>(1);
  TestAllIntegers<int16_t>(3);
  TestAllIntegers<uint32_t>(43129);
  TestAllIntegers<int32_t>(654321);
  TestAllIntegers<uint64_t>(123456789101112);
  TestAllIntegers<int64_t>(91011121249856532);
}

// Tests PrintField with trailing input bytes and only 1 extra output byte.
TEST_F(PrintFieldTest, OtherSizes) {
  static const float kData = 16.78;
  static const ::std::string kString("16.780001");
  static const size_t kExtraInputBytes = 4;
  input_bytes = sizeof(kData) + kExtraInputBytes;
  to_network(&kData, input);
  output_bytes = kString.size() + 1;
  CHECK_LE(output_bytes, sizeof(output));
  ASSERT_TRUE(PrintField(output, &output_bytes, input, &input_bytes,
                         Structure::GetType()->fields[2]->type));
  EXPECT_EQ(kExtraInputBytes, input_bytes);
  EXPECT_EQ(1u, output_bytes);
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
  output_bytes = 4;
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
static const ::aos::common::testing::Structure kStructureValue(true, 973, 3.14);
static const MessageWithArrays kTestMessageWithArrays({{971, 254, 1678}},
                                                      {{kStructureValue,
                                                        kStructureValue}});
static const ::std::string kTestMessageWithArraysString =
    ".aos.common.testing.MessageWithArrays{test_int:[971, 254, 1678], "
    "test_struct:[.aos.common.testing.Structure{struct_bool:T, struct_int:973, "
    "struct_float:3.140000}, .aos.common.testing.Structure{struct_bool:T, "
    "struct_int:973, struct_float:3.140000}]}";

TEST_F(PrintMessageTest, Basic) {
  CHECK_GE(sizeof(input), kTestMessage1.Size());
  input_bytes = kTestMessage1.Serialize(input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestMessage1.GetType()));
  EXPECT_EQ(kTestMessage1String, ::std::string(output));
  EXPECT_EQ(kTestMessage1String.size() + 1, sizeof(output) - output_bytes);
}

TEST_F(PrintMessageTest, OutputTooSmall) {
  CHECK_GE(sizeof(input), kTestMessage1.Size());
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
  CHECK_GE(sizeof(input), kTestStructure1.Size());
  input_bytes = kTestStructure1.Serialize(input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestStructure1.GetType()));
  EXPECT_EQ(kTestStructure1String, ::std::string(output));
  EXPECT_EQ(kTestStructure1String.size() + 1, sizeof(output) - output_bytes);
}

TEST_F(PrintMessageTest, Matrix) {
  static const uint16_t kTestMatrix[] = {971, 254, 1768, 8971, 9971, 973};
  uint16_t test_matrix[sizeof(kTestMatrix) / sizeof(kTestMatrix[0])];
  SerializeMatrix(queue_primitive_types::uint16_t_p, test_matrix, kTestMatrix,
                  3, 2);
  static const ::std::string kOutput =
      "[[971, 8971], [254, 9971], [1768, 973]]";
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMatrix(output, &output_bytes, test_matrix,
                          queue_primitive_types::uint16_t_p, 3, 2));
  EXPECT_EQ(kOutput, ::std::string(output));
  EXPECT_EQ(kOutput.size() + 1, sizeof(output) - output_bytes);
}

TEST_F(PrintMessageTest, Array) {
  CHECK_GE(sizeof(input), kTestMessageWithArrays.Size());
  input_bytes = kTestMessageWithArrays.Serialize(input);
  output_bytes = sizeof(output);
  ASSERT_TRUE(PrintMessage(output, &output_bytes, input, &input_bytes,
                           *kTestMessageWithArrays.GetType()));
  EXPECT_EQ(kTestMessageWithArraysString, ::std::string(output));
  EXPECT_EQ(kTestMessageWithArraysString.size() + 1,
            sizeof(output) - output_bytes);
}

}  // namespace testing
}  // namespace aos
