#include "aos/json_to_flatbuffer.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer_generated.h"
#include "flatbuffers/minireflect.h"

namespace aos {
namespace testing {

class JsonToFlatbufferTest : public ::testing::Test {
 public:
  JsonToFlatbufferTest() {}

  bool JsonAndBack(const ::std::string str) { return JsonAndBack(str, str); }

  bool JsonAndBack(const ::std::string in, const ::std::string out) {
    printf("Testing: %s\n", in.c_str());
    const ::std::vector<uint8_t> fb =
        JsonToFlatbuffer(in.data(), ConfigurationTypeTable());

    if (fb.size() == 0) {
      return false;
    }

    const ::std::string back =
        FlatbufferToJson(fb.data(), ConfigurationTypeTable());

    printf("Back to string: %s\n", back.c_str());

    return back == out;
  }
};

// Tests that the various escapes work as expected.
TEST_F(JsonToFlatbufferTest, ValidEscapes) {
  EXPECT_TRUE(
      JsonAndBack("{ \"foo_string\": \"a\\\"b\\/c\\bd\\fc\\nd\\re\\tf\" }",
                  "{ \"foo_string\": \"a\\\"b/c\\bd\\fc\\nd\\re\\tf\" }"));
}

// Test the easy ones.  Test every type, single, no nesting.
TEST_F(JsonToFlatbufferTest, Basic) {
  EXPECT_TRUE(JsonAndBack("{ \"foo_bool\": true }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_byte\": 5 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_ubyte\": 5 }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_short\": 5 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_ushort\": 5 }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_int\": 5 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_uint\": 5 }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_long\": 5 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_ulong\": 5 }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_float\": 5.0 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_double\": 5.0 }"));

  EXPECT_TRUE(JsonAndBack("{ \"foo_string\": \"baz\" }"));
}

// Tests that we can handle decimal points.
TEST_F(JsonToFlatbufferTest, DecimalPoint) {
  EXPECT_TRUE(JsonAndBack("{ \"foo_float\": 5.1 }"));
  EXPECT_TRUE(JsonAndBack("{ \"foo_double\": 5.1 }"));
}


// Test what happens if you pass a field name that we don't know.
TEST_F(JsonToFlatbufferTest, InvalidFieldName) {
  EXPECT_FALSE(JsonAndBack("{ \"foo\": 5 }"));
}

// Test that adding a duplicate field results in an error.
TEST_F(JsonToFlatbufferTest, DuplicateField) {
  EXPECT_FALSE(
      JsonAndBack("{ \"foo_int\": 5, \"foo_int\": 7 }", "{ \"foo_int\": 7 }"));
}

// Test that various syntax errors are caught correctly
TEST_F(JsonToFlatbufferTest, InvalidSyntax) {
  EXPECT_FALSE(JsonAndBack("{ \"foo_int\": 5"));
  EXPECT_FALSE(JsonAndBack("{ \"foo_int\": 5 "));
  EXPECT_FALSE(JsonAndBack("{ \"foo_string\": \""));
  EXPECT_FALSE(JsonAndBack("{ \"foo_int\": 5 } }"));

  EXPECT_FALSE(JsonAndBack("{ foo_int: 5 }"));

  EXPECT_FALSE(JsonAndBack("{ \"foo_int\": 5, }", "{ \"foo_int\": 5 }"));

  EXPECT_FALSE(
      JsonAndBack("{ \"apps\":\n[\n{\n\"name\": \"woot\"\n},\n{\n\"name\": "
                  "\"wow\"\n} ,\n]\n}"));

  EXPECT_FALSE(JsonAndBack(
      "{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" } ] , }"));

  EXPECT_FALSE(
      JsonAndBack("{ \"vector_foo_string\": [ \"bar\", \"baz\" ] , }"));

  EXPECT_FALSE(
      JsonAndBack("{ \"single_application\": { \"name\": \"woot\" } , }"));
}

// Test arrays of simple types.
TEST_F(JsonToFlatbufferTest, Array) {
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_byte\": [ 9, 7, 1 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_ubyte\": [ 9, 7, 1 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_short\": [ 9, 7, 1 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_ushort\": [ 9, 7, 1 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_int\": [ 9, 7, 1 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_uint\": [ 9, 7, 1 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_long\": [ 9, 7, 1 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_ulong\": [ 9, 7, 1 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_float\": [ 9.0, 7.0, 1.0 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_double\": [ 9.0, 7.0, 1.0 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_float\": [ 9, 7, 1 ] }",
                          "{ \"vector_foo_float\": [ 9.0, 7.0, 1.0 ] }"));
  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_double\": [ 9, 7, 1 ] }",
                          "{ \"vector_foo_double\": [ 9.0, 7.0, 1.0 ] }"));

  EXPECT_TRUE(JsonAndBack("{ \"vector_foo_string\": [ \"bar\", \"baz\" ] }"));
}

// Test nested messages, and arrays of nested messages.
TEST_F(JsonToFlatbufferTest, NestedStruct) {
  EXPECT_TRUE(
      JsonAndBack("{ \"single_application\": { \"name\": \"woot\" } }"));

  EXPECT_TRUE(JsonAndBack(
      "{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" } ] }"));
}

// Test that we can parse an empty message.
TEST_F(JsonToFlatbufferTest, EmptyMessage) {
  // Empty message works.
  EXPECT_TRUE(JsonAndBack("{  }"));
}

// Tests that multiple arrays get properly handled.
TEST_F(JsonToFlatbufferTest, MultipleArrays) {
  EXPECT_TRUE(
      JsonAndBack("{ \"vector_foo_float\": [ 9, 7, 1 ], \"vector_foo_double\": "
                  "[ 9, 7, 1 ] }",
                  "{ \"vector_foo_float\": [ 9.0, 7.0, 1.0 ], "
                  "\"vector_foo_double\": [ 9.0, 7.0, 1.0 ] }"));
}

// TODO(austin): Missmatched values.
// TODO(austin): enums
//
// TODO(austin): unions?

}  // namespace testing
}  // namespace aos
