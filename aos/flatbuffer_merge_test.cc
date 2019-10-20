#include "aos/flatbuffer_merge.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"
#include "flatbuffers/minireflect.h"

namespace aos {
namespace testing {

class FlatbufferMerge : public ::testing::Test {
 public:
  FlatbufferMerge() {}

  void JsonMerge(const ::std::string in1, const ::std::string in2,
                 const ::std::string out) {
    printf("Merging: %s\n", in1.c_str());
    printf("Merging: %s\n", in2.c_str());
    const flatbuffers::DetachedBuffer fb1 = JsonToFlatbuffer(
        static_cast<const char *>(in1.c_str()), ConfigurationTypeTable());

    const ::std::string in1_nested = "{ \"nested_config\": " + in1 + " }";
    const flatbuffers::DetachedBuffer fb1_nested =
        JsonToFlatbuffer(static_cast<const char *>(in1_nested.c_str()),
                         ConfigurationTypeTable());

    const flatbuffers::DetachedBuffer fb2 = JsonToFlatbuffer(
        static_cast<const char *>(in2.c_str()), ConfigurationTypeTable());

    const ::std::string in2_nested = "{ \"nested_config\": " + in2 + " }";
    const flatbuffers::DetachedBuffer fb2_nested =
        JsonToFlatbuffer(static_cast<const char *>(in2_nested.c_str()),
                         ConfigurationTypeTable());

    const ::std::string out_nested = "{ \"nested_config\": " + out + " }";

    const flatbuffers::DetachedBuffer empty =
        JsonToFlatbuffer("{ }", ConfigurationTypeTable());

    ASSERT_NE(fb1.size(), 0u);
    ASSERT_NE(fb2.size(), 0u);
    ASSERT_NE(fb1_nested.size(), 0u);
    ASSERT_NE(fb2_nested.size(), 0u);

    // We now want to run 7 tests.
    //  in1 merged "" -> in1.
    //  in2 merged "" -> in2.
    //  "" merged in1 -> in1.
    //  "" merged in2 -> in2.
    //  nullptr merged in1 -> in1.
    //  in1 merged nullptr -> in1.
    //  in1 merged in2 -> out.

    {
      // in1 merged with "" => in1.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1.data(), empty.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // in2 merged with "" => in2.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb2.data(), empty.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in2, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // "" merged with in1 => in1.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(empty.data(), fb1.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // "" merged with in2 => in2.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(empty.data(), fb2.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in2, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // nullptr merged with in1 => in1.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(nullptr, fb1.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // in1 merged with nullptr => in1.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1.data(), nullptr);
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1, FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // in1 merged with in2 => out.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1.data(), fb2.data());
      ASSERT_NE(fb_merged.size(), 0u);

      const ::std::string merged_output =
          FlatbufferToJson(fb_merged, ConfigurationTypeTable());
      EXPECT_EQ(out, merged_output);

      printf("Merged to: %s\n", merged_output.c_str());
    }

    // Now, to make things extra exciting, nest a config inside a config.  And
    // run all the tests.  This will exercise some fun nested merges and copies.

    {
      // in1_nested merged with "" => in1.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1_nested.data(), empty.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // in2_nested merged with "" => in2_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb2_nested.data(), empty.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in2_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // "" merged with in1_nested => in1_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(empty.data(), fb1_nested.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // "" merged with in2_nested => in2_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(empty.data(), fb2_nested.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in2_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // nullptr merged with in1_nested => in1_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(nullptr, fb1_nested.data());
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // nullptr merged with in1_nested => in1_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1_nested.data(), nullptr);
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(in1_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }

    {
      // in1_nested merged with in2_nested => out_nested.
      flatbuffers::DetachedBuffer fb_merged =
          MergeFlatBuffers<Configuration>(fb1_nested, fb2_nested);
      ASSERT_NE(fb_merged.size(), 0u);

      EXPECT_EQ(out_nested,
                FlatbufferToJson(fb_merged, ConfigurationTypeTable()));
    }
  }
};

// Test the easy ones.  Test every type, single, no nesting.
TEST_F(FlatbufferMerge, Basic) {
  JsonMerge("{ \"foo_bool\": true }", "{ \"foo_bool\": false }",
            "{ \"foo_bool\": false }");

  JsonMerge("{ \"foo_bool\": false }", "{ \"foo_bool\": true }",
            "{ \"foo_bool\": true }");

  JsonMerge("{ \"foo_byte\": 5 }", "{ \"foo_byte\": -7 }",
            "{ \"foo_byte\": -7 }");

  JsonMerge("{ \"foo_ubyte\": 5 }", "{ \"foo_ubyte\": 7 }",
            "{ \"foo_ubyte\": 7 }");

  JsonMerge("{ \"foo_short\": 5 }", "{ \"foo_short\": 7 }",
            "{ \"foo_short\": 7 }");

  JsonMerge("{ \"foo_int\": 5 }", "{ \"foo_int\": 7 }", "{ \"foo_int\": 7 }");

  JsonMerge("{ \"foo_uint\": 5 }", "{ \"foo_uint\": 7 }",
            "{ \"foo_uint\": 7 }");

  JsonMerge("{ \"foo_long\": 5 }", "{ \"foo_long\": 7 }",
            "{ \"foo_long\": 7 }");

  JsonMerge("{ \"foo_ulong\": 5 }", "{ \"foo_ulong\": 7 }",
            "{ \"foo_ulong\": 7 }");

  JsonMerge("{ \"foo_float\": 5.0 }", "{ \"foo_float\": 7.1 }",
            "{ \"foo_float\": 7.1 }");

  JsonMerge("{ \"foo_double\": 5.0 }", "{ \"foo_double\": 7.1 }",
            "{ \"foo_double\": 7.1 }");
}

// Test arrays of simple types.
TEST_F(FlatbufferMerge, Array) {
  JsonMerge("{ \"foo_string\": \"baz\" }", "{ \"foo_string\": \"bar\" }",
            "{ \"foo_string\": \"bar\" }");

  JsonMerge("{ \"vector_foo_bool\": [ true, true, false ] }",
            "{ \"vector_foo_bool\": [ false, true, true, false ] }",
            "{ \"vector_foo_bool\": [ true, true, false, false, "
            "true, true, false ] }");

  JsonMerge("{ \"vector_foo_byte\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_byte\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_byte\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ubyte\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ubyte\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ubyte\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_short\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_short\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_short\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ushort\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ushort\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ushort\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_int\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_int\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_int\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_uint\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_uint\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_uint\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_long\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_long\": [ -3, 1, 3, 2 ] }",
            "{ \"vector_foo_long\": [ 9, 7, 1, -3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_ulong\": [ 9, 7, 1 ] }",
            "{ \"vector_foo_ulong\": [ 3, 1, 3, 2 ] }",
            "{ \"vector_foo_ulong\": [ 9, 7, 1, 3, 1, 3, 2 ] }");

  JsonMerge("{ \"vector_foo_float\": [ 9.0, 7.0, 1.0 ] }",
            "{ \"vector_foo_float\": [ -3.0, 1.3, 3.0, 2.0 ] }",
            "{ \"vector_foo_float\": [ 9.0, 7.0, 1.0, -3.0, 1.3, 3.0, 2.0 ] }");

  JsonMerge(
      "{ \"vector_foo_string\": [ \"9\", \"7\", \"1 \" ] }",
      "{ \"vector_foo_string\": [ \"31\", \"32\" ] }",
      "{ \"vector_foo_string\": [ \"9\", \"7\", \"1 \", \"31\", \"32\" ] }");
}

// Test nested messages, and arrays of nested messages.
TEST_F(FlatbufferMerge, NestedStruct) {
  JsonMerge(
      "{ \"single_application\": { \"name\": \"woot\" } }",
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }",
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }");

  JsonMerge(
      "{ \"single_application\": { \"name\": \"wow\", \"priority\": 7 } }",
      "{ \"single_application\": { \"name\": \"woot\" } }",
      "{ \"single_application\": { \"name\": \"woot\", \"priority\": 7 } }");

  JsonMerge("{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" } ] }",
            "{ \"apps\": [ { \"name\": \"woo2\" }, { \"name\": \"wo3\" } ] }",
            "{ \"apps\": [ { \"name\": \"woot\" }, { \"name\": \"wow\" }, { "
            "\"name\": \"woo2\" }, { \"name\": \"wo3\" } ] }");
}

// TODO(austin): enums
// TODO(austin): unions
// TODO(austin): struct

}  // namespace testing
}  // namespace aos
