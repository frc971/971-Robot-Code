#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/builder.h"
#include "aos/flatbuffers/static_flatbuffers.h"
#include "aos/flatbuffers/test_dir/type_coverage_static.h"
#include "aos/flatbuffers/test_static.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::fbs::testing {

class StaticFlatbuffersFuzzTest : public ::testing::Test {
 protected:
  template <typename T>
  void VerifyJson(const std::string_view data) {
    Builder<T> json_builder = aos::JsonToStaticFlatbuffer<T>(data);

    EXPECT_EQ(data, aos::FlatbufferToJson(json_builder.AsFlatbufferSpan(),
                                          {.multi_line = true}));
  }
};

namespace {
void Combine(const std::span<const std::vector<std::string_view>> &strings,
             std::function<void(const std::vector<std::string_view> &)> handler,
             const std::vector<std::string_view> &current_combination) {
  if (strings.empty()) {
    handler(current_combination);
    return;
  }
  for (const std::string_view &str : strings.front()) {
    std::vector<std::string_view> combination = current_combination;
    combination.push_back(str);
    Combine(strings.subspan(1), handler, combination);
  }
}
void Combine(
    const std::vector<std::vector<std::string_view>> &strings,
    std::function<void(const std::vector<std::string_view> &)> handler) {
  Combine(std::span<const std::vector<std::string_view>>{strings.data(),
                                                         strings.size()},
          handler, {});
}
}  // namespace

// Iterate over lots of variations of different flatbuffers to try to see if we
// can exercise weird corner-cases.
TEST_F(StaticFlatbuffersFuzzTest, JsonFuzzing) {
  std::vector<std::vector<std::string_view>> stanzas{
      {"", "\"scalar\": 1323"},
      {"", "\"vector_of_scalars\": [\n  \n ]",
       "\"vector_of_scalars\": [\n  123\n ]",
       "\"vector_of_scalars\": [\n  123,\n  456\n ]"},
      {"", "\"string\": \"\"", "\"string\": \"abcdef\"",
       "\"string\": \"abcdefghijklmnopqrstuvwxyz\""},
      {
          "",
          "\"vector_of_strings\": [\n  \n ]",
          "\"vector_of_strings\": [\n  \"\",\n  \"abcdef\"\n ]",
          "\"vector_of_strings\": [\n  \"\",\n  \"abcdef\",\n  "
          "\"abcdefghijklmnopqrstuvwxyz\"\n ]",
          "\"vector_of_strings\": [\n  \"\",\n  \"abcdef\",\n  \"971\",\n  "
          "\"abcdefghijklmnopqrstuvwxyz\"\n ]",
          "\"vector_of_strings\": [\n  \"\",\n  \"abcdef\",\n  "
          "\"abcdefghijklmnopqrstuvwxyz\",\n  \"971\",\n  \"123\"\n ]",
          "\"vector_of_strings\": [\n  \"\",\n  \"abcdef\",\n  \"xyz\",\n  "
          "\"971\",\n  \"123\"\n ]",
      },
      {
          "",
          "\"substruct\": {\n  \"x\": 971.0,\n  \"y\": 123.0\n }",
      },
      {
          "",
          "\"subtable\": {\n\n }",
          "\"subtable\": {\n  \"baz\": 1.23\n }",
          "\"subtable\": {\n  \"foo\": 123,\n  \"baz\": 1.23\n }",
      },
      {
          "",
          "\"vector_aligned\": [\n  \n ]",
          "\"vector_aligned\": [\n  678\n ]",
          "\"vector_aligned\": [\n  678,\n  456\n ]",
          "\"vector_aligned\": [\n  7,\n  6,\n  5,\n  4,\n  3,\n  2,\n  1,\n  "
          "0\n ]",
      },
      {
          "",
          "\"vector_of_structs\": [\n  \n ]",
          R"json("vector_of_structs": [
  {
   "x": 1.0,
   "y": 2.0
  }
 ])json",
          R"json("vector_of_structs": [
  {
   "x": 1.0,
   "y": 2.0
  },
  {
   "x": 3.0,
   "y": 4.0
  },
  {
   "x": 5.0,
   "y": 6.0
  }
 ])json",
          R"json("vector_of_structs": [
  {
   "x": 1.0,
   "y": 2.0
  },
  {
   "x": 3.0,
   "y": 4.0
  },
  {
   "x": 5.0,
   "y": 6.0
  },
  {
   "x": 7.0,
   "y": 8.0
  },
  {
   "x": 9.0,
   "y": 10.0
  }
 ])json",
      },
      {
          "",
          "\"vector_of_tables\": [\n  \n ]",
          R"json("vector_of_tables": [
  {

  }
 ])json",
          R"json("vector_of_tables": [
  {
   "foo": 1
  }
 ])json",
          R"json("vector_of_tables": [
  {
   "foo": 1
  },
  {
   "foo": 2
  },
  {
   "foo": 3
  },
  {
   "foo": 4
  },
  {
   "foo": 5
  },
  {
   "foo": 6
  }
 ])json",
      },
      {
          "",
          "\"included_table\": {\n\n }",
          "\"included_table\": {\n  \"foo\": \"A\"\n }",
      },
      {
          "",
          "\"unspecified_length_vector\": [\n  \n ]",
          "\"unspecified_length_vector\": [\n  123\n ]",
          "\"unspecified_length_vector\": [\n  123,\n  100\n ]",
      },
      {
          "",
          "\"unspecified_length_string\": \"\"",
          "\"unspecified_length_string\": \"Hello, World!\"",
      },
      {
          "",
          "\"unspecified_length_vector_of_strings\": [\n  \n ]",
          "\"unspecified_length_vector_of_strings\": [\n  \"\"\n ]",
          "\"unspecified_length_vector_of_strings\": [\n  \"Goodbye, \",\n  "
          "\"World!\"\n ]",
      },
  };
  Combine(stanzas, [this](const std::vector<std::string_view> &strings) {
    std::vector<std::string_view> no_empty_strings;
    for (const std::string_view &str : strings) {
      if (!str.empty()) {
        no_empty_strings.push_back(str);
      }
    }
    if (no_empty_strings.empty()) {
      VerifyJson<TestTableStatic>("{\n\n}");
    } else {
      VerifyJson<TestTableStatic>(
          "{\n " + absl::StrJoin(no_empty_strings, ",\n ") + "\n}");
    }
  });
}
}  // namespace aos::fbs::testing
