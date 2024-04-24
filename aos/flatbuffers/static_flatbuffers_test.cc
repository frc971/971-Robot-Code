#include "aos/flatbuffers/static_flatbuffers.h"

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "external/com_github_google_flatbuffers/src/annotated_binary_text_gen.h"
#include "external/com_github_google_flatbuffers/src/binary_annotator.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/flatbuffers.h"
#include "aos/flatbuffers/builder.h"
#include "aos/flatbuffers/interesting_schemas.h"
#include "aos/flatbuffers/test_dir/include_reflection_static.h"
#include "aos/flatbuffers/test_dir/type_coverage_static.h"
#include "aos/flatbuffers/test_schema.h"
#include "aos/flatbuffers/test_static.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::fbs::testing {

namespace {
// Uses the binary schema to annotate a provided flatbuffer.  Returns the
// annotated flatbuffer.
std::string AnnotateBinaries(
    const aos::NonSizePrefixedFlatbuffer<reflection::Schema> &schema,
    flatbuffers::span<uint8_t> binary_data) {
  flatbuffers::BinaryAnnotator binary_annotator(
      schema.span().data(), schema.span().size(), binary_data.data(),
      binary_data.size());

  auto annotations = binary_annotator.Annotate();
  const std::string schema_filename =
      aos::testing::TestTmpDir() + "/schema.bfbs";

  aos::WriteFlatbufferToFile(schema_filename, schema);

  flatbuffers::AnnotatedBinaryTextGenerator text_generator(
      flatbuffers::AnnotatedBinaryTextGenerator::Options{}, annotations,
      binary_data.data(), binary_data.size());

  text_generator.Generate(aos::testing::TestTmpDir() + "/foo.bfbs",
                          schema_filename);

  return aos::util::ReadFileToStringOrDie(aos::testing::TestTmpDir() +
                                          "/foo.afb");
}
const reflection::Object *GetObjectByName(const reflection::Schema *schema,
                                          std::string_view name) {
  for (const reflection::Object *object : *schema->objects()) {
    if (object->name()->string_view() == name) {
      return object;
    }
  }
  return nullptr;
}

// Accesses all the values in the supplied span. Used to ensure that memory
// sanitizers can observe uninitialized memory.
void TestMemory(std::span<uint8_t> memory) {
  std::stringstream str;
  internal::DebugBytes(memory, str);
  EXPECT_LT(0u, str.view().size());
}
}  // namespace

class StaticFlatbuffersTest : public ::testing::Test {
 protected:
  template <typename T>
  void VerifyJson(const std::string_view data) {
    Builder<T> json_builder = aos::JsonToStaticFlatbuffer<T>(data);

    EXPECT_EQ(data, aos::FlatbufferToJson(json_builder.AsFlatbufferSpan(),
                                          {.multi_line = true}));
  }
  aos::FlatbufferSpan<reflection::Schema> test_schema_{TestTableSchema()};
  aos::FlatbufferSpan<reflection::Schema> interesting_schemas_{
      UnsupportedSchema()};
};

// Test that compiles the same code that is used by an example in
// //aos/documentation/aos/docs/flatbuffers.md.
TEST_F(StaticFlatbuffersTest, DocumentationExample) {
  aos::fbs::VectorAllocator allocator;
  Builder<TestTableStatic> builder(&allocator);
  TestTableStatic *object = builder.get();
  object->set_scalar(123);
  {
    auto vector = object->add_vector_of_scalars();
    CHECK(vector->emplace_back(4));
    CHECK(vector->emplace_back(5));
  }
  {
    auto string = object->add_string();
    string->SetString("Hello, World!");
  }
  {
    auto vector_of_strings = object->add_vector_of_strings();
    auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
    CHECK(sub_string->emplace_back('D'));
  }
  { object->set_substruct({971, 254}); }
  {
    auto subtable = object->add_subtable();
    subtable->set_foo(1234);
  }
  {
    auto vector = object->add_vector_of_structs();
    CHECK(vector->emplace_back({48, 67}));
    CHECK(vector->emplace_back({118, 148}));
    CHECK(vector->emplace_back({971, 973}));
    // Max vector size is three; this should fail.
    CHECK(!vector->emplace_back({1114, 2056}));
  }
  {
    auto vector = object->add_vector_of_tables();
    auto subobject = vector->emplace_back();
    subobject->set_foo(222);
  }
  {
    auto subtable = object->add_included_table();
    subtable->set_foo(included::TestEnum::B);
  }
  ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
  LOG(INFO) << aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                     {.multi_line = true});
  LOG(INFO) << AnnotateBinaries(test_schema_, builder.buffer());
}

// Test that compiles the same code that is used by an example in
// //aos/documentation/aos/docs/flatbuffers.md showing how to convert a
// Populate*() method for adding a subtable to a flatbuffer.
namespace {
flatbuffers::Offset<SubTable> PopulateOld(flatbuffers::FlatBufferBuilder *fbb) {
  SubTable::Builder builder(*fbb);
  builder.add_foo(1234);
  return builder.Finish();
}
void PopulateStatic(SubTableStatic *subtable) { subtable->set_foo(1234); }
}  // namespace
TEST_F(StaticFlatbuffersTest, PopulateMethodConversionExample) {
  // Using a FlatBufferBuilder:
  flatbuffers::FlatBufferBuilder fbb;
  // Note: the PopulateOld() *must* be called prior to creating the builder.
  const flatbuffers::Offset<SubTable> subtable_offset = PopulateOld(&fbb);
  TestTable::Builder testtable_builder(fbb);
  testtable_builder.add_subtable(subtable_offset);
  fbb.Finish(testtable_builder.Finish());
  aos::FlatbufferDetachedBuffer<TestTable> fbb_finished = fbb.Release();

  // Using the static flatbuffer API.
  aos::fbs::VectorAllocator allocator;
  Builder<TestTableStatic> static_builder(&allocator);
  PopulateStatic(CHECK_NOTNULL(static_builder.get()->add_subtable()));

  // And confirm that they both contain the expected flatbuffer:
  const std::string expected = R"json({ "subtable": { "foo": 1234 } })json";
  EXPECT_EQ(expected, aos::FlatbufferToJson(fbb_finished));
  EXPECT_EQ(expected, aos::FlatbufferToJson(static_builder.AsFlatbufferSpan()));
}

TEST_F(StaticFlatbuffersTest, UnsupportedSchema) {
  const reflection::Schema *schema = &interesting_schemas_.message();
  EXPECT_DEATH(
      GenerateCodeForObject(
          schema, GetObjectByName(schema, "aos.fbs.testing.TableWithUnion")),
      "Union not supported");
  GenerateCodeForObject(
      schema, GetObjectByName(schema, "aos.fbs.testing.MissingVectorLength"));
  EXPECT_DEATH(
      GenerateCodeForObject(
          schema,
          GetObjectByName(schema, "aos.fbs.testing.NonIntegerVectorLength")),
      "vector_badlength must specify a positive integer for the "
      "static_length attribute.");
  EXPECT_DEATH(GenerateCodeForObject(
                   schema, GetObjectByName(
                               schema, "aos.fbs.testing.NegativeVectorLength")),
               "Field vector_badlength must have a non-negative "
               "static_length.");
  GenerateCodeForObject(
      schema, GetObjectByName(schema, "aos.fbs.testing.ZeroVectorLength"));
  GenerateCodeForObject(
      schema, GetObjectByName(schema, "aos.fbs.testing.MissingStringLength"));
  GenerateCodeForObject(
      schema,
      GetObjectByName(schema, "aos.fbs.testing.MissingSubStringLength"));
}

// Tests that we can go through and manually build up a big flatbuffer and that
// it stays valid at all points.
TEST_F(StaticFlatbuffersTest, ManuallyConstructFlatbuffer) {
  {
    aos::fbs::VectorAllocator allocator;
    Builder<SubTableStatic> builder(&allocator);
    SubTableStatic *object = builder.get();
    if (!builder.AsFlatbufferSpan().Verify()) {
      LOG(ERROR) << object->SerializationDebugString() << "\nRoot table offset "
                 << *reinterpret_cast<const uoffset_t *>(
                        builder.buffer().data())
                 << "\nraw bytes\n";
      aos::fbs::internal::DebugBytes(builder.buffer(), std::cerr);
      FAIL();
      return;
    }
    EXPECT_EQ("{  }", aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    object->set_foo(123);
    object->set_baz(971);
    CHECK(builder.AsFlatbufferSpan().Verify());
    EXPECT_EQ(123, object->AsFlatbuffer().foo());
    EXPECT_EQ(971, object->AsFlatbuffer().baz());
    EXPECT_EQ(R"json({ "foo": 123, "baz": 971 })json",
              aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    TestMemory(builder.buffer());
  }
  {
    // aos::FixedAllocator
    // allocator(TestTableStatic::kUnalignedBufferSize);
    aos::fbs::VectorAllocator allocator;
    Builder<TestTableStatic> builder(&allocator);
    TestTableStatic *object = builder.get();
    const aos::fbs::testing::TestTable &fbs = object->AsFlatbuffer();
    VLOG(1) << object->SerializationDebugString();
    CHECK(builder.AsFlatbufferSpan().Verify());
    EXPECT_EQ("{  }", aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    {
      ASSERT_FALSE(object->has_scalar());
      object->set_scalar(123);
      EXPECT_TRUE(fbs.has_scalar());
      EXPECT_EQ(123, fbs.scalar());
    }
    EXPECT_EQ(R"json({ "scalar": 123 })json",
              aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    {
      ASSERT_FALSE(object->has_vector_of_scalars());
      auto vector = object->add_vector_of_scalars();
      ASSERT_TRUE(vector->emplace_back(4));
      ASSERT_TRUE(vector->emplace_back(5));
      ASSERT_TRUE(object->has_vector_of_scalars());
      ASSERT_TRUE(fbs.has_vector_of_scalars());
      VLOG(1) << vector->SerializationDebugString();
      EXPECT_TRUE(fbs.has_vector_of_scalars());
      EXPECT_EQ(2u, fbs.vector_of_scalars()->size());
      EXPECT_EQ(4, fbs.vector_of_scalars()->Get(0));
      EXPECT_EQ(5, fbs.vector_of_scalars()->Get(1));
    }
    EXPECT_EQ(R"json({ "scalar": 123, "vector_of_scalars": [ 4, 5 ] })json",
              aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    {
      EXPECT_FALSE(object->has_string());
      auto string = object->add_string();
      EXPECT_TRUE(object->has_string());
      string->SetString("Hello, World!");
      EXPECT_EQ(13u, object->string()->size());
      ASSERT_TRUE(fbs.has_string());
      ASSERT_EQ(13u, fbs.string()->size());
      EXPECT_EQ("Hello, World!", fbs.string()->string_view());
      // Check that we null-terminated correctly.
      EXPECT_EQ(13u, strnlen(fbs.string()->c_str(), 20));
    }
    EXPECT_EQ(
        R"json({ "scalar": 123, "vector_of_scalars": [ 4, 5 ], "string": "Hello, World!" })json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
    {
      EXPECT_FALSE(object->has_vector_of_strings());
      auto vector_of_strings = object->add_vector_of_strings();
      EXPECT_TRUE(object->has_vector_of_strings());
      auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
      ASSERT_TRUE(sub_string->emplace_back('D'));
      EXPECT_TRUE(fbs.has_vector_of_strings());
      ASSERT_EQ(1u, fbs.vector_of_strings()->size());
      ASSERT_EQ(1u, fbs.vector_of_strings()->Get(0)->size());
      EXPECT_EQ('D', fbs.vector_of_strings()->Get(0)->Get(0));
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ]
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      EXPECT_FALSE(object->has_substruct());
      object->set_substruct({971, 254});
      EXPECT_TRUE(object->has_substruct());
      EXPECT_TRUE(fbs.has_substruct());
      EXPECT_EQ(971, fbs.substruct()->x());
      EXPECT_EQ(254, fbs.substruct()->y());
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      auto subtable = object->add_subtable();
      subtable->set_foo(1234);
      EXPECT_TRUE(fbs.has_subtable());
      EXPECT_EQ(1234, fbs.subtable()->foo());
      EXPECT_FALSE(fbs.subtable()->has_baz());
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      auto vector = object->add_vector_of_structs();
      ASSERT_TRUE(vector->emplace_back({48, 67}));
      ASSERT_TRUE(vector->emplace_back({118, 148}));
      ASSERT_TRUE(vector->emplace_back({971, 973}));
      ASSERT_FALSE(vector->emplace_back({1114, 2056}));
      EXPECT_TRUE(fbs.has_vector_of_structs());
      EXPECT_EQ(3u, fbs.vector_of_structs()->size());
      EXPECT_EQ(48, fbs.vector_of_structs()->Get(0)->x());
      EXPECT_EQ(67, fbs.vector_of_structs()->Get(0)->y());
      EXPECT_EQ(118, fbs.vector_of_structs()->Get(1)->x());
      EXPECT_EQ(object->vector_of_structs()->at(1).x(),
                fbs.vector_of_structs()->Get(1)->x());
      EXPECT_EQ((*object->vector_of_structs())[1].x(),
                fbs.vector_of_structs()->Get(1)->x());
      EXPECT_EQ(148, fbs.vector_of_structs()->Get(1)->y());
      EXPECT_EQ(971, fbs.vector_of_structs()->Get(2)->x());
      EXPECT_EQ(973, fbs.vector_of_structs()->Get(2)->y());
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 },
 "vector_of_structs": [
  {
   "x": 48,
   "y": 67
  },
  {
   "x": 118,
   "y": 148
  },
  {
   "x": 971,
   "y": 973
  }
 ]
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      EXPECT_FALSE(object->has_vector_of_tables());
      auto vector = object->add_vector_of_tables();
      EXPECT_TRUE(object->has_vector_of_tables());
      auto subobject = vector->emplace_back();
      subobject->set_foo(222);
      EXPECT_TRUE(fbs.has_vector_of_tables());
      EXPECT_EQ(1u, fbs.vector_of_tables()->size());
      EXPECT_EQ(222, fbs.vector_of_tables()->Get(0)->foo());
      EXPECT_EQ(object->vector_of_tables()->at(0).foo(),
                fbs.vector_of_tables()->Get(0)->foo());
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 },
 "vector_of_structs": [
  {
   "x": 48,
   "y": 67
  },
  {
   "x": 118,
   "y": 148
  },
  {
   "x": 971,
   "y": 973
  }
 ],
 "vector_of_tables": [
  {
   "foo": 222
  }
 ]
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      EXPECT_FALSE(object->has_included_table());
      auto subtable = object->add_included_table();
      EXPECT_TRUE(object->has_included_table());
      subtable->set_foo(included::TestEnum::B);
      ASSERT_TRUE(fbs.has_included_table());
      ASSERT_TRUE(fbs.included_table()->has_foo());
      EXPECT_EQ(included::TestEnum::B, fbs.included_table()->foo());
    }
    EXPECT_EQ(
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 },
 "vector_of_structs": [
  {
   "x": 48,
   "y": 67
  },
  {
   "x": 118,
   "y": 148
  },
  {
   "x": 971,
   "y": 973
  }
 ],
 "vector_of_tables": [
  {
   "foo": 222
  }
 ],
 "included_table": {
  "foo": "B"
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    {
      auto aligned_vector = object->add_vector_aligned();
      ASSERT_EQ(64,
                std::remove_reference<decltype(*aligned_vector)>::type::kAlign);
      ASSERT_EQ(64, TestTableStatic::kAlign);
      ASSERT_TRUE(aligned_vector->emplace_back(444));
      EXPECT_TRUE(fbs.has_vector_aligned());
      EXPECT_EQ(1u, fbs.vector_aligned()->size());
      EXPECT_EQ(0u,
                reinterpret_cast<size_t>(fbs.vector_aligned()->data()) % 64);
      EXPECT_EQ(444, fbs.vector_aligned()->Get(0));
    }
    VLOG(1) << object->SerializationDebugString();
    CHECK(builder.AsFlatbufferSpan().Verify());
    const std::string expected_contents =
        R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 },
 "vector_aligned": [
  444
 ],
 "vector_of_structs": [
  {
   "x": 48,
   "y": 67
  },
  {
   "x": 118,
   "y": 148
  },
  {
   "x": 971,
   "y": 973
  }
 ],
 "vector_of_tables": [
  {
   "foo": 222
  }
 ],
 "included_table": {
  "foo": "B"
 }
})json";
    EXPECT_EQ(expected_contents,
              aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                    {.multi_line = true}));
    VLOG(1) << AnnotateBinaries(test_schema_, builder.buffer());
    VerifyJson<TestTableStatic>(expected_contents);
    {
      auto aligned_vector = object->mutable_vector_aligned();
      ASSERT_TRUE(aligned_vector->reserve(100));
      EXPECT_EQ(100, aligned_vector->capacity());
      ASSERT_TRUE(builder.AsFlatbufferSpan().Verify())
          << aligned_vector->SerializationDebugString();
      EXPECT_EQ(expected_contents,
                aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                      {.multi_line = true}));
      std::vector<int> scalars;
      scalars.push_back(aligned_vector->at(0));
      while (aligned_vector->size() < 100u) {
        scalars.push_back(aligned_vector->size());
        CHECK(aligned_vector->emplace_back(aligned_vector->size()));
      }
      VLOG(1) << aligned_vector->SerializationDebugString();
      VLOG(1) << AnnotateBinaries(test_schema_, builder.buffer());
      EXPECT_EQ(absl::StrFormat(
                    R"json({
 "scalar": 123,
 "vector_of_scalars": [
  4,
  5
 ],
 "string": "Hello, World!",
 "vector_of_strings": [
  "D"
 ],
 "substruct": {
  "x": 971,
  "y": 254
 },
 "subtable": {
  "foo": 1234
 },
 "vector_aligned": [
  %s
 ],
 "vector_of_structs": [
  {
   "x": 48,
   "y": 67
  },
  {
   "x": 118,
   "y": 148
  },
  {
   "x": 971,
   "y": 973
  }
 ],
 "vector_of_tables": [
  {
   "foo": 222
  }
 ],
 "included_table": {
  "foo": "B"
 }
})json",
                    absl::StrJoin(scalars, ",\n  ")),
                aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                      {.multi_line = true}));
    }

    {
      auto unspecified_vector = object->add_unspecified_length_vector();
      ASSERT_NE(nullptr, unspecified_vector);
      ASSERT_EQ(0, unspecified_vector->capacity());
      ASSERT_FALSE(unspecified_vector->emplace_back(0));
      ASSERT_TRUE(unspecified_vector->reserve(2));
      ASSERT_TRUE(unspecified_vector->emplace_back(1));
      ASSERT_TRUE(unspecified_vector->emplace_back(2));
      ASSERT_FALSE(unspecified_vector->emplace_back(3));
      ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
    }
    TestMemory(builder.buffer());
  }
}

// Tests that field clearing (and subsequent resetting) works properly.
TEST_F(StaticFlatbuffersTest, ClearFields) {
  aos::fbs::VectorAllocator allocator;
  Builder<TestTableStatic> builder(&allocator);
  TestTableStatic *object = builder.get();
  // For each field, we will confirm the following:
  // * Clearing a non-existent field causes no issues.
  // * We can set a field, clear it, and have it not be present.
  // * We can set the field again afterwards.
  {
    object->clear_scalar();
    ASSERT_TRUE(builder.Verify());
    object->set_scalar(123);
    EXPECT_EQ(123, object->AsFlatbuffer().scalar());
    object->clear_scalar();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_scalar());
    object->set_scalar(456);
    EXPECT_EQ(456, object->AsFlatbuffer().scalar());
  }
  {
    object->clear_vector_of_scalars();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_vector_of_scalars());
    auto vector = object->add_vector_of_scalars();
    ASSERT_TRUE(vector->emplace_back(4));
    ASSERT_TRUE(vector->emplace_back(5));
    ASSERT_TRUE(vector->emplace_back(6));
    // Deliberately force a resize of the vector to ensure that we can exercise
    // what happens if we clear a non-standard size field.
    ASSERT_FALSE(vector->emplace_back(7));
    ASSERT_TRUE(vector->reserve(4));
    ASSERT_TRUE(vector->emplace_back(7));
    EXPECT_EQ(
        R"json({
 "scalar": 456,
 "vector_of_scalars": [
  4,
  5,
  6,
  7
 ]
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    ASSERT_TRUE(builder.Verify());
    object->clear_vector_of_scalars();
    ASSERT_TRUE(builder.Verify());
    ASSERT_FALSE(object->has_vector_of_scalars())
        << aos::FlatbufferToJson(builder.AsFlatbufferSpan());
    vector = CHECK_NOTNULL(object->add_vector_of_scalars());
    ASSERT_TRUE(builder.Verify());
    EXPECT_EQ(0u, object->AsFlatbuffer().vector_of_scalars()->size());
    ASSERT_TRUE(vector->emplace_back(9));
    ASSERT_TRUE(vector->emplace_back(7));
    ASSERT_TRUE(vector->emplace_back(1));
    // This vector has no knowledge of the past resizing; it should fail to add
    // an extra number.
    ASSERT_FALSE(vector->emplace_back(7));
  }
  {
    object->clear_substruct();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_substruct());
    object->set_substruct(SubStruct{2, 3});
    EXPECT_EQ(
        R"json({
 "scalar": 456,
 "vector_of_scalars": [
  9,
  7,
  1
 ],
 "substruct": {
  "x": 2,
  "y": 3
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    object->clear_substruct();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_substruct());
    object->set_substruct(SubStruct{4, 5});
    EXPECT_EQ(
        R"json({
 "scalar": 456,
 "vector_of_scalars": [
  9,
  7,
  1
 ],
 "substruct": {
  "x": 4,
  "y": 5
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
  }
  {
    object->clear_subtable();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_subtable());
    auto subtable = CHECK_NOTNULL(object->add_subtable());
    subtable->set_baz(9.71);
    EXPECT_EQ(
        R"json({
 "scalar": 456,
 "vector_of_scalars": [
  9,
  7,
  1
 ],
 "substruct": {
  "x": 4,
  "y": 5
 },
 "subtable": {
  "baz": 9.71
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
    object->clear_subtable();
    ASSERT_TRUE(builder.Verify());
    EXPECT_FALSE(object->has_subtable());
    subtable = CHECK_NOTNULL(object->add_subtable());
    subtable->set_baz(16.78);
    EXPECT_EQ(
        R"json({
 "scalar": 456,
 "vector_of_scalars": [
  9,
  7,
  1
 ],
 "substruct": {
  "x": 4,
  "y": 5
 },
 "subtable": {
  "baz": 16.78
 }
})json",
        aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                              {.multi_line = true}));
  }
  TestMemory(builder.buffer());
}

// Try to cover ~all supported scalar/flatbuffer types using JSON convenience
// functions.
TEST_F(StaticFlatbuffersTest, FlatbufferTypeCoverage) {
  VerifyJson<aos::testing::ConfigurationStatic>("{\n\n}");
  std::string populated_config =
      aos::util::ReadFileToStringOrDie(aos::testing::ArtifactPath(
          "aos/flatbuffers/test_dir/type_coverage.json"));
  // Get rid of a pesky new line.
  populated_config = populated_config.substr(0, populated_config.size() - 1);
  VerifyJson<aos::testing::ConfigurationStatic>(populated_config);

  // And now play around with mutating the buffer.
  Builder<aos::testing::ConfigurationStatic> builder =
      aos::JsonToStaticFlatbuffer<aos::testing::ConfigurationStatic>(
          populated_config);
  ASSERT_TRUE(builder.Verify());
  builder.get()->clear_foo_float();
  ASSERT_TRUE(builder.Verify());
  ASSERT_FALSE(builder.get()->AsFlatbuffer().has_foo_float());
  builder.get()->set_foo_float(1.111);
  ASSERT_TRUE(builder.Verify());
  ASSERT_FLOAT_EQ(1.111, builder.get()->AsFlatbuffer().foo_float());
  TestMemory(builder.buffer());
}

TEST_F(StaticFlatbuffersTest, MinimallyAlignedTable) {
  VerifyJson<MinimallyAlignedTableStatic>("{\n \"field\": 123\n}");
  static_assert(4u == alignof(uoffset_t),
                "The alignment of a uoffset_t is expected to be 4.");
  ASSERT_EQ(alignof(uoffset_t), MinimallyAlignedTableStatic::kAlign)
      << "No table should have an alignment of less than the alignment of the "
         "table's root offset.";
}

// Confirm that we can use the SpanAllocator with a span that provides exactly
// the required buffer size.
TEST_F(StaticFlatbuffersTest, ExactSizeSpanAllocator) {
  uint8_t buffer[Builder<TestTableStatic>::kBufferSize];
  aos::fbs::SpanAllocator allocator({buffer, sizeof(buffer)});
  Builder<TestTableStatic> builder(&allocator);
  TestTableStatic *object = builder.get();
  object->set_scalar(123);
  {
    auto vector = object->add_vector_of_scalars();
    ASSERT_TRUE(vector->emplace_back(4));
    ASSERT_TRUE(vector->emplace_back(5));
  }
  {
    auto string = object->add_string();
    string->SetString("Hello, World!");
  }
  {
    auto vector_of_strings = object->add_vector_of_strings();
    auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
    ASSERT_TRUE(sub_string->emplace_back('D'));
  }
  { object->set_substruct({971, 254}); }
  {
    auto subtable = object->add_subtable();
    subtable->set_foo(1234);
  }
  {
    auto vector = object->add_vector_of_structs();
    ASSERT_TRUE(vector->emplace_back({48, 67}));
    ASSERT_TRUE(vector->emplace_back({118, 148}));
    ASSERT_TRUE(vector->emplace_back({971, 973}));
    // Max vector size is three; this should fail.
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
    // We don't have any extra space available.
    ASSERT_FALSE(vector->reserve(4));
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
  }
  {
    auto vector = object->add_vector_of_tables();
    auto subobject = vector->emplace_back();
    subobject->set_foo(222);
  }
  {
    auto subtable = object->add_included_table();
    subtable->set_foo(included::TestEnum::B);
  }
  ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
  VLOG(1) << aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                   {.multi_line = true});
  VLOG(1) << AnnotateBinaries(test_schema_, builder.buffer());
  TestMemory(builder.buffer());
}

// Test that when we provide too small of a span to the Builder that it
// correctly fails.
TEST_F(StaticFlatbuffersTest, TooSmallSpanAllocator) {
  std::vector<uint8_t> buffer;
  buffer.resize(10, 0);
  aos::fbs::SpanAllocator allocator({buffer.data(), buffer.size()});
  EXPECT_DEATH(Builder<TestTableStatic>{&allocator}, "Failed to allocate");
}

// Verify that if we create a span with extra headroom that that lets us
// dynamically alter the size of vectors in the flatbuffers.
TEST_F(StaticFlatbuffersTest, ExtraLargeSpanAllocator) {
  uint8_t buffer[Builder<TestTableStatic>::kBufferSize + 10000];
  aos::fbs::SpanAllocator allocator({buffer, sizeof(buffer)});
  Builder<TestTableStatic> builder(&allocator);
  TestTableStatic *object = builder.get();
  {
    auto vector = object->add_unspecified_length_vector();
    // Confirm that the vector does indeed start out at zero length.
    ASSERT_FALSE(vector->emplace_back(4));
    ASSERT_TRUE(vector->reserve(9000));
    vector->resize(256);
    for (size_t index = 0; index < 256; ++index) {
      vector->at(index) = static_cast<uint8_t>(index);
    }
  }
  ASSERT_EQ(256, object->AsFlatbuffer().unspecified_length_vector()->size());
  size_t expected = 0;
  for (const uint8_t value :
       *object->AsFlatbuffer().unspecified_length_vector()) {
    EXPECT_EQ(expected++, value);
  }
  expected = 0;
  for (const uint8_t value : *object->unspecified_length_vector()) {
    EXPECT_EQ(expected++, value);
  }
  TestMemory(builder.buffer());
}

// Tests that the iterators on the Vector type work.
TEST_F(StaticFlatbuffersTest, IteratorTest) {
  Builder<TestTableStatic> builder(std::make_unique<VectorAllocator>());
  {
    auto vector = builder->add_unspecified_length_vector();
    ASSERT_TRUE(vector->reserve(9000));
    vector->resize(256);
    uint8_t set_value = 0;
    for (uint8_t &destination : *vector) {
      destination = set_value;
      ++set_value;
    }
    uint8_t expected = 0;
    for (const uint8_t value : *builder->unspecified_length_vector()) {
      EXPECT_EQ(expected, value);
      ++expected;
    }
    // Exercise some of the random access iterator functionality to ensure that
    // we have it implemented.
    auto begin_it = vector->begin();
    EXPECT_EQ(begin_it + 256, vector->end());
    EXPECT_EQ(7, *(begin_it + 7));
    EXPECT_EQ(255, *(vector->end() - 1));
    EXPECT_EQ(256, vector->end() - vector->begin());
    EXPECT_EQ(-256, vector->begin() - vector->end());
    static_assert(std::random_access_iterator<decltype(vector->begin())>,
                  "The vector iterator does not meet the requirements of a "
                  "random access iterator.");
  }
  {
    auto vector = builder->add_vector_of_structs();
    vector->resize(3);
    double set_value = 0;
    for (SubStruct &destination : *vector) {
      destination.mutate_x(set_value);
      destination.mutate_y(-set_value);
      set_value += 1.0;
    }
    double expected = 0;
    for (const SubStruct &value : *builder->vector_of_structs()) {
      EXPECT_EQ(expected, value.x());
      EXPECT_EQ(-expected, value.y());
      expected += 1.0;
    }
    static_assert(std::random_access_iterator<decltype(vector->begin())>,
                  "The vector iterator does not meet the requirements of a "
                  "random access iterator.");
  }
  {
    auto vector = builder->add_vector_of_tables();
    vector->resize(3);
    int set_value = 0;
    for (SubTableStatic &destination : *vector) {
      destination.set_foo(set_value);
      set_value += 1;
    }
    int expected = 0;
    for (const SubTableStatic &value : *builder->vector_of_tables()) {
      EXPECT_EQ(expected, value.foo());
      EXPECT_FALSE(value.has_baz());
      expected += 1;
    }
    static_assert(std::random_access_iterator<decltype(vector->begin())>,
                  "The vector iterator does not meet the requirements of a "
                  "random access iterator.");
  }
}

// Confirm that we can use the FixedStackAllocator
TEST_F(StaticFlatbuffersTest, FixedStackAllocator) {
  aos::fbs::FixedStackAllocator<Builder<TestTableStatic>::kBufferSize>
      allocator;
  Builder<TestTableStatic> builder(&allocator);
  TestTableStatic *object = builder.get();
  object->set_scalar(123);
  {
    auto vector = object->add_vector_of_scalars();
    ASSERT_TRUE(vector->emplace_back(4));
    ASSERT_TRUE(vector->emplace_back(5));
  }
  {
    auto string = object->add_string();
    string->SetString("Hello, World!");
  }
  {
    auto vector_of_strings = object->add_vector_of_strings();
    auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
    ASSERT_TRUE(sub_string->emplace_back('D'));
  }
  { object->set_substruct({971, 254}); }
  {
    auto subtable = object->add_subtable();
    subtable->set_foo(1234);
  }
  {
    auto vector = object->add_vector_of_structs();
    ASSERT_TRUE(vector->emplace_back({48, 67}));
    ASSERT_TRUE(vector->emplace_back({118, 148}));
    ASSERT_TRUE(vector->emplace_back({971, 973}));
    // Max vector size is three; this should fail.
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
    // We don't have any extra space available.
    ASSERT_FALSE(vector->reserve(4));
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
  }
  {
    auto vector = object->add_vector_of_tables();
    auto subobject = vector->emplace_back();
    subobject->set_foo(222);
  }
  {
    auto subtable = object->add_included_table();
    subtable->set_foo(included::TestEnum::B);
  }
  ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
  VLOG(1) << aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                   {.multi_line = true});
  VLOG(1) << AnnotateBinaries(test_schema_, builder.buffer());
  TestMemory(builder.buffer());
}

// Uses a small example to manually verify that we can copy from the flatbuffer
// object API.
TEST_F(StaticFlatbuffersTest, ObjectApiCopy) {
  aos::fbs::testing::TestTableT object_t;
  object_t.scalar = 971;
  object_t.vector_of_strings.push_back("971");
  object_t.vector_of_structs.push_back({1, 2});
  object_t.subtable = std::make_unique<SubTableT>();
  aos::fbs::VectorAllocator allocator;
  Builder<TestTableStatic> builder(&allocator);
  ASSERT_TRUE(builder->FromFlatbuffer(object_t));
  ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
  // Note that vectors and strings get set to zero-length, but present, values.
  EXPECT_EQ(
      "{ \"scalar\": 971, \"vector_of_scalars\": [  ], \"string\": \"\", "
      "\"vector_of_strings\": [ \"971\" ], \"subtable\": { \"foo\": 0, "
      "\"baz\": 0 }, \"vector_aligned\": [  ], \"vector_of_structs\": [ { "
      "\"x\": 1, \"y\": 2 } ], \"vector_of_tables\": [  ], "
      "\"unspecified_length_vector\": [  ], \"unspecified_length_string\": "
      "\"\", \"unspecified_length_vector_of_strings\": [  ] }",
      aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
}

// More completely covers our object API copying by comparing the flatbuffer
// Pack() methods to our FromFlatbuffer() methods.
TEST_F(StaticFlatbuffersTest, FlatbufferObjectTypeCoverage) {
  VerifyJson<aos::testing::ConfigurationStatic>("{\n\n}");
  std::string populated_config =
      aos::util::ReadFileToStringOrDie(aos::testing::ArtifactPath(
          "aos/flatbuffers/test_dir/type_coverage.json"));
  Builder<aos::testing::ConfigurationStatic> json_builder =
      aos::JsonToStaticFlatbuffer<aos::testing::ConfigurationStatic>(
          populated_config);
  aos::testing::ConfigurationT object_t;
  json_builder->AsFlatbuffer().UnPackTo(&object_t);

  Builder<aos::testing::ConfigurationStatic> from_object_static;
  ASSERT_TRUE(from_object_static->FromFlatbuffer(object_t));
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(aos::testing::Configuration::Pack(fbb, &object_t));
  aos::FlatbufferDetachedBuffer<aos::testing::Configuration> from_object_raw =
      fbb.Release();
  EXPECT_EQ(aos::FlatbufferToJson(from_object_raw, {.multi_line = true}),
            aos::FlatbufferToJson(from_object_static, {.multi_line = true}));
}

// Tests that we can build code that uses the reflection types.
TEST_F(StaticFlatbuffersTest, IncludeReflectionTypes) {
  VerifyJson<::aos::testing::UseSchemaStatic>("{\n\n}");
}

// Tests that we can use the move constructor on a Builder.
TEST_F(StaticFlatbuffersTest, BuilderMoveConstructor) {
  uint8_t buffer[Builder<TestTableStatic>::kBufferSize];
  aos::fbs::SpanAllocator allocator({buffer, sizeof(buffer)});
  Builder<TestTableStatic> builder_from(&allocator);
  Builder<TestTableStatic> builder(std::move(builder_from));
  TestTableStatic *object = builder.get();
  object->set_scalar(123);
  {
    auto vector = object->add_vector_of_scalars();
    ASSERT_TRUE(vector->emplace_back(4));
    ASSERT_TRUE(vector->emplace_back(5));
  }
  {
    auto string = object->add_string();
    string->SetString("Hello, World!");
  }
  {
    auto vector_of_strings = object->add_vector_of_strings();
    auto sub_string = CHECK_NOTNULL(vector_of_strings->emplace_back());
    ASSERT_TRUE(sub_string->emplace_back('D'));
  }
  { object->set_substruct({971, 254}); }
  {
    auto subtable = object->add_subtable();
    subtable->set_foo(1234);
  }
  {
    auto vector = object->add_vector_of_structs();
    ASSERT_TRUE(vector->emplace_back({48, 67}));
    ASSERT_TRUE(vector->emplace_back({118, 148}));
    ASSERT_TRUE(vector->emplace_back({971, 973}));
    // Max vector size is three; this should fail.
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
    // We don't have any extra space available.
    ASSERT_FALSE(vector->reserve(4));
    ASSERT_FALSE(vector->emplace_back({1114, 2056}));
  }
  {
    auto vector = object->add_vector_of_tables();
    auto subobject = vector->emplace_back();
    subobject->set_foo(222);
  }
  {
    auto subtable = object->add_included_table();
    subtable->set_foo(included::TestEnum::B);
  }
  ASSERT_TRUE(builder.AsFlatbufferSpan().Verify());
  VLOG(1) << aos::FlatbufferToJson(builder.AsFlatbufferSpan(),
                                   {.multi_line = true});
  VLOG(1) << AnnotateBinaries(test_schema_, builder.buffer());
  TestMemory(builder.buffer());
}

}  // namespace aos::fbs::testing
