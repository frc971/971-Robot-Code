#include "aos/json_to_flatbuffer.h"
#include "aos/json_to_flatbuffer_generated.h"
#include "aos/util/file.h"
#include "flatbuffers/reflection.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

class FlatbufferIntrospectionTest : public ::testing::Test {
 public:
  FlatbufferIntrospectionTest()
      : schema_data_(
            util::ReadFileToStringOrDie("aos/json_to_flatbuffer.bfbs")) {
    schema_ = reflection::GetSchema(schema_data_.data());
  }

 protected:
  FlatbufferString<reflection::Schema> schema_data_;
  const reflection::Schema *schema_;
};

TEST_F(FlatbufferIntrospectionTest, IntegerTest) {
  flatbuffers::FlatBufferBuilder builder;
  ConfigurationBuilder config_builder(builder);

  config_builder.add_foo_byte(-5);
  config_builder.add_foo_ubyte(5);
  config_builder.add_foo_bool(true);

  config_builder.add_foo_short(-10);
  config_builder.add_foo_ushort(10);

  config_builder.add_foo_int(-20);
  config_builder.add_foo_uint(20);

  config_builder.add_foo_long(-100);
  config_builder.add_foo_ulong(100);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"foo_bool\": true, \"foo_byte\": -5, \"foo_int\": -20, "
            "\"foo_long\": -100, \"foo_short\": -10, \"foo_ubyte\": 5, "
            "\"foo_uint\": 20, \"foo_ulong\": 100, \"foo_ushort\": 10}");
}

TEST_F(FlatbufferIntrospectionTest, FloatTest) {
  flatbuffers::FlatBufferBuilder builder;
  ConfigurationBuilder config_builder(builder);

  config_builder.add_foo_float(1.0 / 3.0);
  config_builder.add_foo_double(5.0 / 9.0);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"foo_double\": 0.555555555555556, \"foo_float\": 0.333333}");
}

TEST_F(FlatbufferIntrospectionTest, VectorScalarTest) {
  flatbuffers::FlatBufferBuilder builder;

  // Flatbuffers don't like creating vectors simultaneously with table, so do
  // first.
  auto foo_bytes = builder.CreateVector<int8_t>({-3, -2, -1, 0, 1, 2, 3});
  auto foo_ubytes = builder.CreateVector<uint8_t>({0, 1, 2, 3, 4, 5, 6});
  auto foo_bools = builder.CreateVector<uint8_t>({true, false, true, false});

  auto foo_shorts =
      builder.CreateVector<int16_t>({-30, -20, -10, 0, 10, 20, 30});
  auto foo_ushorts =
      builder.CreateVector<uint16_t>({0, 10, 20, 30, 40, 50, 60});

  auto foo_ints =
      builder.CreateVector<int32_t>({-300, -200, -100, 0, 100, 200, 300});
  auto foo_uints =
      builder.CreateVector<uint32_t>({0, 100, 200, 300, 400, 500, 600});

  auto foo_longs =
      builder.CreateVector<int64_t>({-3000, -2000, -1000, 0, 1000, 2000, 3000});
  auto foo_ulongs =
      builder.CreateVector<uint64_t>({0, 1000, 2000, 3000, 4000, 5000, 6000});

  auto foo_floats =
      builder.CreateVector<float>({0.0, 1.0 / 9.0, 2.0 / 9.0, 3.0 / 9.0});
  auto foo_doubles =
      builder.CreateVector<double>({0, 1.0 / 9.0, 2.0 / 9.0, 3.0 / 9.0});

  ConfigurationBuilder config_builder(builder);
  config_builder.add_vector_foo_byte(foo_bytes);
  config_builder.add_vector_foo_ubyte(foo_ubytes);
  config_builder.add_vector_foo_bool(foo_bools);

  config_builder.add_vector_foo_short(foo_shorts);
  config_builder.add_vector_foo_ushort(foo_ushorts);

  config_builder.add_vector_foo_int(foo_ints);
  config_builder.add_vector_foo_uint(foo_uints);

  config_builder.add_vector_foo_long(foo_longs);
  config_builder.add_vector_foo_ulong(foo_ulongs);

  config_builder.add_vector_foo_float(foo_floats);
  config_builder.add_vector_foo_double(foo_doubles);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(
      out,
      "{\"vector_foo_bool\": [true, false, true, false], \"vector_foo_byte\": "
      "[-3, -2, -1, 0, 1, 2, 3], \"vector_foo_double\": [0, 0.111111111111111, "
      "0.222222222222222, 0.333333333333333], \"vector_foo_float\": [0, "
      "0.111111, 0.222222, 0.333333], \"vector_foo_int\": [-300, -200, -100, "
      "0, 100, 200, 300], \"vector_foo_long\": [-3000, -2000, -1000, 0, 1000, "
      "2000, 3000], \"vector_foo_short\": [-30, -20, -10, 0, 10, 20, 30], "
      "\"vector_foo_ubyte\": [0, 1, 2, 3, 4, 5, 6], \"vector_foo_uint\": [0, "
      "100, 200, 300, 400, 500, 600], \"vector_foo_ulong\": [0, 1000, 2000, "
      "3000, 4000, 5000, 6000], \"vector_foo_ushort\": [0, 10, 20, 30, 40, 50, "
      "60]}");
}

TEST_F(FlatbufferIntrospectionTest, StringTest) {
  flatbuffers::FlatBufferBuilder builder;

  auto foo_string = builder.CreateString("I <3 FlatBuffers!");

  ConfigurationBuilder config_builder(builder);
  config_builder.add_foo_string(foo_string);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out, "{\"foo_string\": \"I <3 FlatBuffers!\"}");
}

TEST_F(FlatbufferIntrospectionTest, EnumTest) {
  flatbuffers::FlatBufferBuilder builder;

  ConfigurationBuilder config_builder(builder);
  config_builder.add_foo_enum(BaseType::UShort);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out, "{\"foo_enum\": \"UShort\"}");
}

TEST_F(FlatbufferIntrospectionTest, VectorStringTest) {
  flatbuffers::FlatBufferBuilder builder;

  std::vector<std::vector<std::string>> words{
      {"abc", "acb"}, {"bac", "bca"}, {"cab", "cba"}};
  std::vector<flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>>
      strings;

  for (const auto &v : words) {
    strings.push_back(builder.CreateVectorOfStrings(v));
  }

  std::vector<flatbuffers::Offset<VectorOfStrings>> sub_vectors;

  for (const auto &v : strings) {
    VectorOfStringsBuilder v_builder(builder);
    v_builder.add_str(v);
    sub_vectors.push_back(v_builder.Finish());
  }

  auto foo_vov = builder.CreateVector(sub_vectors);

  VectorOfVectorOfStringBuilder vov_builder(builder);
  vov_builder.add_v(foo_vov);
  auto vov = vov_builder.Finish();

  ConfigurationBuilder config_builder(builder);
  config_builder.add_vector_foo_string(strings[0]);
  config_builder.add_vov(vov);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"vector_foo_string\": [\"abc\", \"acb\"], \"vov\": {\"v\": "
            "[{\"str\": [\"abc\", \"acb\"]}, {\"str\": [\"bac\", \"bca\"]}, "
            "{\"str\": [\"cab\", \"cba\"]}]}}");
}

TEST_F(FlatbufferIntrospectionTest, TableTest) {
  flatbuffers::FlatBufferBuilder builder;

  auto foo_string2 = builder.CreateString("Nested Config String");
  auto foo_bytes2 = builder.CreateVector<int8_t>({6, 7, 8, 9, 10});

  ConfigurationBuilder config_builder2(builder);
  config_builder2.add_foo_byte(10);
  config_builder2.add_foo_string(foo_string2);
  config_builder2.add_vector_foo_byte(foo_bytes2);

  flatbuffers::Offset<Configuration> config_2 = config_builder2.Finish();

  auto foo_string = builder.CreateString("Root Config String");
  auto foo_bytes = builder.CreateVector<int8_t>({0, 1, 2, 3, 4, 5});

  ConfigurationBuilder config_builder(builder);
  config_builder.add_nested_config(config_2);
  config_builder.add_foo_byte(5);
  config_builder.add_foo_string(foo_string);
  config_builder.add_vector_foo_byte(foo_bytes);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"foo_byte\": 5, \"foo_string\": \"Root Config String\", "
            "\"nested_config\": {\"foo_byte\": 10, \"foo_string\": \"Nested "
            "Config String\", \"vector_foo_byte\": [6, 7, 8, 9, 10]}, "
            "\"vector_foo_byte\": [0, 1, 2, 3, 4, 5]}");
}

TEST_F(FlatbufferIntrospectionTest, StructTest) {
  flatbuffers::FlatBufferBuilder builder;

  FooStructNested foo_struct2(10);

  FooStruct foo_struct(5, foo_struct2);

  ConfigurationBuilder config_builder(builder);
  config_builder.add_foo_struct(&foo_struct);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"foo_struct\": {\"foo_byte\": 5, \"nested_struct\": "
            "{\"foo_byte\": 10}}}");
}

TEST_F(FlatbufferIntrospectionTest, VectorStructTest) {
  flatbuffers::FlatBufferBuilder builder;

  FooStructNested foo_struct2(1);

  auto structs = builder.CreateVectorOfStructs(
      std::vector<FooStruct>({{5, foo_struct2}, {10, foo_struct2}}));

  ConfigurationBuilder config_builder(builder);
  config_builder.add_vector_foo_struct(structs);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out,
            "{\"vector_foo_struct\": [{\"foo_byte\": 5, \"nested_struct\": "
            "{\"foo_byte\": 1}}, {\"foo_byte\": 10, \"nested_struct\": "
            "{\"foo_byte\": 1}}]}");
}

TEST_F(FlatbufferIntrospectionTest, VectorEnumTest) {
  flatbuffers::FlatBufferBuilder builder;

  auto enums = builder.CreateVector<BaseType>(
      {BaseType::UShort, BaseType::Obj, BaseType::UInt});

  ConfigurationBuilder config_builder(builder);
  config_builder.add_vector_foo_enum(
      flatbuffers::Offset<flatbuffers::Vector<int8_t>>(enums.o));

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out, "{\"vector_foo_enum\": [\"UShort\", \"Obj\", \"UInt\"]}");
}

TEST_F(FlatbufferIntrospectionTest, StructEnumTest) {
  flatbuffers::FlatBufferBuilder builder;

  StructEnum foo_struct(BaseType::UShort);

  ConfigurationBuilder config_builder(builder);
  config_builder.add_foo_struct_enum(&foo_struct);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());

  EXPECT_EQ(out, "{\"foo_struct_enum\": {\"foo_enum\": \"UShort\"}}");
}

TEST_F(FlatbufferIntrospectionTest, StringEscapeTest) {
  flatbuffers::FlatBufferBuilder builder;

  auto foo_string = builder.CreateString("\"\\\b\f\n\r\t");

  ConfigurationBuilder config_builder(builder);
  config_builder.add_foo_string(foo_string);

  builder.Finish(config_builder.Finish());

  std::string out = FlatbufferToJson(schema_, builder.GetBufferPointer());
  EXPECT_EQ(out, "{\"foo_string\": \"\\\"\\\\\\b\\f\\n\\r\\t\"}");
}

}  // namespace testing
}  // namespace aos
