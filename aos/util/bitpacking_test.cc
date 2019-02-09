#include "aos/util/bitpacking.h"

#include <stdint.h>

#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Tests MaskOnes with small arguments.
TEST(MaskOnesTest, Small) {
  EXPECT_EQ(0u, MaskOnes<uint8_t>(0));
  EXPECT_EQ(0u, MaskOnes<uint64_t>(0));
  EXPECT_EQ(1u, MaskOnes<uint8_t>(1));
  EXPECT_EQ(1u, MaskOnes<uint64_t>(1));
}

// Tests MaskOnes with large arguments.
TEST(MaskOnesTest, Large) {
  EXPECT_EQ(0xFFu, MaskOnes<uint8_t>(8));
  EXPECT_EQ(0x7Fu, MaskOnes<uint8_t>(7));

  EXPECT_EQ(0xFFu, MaskOnes<uint64_t>(8));
  EXPECT_EQ(UINT64_C(0xFFFFFFFFFFFFFFFF), MaskOnes<uint64_t>(64));
  EXPECT_EQ(UINT64_C(0x7FFFFFFFFFFFFFFF), MaskOnes<uint64_t>(63));
}

// Tests some simple non-edge-case use cases for PackBits.
TEST(PackBitsTest, Basic) {
  {
    std::array<char, 3> buffer{};
    PackBits<uint8_t, 8, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 3>{}), buffer);
    PackBits<uint8_t, 8, 0>(9, buffer);
    EXPECT_EQ((std::array<char, 3>{{9, 0, 0}}), buffer);
    PackBits<uint8_t, 8, 8>(7, buffer);
    EXPECT_EQ((std::array<char, 3>{{9, 7, 0}}), buffer);
    PackBits<uint8_t, 8, 16>(1, buffer);
    EXPECT_EQ((std::array<char, 3>{{9, 7, 1}}), buffer);
  }
  {
    std::array<char, 3> buffer{};
    PackBits<uint16_t, 16, 0>(0xdead, buffer);
    EXPECT_EQ((std::array<char, 3>{
                  {static_cast<char>(0xad), static_cast<char>(0xde), 0}}),
              buffer);
  }
  {
    std::array<char, 3> buffer{};
    PackBits<uint8_t, 4, 0>(0xd7, buffer);
    EXPECT_EQ((std::array<char, 3>{{0x7, 0, 0}}), buffer);
  }
  {
    std::array<char, 3> buffer{};
    PackBits<uint8_t, 4, 4>(0xd7, buffer);
    EXPECT_EQ((std::array<char, 3>{{0x70, 0, 0}}), buffer);
  }
}

// Verifies that PackBits puts bits in an order consistent with increasing
// offsets.
TEST(PackBitsTest, Consistency) {
  {
    std::array<char, 3> buffer1{};
    PackBits<uint8_t, 8, 0>(0x80, buffer1);
    std::array<char, 3> buffer2{};
    PackBits<uint8_t, 1, 7>(0x1, buffer2);
    EXPECT_EQ(buffer1, buffer2);
  }
  {
    std::array<char, 1> buffer1{{static_cast<char>(0xFF)}};
    PackBits<uint8_t, 8, 0>(0x7F, buffer1);
    std::array<char, 1> buffer2{{static_cast<char>(0xFF)}};
    PackBits<uint8_t, 1, 7>(0x0, buffer2);
    EXPECT_EQ(buffer1, buffer2);
  }
  {
    std::array<char, 1> buffer1{};
    PackBits<uint8_t, 3, 5>(0x7, buffer1);
    std::array<char, 1> buffer2{};
    PackBits<uint8_t, 5, 3>(0x3C, buffer2);
    EXPECT_EQ(buffer1, buffer2);
  }
  {
    std::array<char, 1> buffer1{{static_cast<char>(0xFF)}};
    PackBits<uint8_t, 3, 5>(0x0, buffer1);
    std::array<char, 1> buffer2{{static_cast<char>(0xFF)}};
    PackBits<uint8_t, 5, 3>(0x03, buffer2);
    EXPECT_EQ(buffer1, buffer2);
  }
}

// Tests some simple non-edge-case use cases for UnpackBits.
TEST(UnpackBitsTest, Basic) {
  {
    std::array<char, 3> buffer{};
    EXPECT_EQ(0u, (UnpackBits<uint8_t, 8, 0>(buffer)));
    buffer = {{9, 0, 0}};
    EXPECT_EQ(9u, (UnpackBits<uint8_t, 8, 0>(buffer)));
    buffer = {{9, 7, 0}};
    EXPECT_EQ(9u, (UnpackBits<uint8_t, 8, 0>(buffer)));
    EXPECT_EQ(7u, (UnpackBits<uint8_t, 8, 8>(buffer)));
    buffer = {{9, 7, 1}};
    EXPECT_EQ(9u, (UnpackBits<uint8_t, 8, 0>(buffer)));
    EXPECT_EQ(7u, (UnpackBits<uint8_t, 8, 8>(buffer)));
    EXPECT_EQ(1u, (UnpackBits<uint8_t, 8, 16>(buffer)));
  }
  {
    const std::array<char, 3> buffer = {
        {static_cast<char>(0xad), static_cast<char>(0xde), 0}};
    EXPECT_EQ(0xdead, (UnpackBits<uint16_t, 16, 0>(buffer)));
  }
  {
    const std::array<char, 3> buffer = {{static_cast<char>(0xF7), 0, 0}};
    EXPECT_EQ(7u, (UnpackBits<uint8_t, 4, 0>(buffer)));
  }
  {
    const std::array<char, 3> buffer = {{static_cast<char>(0x7F), 0, 0}};
    EXPECT_EQ(7u, (UnpackBits<uint8_t, 4, 4>(buffer)));
  }
}

// Tests PackBits split across multiple bytes.
TEST(PackBitsTest, AcrossBytes) {
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 7>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x80), static_cast<char>(0x7F)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 6>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xC0), static_cast<char>(0x3F)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 5>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xE0), static_cast<char>(0x1F)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 4>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xF0), static_cast<char>(0x0F)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 3>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xF8), static_cast<char>(0x07)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 2>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xFC), static_cast<char>(0x03)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{};
    PackBits<uint8_t, 8, 1>(0xFF, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xFE), static_cast<char>(0x01)}}),
              buffer);
  }
}

// Tests UnpackBits split across multiple bytes.
TEST(UnpackBitsTest, AcrossBytes) {
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0x80), static_cast<char>(0x7F)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 7>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xC0), static_cast<char>(0x3F)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 6>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xE0), static_cast<char>(0x1F)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 5>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xF0), static_cast<char>(0x0F)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 4>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xF8), static_cast<char>(0x07)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 3>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xFC), static_cast<char>(0x03)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 2>(buffer)));
  }
  {
    const std::array<char, 2> buffer = {
        {static_cast<char>(0xFE), static_cast<char>(0x01)}};
    EXPECT_EQ(0xFFu, (UnpackBits<uint8_t, 8, 1>(buffer)));
  }
}

// Verifies that PackBits avoids touching adjacent bits.
TEST(PackBitsTest, AdjacentBits) {
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint8_t, 1, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xFE), static_cast<char>(0xFF)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint8_t, 7, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x80), static_cast<char>(0xFF)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint8_t, 8, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x00), static_cast<char>(0xFF)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 9, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x00), static_cast<char>(0xFE)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 14, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x00), static_cast<char>(0xC0)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 15, 0>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x00), static_cast<char>(0x80)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 15, 1>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0x01), static_cast<char>(0x00)}}),
              buffer);
  }
  {
    std::array<char, 2> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 6, 8>(0, buffer);
    EXPECT_EQ((std::array<char, 2>{
                  {static_cast<char>(0xFF), static_cast<char>(0xC0)}}),
              buffer);
  }
  {
    std::array<char, 4> buffer{
        {static_cast<char>(0xFF), static_cast<char>(0xFF),
         static_cast<char>(0xFF), static_cast<char>(0xFF)}};
    PackBits<uint16_t, 6, 24>(0, buffer);
    EXPECT_EQ((std::array<char, 4>{
                  {static_cast<char>(0xFF), static_cast<char>(0xFF),
                   static_cast<char>(0xFF), static_cast<char>(0xC0)}}),
              buffer);
  }
}

// Tests FloatToIntLinear with values near or outside of its boundaries.
TEST(FloatToIntLinearTest, OutOfBounds) {
  EXPECT_EQ(0u, (FloatToIntLinear<1>(0.0f, 1.0f, 0.0f)));
  EXPECT_EQ(0u, (FloatToIntLinear<1>(0.0f, 1.0f, -0.1f)));
  EXPECT_EQ(0u, (FloatToIntLinear<1>(0.0f, 1.0f, -1.0f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 1.0f, 1.0f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 1.0f, 1.1f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 1.0f, 2.0f)));

  EXPECT_EQ(0u, (FloatToIntLinear<1>(0.0f, 4.0f, 0.0f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 4.0f, 4.0f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 4.0f, 10.0f)));

  EXPECT_EQ(0u, (FloatToIntLinear<3>(0.0f, 4.0f, 0.0f)));
  EXPECT_EQ(0u, (FloatToIntLinear<3>(0.0f, 4.0f, -100.0f)));
  EXPECT_EQ(7u, (FloatToIntLinear<3>(0.0f, 4.0f, 4.0f)));
  EXPECT_EQ(7u, (FloatToIntLinear<3>(0.0f, 4.0f, 4.01f)));

  EXPECT_EQ(0u, (FloatToIntLinear<3>(-3.0f, 5.0f, -3.0f)));
  EXPECT_EQ(0u, (FloatToIntLinear<3>(-3.0f, 5.0f, -3.1f)));
  EXPECT_EQ(7u, (FloatToIntLinear<3>(-3.0f, 5.0f, 5.0f)));
  EXPECT_EQ(7u, (FloatToIntLinear<3>(-3.0f, 5.0f, 5.1f)));
}

// Tests that FloatToIntLinear rounds correctly at the boundaries between output
// values.
TEST(FloatToIntLinearTest, Rounding) {
  EXPECT_EQ(0u, (FloatToIntLinear<1>(0.0f, 1.0f, 0.49f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(0.0f, 1.0f, 0.51f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(-1.0f, 0.0f, -0.49f)));
  EXPECT_EQ(0u, (FloatToIntLinear<1>(-1.0f, 0.0f, -0.51f)));
  EXPECT_EQ(1u, (FloatToIntLinear<1>(-1.0f, 1.0f, 0.01f)));
  EXPECT_EQ(0u, (FloatToIntLinear<1>(-1.0f, 1.0f, -0.01f)));

  EXPECT_EQ(0u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.124f)));
  EXPECT_EQ(1u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.126f)));
  EXPECT_EQ(1u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.249f)));
  EXPECT_EQ(2u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.251f)));
  EXPECT_EQ(2u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.374f)));
  EXPECT_EQ(3u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.376f)));
  EXPECT_EQ(3u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.499f)));
  EXPECT_EQ(4u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.501f)));
  EXPECT_EQ(4u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.624f)));
  EXPECT_EQ(5u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.626f)));
  EXPECT_EQ(5u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.749f)));
  EXPECT_EQ(6u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.751f)));
  EXPECT_EQ(6u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.874f)));
  EXPECT_EQ(7u, (FloatToIntLinear<3>(0.0f, 1.0f, 0.876f)));
}

// Tests IntToFloatLinear with values near or outside of its boundaries.
TEST(IntToFloatLinearTest, OutOfBounds) {
  EXPECT_EQ(0.25f, (IntToFloatLinear<1>(0.0f, 1.0f, 0)));
  EXPECT_EQ(0.75f, (IntToFloatLinear<1>(0.0f, 1.0f, 1)));
  EXPECT_EQ(0.75f, (IntToFloatLinear<1>(0.0f, 1.0f, 2)));
  EXPECT_EQ(0.75f, (IntToFloatLinear<1>(0.0f, 1.0f, 3)));

  EXPECT_EQ(1.0f, (IntToFloatLinear<1>(0.0f, 4.0f, 0)));
  EXPECT_EQ(3.0f, (IntToFloatLinear<1>(0.0f, 4.0f, 1)));

  EXPECT_EQ(0.0625f, (IntToFloatLinear<3>(0.0f, 1.0f, 0)));
  EXPECT_EQ(0.9375f, (IntToFloatLinear<3>(0.0f, 1.0f, 7)));
  EXPECT_EQ(0.9375f, (IntToFloatLinear<3>(0.0f, 1.0f, 8)));
}

// Tests IntToFloatLinear with some specific values which are easy to calculate
// by hand.
TEST(IntToFloatLinearTest, Values) {
  EXPECT_EQ(0.125f, (IntToFloatLinear<2>(0.0f, 1.0f, 0)));
  EXPECT_EQ(0.375f, (IntToFloatLinear<2>(0.0f, 1.0f, 1)));
  EXPECT_EQ(0.625f, (IntToFloatLinear<2>(0.0f, 1.0f, 2)));
  EXPECT_EQ(0.875f, (IntToFloatLinear<2>(0.0f, 1.0f, 3)));

  EXPECT_EQ(0.0625f, (IntToFloatLinear<3>(0.0f, 1.0f, 0)));
  EXPECT_EQ(0.1875f, (IntToFloatLinear<3>(0.0f, 1.0f, 1)));
  EXPECT_EQ(0.3125f, (IntToFloatLinear<3>(0.0f, 1.0f, 2)));
  EXPECT_EQ(0.4375f, (IntToFloatLinear<3>(0.0f, 1.0f, 3)));
  EXPECT_EQ(0.5625f, (IntToFloatLinear<3>(0.0f, 1.0f, 4)));
  EXPECT_EQ(0.6875f, (IntToFloatLinear<3>(0.0f, 1.0f, 5)));
  EXPECT_EQ(0.8125f, (IntToFloatLinear<3>(0.0f, 1.0f, 6)));
  EXPECT_EQ(0.9375f, (IntToFloatLinear<3>(0.0f, 1.0f, 7)));

  EXPECT_EQ(-0.875f, (IntToFloatLinear<2>(-1.0f, 0.0f, 0)));
  EXPECT_EQ(-0.625f, (IntToFloatLinear<2>(-1.0f, 0.0f, 1)));
  EXPECT_EQ(-0.375f, (IntToFloatLinear<2>(-1.0f, 0.0f, 2)));
  EXPECT_EQ(-0.125f, (IntToFloatLinear<2>(-1.0f, 0.0f, 3)));

  EXPECT_EQ(-0.75f, (IntToFloatLinear<2>(-1.0f, 1.0f, 0)));
  EXPECT_EQ(-0.25f, (IntToFloatLinear<2>(-1.0f, 1.0f, 1)));
  EXPECT_EQ(0.25f, (IntToFloatLinear<2>(-1.0f, 1.0f, 2)));
  EXPECT_EQ(0.75f, (IntToFloatLinear<2>(-1.0f, 1.0f, 3)));
}

}  // namespace testing
}  // namespace aos
