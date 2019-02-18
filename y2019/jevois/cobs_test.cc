#include "y2019/jevois/cobs.h"

#include "aos/testing/test_logging.h"
#include "gtest/gtest.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace jevois {

// Tests the size conversions for some known, simple values.
TEST(CobsMaxEncodedSizeTest, Simple) {
  EXPECT_EQ(0u, CobsMaxEncodedSize(0));
  EXPECT_EQ(2u, CobsMaxEncodedSize(1));
  EXPECT_EQ(3u, CobsMaxEncodedSize(2));

  EXPECT_EQ(254u, CobsMaxEncodedSize(253));
  EXPECT_EQ(255u, CobsMaxEncodedSize(254));
  EXPECT_EQ(257u, CobsMaxEncodedSize(255));
}

class CobsTest : public ::testing::Test {
 public:
  CobsTest() { aos::testing::EnableTestLogging(); }

  template <size_t min_buffer_size = 0, size_t number_elements>
  void EncodeAndDecode(const char (&input_data)[number_elements]) {
    static constexpr size_t input_size =
        (min_buffer_size > number_elements) ? min_buffer_size : number_elements;
    EncodeAndDecode<input_size>(gsl::span<const char>(input_data));
  }

  template <size_t max_decoded_size>
  void EncodeAndDecode(const gsl::span<const char> decoded_input) {
    std::array<char, CobsMaxEncodedSize(max_decoded_size)> encoded_buffer;
    const auto encoded =
        CobsEncode<max_decoded_size>(decoded_input, &encoded_buffer);
    ASSERT_LE(encoded.size(), encoded_buffer.size());
    ASSERT_EQ(encoded.data(), &encoded_buffer.front());

    std::array<char, max_decoded_size> decoded_buffer;
    const auto decoded = CobsDecode<max_decoded_size>(encoded, &decoded_buffer);
    ASSERT_LE(decoded.size(), decoded_buffer.size());
    ASSERT_EQ(decoded.data(), &decoded_buffer.front());
    ASSERT_EQ(decoded.size(), decoded_input.size());
    for (int i = 0; i < decoded.size(); ++i) {
      EXPECT_EQ(decoded[i], decoded_input[i]);
    }
  }
};

// Tests various small buffers.
TEST_F(CobsTest, Small) {
  EncodeAndDecode<1>(std::array<char, 0>{});
  EncodeAndDecode<5>(std::array<char, 0>{});
  {
    const char data[] = {0};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {1};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(254)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(255)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {0, 1};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {0, static_cast<char>(254)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {0, static_cast<char>(255)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {0, 1, static_cast<char>(254)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {0, 1, static_cast<char>(255)};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(254), 1};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(255), 1};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(254), 0};
    EncodeAndDecode(data);
  }
  {
    const char data[] = {static_cast<char>(255), 0};
    EncodeAndDecode(data);
  }
}

// Tests encoding arrays with approximately one full chunk. This exposes some
// corner cases in the binary format.
TEST_F(CobsTest, AroundOneChunk) {
  char data[256];
  for (size_t i = 0; i < sizeof(data); ++i) {
    data[i] = (i * 7) & 0xFF;
  }
  const gsl::span<char> data_span = data;
  for (int i = 253; i <= 256; ++i) {
    EncodeAndDecode<256>(data_span.subspan(0, i));
  }
  for (int i = 253; i <= 255; ++i) {
    EncodeAndDecode<255>(data_span.subspan(0, i));
  }
  for (int i = 253; i <= 254; ++i) {
    EncodeAndDecode<254>(data_span.subspan(0, i));
  }
  EncodeAndDecode<253>(data_span.subspan(0, 253));
}

// Tests parsing a few packets, one byte at a time.
TEST(CobsPacketizerTest, BasicSingleByte) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{1}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{2}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{3}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 3>{{1, 2, 3}}),
            packetizer.received_packet());
  packetizer.clear_received_packet();
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{5}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());

  ASSERT_FALSE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{9}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());
  packetizer.ParseData(std::array<char, 1>{{7}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 2>{{9, 7}}),
            packetizer.received_packet());
}

// Tests parsing a few packets, one span per packet.
TEST(CobsPacketizerTest, BasicSinglePacket) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{1, 2, 3}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 3>{{1, 2, 3}}),
            packetizer.received_packet());
  packetizer.clear_received_packet();
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{5}});
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());

  ASSERT_FALSE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 2>{{9, 7}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 2>{{9, 7}}),
            packetizer.received_packet());
}

// Tests parsing a few packets, one span per packet including its terminator.
TEST(CobsPacketizerTest, BasicSinglePacketWithTerminator) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 4>{{1, 2, 3, 0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 3>{{1, 2, 3}}),
            packetizer.received_packet());
  packetizer.clear_received_packet();
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 2>{{5, 0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());

  ASSERT_FALSE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{9, 7, 0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 2>{{9, 7}}),
            packetizer.received_packet());
}

// Tests parsing a packet in the same span as the previous terminator.
TEST(CobsPacketizerTest, OverlappingEnd) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{1, 2, 3}});
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 2>{{0, 5}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 3>{{1, 2, 3}}),
            packetizer.received_packet());
  packetizer.clear_received_packet();
  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());
}

// Tests parsing a packet in the same span as the previous terminator, including
// its terminator (so we skip the first packet).
TEST(CobsPacketizerTest, OverlappingTwoEnds) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{1, 2, 3}});
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{0, 5, 0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  // We skip the {{1, 2, 3}} packet (arbitrarily; either that packet or this one
  // has to be skipped).
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{5}}),
            packetizer.received_packet());
}

// Tests parsing a packet in the same span as the previous terminator, including
// its terminator (so we skip the first packet), and then starting another new
// packet.
TEST(CobsPacketizerTest, OverlappingTwoEndsAndPartial) {
  CobsPacketizer<5> packetizer;

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 3>{{1, 2, 3}});
  ASSERT_TRUE(packetizer.received_packet().empty());

  ASSERT_TRUE(packetizer.received_packet().empty());
  packetizer.ParseData(std::array<char, 4>{{0, 5, 0, 8}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  // We skip the {{5}} packet (arbitrarily; either that packet or this one has
  // to be skipped).
  EXPECT_EQ(gsl::span<const char>(std::array<char, 3>{{1, 2, 3}}),
            packetizer.received_packet());

  packetizer.ParseData(std::array<char, 1>{{0}});
  ASSERT_FALSE(packetizer.received_packet().empty());
  EXPECT_EQ(gsl::span<const char>(std::array<char, 1>{{8}}),
            packetizer.received_packet());
}

}  // namespace jevois
}  // namespace frc971
