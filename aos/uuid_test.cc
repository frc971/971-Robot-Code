#include "aos/uuid.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace aos {
namespace testing {

// Tests that random UUIDs are actually random, and we can convert them to a
// string.  Not very exhaustive, but it is a good smoke test.
TEST(UUIDTest, GetOne) {
  LOG(INFO) << UUID::Random();

  UUID r = UUID::Random();

  std::stringstream ss;
  ss << r;

  UUID r2 = UUID::FromString(ss.str());
  EXPECT_EQ(r2, r);

  EXPECT_NE(UUID::Random(), UUID::Random());
  EXPECT_NE(UUID::Random(), UUID::Zero());
  EXPECT_EQ(UUID::Zero(), UUID::Zero());
}

// Tests that converting to and from various formats produces the same UUID.
TEST(UUIDTest, FromStringOrSpan) {
  std::string_view str = "4b88ab00-556a-455b-a395-17d1a0c6f906";
  std::array<uint8_t, UUID::kDataSize> data = {
      0x4b, 0x88, 0xab, 0x00, 0x55, 0x6a, 0x45, 0x5b,
      0xa3, 0x95, 0x17, 0xd1, 0xa0, 0xc6, 0xf9, 0x06};

  const UUID u_str = UUID::FromString(str);
  const UUID u_span = UUID::FromSpan({data.data(), data.size()});

  EXPECT_EQ(u_str.span(), absl::Span<uint8_t>(data.data(), data.size()));
  EXPECT_EQ(u_span.span(), absl::Span<uint8_t>(data.data(), data.size()));
  EXPECT_EQ(u_str.ToString(), str);
  EXPECT_EQ(u_span.ToString(), str);

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
      fbb.CreateVector(data.data(), data.size());

  const flatbuffers::Vector<uint8_t> *data_vector =
      flatbuffers::GetTemporaryPointer(fbb, data_offset);

  const UUID u2 = UUID::FromVector(data_vector);

  EXPECT_EQ(u_str, u2);
  EXPECT_EQ(u_span, u2);
}

}  // namespace testing
}  // namespace aos
