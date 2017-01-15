#include "aos/vision/blob/codec.h"

#include <algorithm>
#include "gtest/gtest.h"

namespace aos {
namespace vision {

TEST(CodecTest, WriteRead) {
  BlobList blobl;
  {
    std::vector<std::vector<ImageRange>> ranges;
    ranges.emplace_back(std::vector<ImageRange>{{10, 11}});
    ranges.emplace_back(std::vector<ImageRange>{{15, 17}});
    ranges.emplace_back(std::vector<ImageRange>{{19, 30}});
    blobl.emplace_back(RangeImage(10, std::move(ranges)));
  }

  {
    std::vector<std::vector<ImageRange>> ranges;
    ranges.emplace_back(std::vector<ImageRange>{{18, 19}});
    ranges.emplace_back(std::vector<ImageRange>{{12, 13}});
    ranges.emplace_back(std::vector<ImageRange>{{12, 17}});
    blobl.emplace_back(RangeImage(13, std::move(ranges)));
  }

  std::string out;
  SerializeBlobTo(blobl, &out);
  BlobList blobl2;
  size_t real_len = ParseBlobList(&blobl2, out.data()) - out.data();
  EXPECT_EQ(ShortDebugPrint(blobl), ShortDebugPrint(blobl2));

  EXPECT_EQ(real_len, CalculateSize(blobl));
}

}  // namespace vision
}  // namespace aos
