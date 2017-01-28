#include "aos/vision/blob/codec.h"

namespace aos {
namespace vision {

size_t CalculateSize(const BlobList &blob_list) {
  size_t count = Int16Codec::kSize;
  for (const auto &blob : blob_list) {
    count += 2 * Int16Codec::kSize;
    for (int i = 0; i < static_cast<int>(blob.ranges().size()); ++i) {
      count +=
          Int16Codec::kSize + 2 * Int16Codec::kSize * blob.ranges()[i].size();
    }
  }
  return count;
}

void SerializeBlob(const BlobList &blob_list, char *data) {
  data = Int16Codec::Write(data, static_cast<uint16_t>(blob_list.size()));
  for (const auto &blob : blob_list) {
    data = Int16Codec::Write(data, static_cast<uint16_t>(blob.min_y()));
    data = Int16Codec::Write(data, static_cast<uint16_t>(blob.ranges().size()));
    for (int i = 0; i < (int)blob.ranges().size(); ++i) {
      data = Int16Codec::Write(data,
                               static_cast<uint16_t>(blob.ranges()[i].size()));
      for (const auto &range : blob.ranges()[i]) {
        data = Int16Codec::Write(data, static_cast<uint16_t>(range.st));
        data = Int16Codec::Write(data, static_cast<uint16_t>(range.ed));
      }
    }
  }
}

const char *ParseBlobList(BlobList *blob_list, const char *data) {
  int num_items = Int16Codec::Read(&data);
  blob_list->clear();
  blob_list->reserve(num_items);
  for (int i = 0; i < num_items; ++i) {
    std::vector<std::vector<ImageRange>> ranges_image;
    int min_y = Int16Codec::Read(&data);
    int num_ranges = Int16Codec::Read(&data);
    ranges_image.reserve(num_ranges);
    for (int j = 0; j < num_ranges; ++j) {
      ranges_image.emplace_back();
      auto &ranges = ranges_image.back();
      int num_sub_ranges = Int16Codec::Read(&data);
      for (int k = 0; k < num_sub_ranges; ++k) {
        int st = Int16Codec::Read(&data);
        int ed = Int16Codec::Read(&data);
        ranges.emplace_back(st, ed);
      }
    }
    blob_list->emplace_back(min_y, std::move(ranges_image));
  }
  return data;
}

}  // namespace vision
}  // namespace aos
