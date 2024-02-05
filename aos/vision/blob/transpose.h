#ifndef AOS_VISION_BLOB_TRANSPOSE_H_
#define AOS_VISION_BLOB_TRANSPOSE_H_

#include "aos/vision/blob/range_image.h"

namespace aos::vision {

RangeImage Transpose(const RangeImage &img);
inline std::vector<RangeImage> Transpose(const std::vector<RangeImage> &imgs) {
  std::vector<RangeImage> out;
  out.reserve(imgs.size());
  for (const auto &img : imgs) out.push_back(Transpose(img));
  return out;
}

}  // namespace aos::vision

#endif  // AOS_VISION_BLOB_TRANSPOSE_H_
