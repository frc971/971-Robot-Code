#ifndef _AOS_VIISON_BLOB_THRESHOLD_H_
#define _AOS_VIISON_BLOB_THRESHOLD_H_

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

// ThresholdFn should be a lambda.
template <typename ThresholdFn>
RangeImage DoThreshold(const ImagePtr &img, ThresholdFn &&fn) {
  std::vector<std::vector<ImageRange>> ranges;
  ranges.reserve(img.fmt().h);
  for (int y = 0; y < img.fmt().h; ++y) {
    bool p_score = false;
    int pstart = -1;
    std::vector<ImageRange> rngs;
    for (int x = 0; x < img.fmt().w; ++x) {
      if (fn(img.get_px(x, y)) != p_score) {
        if (p_score) {
          rngs.emplace_back(ImageRange(pstart, x));
        } else {
          pstart = x;
        }
        p_score = !p_score;
      }
    }
    if (p_score) {
      rngs.emplace_back(ImageRange(pstart, img.fmt().w));
    }
    ranges.push_back(rngs);
  }
  return RangeImage(0, std::move(ranges));
}

}  // namespace vision
}  // namespace aos

#endif  //  _AOS_VIISON_BLOB_THRESHOLD_H_
