#ifndef _AOS_VIISON_BLOB_THRESHOLD_H_
#define _AOS_VIISON_BLOB_THRESHOLD_H_

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

// ThresholdFn should be a lambda.
template <typename ThresholdFn>
RangeImage DoThreshold(ImageFormat fmt, ThresholdFn &&fn) {
  std::vector<std::vector<ImageRange>> ranges;
  ranges.reserve(fmt.h);
  for (int y = 0; y < fmt.h; ++y) {
    bool p_score = false;
    int pstart = -1;
    std::vector<ImageRange> rngs;
    for (int x = 0; x < fmt.w; ++x) {
      if (fn(x, y) != p_score) {
        if (p_score) {
          rngs.emplace_back(ImageRange(pstart, x));
        } else {
          pstart = x;
        }
        p_score = !p_score;
      }
    }
    if (p_score) {
      rngs.emplace_back(ImageRange(pstart, fmt.w));
    }
    ranges.push_back(rngs);
  }
  return RangeImage(0, std::move(ranges));
}

// ThresholdFn should be a lambda.
template <typename ThresholdFn>
RangeImage DoThreshold(const ImagePtr &img, ThresholdFn &&fn) {
  return DoThreshold(img.fmt(),
                     [&](int x, int y) { return fn(img.get_px(x, y)); });
}

// YUYV image types:
inline RangeImage DoThresholdYUYV(ImageFormat fmt, const char *data,
                                  uint8_t value) {
  return DoThreshold(fmt, [&](int x, int y) {
    uint8_t v = data[y * fmt.w * 2 + x * 2];
    return v > value;
  });
}

}  // namespace vision
}  // namespace aos

#endif  //  _AOS_VIISON_BLOB_THRESHOLD_H_
