#ifndef AOS_VISION_BLOB_MOVE_SCALE_H_
#define AOS_VISION_BLOB_MOVE_SCALE_H_

#include <limits>
#include <vector>

#include "aos/vision/blob/range_image.h"
#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

// Sums img into bbox. bbox is constructed empty and grows with each call
// to GetBBox.
void GetBBox(const RangeImage &img, ImageBBox *bbox);
inline void GetBBox(const std::vector<RangeImage> &imgs, ImageBBox *bbox) {
  for (const auto &img : imgs) GetBBox(img, bbox);
}

std::vector<RangeImage> MoveScale(const std::vector<RangeImage> &imgs, int dx,
                                  int dy, int scale);

RangeImage MoveScale(const RangeImage &img, int dx, int dy, int scale);

}  // namespace vision
}  // namespace aos

#endif  // AOS_VISION_BLOB_MOVE_SCALE_H_
