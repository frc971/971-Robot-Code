#ifndef AOS_VISION_BLOB_MOVE_SCALE_H_
#define AOS_VISION_BLOB_MOVE_SCALE_H_

#include <vector>
#include <limits>

#include "aos/vision/blob/range_image.h"

namespace aos {
namespace vision {

// Bounding box for a RangeImage.
struct ImageBBox {
  int minx = std::numeric_limits<int>::max();
  int maxx = std::numeric_limits<int>::min();
  int miny = std::numeric_limits<int>::max();
  int maxy = std::numeric_limits<int>::min();
};

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
