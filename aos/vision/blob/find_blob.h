#ifndef AOS_VISION_BLOB_FIND_BLOB_H_
#define AOS_VISION_BLOB_FIND_BLOB_H_

#include "aos/vision/blob/range_image.h"

namespace aos::vision {

// Uses disjoint sets to group ranges into disjoint RangeImage.
// ranges that overlap are grouped into the same output RangeImage.
BlobList FindBlobs(const RangeImage &input_image);

}  // namespace aos::vision

#endif  // AOS_VISION_BLOB_FIND_BLOB_H_
