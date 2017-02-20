#ifndef _AOS_VISION_BLOB_RANGE_IMAGE_H_
#define _AOS_VISION_BLOB_RANGE_IMAGE_H_

#include <vector>

#include "aos/vision/image/image_types.h"

namespace aos {
namespace vision {

struct Point {
  int x;
  int y;
};

struct ImageRange {
  ImageRange(int a, int b) : st(a), ed(b) {}
  int st;
  int ed;
  int last() const { return ed - 1; }
  int calc_width() const { return ed - st; }

  bool operator<(const ImageRange &o) const { return st < o.st; }
};

// Image in pre-thresholded run-length encoded format.
class RangeImage {
 public:
  RangeImage(int min_y, std::vector<std::vector<ImageRange>> &&ranges)
      : min_y_(min_y), ranges_(std::move(ranges)) {}
  explicit RangeImage(int l) { ranges_.reserve(l); }
  RangeImage() {}

  int size() const { return ranges_.size(); }

  int npixels();
  int calc_area() const;

  void Flip(ImageFormat fmt) { Flip(fmt.w, fmt.h); }
  void Flip(int image_width, int image_height);

  std::vector<std::vector<ImageRange>>::const_iterator begin() const {
    return ranges_.begin();
  }

  std::vector<std::vector<ImageRange>>::const_iterator end() const {
    return ranges_.end();
  }

  const std::vector<std::vector<ImageRange>> &ranges() const { return ranges_; }

  int min_y() const { return min_y_; }

  int mini() const { return min_y_; }

  int height() const { return min_y_ + ranges_.size(); }

 private:
  // minimum index in y where the blob starts
  int min_y_ = 0;

  // ranges are always sorted in y then x order
  std::vector<std::vector<ImageRange>> ranges_;

  // Cached pixel count.
  int npixelsc_ = -1;
};

typedef std::vector<RangeImage> BlobList;
typedef std::vector<const RangeImage *> BlobLRef;

void DrawRangeImage(const RangeImage &rimg, ImagePtr outbuf, PixelRef color);

// Merge sort of multiple range images into a single range image.
// They must not overlap.
RangeImage MergeRangeImage(const BlobList &blobl);

// Debug print range image as ranges.
std::string ShortDebugPrint(const BlobList &blobl);
// Debug print range image as ### for the ranges.
void DebugPrint(const BlobList &blobl);

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_RANGE_IMAGE_H_
