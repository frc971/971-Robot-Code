#ifndef _AOS_VISION_IMAGE_IMAGE_TYPES_H_
#define _AOS_VISION_IMAGE_IMAGE_TYPES_H_

#include <stdint.h>
#include <string.h>
#include <memory>
#include <sstream>

#include <experimental/string_view>
#include "aos/common/logging/logging.h"

namespace aos {
namespace vision {

// Bounding box for a RangeImage.
struct ImageBBox {
  int minx = std::numeric_limits<int>::max();
  int maxx = std::numeric_limits<int>::min();
  int miny = std::numeric_limits<int>::max();
  int maxy = std::numeric_limits<int>::min();
};

// This will go into c++17. No sense writing my own version.
using DataRef = std::experimental::string_view;

// Represents the dimensions of an image.
struct ImageFormat {
  ImageFormat() : w(0), h(0) {}
  ImageFormat(int nw, int nh) : w(nw), h(nh) {}
  std::string ToString() const {
    std::ostringstream s;
    s << "ImageFormat {" << w << ", " << h << "}";
    return s.str();
  }
  int ImgSize() const { return w * h; }
  bool Equals(const ImageFormat &other) const {
    return w == other.w && h == other.h;
  }

  int w;
  int h;
};

// Alias for RGB triple. Should be align 1 size 3.
struct PixelRef {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Just to be extra safe.
static_assert(sizeof(PixelRef) == 3, "Problem with cows in fields!");
static_assert(alignof(PixelRef) == 1, "Problem with cows in fields!");

// ptr version of a ValueArray. Allows operations on buffers owned by other
// entities. Prefer this for const-ref versions.
//
// Templatized because there was a grayscale version, and cairo buffers are
// only RGBA.
template <typename ImageType>
class Array2dPtr {
 public:
  Array2dPtr() : Array2dPtr({0, 0}, nullptr) {}
  Array2dPtr(ImageFormat fmt, ImageType *data) : fmt_(fmt), data_(data) {}
  ImageType &get_px(int x, int y) const {
#ifndef NDEBUG
    if (x < 0 || x >= fmt_.w || y < 0 || y >= fmt_.h) {
      LOG(FATAL, "%d, %d out of range [%dx %d]\n", x, y, fmt_.w, fmt_.h);
    }
#endif  // NBOUNDSCHECK
    return data_[(x + y * fmt_.w)];
  }
  void CopyFrom(const Array2dPtr &other) const {
    memcpy(data_, other.data_, sizeof(ImageType) * fmt_.ImgSize());
  }

  const ImageFormat &fmt() const { return fmt_; }
  ImageType *data() const { return data_; }

 private:
  ImageFormat fmt_;
  ImageType *data_;
};

// unique_ptr version of above.
template <typename ImageType>
class ValueArray2d {
 public:
  ValueArray2d() : fmt_({0, 0}) {}
  explicit ValueArray2d(ImageFormat fmt) : fmt_(fmt) {
    data_.reset(new ImageType[fmt.ImgSize()]);
  }

  Array2dPtr<ImageType> get() {
    return Array2dPtr<ImageType>{fmt_, data_.get()};
  }

  const ImageFormat &fmt() const { return fmt_; }
  ImageType *data() { return data_.get(); }

 private:
  ImageFormat fmt_;
  std::unique_ptr<ImageType[]> data_;
};

using ImagePtr = Array2dPtr<PixelRef>;
using ImageValue = ValueArray2d<PixelRef>;

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_IMAGE_IMAGE_TYPES_H_
