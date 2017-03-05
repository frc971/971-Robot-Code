#ifndef _AOS_VISION_BLOB_STREAM_VIEW_H_
#define _AOS_VISION_BLOB_STREAM_VIEW_H_

#include "aos/vision/blob/range_image.h"
#include "aos/vision/debug/debug_window.h"
#include "aos/vision/image/image_types.h"

#include <memory>

namespace aos {
namespace vision {

class BlobStreamViewer : public DebugWindow {
 public:
  BlobStreamViewer() : DebugWindow(false) {}
  explicit BlobStreamViewer(bool flip) : DebugWindow(flip) {}

  void Submit(ImageFormat fmt, const BlobList &blob_list) {
    SetFormatAndClear(fmt);
    DrawBlobList(blob_list, {255, 255, 255});
  }

  inline void SetFormatAndClear(ImageFormat fmt) {
    if (!image_.fmt().Equals(fmt)) {
      printf("resizing data: %d, %d\n", fmt.w, fmt.h);
      image_ = ImageValue(fmt);
      UpdateImage(image_.get());
    }
    memset(image_.data(), 0, fmt.ImgSize() * sizeof(PixelRef));
  }

  template <typename PixelCallback>
  inline void ForPxInBlobList(const BlobList &blob_list,
                              PixelCallback pixel_callback) {
    ImagePtr ptr = img();
    auto fmt = ptr.fmt();
    for (const auto &blob : blob_list) {
      for (int i = 0; i < (int)blob.ranges().size(); ++i) {
        int y = blob.min_y() + i;
        if (y >= 0 && y < fmt.h) {
          for (const auto &range : blob.ranges()[i]) {
            for (int j = std::max(0, range.st); j < std::min(fmt.w, range.ed);
                 ++j) {
              pixel_callback(ptr.get_px(j, y));
            }
          }
        }
      }
    }
  }

  inline void DrawBlobList(const BlobList &blob_list, PixelRef color) {
    ForPxInBlobList(blob_list, [&](PixelRef &px) { px = color; });
  }

  inline void DrawSecondBlobList(const BlobList &blob_list, PixelRef color1,
                                 PixelRef color2) {
    ForPxInBlobList(blob_list, [&](PixelRef &px) {
      if (px.r == 0 && px.g == 0 && px.b == 0) {
        px = color1;
      } else {
        px = color2;
      }
    });
  }

  inline void DrawSecondBlobList(const BlobList &blob_list, PixelRef color1,
                                 PixelRef color2, PixelRef prev_color) {
    ForPxInBlobList(blob_list, [&](PixelRef &px) {
      if (px.r == prev_color.r && px.g == prev_color.g &&
          px.b == prev_color.b) {
        px = color2;
      } else {
        px = color1;
      }
    });
  }

  // Backwards compatible.
  DebugWindow *view() { return this; }

  ImagePtr img() { return image_.get(); }

 private:
  ImageValue image_;
};

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_STREAM_VIEW_H_
