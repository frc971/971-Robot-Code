#ifndef _AOS_VISION_BLOB_STREAM_VIEW_H_
#define _AOS_VISION_BLOB_STREAM_VIEW_H_

#include "aos/vision/blob/range_image.h"
#include "aos/vision/debug/debug_viewer.h"
#include "aos/vision/image/image_types.h"

#include <memory>

namespace aos {
namespace vision {

class BlobStreamViewer : public DebugViewer {
 public:
  BlobStreamViewer() : DebugViewer(false) {}
  explicit BlobStreamViewer(bool flip) : DebugViewer(flip) {}

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

  inline void DrawBlobList(const BlobList &blob_list, PixelRef color) {
    ImagePtr ptr = img();
    for (const RangeImage &blob : blob_list) {
      for (int i = 0; i < (int)blob.ranges().size(); ++i) {
        for (const auto &range : blob.ranges()[i]) {
          for (int j = range.st; j < range.ed; ++j) {
            ptr.get_px(j, i + blob.min_y()) = color;
          }
        }
      }
    }
  }

  inline void DrawSecondBlobList(const BlobList &blob_list, PixelRef color1,
                                 PixelRef color2) {
    ImagePtr ptr = img();
    for (const auto &blob : blob_list) {
      for (int i = 0; i < (int)blob.ranges().size(); ++i) {
        for (const auto &range : blob.ranges()[i]) {
          for (int j = range.st; j < range.ed; ++j) {
            auto px = ptr.get_px(j, i + blob.min_y());
            if (px.r == 0 && px.g == 0 && px.b == 0) {
              ptr.get_px(j, i + blob.min_y()) = color1;
            } else {
              ptr.get_px(j, i + blob.min_y()) = color2;
            }
          }
        }
      }
    }
  }

  inline void DrawSecondBlobList(const BlobList &blob_list, PixelRef color1,
                                 PixelRef color2, PixelRef prev_color) {
    ImagePtr ptr = img();
    for (const auto &blob : blob_list) {
      for (int i = 0; i < (int)blob.ranges().size(); ++i) {
        for (const auto &range : blob.ranges()[i]) {
          for (int j = range.st; j < range.ed; ++j) {
            auto px = ptr.get_px(j, i + blob.min_y());
            if (px.r == prev_color.r && px.g == prev_color.g &&
                px.b == prev_color.b) {
              ptr.get_px(j, i + blob.min_y()) = color2;
            } else {
              ptr.get_px(j, i + blob.min_y()) = color1;
            }
          }
        }
      }
    }
  }

  // Backwards compatible.
  DebugViewer *view() { return this; }

  ImagePtr img() { return image_.get(); }

 private:
  ImageValue image_;
};

}  // namespace vision
}  // namespace aos

#endif  // _AOS_VISION_BLOB_STREAM_VIEW_H_
