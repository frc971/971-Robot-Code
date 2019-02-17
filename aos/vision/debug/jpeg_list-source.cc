#include "aos/vision/debug/debug_framework.h"

#include "aos/vision/image/image_dataset.h"

#include <gdk/gdk.h>
#include <fstream>
#include <string>

namespace aos {
namespace vision {

class JpegListImageSource : public ImageSource {
 public:
  void Init(const std::string &jpeg_list_filename,
            DebugFrameworkInterface *interface) override {
    interface_ = interface;
    images_ = LoadDataset(jpeg_list_filename);
    fprintf(stderr, "loaded %lu items\n", images_.size());
    if (!images_.empty()) {
      SetCurrentFrame();
      interface_->InstallKeyPress([this](uint32_t keyval) {
        if (keyval == GDK_KEY_Left && idx_ > 0) {
          --idx_;
        } else if (keyval == GDK_KEY_Right && idx_ < images_.size()) {
          idx_ = (idx_ + 1) % images_.size();
        } else {
          return;
        }
        SetCurrentFrame();
      });
    }
  }

  void SetCurrentFrame() {
    const auto &frame = images_[idx_];
    if (frame.is_jpeg) {
      interface_->NewJpeg(frame.data);
    } else {
      const auto &data = frame.data;
      interface_->NewImage({640, 480},
                           [&](ImagePtr img_data) {
                             for (int y = 0; y < 480; ++y) {
                               for (int x = 0; x < 640; ++x) {
                                 uint8_t v = data[y * 640 * 2 + x * 2 + 0];
                                 img_data.get_px(x, y) = PixelRef{v, v, v};
                               }
                             }
                             return false;
                           });
    }
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is the name of a file with each jpeg filename on a new line.
    This viewer source will load each jpeg individually and cycle through them
    with the arrow keys.
)"[1];
  }

 private:
  DebugFrameworkInterface *interface_ = nullptr;
  std::vector<DatasetFrame> images_;
  size_t idx_ = 0;
};

REGISTER_IMAGE_SOURCE("jpeg_list", JpegListImageSource);

}  // namespace vision
}  // namespace aos
