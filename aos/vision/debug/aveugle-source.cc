#include <gdk/gdk.h>

#include <fstream>
#include <string>

#include "aos/vision/debug/debug_framework.h"
#include "aos/vision/image/camera_params.pb.h"
#include "aos/vision/image/image_stream.h"

namespace aos::vision {

class AveugleImageSource : public ImageSource {
 public:
  void Init(const std::string &jpeg_list_filename,
            DebugFrameworkInterface *interface) override {
    // TODO: These camera params make this ugly and less generic.
    image_stream_.reset(new ImageStream(jpeg_list_filename,
                                        interface->camera_params(), interface));
  }

  const char *GetHelpMessage() override {
    return &R"(
    format_spec is filename of the camera device.
    example: jevois:/dev/video1
    This viewer source will stream video from a jevois camera of your choice.
)"[1];
  }

  class ImageStream : public ImageStreamEvent {
   public:
    ImageStream(const std::string &fname, aos::vision::CameraParams params,
                DebugFrameworkInterface *interface)
        : ImageStreamEvent(fname, params), interface_(interface) {
      interface_->Loop()->Add(this);

      interface_->InstallKeyPress([this](uint32_t keyval) {
        // Takes a picture when you press 'a'.
        // TODO(parker): Allow setting directory.
        if (keyval == GDK_KEY_a) {
          std::ofstream ofs(std::string("/tmp/debug_viewer_jpeg_") +
                                std::to_string(i_) + ".yuyv",
                            std::ofstream::out);
          ofs << prev_data_;
          ofs.close();
          ++i_;
        }
      });
      interface_->InstallSetExposure(
          [this](int abs_exp) { this->SetExposure(abs_exp); });
    }
    void ProcessImage(DataRef data, aos::monotonic_clock::time_point) override {
      prev_data_ = std::string(data);
      interface_->NewImage({640, 480}, [&](ImagePtr img_data) {
        for (int y = 0; y < 480; ++y) {
          for (int x = 0; x < 640; ++x) {
            uint8_t v = data[y * 640 * 2 + x * 2 + 0];
            img_data.get_px(x, y) = PixelRef{v, v, v};
          }
        }
        return false;
      });
    }

   private:
    int i_ = 0;
    std::string prev_data_;
    DebugFrameworkInterface *interface_;
  };

  std::unique_ptr<ImageStream> image_stream_;
};

REGISTER_IMAGE_SOURCE("jevois", AveugleImageSource);

}  // namespace aos::vision
