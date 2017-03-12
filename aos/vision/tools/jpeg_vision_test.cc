// This file is to collect data from a camera to use for debug and testing. This
// should not placed on a robot. This is okay as it is a utility of limited use
// only.

#include <gtk/gtk.h>
#include <netdb.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <memory>
#include <vector>

#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/vision/blob/range_image.h"
#include "aos/vision/blob/stream_view.h"
#include "aos/vision/blob/threshold.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/events/tcp_server.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/image/jpeg_routines.h"
#include "aos/vision/image/reader.h"
#include "aos/vision/math/vector.h"

namespace aos {
namespace vision {

void DrawVLine(ImagePtr ptr, int x, PixelRef color = {255, 0, 0}) {
  for (int y = 0; y < ptr.fmt().h; ++y) {
    ptr.get_px(x, y) = color;
  }
}
void DrawHLine(ImagePtr ptr, int y, PixelRef color = {255, 0, 0}) {
  for (int x = 0; x < ptr.fmt().w; ++x) {
    ptr.get_px(x, y) = color;
  }
}

// Connects up a camera with our processing.
class ChannelImageStream : public ImageStreamEvent {
 public:
  ChannelImageStream(const std::string &fname,
                     const aos::vision::CameraParams &params)
      : ImageStreamEvent(fname, params), view_(false) {
    // Lambda to record image data to a file on key press.
    view_.view()->key_press_event = [this](uint32_t keyval) {
      if (keyval == 'j') {
        std::ofstream ofs("/tmp/test.jpg", std::ofstream::out);
        ofs << prev_data_;
        ofs.close();
      } else if (keyval == 'a') {
        ++dx;
      } else if (keyval == 'd') {
        --dx;
      } else if (keyval == 'w') {
        ++dy;
      } else if (keyval == 's') {
        --dy;
      }
      fprintf(stderr, "dx: %d dy: %d\n", dx, dy);
    };
  }

  // Handle an image from the camera.
  void ProcessImage(DataRef data,
                    aos::monotonic_clock::time_point /*timestamp*/) {
    ImageFormat fmt = GetFmt(data);
    if (!fmt.Equals(view_.img().fmt())) view_.SetFormatAndClear(fmt);
    if (!ProcessJpeg(data, view_.img().data())) return;

    ImagePtr img_ptr = view_.img();
    prev_data_ = data.to_string();

    // Threshold the image with the given lambda.
    RangeImage rimg = DoThreshold(img_ptr, [](PixelRef &px) {
      if (px.g > 88) {
        uint8_t min = std::min(px.b, px.r);
        uint8_t max = std::max(px.b, px.r);
        if (min >= px.g || max >= px.g) return false;
        uint8_t a = px.g - min;
        uint8_t b = px.g - max;
        return (a > 10 && b > 10);
      }
      return false;
    });

    DrawVLine(img_ptr, fmt.w / 4);
    DrawVLine(img_ptr, fmt.w / 2 + dy, {0, 255, 0});
    DrawVLine(img_ptr, fmt.w - fmt.w / 4);

    DrawHLine(img_ptr, fmt.h / 4);
    DrawHLine(img_ptr, fmt.h / 2 + dx, {0, 255, 0});
    DrawHLine(img_ptr, fmt.h - fmt.h / 4);
    view_.DrawBlobList({rimg}, {255, 255, 255});

    view_.Redraw();
  }

 private:
  std::string prev_data_;

  // responsible for handling drawing
  BlobStreamViewer view_;

  int dx = 0;
  int dy = 0;
};
}  // namespace aos
}  // namespace vision

int main(int argc, char *argv[]) {
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));
  aos::events::EpollLoop loop;
  gtk_init(&argc, &argv);

  aos::vision::CameraParams params;

  aos::vision::ChannelImageStream strm1("/dev/video1", params);

  loop.Add(&strm1);
  loop.RunWithGtkMain();
}
