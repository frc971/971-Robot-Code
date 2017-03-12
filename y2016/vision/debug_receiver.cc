#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <unistd.h>

#include <vector>
#include <memory>

#include <gtk/gtk.h>
#include "aos/vision/image/image_types.h"
#include "aos/vision/image/jpeg_routines.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/tcp_client.h"
#include "aos/vision/events/epoll_events.h"
#include "aos/vision/blob/range_image.h"
#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/stream_view.h"

#include "y2016/vision/vision_data.pb.h"
#include "y2016/vision/stereo_geometry.h"
#include "y2016/vision/blob_filters.h"

using namespace aos::vision;

class StereoViewer {
 public:
  StereoViewer(int width, int height)
      : blob_filt_(ImageFormat(width, height), 40, 100, 250000) {
    overlays_.push_back(&overlay_);
    view_.view()->SetOverlays(&overlays_);

    // Uncomment to enable blob_filt_ overlay.
    // blob_filt_.EnableOverlay(&overlay_);
    finder_.EnableOverlay(&overlay_);
  }

  virtual ~StereoViewer() {}

  void SetBlob(int camera_index, BlobList &blobl) {
    if (camera_index == 0) {
      left_blobs.swap(blobl);
      new_left = true;
    } else {
      right_blobs.swap(blobl);
      new_right = true;
    }
  }

  void Process(ImageFormat fmt) {
    if (new_left && new_right) {
      overlay_.Reset();
      DrawCross(overlay_, Vector<2>(fmt.w / 2.0, fmt.h / 2.0), {0, 255, 0});

      view_.SetFormatAndClear(fmt);
      printf("right (%d) left (%d)\n", (int)left_blobs.size(),
             (int)right_blobs.size());
      std::vector<std::pair<Vector<2>, Vector<2>>> cornersA =
          finder_.Find(blob_filt_.FilterBlobs(left_blobs));
      std::vector<std::pair<Vector<2>, Vector<2>>> cornersB =
          finder_.Find(blob_filt_.FilterBlobs(right_blobs));
      view_.DrawBlobList(left_blobs, {255, 0, 0});
      view_.DrawSecondBlobList(right_blobs, {0, 255, 0}, {0, 0, 255});
      view_.view()->Redraw();
      new_left = false;
      new_right = false;
    }
  }

 private:
  void DrawCross(PixelLinesOverlay &overlay, Vector<2> center, PixelRef color) {
    overlay.AddLine(Vector<2>(center.x() - 50, center.y()),
                     Vector<2>(center.x() + 50, center.y()), color);
    overlay.AddLine(Vector<2>(center.x(), center.y() - 50),
                     Vector<2>(center.x(), center.y() + 50), color);
  }

  // where we darw for debugging
  PixelLinesOverlay overlay_;

  // container for viewer
  std::vector<OverlayBase *> overlays_;
  BlobStreamViewer view_;

  bool new_left = false;
  BlobList left_blobs;
  bool new_right = false;
  BlobList right_blobs;

  // our blob processing object
  HistogramBlobFilter blob_filt_;

  // corner finder to align aiming
  CornerFinder finder_;
};

class BufferedLengthDelimReader {
 public:
  union data_len {
    uint32_t len;
    char buf[4];
  };
  BufferedLengthDelimReader() {
    num_read_ = 0;
    img_read_ = -1;
  }
  template <typename Lamb>
  void up(int fd, Lamb lam) {
    ssize_t count;
    if (img_read_ < 0) {
      count = read(fd, &len_.buf[num_read_], sizeof(len_.buf) - num_read_);
      if (count < 0) return;
      num_read_ += count;
      if (num_read_ < 4) return;
      num_read_ = 0;
      img_read_ = 0;
      data_.clear();
      if (len_.len > 200000) {
        printf("bad size: %d\n", len_.len);
        exit(-1);
      }
      data_.resize(len_.len);
    } else {
      count = read(fd, &data_[img_read_], len_.len - img_read_);
      if (count < 0) return;
      img_read_ += count;
      if (img_read_ < (int)len_.len) return;
      lam(DataRef{&data_[0], len_.len});
      img_read_ = -1;
    }
  }
 private:
  data_len len_;
  int num_read_;
  std::vector<char> data_;
  int img_read_;
};

class ProtoClient : public aos::events::TcpClient {
 public:
  ProtoClient(int camera_index, int width, int height, const char *hostname,
              int portno, StereoViewer *stereo)
      : aos::events::TcpClient(hostname, portno),
        camera_index_(camera_index),
        fmt_(width, height),
        stereo_(stereo) {}

  void ReadEvent() override {
    read_.up(fd(), [&](DataRef data) {
      BlobList blobl;
      y2016::vision::VisionData target;
      if (target.ParseFromArray(data.data(), data.size())) {
        auto &raw = target.raw();
        ParseBlobList(&blobl, raw.data());
        stereo_->SetBlob(camera_index_, blobl);
        stereo_->Process(fmt_);
      }
    });
  }

  int camera_index_;

  ImageFormat fmt_;

  BufferedLengthDelimReader read_;
  std::unique_ptr<PixelRef[]> outbuf;
  ImagePtr ptr;
  StereoViewer *stereo_;

 private:
};

int main(int argc, char *argv[]) {
  aos::events::EpollLoop loop;
  gtk_init(&argc, &argv);

  y2016::vision::Calibration calib =
      y2016::vision::StereoGeometry("competition").calibration();

  StereoViewer stereo(calib.camera_image_width(), calib.camera_image_height());

  ProtoClient client0(0, calib.camera_image_width(),
                      calib.camera_image_height(),
                      calib.jetson_ip_addr().data(), 8082, &stereo);
  ProtoClient client1(1, calib.camera_image_width(),
                      calib.camera_image_height(),
                      calib.jetson_ip_addr().data(), 8083, &stereo);

  loop.Add(&client0);
  loop.Add(&client1);
  loop.RunWithGtkMain();
  return EXIT_SUCCESS;
}
