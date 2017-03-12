#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "aos/common/logging/implementations.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/image/jpeg_routines.h"
#include "aos/vision/image/reader.h"
#include "y2016/vision/blob_filters.h"
#include "y2016/vision/stereo_geometry.h"
#include "y2016/vision/vision_data.pb.h"

namespace y2016 {
namespace vision {
using aos::vision::ImageStreamEvent;
using aos::vision::DataRef;
using aos::events::TCPServer;
using aos::vision::BlobLRef;
using aos::vision::Vector;
using aos::vision::Int32Codec;
using aos::vision::BlobList;
using aos::vision::RangeImage;
using aos::vision::PixelRef;
using aos::vision::ImageValue;
using aos::vision::HistogramBlobFilter;
using aos::vision::CornerFinder;
using aos::vision::Int64Codec;
using aos::events::TXUdpSocket;
using aos::events::DataSocket;
using aos::vision::ImageFormat;

::aos::vision::CameraParams GetCameraParams(const Calibration &calibration) {
  ::aos::vision::CameraParams params;
  params.set_width(calibration.camera_image_width());
  params.set_height(calibration.camera_image_height());
  params.set_exposure(calibration.camera_exposure());
  params.set_brightness(calibration.camera_brightness());
  params.set_gain(calibration.camera_gain());
  params.set_fps(calibration.camera_fps());
  return params;
}

int64_t Nanos(aos::monotonic_clock::duration time_diff) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff)
      .count();
}

int64_t NowNanos() {
  return Nanos(aos::monotonic_clock::now().time_since_epoch());
}

inline bool FileExist(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

class ImageSender : public ImageStreamEvent {
 public:
  ImageSender(int camera_index, aos::vision::CameraParams params,
              const std::string &fname, const std::string &ipadder, int port)
      : ImageStreamEvent(fname, params),
        camera_index_(camera_index),
        udp_serv_(ipadder, 8080),
        tcp_serv_(port),
        blob_filt_(ImageFormat(params.width(), params.height()), 40, 750,
                   250000),
        finder_(0.25, 35) {
    int index = 0;
    while (true) {
      std::string file = "./logging/blob_record_" + std::to_string(index) +
                         "_" + std::to_string(camera_index_) + ".dat";
      if (FileExist(file)) {
        index++;
      } else {
        printf("Logging to file (%s)\n", file.c_str());
        ofst_.open(file);
        assert(ofst_.is_open());
        break;
      }
    }
  }

  ~ImageSender() { ofst_.close(); }

  void ProcessImage(DataRef data, aos::monotonic_clock::time_point tp) {
    int64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            tp.time_since_epoch())
                            .count();
    DecodeJpeg(data, &image_);
    auto fmt = image_.fmt();

    RangeImage rimg =
        DoThreshold(image_.get(), [](PixelRef &px) { return (px.g > 88); });

    // flip the right image as this camera is mount backward
    if (camera_index_ == 0) {
      rimg.Flip(fmt.w, fmt.h);
    }

    BlobList blobl = aos::vision::FindBlobs(rimg);
    auto whatever = blob_filt_.FilterBlobs(blobl);

    VisionData target;
    target.set_camera_index(camera_index_);
    target.set_image_timestamp(timestamp);

    if (!whatever.empty()) {
      std::vector<std::pair<Vector<2>, Vector<2>>> corners =
          finder_.Find(whatever);

      if (!corners.empty()) {
        for (int i = 0; i < (int)corners.size(); i++) {
          Target *pos = target.add_target();
          pos->set_left_corner_x(corners[i].first.x());
          pos->set_left_corner_y(corners[i].first.y());
          pos->set_right_corner_x(corners[i].second.x());
          pos->set_right_corner_y(corners[i].second.y());
        }
      }
    }
    target.set_send_timestamp(NowNanos());

    // always send data
    std::string dat;
    if (target.SerializeToString(&dat)) {
      if (print_once_) {
        printf("Beginning data streaming camera %d...\n", camera_index_);
        print_once_ = false;
      }

      // Send only target over udp.
      udp_serv_.Send(dat.data(), dat.size());
    }

    // Write blob to file for logging.
    int blob_size = CalculateSize(blobl);
    int tmp_size = blob_size + sizeof(int32_t) + sizeof(uint64_t);
    char *buf;
    blob_data_.resize(tmp_size, 0);
    {
      buf = Int32Codec::Write(&blob_data_[0], tmp_size);
      buf = Int64Codec::Write(buf, timestamp);
      SerializeBlob(blobl, buf);
    }
    ofst_.write(&blob_data_[0], tmp_size);

    // Add blob debug
    bool debug = true;
    if (debug) {
      target.set_raw(buf, blob_size);
      if (target.SerializeToString(&dat)) {
        tcp_serv_.Broadcast([&](DataSocket *client) { client->Emit(dat); });
      }
    }

    bool timing = false;
    if (timing) {
      if (n_time > 0) {
        auto now = aos::monotonic_clock::now();
        printf("%g, %g\n",
               (((double)Nanos(now - tstart)) / (double)(n_time)) / 1e6,
               (double)Nanos(now - tp) / 1e6);
      } else {
        tstart = aos::monotonic_clock::now();
      }
      ++n_time;
    }
  }

  TCPServer<DataSocket> *GetTCPServ() { return &tcp_serv_; }

  // print when we start
  bool print_once_ = true;

  // left or right camera
  int camera_index_;

  // udp socket on which to send to robot
  TXUdpSocket udp_serv_;

  // tcp socket on which to debug to laptop
  TCPServer<DataSocket> tcp_serv_;

  // our blob processing object
  HistogramBlobFilter blob_filt_;

  // corner finder to align aiming
  CornerFinder finder_;

  ImageValue image_;
  std::string blob_data_;
  std::ofstream ofst_;
  aos::monotonic_clock::time_point tstart;
  int n_time = 0;

 private:
};

void RunCamera(int camera_index, aos::vision::CameraParams params,
               const std::string &device, const std::string &ip_addr,
               int port) {
  printf("Creating camera %d (%d, %d).\n", camera_index, params.width(),
         params.height());
  ImageSender strm(camera_index, params, device, ip_addr, port);

  aos::events::EpollLoop loop;
  loop.Add(strm.GetTCPServ());
  loop.Add(&strm);
  printf("Running Camera (%d)\n", camera_index);
  loop.Run();
}

}  // namespace vision
}  // namespace y2016

int main(int, char **) {
  using namespace y2016::vision;
  StereoGeometry stereo("./stereo_rig.calib");
  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));
  std::thread cam0([stereo]() {
    RunCamera(0, GetCameraParams(stereo.calibration()),
              stereo.calibration().right_camera_name(),
              stereo.calibration().roborio_ip_addr(), 8080);
  });
  std::thread cam1([stereo]() {
    RunCamera(1, GetCameraParams(stereo.calibration()),
              stereo.calibration().left_camera_name(),
              stereo.calibration().roborio_ip_addr(), 8080);
  });
  cam0.join();
  cam1.join();

  return EXIT_SUCCESS;
}
