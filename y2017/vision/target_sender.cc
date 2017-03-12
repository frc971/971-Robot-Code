#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
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
#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/blob/range_image.h"
#include "aos/vision/blob/threshold.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "aos/vision/image/image_stream.h"
#include "aos/vision/image/jpeg_routines.h"
#include "aos/vision/image/reader.h"
#include "y2017/vision/target_finder.h"
#include "y2017/vision/vision_config.pb.h"
#include "y2017/vision/vision_result.pb.h"

namespace y2017 {
namespace vision {

using aos::events::TCPServer;
using aos::vision::DataRef;
using aos::vision::Int32Codec;
using aos::vision::ImageValue;
using aos::vision::Int64Codec;
using aos::events::TXUdpSocket;
using aos::events::DataSocket;
using aos::vision::ImageFormat;
using aos::vision::ImageStreamEvent;

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

class BlobLog {
 public:
  explicit BlobLog(const char *prefix, const char *extension) {
    int index = 0;
    while (true) {
      std::string file = prefix + std::to_string(index) + extension;
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

  ~BlobLog() { ofst_.close(); }

  void WriteLogEntry(DataRef data) { ofst_.write(&data[0], data.size()); }

 private:
  std::ofstream ofst_;
};

class ImageSender : public ImageStreamEvent {
 public:
  ImageSender(aos::vision::CameraParams params, GameSpecific game_cfg,
              const std::string &fname, const std::string &ipadder, int port)
      : ImageStreamEvent(fname, params),
        game_cfg_(game_cfg),
        udp_serv_(ipadder, 8080),
        tcp_serv_(port),
        log_("./logging/blob_record_", ".dat") {}

  void WriteAndSendBlob(ImageFormat fmt, int64_t timestamp,
                        aos::vision::BlobList blobl) {
    // Write blob to file for logging.
    int blob_size = CalculateSize(blobl);
    int tmp_size = blob_size + sizeof(int32_t) * 3 + sizeof(uint64_t);
    char *buf;
    std::string blob_data;
    blob_data.resize(tmp_size, 0);
    {
      buf = Int32Codec::Write(&blob_data[0], tmp_size);
      buf = Int64Codec::Write(buf, timestamp);
      buf = Int32Codec::Write(buf, fmt.w);
      buf = Int32Codec::Write(buf, fmt.h);
      SerializeBlob(blobl, buf);
    }
    log_.WriteLogEntry(blob_data);

    // Send the blob back to the debug-viewer
    tcp_serv_.Broadcast([&](DataSocket *client) { client->Emit(blob_data); });
  }

  void ProcessImage(DataRef data, aos::monotonic_clock::time_point tp) {
    using namespace aos::vision;
    int64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            tp.time_since_epoch())
                            .count();
    DecodeJpeg(data, &image_);
    ImageFormat fmt = image_.fmt();

    RangeImage rimg = finder_.Threshold(image_.get());
    BlobList blobl = aos::vision::FindBlobs(rimg);

    // Remove bad blobs before we log.
    finder_.PreFilter(blobl);

    // Write to the logi and stream to the debug-viewer.
    WriteAndSendBlob(fmt, timestamp, blobl);

    // Calculate each component.
    std::vector<TargetComponent> target_component_list =
        finder_.FillTargetComponentList(blobl);

    // Put the compenents together into targets and pick the best.
    y2017::vision::Target final_target;
    bool found_target =
        finder_.FindTargetFromComponents(target_component_list, &final_target);

    std::string dat;
    VisionResult result;
    result.set_image_timestamp(timestamp);
    result.set_send_timestamp(NowNanos());
    if (found_target) {
      result.mutable_target()->set_x(final_target.screen_coord.x());
      result.mutable_target()->set_y(final_target.screen_coord.y());
    }
    // always send data
    if (result.SerializeToString(&dat)) {
      if (print_once_) {
        printf("Beginning data streaming camera...\n");
        print_once_ = false;
      }

      // Send only target over udp.
      udp_serv_.Send(dat.data(), dat.size());
    }
  }

  TCPServer<DataSocket> *GetTCPServ() { return &tcp_serv_; }

  // Configuration related to the game.
  GameSpecific game_cfg_;

  // target selction code.
  TargetFinder finder_;

  // print when we start
  bool print_once_ = true;

  // udp socket on which to send to robot
  TXUdpSocket udp_serv_;

  // tcp socket on which to debug to laptop
  TCPServer<DataSocket> tcp_serv_;

  ImageValue image_;
  BlobLog log_;
  aos::monotonic_clock::time_point tstart;

 private:
};

void RunCamera(aos::vision::CameraParams settings, GameSpecific game_cfg,
               const std::string &device, const std::string &ip_addr,
               int port) {
  printf("Creating camera (%dx%d).\n", settings.width(), settings.height());
  ImageSender strm(settings, game_cfg, device, ip_addr, port);

  aos::events::EpollLoop loop;
  loop.Add(strm.GetTCPServ());
  loop.Add(&strm);
  printf("Running Camera\n");
  loop.Run();
}

bool ReadConfiguration(const std::string &file_name, VisionConfig *cfg) {
  using namespace google::protobuf::io;
  using namespace google::protobuf;
  if (cfg == nullptr) {
    return false;
  }

  // fd will close when it goes out of scope.
  std::ifstream fd(file_name);
  if (!fd.is_open()) {
    fprintf(stderr, "File (%s) not found.\n", file_name.c_str());
    return false;
  }
  IstreamInputStream is(&fd);
  if (!TextFormat::Parse(&is, cfg)) {
    fprintf(stderr, "Unable to parse file (%s).\n", file_name.c_str());
    return false;
  }

  return true;
}

}  // namespace vision
}  // namespace y2017

int main(int, char **) {
  using namespace y2017::vision;

  ::aos::logging::Init();
  ::aos::logging::AddImplementation(
      new ::aos::logging::StreamLogImplementation(stdout));
  VisionConfig cfg;
  if (ReadConfiguration("ConfigFile.pb.ascii", &cfg)) {
    if (cfg.robot_configs().count("Laptop") != 1) {
      fprintf(stderr, "Could not find config key.\n");
      return -1;
    }
    const RobotConfig &rbt = cfg.robot_configs().at("Laptop");
    RunCamera(cfg.camera_params(), cfg.game_params(), rbt.camera_device_path(),
              rbt.roborio_ipaddr(), rbt.port());
  }

  return EXIT_SUCCESS;
}
