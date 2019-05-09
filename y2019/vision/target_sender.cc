#include "y2019/vision/target_finder.h"

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <random>
#include <thread>

#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/events/socket_types.h"
#include "aos/vision/events/udp.h"
#include "y2019/jevois/camera/image_stream.h"
#include "y2019/jevois/camera/reader.h"
#include "y2019/jevois/serial.h"
#include "y2019/jevois/structures.h"
#include "y2019/jevois/uart.h"
#include "y2019/vision/image_writer.h"

// This has to be last to preserve compatibility with other headers using AOS
// logging.
#include "glog/logging.h"

using ::aos::events::DataSocket;
using ::aos::events::RXUdpSocket;
using ::aos::events::TCPServer;
using ::aos::vision::DataRef;
using ::aos::vision::Int32Codec;
using ::aos::monotonic_clock;
using ::y2019::jevois::open_via_terminos;
using aos::vision::Segment;

class CameraStream : public ::y2019::camera::ImageStreamEvent {
 public:
  CameraStream(::aos::vision::CameraParams params, const ::std::string &fname)
      : ImageStreamEvent(fname, params) {}

  void ProcessImage(DataRef data, monotonic_clock::time_point monotonic_now) {
    LOG(INFO) << "got frame: " << data.size();

    if (on_frame_) on_frame_(data, monotonic_now);
  }

  void set_on_frame(const std::function<
                    void(DataRef, monotonic_clock::time_point)> &on_frame) {
    on_frame_ = on_frame;
  }

 private:
  std::function<void(DataRef, monotonic_clock::time_point)> on_frame_;
};

int open_terminos(const char *tty_name) { return open_via_terminos(tty_name); }

std::string GetFileContents(const std::string &filename) {
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in) {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return (contents);
  }
  fprintf(stderr, "Could not read file: %s\n", filename.c_str());
  exit(-1);
}

using aos::vision::ImageRange;
using aos::vision::RangeImage;
using aos::vision::ImageFormat;
using y2019::vision::TargetFinder;
using y2019::vision::IntermediateResult;
using y2019::vision::Target;

class TargetProcessPool {
 public:
  // The number of threads we'll use.
  static constexpr int kThreads = 4;

  TargetProcessPool(TargetFinder *finder);
  ~TargetProcessPool();

  std::vector<IntermediateResult> Process(std::vector<const Target *> &&inputs,
                                          bool verbose);

 private:
  // The main function for a thread.
  void RunThread();

  std::array<std::thread, kThreads> threads_;
  // Coordinates access to results_/inputs_ and coordinates with
  // condition_variable_.
  std::mutex mutex_;
  // Signals changes to results_/inputs_ and quit_.
  std::condition_variable condition_variable_;
  bool quit_ = false;

  bool verbose_ = false;
  std::vector<const Target *> inputs_;
  std::vector<IntermediateResult> results_;

  TargetFinder *const finder_;
};

TargetProcessPool::TargetProcessPool(TargetFinder *finder) : finder_(finder) {
  for (int i = 0; i < kThreads; ++i) {
    threads_[i] = std::thread([this]() { RunThread(); });
  }
}

TargetProcessPool::~TargetProcessPool() {
  {
    std::unique_lock<std::mutex> locker(mutex_);
    quit_ = true;
    condition_variable_.notify_all();
  }
  for (int i = 0; i < kThreads; ++i) {
    threads_[i].join();
  }
}

std::vector<IntermediateResult> TargetProcessPool::Process(
    std::vector<const Target *> &&inputs, bool verbose) {
  inputs_ = std::move(inputs);
  results_.clear();
  verbose_ = verbose;
  const size_t number_targets = inputs_.size();
  {
    std::unique_lock<std::mutex> locker(mutex_);
    condition_variable_.notify_all();
    while (results_.size() < number_targets) {
      condition_variable_.wait(locker);
    }
  }
  return std::move(results_);
}

void TargetProcessPool::RunThread() {
  while (true) {
    const Target *my_input;
    {
      std::unique_lock<std::mutex> locker(mutex_);
      while (inputs_.empty()) {
        if (quit_) {
          return;
        }
        condition_variable_.wait(locker);
      }
      my_input = inputs_.back();
      inputs_.pop_back();
    }
    IntermediateResult my_output =
        finder_->ProcessTargetToResult(*my_input, false);
    {
      std::unique_lock<std::mutex> locker(mutex_);
      results_.emplace_back(std::move(my_output));
      condition_variable_.notify_all();
    }
  }
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  using namespace y2019::vision;
  using frc971::jevois::CameraCommand;
  // gflags::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  int itsDev = open_terminos("/dev/ttyS0");
  frc971::jevois::CobsPacketizer<frc971::jevois::uart_to_camera_size()> cobs;
  // Uncomment these to printf directly to stdout to get debug info...
  // dup2(itsDev, 1);
  // dup2(itsDev, 2);

  TargetFinder finder;
  TargetProcessPool process_pool(&finder);
  ImageWriter writer;
  uint32_t image_count = 0;
  bool log_images = false;

  aos::vision::CameraParams params0;
  params0.set_exposure(60);
  params0.set_brightness(40);
  params0.set_width(640);
  params0.set_fps(25);
  params0.set_height(480);

  aos::vision::FastYuyvYPooledThresholder thresholder;

  // A source of psuedorandom numbers which gives different numbers each time we
  // need to drop targets.
  std::minstd_rand random_engine;

  ::std::unique_ptr<CameraStream> camera0(
      new CameraStream(params0, "/dev/video0"));
  camera0->set_on_frame([&](DataRef data,
                            monotonic_clock::time_point monotonic_now) {
    aos::vision::ImageFormat fmt{640, 480};
    aos::vision::BlobList imgs =
        aos::vision::FindBlobs(thresholder.Threshold(fmt, data.data(), 120));
    const int num_pixels = finder.PixelCount(&imgs);
    LOG(INFO) << "Number pixels: " << num_pixels;
    finder.PreFilter(&imgs);
    LOG(INFO) << "Blobs: " << imgs.size();

    constexpr bool verbose = false;
    ::std::vector<Polygon> raw_polys;
    for (const RangeImage &blob : imgs) {
      // Convert blobs to contours in the corrected space.
      ContourNode *contour = finder.GetContour(blob);
      ::std::vector<::Eigen::Vector2f> unwarped_contour =
          finder.UnWarpContour(contour);
      const Polygon polygon =
          finder.FindPolygon(::std::move(unwarped_contour), verbose);
      if (!polygon.segments.empty()) {
        raw_polys.push_back(polygon);
      }
    }
    LOG(INFO) << "Polygons: " << raw_polys.size();

    // Calculate each component side of a possible target.
    ::std::vector<TargetComponent> target_component_list =
        finder.FillTargetComponentList(raw_polys, verbose);
    LOG(INFO) << "Components: " << target_component_list.size();

    // Put the compenents together into targets.
    ::std::vector<Target> target_list =
        finder.FindTargetsFromComponents(target_component_list, verbose);
    static constexpr size_t kMaximumPotentialTargets = 8;
    LOG(INFO) << "Potential Targets (will filter to "
              << kMaximumPotentialTargets << "): " << target_list.size();

    // A list of all the indices into target_list which we're going to actually
    // use.
    std::vector<int> target_list_indices;
    target_list_indices.resize(target_list.size());
    for (size_t i = 0; i < target_list.size(); ++i) {
      target_list_indices[i] = i;
    }
    // Drop random elements until we get sufficiently few of them. We drop
    // different elements each time to ensure we will see different valid
    // targets on successive frames, which provides more useful information to
    // the localization.
    while (target_list_indices.size() > kMaximumPotentialTargets) {
      std::uniform_int_distribution<size_t> distribution(
          0, target_list_indices.size() - 1);
      const size_t index = distribution(random_engine);
      target_list_indices.erase(target_list_indices.begin() + index);
    }

    // Use the solver to generate an intermediate version of our results.
    std::vector<const Target *> inputs;
    for (size_t index : target_list_indices) {
      inputs.push_back(&target_list[index]);
    }
    std::vector<IntermediateResult> results =
        process_pool.Process(std::move(inputs), verbose);
    LOG(INFO) << "Raw Results: " << results.size();

    results = finder.FilterResults(results, 30, verbose);
    LOG(INFO) << "Results: " << results.size();

    int desired_exposure;
    if (finder.TestExposure(results, num_pixels, &desired_exposure)) {
      camera0->SetExposure(desired_exposure);
    }

    frc971::jevois::CameraFrame frame{};

    for (size_t i = 0; i < results.size() && i < frame.targets.max_size();
         ++i) {
      const auto &result = results[i].extrinsics;
      frame.targets.push_back(frc971::jevois::Target{
          static_cast<float>(result.z), static_cast<float>(result.y),
          static_cast<float>(result.r2), static_cast<float>(result.r1)});
    }

    frame.age = std::chrono::duration_cast<frc971::jevois::camera_duration>(
        aos::monotonic_clock::now() - monotonic_now);

    // If we succeed in writing our delimiter, then write out the rest of
    // the frame. If not, no point in continuing.
    if (write(itsDev, "\0", 1) == 1) {
      const auto serialized_frame = frc971::jevois::UartPackToTeensy(frame);
      // We don't really care if this succeeds or not. If it fails for some
      // reason, we'll just try again with the next frame, and the other end
      // will find the new packet just fine.
      ssize_t n =
          write(itsDev, serialized_frame.data(), serialized_frame.size());

      if (n != (ssize_t)serialized_frame.size()) {
        LOG(INFO) << "Some problem happened";
      }
    }

    if (log_images) {
      if ((image_count % 5) == 0) {
        writer.WriteImage(data);
      }
      ++image_count;
    }
  });

  aos::events::EpollLoop loop;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    camera0->ReadEvent();

    {
      constexpr size_t kBufferSize = frc971::jevois::uart_to_teensy_size();
      char data[kBufferSize];
      ssize_t n = read(itsDev, &data[0], kBufferSize);
      if (n >= 1) {
        cobs.ParseData(gsl::span<const char>(&data[0], n));
        auto packet = cobs.received_packet();
        if (!packet.empty()) {
          auto calibration_question =
              frc971::jevois::UartUnpackToCamera(packet);
          if (calibration_question) {
            const auto &calibration = *calibration_question;
            IntrinsicParams *intrinsics = finder.mutable_intrinsics();
            intrinsics->mount_angle = calibration.calibration(0, 0);
            intrinsics->focal_length = calibration.calibration(0, 1);
            intrinsics->barrel_mount = calibration.calibration(0, 2);

            switch (calibration.camera_command) {
              case CameraCommand::kNormal:
              case CameraCommand::kAs:
                log_images = false;
                break;
              case CameraCommand::kLog:
                log_images = true;
                break;
              case CameraCommand::kUsb:
                return 0;
              case CameraCommand::kCameraPassthrough:
                return system("touch /tmp/do_not_export_sd_card");
            }
          } else {
            fprintf(stderr, "bad frame\n");
          }
          cobs.clear_received_packet();
        }
      }
    }
  }

  // TODO: Fix event loop on jevois:
  // loop.Add(camera0.get());
  // loop.Run();
}
