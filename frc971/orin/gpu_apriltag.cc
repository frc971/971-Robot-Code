#include <chrono>

#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/tag16h5.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "frc971/orin/apriltag.h"
#include "frc971/vision/charuco_lib.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_string(channel, "/camera1", "Channel name");
DEFINE_double(min_decision_margin, 50.0,
              "Minimum decision margin (confidence) for an apriltag detection");

DEFINE_bool(debug, false, "If true, write debug images.");

namespace frc971::apriltag {

namespace chrono = std::chrono;

apriltag_detector_t *MakeTagDetector(apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = FLAGS_debug;

  return tag_detector;
}

class Detector {
 public:
  Detector(aos::EventLoop *event_loop, std::string_view channel_name,
           size_t width = 1456, size_t height = 1088)
      : tag_family_(tag16h5_create()),
        tag_detector_(MakeTagDetector(tag_family_)),
        gpu_detector_(width, height, tag_detector_),
        image_callback_(
            event_loop, channel_name,
            [&](cv::Mat image_color_mat,
                const aos::monotonic_clock::time_point eof) {
              HandleImage(image_color_mat, eof);
            },
            chrono::milliseconds(45)) {
    image_callback_.set_format(frc971::vision::ImageCallback::Format::YUYV2);
  }

  virtual ~Detector() {
    apriltag_detector_destroy(tag_detector_);
    free(tag_family_);
  }

  void HandleImage(cv::Mat color_image,
                   aos::monotonic_clock::time_point /*eof*/) {
    const aos::monotonic_clock::time_point start_time =
        aos::monotonic_clock::now();
    gpu_detector_.Detect(color_image.data);

    const zarray_t *detections = gpu_detector_.Detections();

    const aos::monotonic_clock::time_point end_time =
        aos::monotonic_clock::now();
    for (int i = 0; i < zarray_size(detections); ++i) {
      const apriltag_detection_t *gpu_detection;

      zarray_get(detections, i, &gpu_detection);

      bool valid = gpu_detection->decision_margin > FLAGS_min_decision_margin;

      if (valid) {
        AOS_LOG(
            INFO,
            "Found GPU %s tag number %d hamming %d margin %f  (%f, %f), (%f, "
            "%f), (%f, %f), (%f, %f) in %f ms\n",
            valid ? "valid" : "invalid", gpu_detection->id,
            gpu_detection->hamming, gpu_detection->decision_margin,
            gpu_detection->p[0][0], gpu_detection->p[0][1],
            gpu_detection->p[1][0], gpu_detection->p[1][1],
            gpu_detection->p[2][0], gpu_detection->p[2][1],
            gpu_detection->p[3][0], gpu_detection->p[3][1],
            std::chrono::duration<float, std::milli>(end_time - start_time)
                .count());
      }
    }
  }

  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;

  GpuDetector gpu_detector_;

  frc971::vision::ImageCallback image_callback_;
};

void AprilViewerMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  Detector detector(&event_loop, FLAGS_channel);

  // TODO(austin): Figure out our core pinning strategy.
  // event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  struct sched_param param;
  param.sched_priority = 21;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  // TODO(austin): Pre-warm it...
  event_loop.Run();
}

}  // namespace frc971::apriltag

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::apriltag::AprilViewerMain();

  return 0;
}
