#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/vision/media_device.h"
#include "frc971/vision/v4l2_reader.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

namespace y2022 {
namespace vision {
namespace {

using namespace frc971::vision;

class CameraReader {
 public:
  CameraReader(aos::EventLoop *event_loop, V4L2ReaderBase *reader)
      : event_loop_(event_loop),
        reader_(reader),
        read_image_timer_(event_loop->AddTimer([this]() { ReadImage(); })) {
    event_loop->OnRun(
        [this]() { read_image_timer_->Setup(event_loop_->monotonic_now()); });
  }

  void ReadImage() {
    if (!reader_->ReadLatestImage()) {
      read_image_timer_->Setup(event_loop_->monotonic_now() +
                               std::chrono::milliseconds(10));
      return;
    }
    reader_->SendLatestImage();

    read_image_timer_->Setup(event_loop_->monotonic_now());
  }

 private:
  aos::EventLoop *event_loop_;
  V4L2ReaderBase *reader_;

  aos::TimerHandler *const read_image_timer_;
};

void CameraReaderMain() {
  std::optional<MediaDevice> media_device = FindMediaDevice("platform:rkisp1");

  if (VLOG_IS_ON(1)) {
    media_device->Log();
  }

  media_device->Reset();

  media_device->Enable(
      media_device->FindLink("ov5647 4-0036", 0, "rkisp1_csi", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_csi", 1, "rkisp1_isp", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_isp", 2, "rkisp1_resizer_selfpath", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_isp", 2, "rkisp1_resizer_mainpath", 0));

  media_device->FindEntity("ov5647 4-0036")
      ->pads()[0]
      ->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);

  Entity *rkisp1_csi = media_device->FindEntity("rkisp1_csi");
  rkisp1_csi->pads()[0]->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);
  rkisp1_csi->pads()[1]->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);

  // TODO(austin): Should we set this on the link?
  // TODO(austin): Need to update crop too.
  Entity *rkisp1_isp = media_device->FindEntity("rkisp1_isp");
  rkisp1_isp->pads(0)->SetSubdevCrop(1296, 972);
  rkisp1_isp->pads(0)->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);

  rkisp1_isp->pads(2)->SetSubdevCrop(1296, 972);
  rkisp1_isp->pads(2)->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_YUYV8_2X8);

  Entity *rkisp1_resizer_selfpath =
      media_device->FindEntity("rkisp1_resizer_selfpath");
  rkisp1_resizer_selfpath->pads(0)->SetSubdevFormat(1296, 972,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(1)->SetSubdevFormat(1296, 972,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(0)->SetSubdevCrop(1296, 972);

  Entity *rkisp1_resizer_mainpath =
      media_device->FindEntity("rkisp1_resizer_mainpath");
  rkisp1_resizer_mainpath->pads(0)->SetSubdevFormat(1296, 972,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_mainpath->pads(1)->SetSubdevFormat(1296 / 2, 972 / 2,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_mainpath->pads(0)->SetSubdevCrop(1296 / 2, 972 / 2);

  Entity *rkisp1_mainpath = media_device->FindEntity("rkisp1_mainpath");
  rkisp1_mainpath->SetFormat(1296 / 2, 972 / 2, V4L2_PIX_FMT_YUV422P);

  Entity *rkisp1_selfpath = media_device->FindEntity("rkisp1_selfpath");
  rkisp1_selfpath->SetFormat(1296, 972, V4L2_PIX_FMT_YUYV);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(55);

  RockchipV4L2Reader v4l2_reader(&event_loop, rkisp1_selfpath->device());

  // TODO(austin): Figure out exposure and stuff.
  /*
  const uint32_t exposure =
      (FLAGS_use_outdoors ? FLAGS_outdoors_exposure : FLAGS_exposure);
  if (exposure > 0) {
    LOG(INFO) << "Setting camera to Manual Exposure mode with exposure = "
              << exposure << " or " << static_cast<double>(exposure) / 10.0
              << " ms";
    v4l2_reader.SetExposure(exposure);
  } else {
    LOG(INFO) << "Setting camera to use Auto Exposure";
    v4l2_reader.UseAutoExposure();
  }
  */

  CameraReader camera_reader(&event_loop, &v4l2_reader);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2022

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::CameraReaderMain();
}
