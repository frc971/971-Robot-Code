#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc971/vision/media_device.h"
#include "frc971/vision/v4l2_reader.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

namespace y2023 {
namespace vision {
namespace {

using namespace frc971::vision;

void CameraReaderMain() {
  std::optional<MediaDevice> media_device = FindMediaDevice("platform:rkisp1");

  if (VLOG_IS_ON(1)) {
    media_device->Log();
  }

  media_device->Reset();

  Entity *ov5647 = media_device->FindEntity("ov5647 4-0036");
  ov5647->pads()[0]->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);

  Entity *rkisp1_csi = media_device->FindEntity("rkisp1_csi");
  rkisp1_csi->pads()[0]->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);
  rkisp1_csi->pads()[1]->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);

  // TODO(austin): Should we set this on the link?
  Entity *rkisp1_isp = media_device->FindEntity("rkisp1_isp");
  rkisp1_isp->pads(0)->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_SBGGR10_1X10);
  rkisp1_isp->pads(0)->SetSubdevCrop(1296, 972);

  rkisp1_isp->pads(2)->SetSubdevFormat(1296, 972, MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_isp->pads(2)->SetSubdevCrop(1296, 972);

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
  rkisp1_resizer_mainpath->pads(0)->SetSubdevCrop(1296, 972);

  rkisp1_resizer_mainpath->pads(1)->SetSubdevFormat(1296 / 2, 972 / 2,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);

  Entity *rkisp1_mainpath = media_device->FindEntity("rkisp1_mainpath");
  rkisp1_mainpath->SetFormat(1296 / 2, 972 / 2, V4L2_PIX_FMT_YUV422P);

  Entity *rkisp1_selfpath = media_device->FindEntity("rkisp1_selfpath");
  rkisp1_selfpath->SetFormat(1296, 972, V4L2_PIX_FMT_YUYV);

  media_device->Enable(
      media_device->FindLink("ov5647 4-0036", 0, "rkisp1_csi", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_csi", 1, "rkisp1_isp", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_isp", 2, "rkisp1_resizer_selfpath", 0));
  media_device->Enable(
      media_device->FindLink("rkisp1_isp", 2, "rkisp1_resizer_mainpath", 0));

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(55);
  event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({2}));


  RockchipV4L2Reader v4l2_reader(&event_loop, event_loop.epoll(),
                                 rkisp1_selfpath->device(), ov5647->device());

  v4l2_reader.SetGain(1000);
  v4l2_reader.SetExposure(1000);

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::CameraReaderMain();
}
