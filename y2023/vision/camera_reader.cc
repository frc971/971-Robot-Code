#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc971/vision/media_device.h"
#include "frc971/vision/v4l2_reader.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_bool(lowlight_camera, false, "Switch to use imx462 image sensor.");

namespace y2023 {
namespace vision {
namespace {

using namespace frc971::vision;

void CameraReaderMain() {
  std::optional<MediaDevice> media_device = FindMediaDevice("platform:rkisp1");

  if (VLOG_IS_ON(1)) {
    media_device->Log();
  }

  int width = 1296;
  int height = 972;
  int color_format = MEDIA_BUS_FMT_SBGGR10_1X10;
  std::string camera_device_string = "ov5647 4-0036";
  if (FLAGS_lowlight_camera) {
    width = 1920;
    height = 1080;
    color_format = MEDIA_BUS_FMT_SRGGB10_1X10;
    camera_device_string = "imx290 4-0036";
  }

  media_device->Reset();

  Entity *camera = media_device->FindEntity(camera_device_string);
  camera->pads()[0]->SetSubdevFormat(width, height, color_format);

  Entity *rkisp1_csi = media_device->FindEntity("rkisp1_csi");
  rkisp1_csi->pads()[0]->SetSubdevFormat(width, height, color_format);
  rkisp1_csi->pads()[1]->SetSubdevFormat(width, height, color_format);

  // TODO(austin): Should we set this on the link?
  Entity *rkisp1_isp = media_device->FindEntity("rkisp1_isp");
  rkisp1_isp->pads(0)->SetSubdevFormat(width, height, color_format);
  rkisp1_isp->pads(0)->SetSubdevCrop(width, height);

  rkisp1_isp->pads(2)->SetSubdevFormat(width, height, MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_isp->pads(2)->SetSubdevCrop(width, height);

  Entity *rkisp1_resizer_selfpath =
      media_device->FindEntity("rkisp1_resizer_selfpath");
  rkisp1_resizer_selfpath->pads(0)->SetSubdevFormat(width, height,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(1)->SetSubdevFormat(width, height,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(0)->SetSubdevCrop(width, height);

  Entity *rkisp1_resizer_mainpath =
      media_device->FindEntity("rkisp1_resizer_mainpath");
  rkisp1_resizer_mainpath->pads(0)->SetSubdevFormat(width, height,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);

  rkisp1_resizer_mainpath->pads(0)->SetSubdevCrop(width, height);
  rkisp1_resizer_mainpath->pads(1)->SetSubdevFormat(width / 2, height / 2,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);

  Entity *rkisp1_mainpath = media_device->FindEntity("rkisp1_mainpath");
  rkisp1_mainpath->SetFormat(width / 2, height / 2, V4L2_PIX_FMT_YUV422P);

  Entity *rkisp1_selfpath = media_device->FindEntity("rkisp1_selfpath");
  rkisp1_selfpath->SetFormat(width, height, V4L2_PIX_FMT_YUYV);

  media_device->Enable(
      media_device->FindLink(camera_device_string, 0, "rkisp1_csi", 0));
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
                                 rkisp1_selfpath->device(), camera->device());

  if (FLAGS_lowlight_camera) {
    v4l2_reader.SetGain(72);
    v4l2_reader.SetExposure(30);
    v4l2_reader.SetBlanking(2480, 45);
  } else {
    v4l2_reader.SetGainExt(1000);
    v4l2_reader.SetExposure(1000);
  }

  event_loop.Run();
}

}  // namespace
}  // namespace vision
}  // namespace y2023

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2023::vision::CameraReaderMain();
}
