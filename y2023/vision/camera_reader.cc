#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc971/vision/media_device.h"
#include "frc971/vision/v4l2_reader.h"
#include "y2023/vision/rkisp1-config.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");
DEFINE_bool(lowlight_camera, true, "Switch to use imx462 image sensor.");

DEFINE_double(red, 1.252, "Red gain");
DEFINE_double(green, 1, "Green gain");
DEFINE_double(blue, 1.96, "Blue gain");
DEFINE_double(exposure, 150, "Camera exposure");

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
    camera_device_string = "arducam-pivariety 4-000c";
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
  rkisp1_resizer_selfpath->pads(1)->SetSubdevFormat(
      width * 2 / 3, height * 2 / 3, MEDIA_BUS_FMT_YUYV8_2X8);
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
  rkisp1_selfpath->SetFormat(width * 2 / 3, height * 2 / 3, V4L2_PIX_FMT_YUYV);

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
    v4l2_reader.SetGainExt(100);
    v4l2_reader.SetVerticalBlanking(1000);
    v4l2_reader.SetExposure(FLAGS_exposure);
  } else {
    v4l2_reader.SetGainExt(1000);
    v4l2_reader.SetExposure(1000);
  }

  {
    Entity *rkisp1_params = media_device->FindEntity("rkisp1_params");

    LOG(INFO) << "Opening " << rkisp1_params->device();
    aos::ScopedFD fd(open(rkisp1_params->device().c_str(), O_RDWR));
    PCHECK(fd >= 0);

    struct v4l2_capability capability;
    memset(&capability, 0, sizeof(capability));
    PCHECK(ioctl(fd.get(), VIDIOC_QUERYCAP, &capability) == 0);
    CHECK(capability.device_caps & V4L2_CAP_META_OUTPUT);

    // V4L2_META_FMT_RK_ISP1_PARAMS
    // RK1P
    uint32_t meta_params_format = (uint32_t)('R') | ((uint32_t)('K') << 8) |
                                  ((uint32_t)('1') << 16) |
                                  ((uint32_t)('P') << 24);
    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_META_OUTPUT;

    PCHECK(ioctl(fd.get(), VIDIOC_G_FMT, &format) == 0);
    CHECK_EQ(format.fmt.meta.buffersize, 3048ul);
    CHECK_EQ(format.fmt.meta.dataformat, meta_params_format);

    struct v4l2_requestbuffers request;
    memset(&request, 0, sizeof(request));
    request.count = 1;
    request.type = V4L2_BUF_TYPE_META_OUTPUT;
    request.memory = V4L2_MEMORY_USERPTR;
    PCHECK(ioctl(fd.get(), VIDIOC_REQBUFS, &request) == 0);

    struct rkisp1_params_cfg configuration;
    memset(&configuration, 0, sizeof(configuration));

    configuration.module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

    configuration.others.awb_gain_config.gain_red = 256 * FLAGS_red;
    configuration.others.awb_gain_config.gain_green_r = 256 * FLAGS_green;
    configuration.others.awb_gain_config.gain_blue = 256 * FLAGS_blue;
    configuration.others.awb_gain_config.gain_green_b = 256 * FLAGS_green;

    // Enable the AWB gains
    configuration.module_en_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;
    configuration.module_ens |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.memory = V4L2_MEMORY_USERPTR;
    buffer.index = 0;
    buffer.type = V4L2_BUF_TYPE_META_OUTPUT;
    buffer.m.userptr = reinterpret_cast<uintptr_t>(&configuration);
    buffer.length = format.fmt.meta.buffersize;

    int type = V4L2_BUF_TYPE_META_OUTPUT;
    PCHECK(ioctl(fd.get(), VIDIOC_STREAMON, &type) == 0);

    PCHECK(ioctl(fd.get(), VIDIOC_QBUF, &buffer) == 0);
    CHECK(buffer.flags & V4L2_BUF_FLAG_QUEUED);

    PCHECK(ioctl(fd.get(), VIDIOC_DQBUF, &buffer) == 0);
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
