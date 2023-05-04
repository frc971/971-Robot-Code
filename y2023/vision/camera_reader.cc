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
DEFINE_int32(gain, 150, "analogue_gain");

DEFINE_double(red, 1.252, "Red gain");
DEFINE_double(green, 1, "Green gain");
DEFINE_double(blue, 1.96, "Blue gain");
DEFINE_double(exposure, 150, "Camera exposure");
DEFINE_bool(send_downsized_images, false,
            "Whether to send downsized image for driver cam streaming.");

namespace y2023 {
namespace vision {
namespace {

using namespace frc971::vision;

void CameraReaderMain() {
  std::optional<MediaDevice> media_device = FindMediaDevice("platform:rkisp1");

  if (VLOG_IS_ON(1)) {
    media_device->Log();
  }

  const int kWidth = (FLAGS_lowlight_camera ? 1920 : 1296);
  const int kHeight = (FLAGS_lowlight_camera ? 1080 : 972);
  const int kColorFormat = (FLAGS_lowlight_camera ? MEDIA_BUS_FMT_SRGGB10_1X10
                                                  : MEDIA_BUS_FMT_SBGGR10_1X10);

  const std::string_view kCameraDeviceString =
      (FLAGS_lowlight_camera ? "arducam-pivariety 4-000c" : "ov5647 4-0036");

  // Scale down the selfpath images so we can log at 30 Hz (but still detect
  // april tags at a far enough distance)
  const double kSelfpathScalar = 2.0 / 3.0;
  const int kSelfpathWidth = kWidth * kSelfpathScalar;
  const int kSelfpathHeight = kHeight * kSelfpathScalar;

  // Send heavily downsized images to the drivercam. They go over the network,
  // and in this case frame rate is more important than quality
  constexpr int kMainpathWidth = 640;
  constexpr int kMainpathHeight = 480;

  media_device->Reset();

  Entity *camera = media_device->FindEntity(kCameraDeviceString);
  camera->pads()[0]->SetSubdevFormat(kWidth, kHeight, kColorFormat);

  Entity *rkisp1_csi = media_device->FindEntity("rkisp1_csi");
  rkisp1_csi->pads()[0]->SetSubdevFormat(kWidth, kHeight, kColorFormat);
  rkisp1_csi->pads()[1]->SetSubdevFormat(kWidth, kHeight, kColorFormat);

  // TODO(austin): Should we set this on the link?
  Entity *rkisp1_isp = media_device->FindEntity("rkisp1_isp");
  rkisp1_isp->pads(0)->SetSubdevFormat(kWidth, kHeight, kColorFormat);
  rkisp1_isp->pads(0)->SetSubdevCrop(kWidth, kHeight);

  rkisp1_isp->pads(2)->SetSubdevFormat(kWidth, kHeight,
                                       MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_isp->pads(2)->SetSubdevCrop(kWidth, kHeight);

  Entity *rkisp1_resizer_selfpath =
      media_device->FindEntity("rkisp1_resizer_selfpath");
  rkisp1_resizer_selfpath->pads(0)->SetSubdevFormat(kWidth, kHeight,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(1)->SetSubdevFormat(
      kSelfpathWidth, kSelfpathHeight, MEDIA_BUS_FMT_YUYV8_2X8);
  rkisp1_resizer_selfpath->pads(0)->SetSubdevCrop(kWidth, kHeight);

  Entity *rkisp1_resizer_mainpath =
      media_device->FindEntity("rkisp1_resizer_mainpath");
  rkisp1_resizer_mainpath->pads(0)->SetSubdevFormat(kWidth, kHeight,
                                                    MEDIA_BUS_FMT_YUYV8_2X8);

  rkisp1_resizer_mainpath->pads(0)->SetSubdevCrop(kWidth, kHeight);
  rkisp1_resizer_mainpath->pads(1)->SetSubdevFormat(
      kMainpathWidth, kMainpathHeight, MEDIA_BUS_FMT_YUYV8_2X8);

  Entity *rkisp1_mainpath = media_device->FindEntity("rkisp1_mainpath");
  rkisp1_mainpath->SetFormat(kMainpathWidth, kMainpathHeight,
                             V4L2_PIX_FMT_YUYV);

  Entity *rkisp1_selfpath = media_device->FindEntity("rkisp1_selfpath");
  rkisp1_selfpath->SetFormat(kSelfpathWidth, kSelfpathHeight,
                             V4L2_PIX_FMT_YUYV);

  media_device->Enable(
      media_device->FindLink(kCameraDeviceString, 0, "rkisp1_csi", 0));
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

  // Reader for vision processing
  RockchipV4L2Reader v4l2_reader_selfpath(&event_loop, event_loop.epoll(),
                                          rkisp1_selfpath->device(),
                                          camera->device());
  if (FLAGS_lowlight_camera) {
    v4l2_reader_selfpath.SetGainExt(FLAGS_gain);
    v4l2_reader_selfpath.SetVerticalBlanking(1000);
    v4l2_reader_selfpath.SetExposure(FLAGS_exposure);
  } else {
    v4l2_reader_selfpath.SetGainExt(1000);
    v4l2_reader_selfpath.SetExposure(1000);
  }

  std::unique_ptr<RockchipV4L2Reader> v4l2_reader_mainpath;
  if (FLAGS_send_downsized_images) {
    // Reader for driver cam streaming on logger pi, sending lower res images
    v4l2_reader_mainpath = std::make_unique<RockchipV4L2Reader>(
        &event_loop, event_loop.epoll(), rkisp1_mainpath->device(),
        camera->device(), "/camera/downsized");
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
