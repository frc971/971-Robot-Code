#include <chrono>
#include <filesystem>
#include <thread>

#include "glog/logging.h"

#include "Argus/Argus.h"
#include "Argus/EGLStream.h"
#include "Argus/Types.h"
#include "Argus/utils/Error.h"
#include "EGLStream/FrameConsumer.h"
#include "EGLStream/Image.h"
#include "EGLStream/NV/ImageNativeBuffer.h"
#include "HalideBuffer.h"
#include "HalideRuntime.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/orin/ycbcr.h"
#include "frc971/orin/ycbcr422.h"
#include "frc971/vision/vision_generated.h"
#include "nvbufsurface.h"

DEFINE_string(config, "aos_config.json", "Path to the config file to use.");

DEFINE_int32(colorformat, NVBUF_COLOR_FORMAT_NV16,
             "Mode to use.  Don't change unless you know what you are doing.");
DEFINE_int32(camera, 0, "Camera number");
DEFINE_int32(mode, 0, "Mode number to use.");
DEFINE_int32(exposure, 200000, "Exposure number to use.");
DEFINE_int32(gain, 5, "gain number to use.");
DEFINE_int32(width, 1456, "Image width");
DEFINE_int32(height, 1088, "Image height");
DEFINE_double(rgain, 1.0, "R gain");
DEFINE_double(g1gain, 1.0, "G gain");
DEFINE_double(g2gain, 1.0, "G gain");
DEFINE_double(bgain, 1.0, "B gain");
DEFINE_string(channel, "/camera", "Channel name for the image.");

namespace frc971 {

namespace chrono = std::chrono;

// Converts a multiplanar 422 image into a single plane 422 image at the
// provided memory location sutable for putting in a flatbuffer.
void YCbCr422(NvBufSurface *nvbuf_surf, uint8_t *data_pointer) {
  CHECK_EQ(nvbuf_surf->surfaceList->planeParams.width[0],
           nvbuf_surf->surfaceList->planeParams.width[1] * 2);
  CHECK_EQ(nvbuf_surf->surfaceList->planeParams.height[0],
           nvbuf_surf->surfaceList->planeParams.height[1]);
  CHECK_EQ(nvbuf_surf->surfaceList->planeParams.pitch[0], 0x600u);
  CHECK_EQ(nvbuf_surf->surfaceList->planeParams.pitch[1], 0x600u);
  std::array<halide_dimension_t, 2> y_dimensions{{
      {
          /*.min =*/0,
          /*.extent =*/
          static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.width[0]),
          /*.stride =*/1,
          /*.flags =*/0,
      },
      {
          /*.min = */ 0,
          /*.extent =*/
          static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.height[0]),
          /*.stride =*/
          static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.pitch[0]),
          /*.flags =*/0,
      },
  }};

  Halide::Runtime::Buffer<uint8_t, 2> y(
      reinterpret_cast<uint8_t *>(nvbuf_surf->surfaceList->mappedAddr.addr[0]),
      y_dimensions.size(), y_dimensions.data());

  std::array<halide_dimension_t, 3> cbcr_dimensions{
      {{
           /*.min =*/0,
           /*.extent =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.width[1]),
           /*.stride =*/2,
           /*.flags =*/0,
       },
       {
           /*.min =*/0,
           /*.extent =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.height[1]),
           /*.stride =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.pitch[1]),
           /*.flags =*/0,
       },
       {
           /*.min =*/0,
           /*.extent =*/2,
           /*.stride =*/1,
           /*.flags =*/0,
       }}};

  Halide::Runtime::Buffer<uint8_t, 3> cbcr(
      reinterpret_cast<uint8_t *>(nvbuf_surf->surfaceList->mappedAddr.addr[1]),
      cbcr_dimensions.size(), cbcr_dimensions.data());

  std::array<halide_dimension_t, 3> output_dimensions{
      {{
           /*.min =*/0,
           /*.extent =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.width[0]),
           /*.stride =*/2,
           /*.flags =*/0,
       },
       {
           /*.min =*/0,
           /*.extent =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.height[0]),
           /*.stride =*/
           static_cast<int32_t>(nvbuf_surf->surfaceList->planeParams.width[0] *
                                2),
           /*.flags =*/0,
       },
       {
           /*.min =*/0,
           /*.extent =*/2,
           /*.stride =*/1,
           /*.flags =*/0,
       }}};

  Halide::Runtime::Buffer<uint8_t, 3> output(
      data_pointer, output_dimensions.size(), output_dimensions.data());
  ycbcr422(y, cbcr, output);
}

// Helper class to tie a NvBufSurface to an Argus::Buffer.
class DmaBuffer {
 public:
  // Creates a DmaBuffer.  This is a static method so we can make sure it ends
  // up as a unique_ptr so the pointer value doesn't change and break all the
  // links.
  static std::unique_ptr<DmaBuffer> Create(
      const Argus::Size2D<uint32_t> &size, NvBufSurfaceColorFormat color_format,
      NvBufSurfaceLayout layout = NVBUF_LAYOUT_PITCH) {
    std::unique_ptr<DmaBuffer> buffer(new DmaBuffer());

    NvBufSurfaceAllocateParams params;

    params.memtag = NvBufSurfaceTag_CAMERA;
    params.params.width = size.width();
    params.params.height = size.height();
    params.params.colorFormat = color_format;
    params.params.layout = layout;
    params.params.isContiguous = true;
    params.disablePitchPadding = true;
    params.params.memType = NVBUF_MEM_SURFACE_ARRAY;

    NvBufSurface *nvbuf_surf = 0;
    CHECK_EQ(NvBufSurfaceAllocate(&nvbuf_surf, 1, &params), 0);
    buffer->fd_ = nvbuf_surf->surfaceList[0].bufferDesc;

    return buffer;
  }

  // Extracts the DmaBuffer from the Argus::Buffer.
  static DmaBuffer *FromArgusBuffer(Argus::Buffer *buffer) {
    Argus::IBuffer *i_buffer = Argus::interface_cast<Argus::IBuffer>(buffer);
    const DmaBuffer *dmabuf =
        static_cast<const DmaBuffer *>(i_buffer->getClientData());

    return const_cast<DmaBuffer *>(dmabuf);
  }

  // Returns the DMA buffer handle.
  int fd() const { return fd_; }

  // Sets and gets the Argus::Buffer pointer.
  void set_argus_buffer(Argus::Buffer *buffer) { buffer_ = buffer; }
  Argus::Buffer *get_argus_buffer() const { return buffer_; }

  virtual ~DmaBuffer() {
    if (fd_ >= 0) {
      NvBufSurface *nvbuf_surf = 0;
      NvBufSurfaceFromFd(fd_, (void **)(&nvbuf_surf));
      if (nvbuf_surf != NULL) {
        NvBufSurfaceDestroy(nvbuf_surf);
      }
    }
  }

 private:
  // Private to force people to use Create() above.
  DmaBuffer() {}

  int fd_ = -1;
  Argus::Buffer *buffer_ = nullptr;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(FLAGS_config);

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(55);

  aos::Sender<frc971::vision::CameraImage> sender =
      event_loop.MakeSender<frc971::vision::CameraImage>(FLAGS_channel);

  LOG(INFO) << "Started";
  // Initialize the Argus camera provider.
  Argus::UniqueObj<Argus::CameraProvider> camera_provider;
  camera_provider =
      Argus::UniqueObj<Argus::CameraProvider>(Argus::CameraProvider::create());

  // Get the ICameraProvider interface from the global CameraProvider
  Argus::ICameraProvider *i_camera_provider =
      Argus::interface_cast<Argus::ICameraProvider>(camera_provider);
  if (!i_camera_provider) {
    ORIGINATE_ERROR("Failed to get ICameraProvider interface");
  }

  // Get the camera devices.
  std::vector<Argus::CameraDevice *> camera_devices;
  i_camera_provider->getCameraDevices(&camera_devices);
  if (camera_devices.size() == 0) {
    ORIGINATE_ERROR("there are %d cameras", (unsigned)camera_devices.size());
  }

  LOG(INFO) << "Found " << camera_devices.size() << " cameras";
  for (Argus::CameraDevice *camera : camera_devices) {
    Argus::ICameraProperties *iCameraProperties =
        Argus::interface_cast<Argus::ICameraProperties>(camera);
    LOG(INFO) << "Camera " << iCameraProperties->getModelName();
  }

  std::vector<Argus::SensorMode *> sensor_modes;
  Argus::ICameraProperties *iCameraProperties =
      Argus::interface_cast<Argus::ICameraProperties>(
          camera_devices[FLAGS_camera]);
  if (!iCameraProperties)
    ORIGINATE_ERROR("Failed to get ICameraProperties Interface");
  // Get available Sensor Modes
  iCameraProperties->getAllSensorModes(&sensor_modes);
  LOG(INFO) << "Found " << sensor_modes.size() << " modes";

  for (Argus::SensorMode *mode : sensor_modes) {
    Argus::ISensorMode *imode = Argus::interface_cast<Argus::ISensorMode>(mode);
    LOG(INFO) << imode->getResolution().width() << " x "
              << imode->getResolution().height();
    LOG(INFO) << "type " << imode->getSensorModeType().getName();
    LOG(INFO) << "exposure min " << imode->getExposureTimeRange().min();
    LOG(INFO) << "exposure max " << imode->getExposureTimeRange().max();
  }
  if (sensor_modes.size() <= 0) {
    ORIGINATE_ERROR("Preview Sensor Mode %d not available", 0);
  }

  Argus::ISensorMode *i_sensor_mode =
      Argus::interface_cast<Argus::ISensorMode>(sensor_modes[FLAGS_mode]);
  if (!i_sensor_mode) {
    ORIGINATE_ERROR("Failed to get SensorMode interface");
  }

  {
    auto range = i_sensor_mode->getFrameDurationRange();
    LOG(INFO) << "Min: " << range.min() << ", " << range.max();
    LOG(INFO) << "type " << i_sensor_mode->getSensorModeType().getName();
  }

  // Create the capture session using the first device and get the core
  // interface.
  Argus::UniqueObj<Argus::CaptureSession> capture_session;
  capture_session.reset(
      i_camera_provider->createCaptureSession(camera_devices[FLAGS_camera]));
  Argus::ICaptureSession *i_capture_session =
      Argus::interface_cast<Argus::ICaptureSession>(capture_session);
  if (!i_capture_session) {
    ORIGINATE_ERROR("Failed to create CaptureSession");
  }

  EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  CHECK_NE(egl_display, EGL_NO_DISPLAY) << ": Failed to open display";

  // Create the OutputStream.
  Argus::UniqueObj<Argus::OutputStreamSettings> stream_settings(
      i_capture_session->createOutputStreamSettings(Argus::STREAM_TYPE_BUFFER));

  Argus::IBufferOutputStreamSettings *i_buffer_output_stream_settings =
      Argus::interface_cast<Argus::IBufferOutputStreamSettings>(
          stream_settings);
  CHECK(i_buffer_output_stream_settings != nullptr);
  i_buffer_output_stream_settings->setBufferType(Argus::BUFFER_TYPE_EGL_IMAGE);
  i_buffer_output_stream_settings->setMetadataEnable(true);
  LOG(INFO) << "Type: "
            << i_buffer_output_stream_settings->getBufferType().getName();

  Argus::UniqueObj<Argus::OutputStream> output_stream(
      i_capture_session->createOutputStream(stream_settings.get()));
  LOG(INFO) << "Got sream";

  Argus::IBufferOutputStream *i_buffer_output_stream =
      Argus::interface_cast<Argus::IBufferOutputStream>(output_stream);
  CHECK(i_buffer_output_stream != nullptr);

  // Build the DmaBuffers
  std::array<std::unique_ptr<DmaBuffer>, 10> native_buffers;
  for (size_t i = 0; i < native_buffers.size(); ++i) {
    native_buffers[i] = DmaBuffer::Create(
        i_sensor_mode->getResolution(),
        static_cast<NvBufSurfaceColorFormat>(FLAGS_colorformat),
        NVBUF_LAYOUT_PITCH);
  }

  std::array<NvBufSurface *, 10> surf;

  // Create EGLImages from the native buffers
  std::array<EGLImageKHR, 10> egl_images;
  for (size_t i = 0; i < egl_images.size(); i++) {
    int ret = 0;

    ret = NvBufSurfaceFromFd(native_buffers[i]->fd(), (void **)(&surf[i]));
    CHECK(ret == 0) << ": NvBufSurfaceFromFd failed";

    ret = NvBufSurfaceMapEglImage(surf[i], 0);
    CHECK(ret == 0) << ": NvBufSurfaceMapEglImage failed";

    egl_images[i] = surf[i]->surfaceList[0].mappedAddr.eglImage;
    CHECK(egl_images[i] != EGL_NO_IMAGE_KHR) << ": Failed to create EGLImage";
  }

  // Create the BufferSettings object to configure Buffer creation.
  Argus::UniqueObj<Argus::BufferSettings> buffer_settings(
      i_buffer_output_stream->createBufferSettings());
  Argus::IEGLImageBufferSettings *i_buffer_settings =
      Argus::interface_cast<Argus::IEGLImageBufferSettings>(buffer_settings);
  if (!i_buffer_settings) ORIGINATE_ERROR("Failed to create BufferSettings");

  // Create the Buffers for each EGLImage (and release to the stream for initial
  // capture use)
  std::array<Argus::UniqueObj<Argus::Buffer>, 10> buffers;
  for (size_t i = 0; i < buffers.size(); i++) {
    i_buffer_settings->setEGLImage(egl_images[i]);
    i_buffer_settings->setEGLDisplay(egl_display);
    buffers[i].reset(
        i_buffer_output_stream->createBuffer(buffer_settings.get()));
    Argus::IBuffer *i_buffer =
        Argus::interface_cast<Argus::IBuffer>(buffers[i]);

    // Ties Argus::Buffer and DmaBuffer together.
    i_buffer->setClientData(native_buffers[i].get());
    native_buffers[i]->set_argus_buffer(buffers[i].get());

    CHECK(Argus::interface_cast<Argus::IEGLImageBuffer>(buffers[i]) != nullptr)
        << ": Failed to create Buffer";

    if (i_buffer_output_stream->releaseBuffer(buffers[i].get()) !=
        Argus::STATUS_OK)
      ORIGINATE_ERROR("Failed to release Buffer for capture use");
  }

  Argus::UniqueObj<Argus::Request> request(i_capture_session->createRequest());
  Argus::IRequest *i_request = Argus::interface_cast<Argus::IRequest>(request);
  CHECK(i_request);

  Argus::IAutoControlSettings *i_auto_control_settings =
      Argus::interface_cast<Argus::IAutoControlSettings>(
          i_request->getAutoControlSettings());
  CHECK(i_auto_control_settings != nullptr);
  i_auto_control_settings->setAwbMode(Argus::AWB_MODE_OFF);

  i_auto_control_settings->setAeLock(false);
  Argus::Range<float> isp_digital_gain_range;
  isp_digital_gain_range.min() = 1;
  isp_digital_gain_range.max() = 1;
  i_auto_control_settings->setIspDigitalGainRange(isp_digital_gain_range);

  Argus::IEdgeEnhanceSettings *i_ee_settings =
      Argus::interface_cast<Argus::IEdgeEnhanceSettings>(request);
  CHECK(i_ee_settings != nullptr);

  i_ee_settings->setEdgeEnhanceStrength(0);

  i_request->enableOutputStream(output_stream.get());

  Argus::ISourceSettings *i_source_settings =
      Argus::interface_cast<Argus::ISourceSettings>(
          i_request->getSourceSettings());
  CHECK(i_source_settings != nullptr);

  i_source_settings->setFrameDurationRange(
      i_sensor_mode->getFrameDurationRange().min());
  i_source_settings->setSensorMode(sensor_modes[FLAGS_mode]);

  Argus::Range<float> sensor_mode_analog_gain_range;
  sensor_mode_analog_gain_range.min() = FLAGS_gain;
  sensor_mode_analog_gain_range.max() = FLAGS_gain;
  i_source_settings->setGainRange(sensor_mode_analog_gain_range);

  Argus::Range<uint64_t> limit_exposure_time_range;
  limit_exposure_time_range.min() = FLAGS_exposure;
  limit_exposure_time_range.max() = FLAGS_exposure;
  i_source_settings->setExposureTimeRange(limit_exposure_time_range);

  if (i_capture_session->repeat(request.get()) != Argus::STATUS_OK) {
    LOG(ERROR) << "Failed to submit repeat";
  }

  LOG(INFO) << "Session submitted";

  // Run.
  //
  // TODO(austin): Use the event loop a bit better...  That'll let us set
  // priority + get stats.  Timer which always repeats "now" ?
  aos::monotonic_clock::time_point last_time = aos::monotonic_clock::epoch();
  while (true) {
    VLOG(1) << "Going for frame";
    Argus::Status status;

    Argus::Buffer *buffer = i_buffer_output_stream->acquireBuffer(
        std::chrono::nanoseconds(std::chrono::seconds(5)).count(), &status);

    if (status == Argus::STATUS_END_OF_STREAM) {
      break;
    }

    const aos::monotonic_clock::time_point now = aos::monotonic_clock::now();

    DmaBuffer *dmabuf = DmaBuffer::FromArgusBuffer(buffer);
    int dmabuf_fd = dmabuf->fd();

    Argus::IBuffer *ibuffer = Argus::interface_cast<Argus::IBuffer>(buffer);
    CHECK(ibuffer != nullptr);

    const Argus::CaptureMetadata *metadata = ibuffer->getMetadata();
    const Argus::ICaptureMetadata *imetadata =
        Argus::interface_cast<const Argus::ICaptureMetadata>(metadata);

    NvBufSurface *nvbuf_surf = 0;
    CHECK_EQ(NvBufSurfaceFromFd(dmabuf_fd, (void **)(&nvbuf_surf)), 0);

    CHECK_EQ(NvBufSurfaceMap(nvbuf_surf, -1, -1, NVBUF_MAP_READ), 0);
    VLOG(1) << "Mapped";
    NvBufSurfaceSyncForCpu(nvbuf_surf, -1, -1);

    VLOG(1) << "Planes " << nvbuf_surf->surfaceList->planeParams.num_planes
            << " colorFormat " << nvbuf_surf->surfaceList->colorFormat;
    for (size_t i = 0; i < nvbuf_surf->surfaceList->planeParams.num_planes;
         ++i) {
      VLOG(1) << "Address "
              << static_cast<void *>(
                     nvbuf_surf->surfaceList->mappedAddr.addr[i])
              << ", pitch " << nvbuf_surf->surfaceList->planeParams.pitch[i]
              << " height " << nvbuf_surf->surfaceList->planeParams.height[i]
              << " width " << nvbuf_surf->surfaceList->planeParams.width[i]
              << " bytes per pixel "
              << nvbuf_surf->surfaceList->planeParams.bytesPerPix[i];
    }
    CHECK_EQ(nvbuf_surf->surfaceList->planeParams.width[0],
             static_cast<size_t>(FLAGS_width));
    CHECK_EQ(nvbuf_surf->surfaceList->planeParams.height[0],
             static_cast<size_t>(FLAGS_height));

    aos::Sender<frc971::vision::CameraImage>::Builder builder =
        sender.MakeBuilder();

    uint8_t *data_pointer = nullptr;
    builder.fbb()->StartIndeterminateVector(FLAGS_width * FLAGS_height * 2, 1,
                                            64, &data_pointer);

    YCbCr422(nvbuf_surf, data_pointer);
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->EndIndeterminateVector(FLAGS_width * FLAGS_height * 2,
                                              1);

    auto image_builder = builder.MakeBuilder<frc971::vision::CameraImage>();
    image_builder.add_data(data_offset);
    image_builder.add_rows(FLAGS_height);
    image_builder.add_cols(FLAGS_width);
    image_builder.add_monotonic_timestamp_ns(imetadata->getFrameReadoutTime());
    builder.CheckOk(builder.Send(image_builder.Finish()));

    const aos::monotonic_clock::time_point after_send =
        aos::monotonic_clock::now();

    CHECK_EQ(NvBufSurfaceUnMap(nvbuf_surf, -1, -1), 0);

    CHECK(imetadata);
    VLOG(1) << "Got " << imetadata->getCaptureId() << " delay "
            << chrono::duration<double>(
                   chrono::nanoseconds((now.time_since_epoch().count() -
                                        (imetadata->getSensorTimestamp() +
                                         imetadata->getFrameReadoutTime()))))
                   .count()
            << " mmap " << chrono::duration<double>(after_send - now).count()
            << "sec dt " << chrono::duration<double>(now - last_time).count()
            << "sec " << dmabuf << " exposure "
            << imetadata->getSensorExposureTime();
    i_buffer_output_stream->releaseBuffer(buffer);

    last_time = now;
  }

  i_capture_session->stopRepeat();
  i_buffer_output_stream->endOfStream();
  i_capture_session->waitForIdle();

  output_stream.reset();

  for (uint32_t i = 0; i < surf.size(); i++) {
    NvBufSurfaceUnMapEglImage(surf[i], 0);
  }

  eglTerminate(egl_display);
  return 0;
}

};  // namespace frc971

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc971::Main();
}

// I tried every different format option.  Here's what worked and didn't work.
//
// NVBUF_COLOR_FORMAT_RGB,

// NVBUF_COLOR_FORMAT_YUYV,  // Failed
// NVBUF_COLOR_FORMAT_NV24,  // Works
//  NVBUF_COLOR_FORMAT_UYVY,  // Failed
// NVBUF_COLOR_FORMAT_YUV420,  // Failed with error.

// NVBUF_COLOR_FORMAT_GRAY8,  // unsupported
// NVBUF_COLOR_FORMAT_YUV420,  // unsupported
// NVBUF_COLOR_FORMAT_YVU420,  // unsupported

// NVBUF_COLOR_FORMAT_YUV420_ER,  // unsupported
// NVBUF_COLOR_FORMAT_YVU420_ER,  // unsupported
//
///** Specifies BT.601 colorspace - Y/CbCr 4:2:0 multi-planar. */
// NVBUF_COLOR_FORMAT_NV12,  // Works!  pitch 2048 height 1080 width
// 1920 colorFormat 6 planes 2 bytes per pixel 1  delay 0.00203304
// mmap 0.000340288sec dt 0.0166379sec
//
///** Specifies BT.601 colorspace - Y/CbCr ER 4:2:0 multi-planar. */
// NVBUF_COLOR_FORMAT_NV12_ER,  // Works!  pitch 2048 height 1080
// width 1920 colorFormat 7 planes 2 bytes per pixel 1
///** Specifies BT.601 colorspace - Y/CbCr 4:2:0 multi-planar. */
// NVBUF_COLOR_FORMAT_NV21,  // Works!  pitch 2048 height 1080 width
// 1920 colorFormat 8 planes 2 bytes per pixel 1
///** Specifies BT.601 colorspace - Y/CbCr ER 4:2:0 multi-planar. */
// NVBUF_COLOR_FORMAT_NV21_ER,  // Works!  pitch 2048 height 1080
// width 1920 colorFormat 9 planes 2 bytes per pixel 1
//
//
// NVBUF_COLOR_FORMAT_UYVY,  // works with an error?!?
// NVBUF_COLOR_FORMAT_UYVY_ER,  // unsupported 11
// NVBUF_COLOR_FORMAT_VYUY,  // unsupported 12
// NVBUF_COLOR_FORMAT_VYUY_ER,  // unsupported 13
// NVBUF_COLOR_FORMAT_YUYV,  // unsupported 14
// NVBUF_COLOR_FORMAT_YUYV_ER,  // unsupported 15
// NVBUF_COLOR_FORMAT_YVYU,  // unsupported 16
// NVBUF_COLOR_FORMAT_YVYU_ER,  // unsupported 17
// NVBUF_COLOR_FORMAT_YUV444,  // unsupported 18
// NVBUF_COLOR_FORMAT_RGBA,    // unsupported 19
// NVBUF_COLOR_FORMAT_BGRA,    // unsupported 20
// NVBUF_COLOR_FORMAT_ARGB,    // unsupported 21
// NVBUF_COLOR_FORMAT_ABGR,    // unsupported 22
// NVBUF_COLOR_FORMAT_RGBx,    // unsupported 23
// NVBUF_COLOR_FORMAT_BGRx,    // unsupported 24
// NVBUF_COLOR_FORMAT_xRGB,    // unsupported 25
// NVBUF_COLOR_FORMAT_xBGR,    // unsupported 26
// NVBUF_COLOR_FORMAT_RGB,    // unsupported 27
// NVBUF_COLOR_FORMAT_BGR,    // unsupported 28
// NVBUF_COLOR_FORMAT_NV12_10LE, // unsupported 29
// NVBUF_COLOR_FORMAT_NV12_12LE, // unsupported 30
// NVBUF_COLOR_FORMAT_YUV420_709,    // unsupported 31
// NVBUF_COLOR_FORMAT_YUV420_709_ER,    // unsupported 32
// NVBUF_COLOR_FORMAT_NV12_709,    // works pitch 2048 height 1080
// width 1920 colorFormat 33 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_NV12_709_ER,    // works pitch 2048 height 1080
// width 1920 colorFormat 34 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_YUV420_2020,    // unsupported 35
// NVBUF_COLOR_FORMAT_NV12_2020,    // unsupported 36
// NVBUF_COLOR_FORMAT_NV12_10LE_ER,    // unsupported 37
// NVBUF_COLOR_FORMAT_NV12_10LE_709,    // unsupported 38
// NVBUF_COLOR_FORMAT_NV12_10LE_709_ER,    // unsupported 39
// NVBUF_COLOR_FORMAT_NV12_10LE_2020,    // unsupported 40
// NVBUF_COLOR_FORMAT_SIGNED_R16G16,    // unsupported 41
// NVBUF_COLOR_FORMAT_R8_G8_B8,    // unsupported 42
// NVBUF_COLOR_FORMAT_B8_G8_R8,    // unsupported 43
// NVBUF_COLOR_FORMAT_R32F_G32F_B32F,    // unsupported 44
// NVBUF_COLOR_FORMAT_B32F_G32F_R32F,    // unsupported 45
// NVBUF_COLOR_FORMAT_YUV422,    // unsupported 46
// NVBUF_COLOR_FORMAT_NV21_10LE,    // unsupported 47
// NVBUF_COLOR_FORMAT_NV21_12LE,    // unsupported 48
// NVBUF_COLOR_FORMAT_NV12_12LE_2020,    // unsupported 49
///** Specifies BT.601 colorspace - Y/CbCr 4:2:2 multi-planar. */
// NVBUF_COLOR_FORMAT_NV16,    // works  pitch 2048 height 1080 width
// 1920 colorFormat 50 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_NV16_10LE,    // unsupported 51
///** Specifies BT.601 colorspace - Y/CbCr 4:4:4 multi-planar. */
// NVBUF_COLOR_FORMAT_NV24,    // works     pitch 2048 height 1080
// width 1920 colorFormat 52 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_NV24_10LE,    // unsupported 53
//
// NVBUF_COLOR_FORMAT_NV16_ER,    // works   pitch 2048 height 1080
// width 1920 colorFormat 54 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_NV24_ER,    // works    pitch 2048 height 1080
// width 1920 colorFormat 55 planes 2 bytes per pixel 1
// NVBUF_COLOR_FORMAT_NV16_709,    // unsupported 56
// NVBUF_COLOR_FORMAT_NV24_709,    // unsupported 57
// NVBUF_COLOR_FORMAT_NV16_709_ER,    // unsupported 58
// NVBUF_COLOR_FORMAT_NV24_709_ER,    // unsupported 59
// NVBUF_COLOR_FORMAT_NV24_10LE_709,    // unsupported 60
// NVBUF_COLOR_FORMAT_NV24_10LE_709_ER,    // unsupported 61
// NVBUF_COLOR_FORMAT_NV24_10LE_2020,    // unsupported 62
// NVBUF_COLOR_FORMAT_NV24_12LE_2020,    // unsupported 63
// NVBUF_COLOR_FORMAT_RGBA_10_10_10_2_709,    // unsupported 64
// NVBUF_COLOR_FORMAT_RGBA_10_10_10_2_2020,    // unsupported 65
// NVBUF_COLOR_FORMAT_BGRA_10_10_10_2_709,    // unsupported 66
// NVBUF_COLOR_FORMAT_BGRA_10_10_10_2_2020,    // unsupported 67
// NVBUF_COLOR_FORMAT_A32,    // unsupported 68
// NVBUF_COLOR_FORMAT_UYVP,    // unsupported 69
// NVBUF_COLOR_FORMAT_UYVP_ER    // unsupported 70

// NVBUF_COLOR_FORMAT_ABGR,
// NVBUF_COLOR_FORMAT_ARGB,
