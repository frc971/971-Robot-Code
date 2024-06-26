#include <dirent.h>

#include <chrono>
#include <filesystem>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

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
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc971/orin/ycbcr.h"
#include "frc971/orin/ycbcr422.h"
#include "frc971/vision/vision_generated.h"
#include "nvbufsurface.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(int32_t, colorformat, NVBUF_COLOR_FORMAT_NV16,
          "Mode to use.  Don't change unless you know what you are doing.");
ABSL_FLAG(int32_t, camera, 0, "Camera number");
ABSL_FLAG(int32_t, mode, 0, "Mode number to use.");
ABSL_FLAG(int32_t, exposure, 100, "Exposure number to use.");
ABSL_FLAG(int32_t, gain, 5, "gain number to use.");
ABSL_FLAG(int32_t, width, 1456, "Image width");
ABSL_FLAG(int32_t, height, 1088, "Image height");
ABSL_FLAG(double, rgain, 1.0, "R gain");
ABSL_FLAG(double, g1gain, 1.0, "G gain");
ABSL_FLAG(double, g2gain, 1.0, "G gain");
ABSL_FLAG(double, bgain, 1.0, "B gain");
ABSL_FLAG(std::string, channel, "/camera", "Channel name for the image.");

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
    aos::ScopedNotRealtime nrt;
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

// Class to make it easy to interact with an Argus camera inside an event loop.
class ArgusCamera {
 public:
  ArgusCamera(Argus::ICameraProvider *i_camera_provider,
              Argus::CameraDevice *camera_device) {
    std::vector<Argus::SensorMode *> sensor_modes;
    Argus::ICameraProperties *i_camera_properties =
        Argus::interface_cast<Argus::ICameraProperties>(camera_device);
    CHECK(i_camera_properties) << "Failed to get ICameraProperties Interface";
    // Get available Sensor Modes
    i_camera_properties->getAllSensorModes(&sensor_modes);
    LOG(INFO) << "Found " << sensor_modes.size() << " modes";

    for (Argus::SensorMode *mode : sensor_modes) {
      Argus::ISensorMode *imode =
          Argus::interface_cast<Argus::ISensorMode>(mode);
      LOG(INFO) << imode->getResolution().width() << " x "
                << imode->getResolution().height();
      LOG(INFO) << "type " << imode->getSensorModeType().getName();
      LOG(INFO) << "exposure min " << imode->getExposureTimeRange().min();
      LOG(INFO) << "exposure max " << imode->getExposureTimeRange().max();
    }
    CHECK_GT(sensor_modes.size(), 0u);

    Argus::ISensorMode *i_sensor_mode =
        Argus::interface_cast<Argus::ISensorMode>(
            sensor_modes[absl::GetFlag(FLAGS_mode)]);
    CHECK(i_sensor_mode);

    {
      auto range = i_sensor_mode->getFrameDurationRange();
      LOG(INFO) << "Frame duration min: " << range.min() << ", " << range.max()
                << ", type " << i_sensor_mode->getSensorModeType().getName();
    }

    // Create the capture session using the first device and get the core
    // interface.
    capture_session_.reset(
        i_camera_provider->createCaptureSession(camera_device));
    i_capture_session_ =
        Argus::interface_cast<Argus::ICaptureSession>(capture_session_);
    CHECK(i_capture_session_);

    CHECK_NE(egl_display_, EGL_NO_DISPLAY) << ": Failed to open display";

    // Create the OutputStream.
    stream_settings_.reset(i_capture_session_->createOutputStreamSettings(
        Argus::STREAM_TYPE_BUFFER));

    Argus::IBufferOutputStreamSettings *i_buffer_output_stream_settings =
        Argus::interface_cast<Argus::IBufferOutputStreamSettings>(
            stream_settings_);
    CHECK(i_buffer_output_stream_settings != nullptr);
    i_buffer_output_stream_settings->setBufferType(
        Argus::BUFFER_TYPE_EGL_IMAGE);
    i_buffer_output_stream_settings->setMetadataEnable(true);
    LOG(INFO) << "Type: "
              << i_buffer_output_stream_settings->getBufferType().getName();

    output_stream_.reset(
        i_capture_session_->createOutputStream(stream_settings_.get()));
    LOG(INFO) << "Got image stream";

    i_buffer_output_stream_ =
        Argus::interface_cast<Argus::IBufferOutputStream>(output_stream_);
    CHECK(i_buffer_output_stream_ != nullptr);

    // Build the DmaBuffers
    for (size_t i = 0; i < native_buffers_.size(); ++i) {
      native_buffers_[i] =
          DmaBuffer::Create(i_sensor_mode->getResolution(),
                            static_cast<NvBufSurfaceColorFormat>(
                                absl::GetFlag(FLAGS_colorformat)),
                            NVBUF_LAYOUT_PITCH);
    }

    // Create EGLImages from the native buffers
    for (size_t i = 0; i < egl_images_.size(); i++) {
      int ret = 0;

      ret = NvBufSurfaceFromFd(native_buffers_[i]->fd(), (void **)(&surf_[i]));
      CHECK(ret == 0) << ": NvBufSurfaceFromFd failed";

      ret = NvBufSurfaceMapEglImage(surf_[i], 0);
      // This check typically fails from having X forwarding enabled.
      // Always call argus_camera without X forwarding.
      CHECK(ret == 0) << ": NvBufSurfaceMapEglImage failed.  Make sure X "
                         "forwarding is not enabled.";

      egl_images_[i] = surf_[i]->surfaceList[0].mappedAddr.eglImage;
      CHECK(egl_images_[i] != EGL_NO_IMAGE_KHR)
          << ": Failed to create EGLImage";
    }

    // Create the BufferSettings object to configure Buffer creation.
    buffer_settings_.reset(i_buffer_output_stream_->createBufferSettings());
    Argus::IEGLImageBufferSettings *i_buffer_settings =
        Argus::interface_cast<Argus::IEGLImageBufferSettings>(buffer_settings_);
    CHECK(i_buffer_settings);

    // Create the Buffers for each EGLImage (and release to the stream for
    // initial capture use)
    for (size_t i = 0; i < buffers_.size(); i++) {
      i_buffer_settings->setEGLImage(egl_images_[i]);
      i_buffer_settings->setEGLDisplay(egl_display_);
      buffers_[i].reset(
          i_buffer_output_stream_->createBuffer(buffer_settings_.get()));
      Argus::IBuffer *i_buffer =
          Argus::interface_cast<Argus::IBuffer>(buffers_[i]);

      // Ties Argus::Buffer and DmaBuffer together.
      i_buffer->setClientData(native_buffers_[i].get());
      native_buffers_[i]->set_argus_buffer(buffers_[i].get());

      CHECK(Argus::interface_cast<Argus::IEGLImageBuffer>(buffers_[i]) !=
            nullptr)
          << ": Failed to create Buffer";

      CHECK_EQ(i_buffer_output_stream_->releaseBuffer(buffers_[i].get()),
               Argus::STATUS_OK)
          << "Failed to release Buffer for capture use";
    }

    request_.reset(i_capture_session_->createRequest());
    Argus::IRequest *i_request =
        Argus::interface_cast<Argus::IRequest>(request_);
    CHECK(i_request);

    Argus::IAutoControlSettings *i_auto_control_settings =
        Argus::interface_cast<Argus::IAutoControlSettings>(
            i_request->getAutoControlSettings());
    CHECK(i_auto_control_settings != nullptr);
    CHECK_EQ(i_auto_control_settings->setAwbMode(Argus::AWB_MODE_OFF),
             Argus::STATUS_OK);

    i_auto_control_settings->setAeLock(false);
    Argus::Range<float> isp_digital_gain_range;
    isp_digital_gain_range.min() = 1;
    isp_digital_gain_range.max() = 1;
    CHECK_EQ(
        i_auto_control_settings->setIspDigitalGainRange(isp_digital_gain_range),
        Argus::STATUS_OK);

    Argus::IEdgeEnhanceSettings *i_ee_settings =
        Argus::interface_cast<Argus::IEdgeEnhanceSettings>(request_);
    CHECK(i_ee_settings != nullptr);

    CHECK_EQ(i_ee_settings->setEdgeEnhanceStrength(0), Argus::STATUS_OK);

    Argus::ISourceSettings *i_source_settings =
        Argus::interface_cast<Argus::ISourceSettings>(
            i_request->getSourceSettings());
    CHECK(i_source_settings != nullptr);

    i_source_settings->setFrameDurationRange(
        i_sensor_mode->getFrameDurationRange().min());
    CHECK_EQ(i_source_settings->setSensorMode(
                 sensor_modes[absl::GetFlag(FLAGS_mode)]),
             Argus::STATUS_OK);

    Argus::Range<float> sensor_mode_analog_gain_range;
    sensor_mode_analog_gain_range.min() = absl::GetFlag(FLAGS_gain);
    sensor_mode_analog_gain_range.max() = absl::GetFlag(FLAGS_gain);
    CHECK_EQ(i_source_settings->setGainRange(sensor_mode_analog_gain_range),
             Argus::STATUS_OK);

    Argus::Range<uint64_t> limit_exposure_time_range;
    limit_exposure_time_range.min() = absl::GetFlag(FLAGS_exposure) * 1000;
    limit_exposure_time_range.max() = absl::GetFlag(FLAGS_exposure) * 1000;
    CHECK_EQ(i_source_settings->setExposureTimeRange(limit_exposure_time_range),
             Argus::STATUS_OK);

    i_request->enableOutputStream(output_stream_.get());
  }

  void Start() {
    if (i_capture_session_->repeat(request_.get()) != Argus::STATUS_OK) {
      LOG(ERROR) << "Failed to submit repeat";
    }

    LOG(INFO) << "Session submitted";
  }

  // Class to manage an image buffer and return it when we are done.
  class MappedBuffer {
   public:
    MappedBuffer(Argus::IBufferOutputStream *i_buffer_output_stream,
                 Argus::Buffer *buffer)
        : i_buffer_output_stream_(i_buffer_output_stream), buffer_(buffer) {
      if (buffer_ == nullptr) {
        return;
      }

      start_time_ = aos::monotonic_clock::now();

      dmabuf_ = DmaBuffer::FromArgusBuffer(buffer_);

      int dmabuf_fd = dmabuf_->fd();

      CHECK_EQ(NvBufSurfaceFromFd(dmabuf_fd, (void **)(&nvbuf_surf_)), 0);

      CHECK_EQ(NvBufSurfaceMap(nvbuf_surf_, -1, -1, NVBUF_MAP_READ), 0);
      VLOG(1) << "Mapped";
      NvBufSurfaceSyncForCpu(nvbuf_surf_, -1, -1);

      VLOG(1) << "Planes " << nvbuf_surf_->surfaceList->planeParams.num_planes
              << " colorFormat " << nvbuf_surf_->surfaceList->colorFormat;
      for (size_t i = 0; i < nvbuf_surf_->surfaceList->planeParams.num_planes;
           ++i) {
        VLOG(1) << "Address "
                << static_cast<void *>(
                       nvbuf_surf_->surfaceList->mappedAddr.addr[i])
                << ", pitch " << nvbuf_surf_->surfaceList->planeParams.pitch[i]
                << " height " << nvbuf_surf_->surfaceList->planeParams.height[i]
                << " width " << nvbuf_surf_->surfaceList->planeParams.width[i]
                << " bytes per pixel "
                << nvbuf_surf_->surfaceList->planeParams.bytesPerPix[i];
      }
      CHECK_EQ(nvbuf_surf_->surfaceList->planeParams.width[0],
               static_cast<size_t>(absl::GetFlag(FLAGS_width)));
      CHECK_EQ(nvbuf_surf_->surfaceList->planeParams.height[0],
               static_cast<size_t>(absl::GetFlag(FLAGS_height)));
    }
    MappedBuffer(const MappedBuffer &other) = delete;
    MappedBuffer &operator=(const MappedBuffer &other) = delete;
    MappedBuffer(MappedBuffer &&other) noexcept {
      buffer_ = other.buffer_;
      dmabuf_ = other.dmabuf_;
      nvbuf_surf_ = other.nvbuf_surf_;
      i_buffer_output_stream_ = other.i_buffer_output_stream_;
      start_time_ = other.start_time_;
      other.buffer_ = nullptr;
      other.dmabuf_ = nullptr;
      other.nvbuf_surf_ = nullptr;
    }

    NvBufSurface *nvbuf_surf() { return nvbuf_surf_; }

    const Argus::ICaptureMetadata *imetadata() {
      Argus::IBuffer *ibuffer = Argus::interface_cast<Argus::IBuffer>(buffer_);
      CHECK(ibuffer != nullptr);

      aos::ScopedNotRealtime nrt;
      const Argus::CaptureMetadata *metadata = ibuffer->getMetadata();
      const Argus::ICaptureMetadata *imetadata =
          Argus::interface_cast<const Argus::ICaptureMetadata>(metadata);
      return imetadata;
    }

    aos::monotonic_clock::time_point start_time() const { return start_time_; }

    virtual ~MappedBuffer() {
      if (buffer_ != nullptr) {
        CHECK_EQ(NvBufSurfaceUnMap(nvbuf_surf_, -1, -1), 0);
        aos::ScopedNotRealtime nrt;
        i_buffer_output_stream_->releaseBuffer(buffer_);
      }
    }

   private:
    Argus::IBufferOutputStream *i_buffer_output_stream_;

    Argus::Buffer *buffer_;

    DmaBuffer *dmabuf_ = nullptr;

    NvBufSurface *nvbuf_surf_ = nullptr;

    aos::monotonic_clock::time_point start_time_;
  };

  MappedBuffer NextImageBlocking() {
    VLOG(1) << "Going for frame";

    Argus::Buffer *buffer;
    {
      Argus::Status status;
      aos::ScopedNotRealtime nrt;

      buffer = i_buffer_output_stream_->acquireBuffer(
          std::chrono::nanoseconds(std::chrono::seconds(5)).count(), &status);

      if (status == Argus::STATUS_END_OF_STREAM) {
        return MappedBuffer(nullptr, nullptr);
      }
    }

    // const aos::monotonic_clock::time_point now = aos::monotonic_clock::now();
    return MappedBuffer(i_buffer_output_stream_, buffer);
  }

  void Stop() {
    i_capture_session_->stopRepeat();
    i_buffer_output_stream_->endOfStream();
    i_capture_session_->waitForIdle();
  }

  virtual ~ArgusCamera() {
    output_stream_.reset();

    for (uint32_t i = 0; i < surf_.size(); i++) {
      NvBufSurfaceUnMapEglImage(surf_[i], 0);
    }
    eglTerminate(egl_display_);
  }

 private:
  Argus::UniqueObj<Argus::CaptureSession> capture_session_;
  Argus::ICaptureSession *i_capture_session_;

  EGLDisplay egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);

  Argus::UniqueObj<Argus::OutputStreamSettings> stream_settings_;

  Argus::UniqueObj<Argus::OutputStream> output_stream_;
  Argus::IBufferOutputStream *i_buffer_output_stream_;

  std::array<std::unique_ptr<DmaBuffer>, 10> native_buffers_;

  std::array<NvBufSurface *, 10> surf_;

  std::array<EGLImageKHR, 10> egl_images_;

  Argus::UniqueObj<Argus::BufferSettings> buffer_settings_;

  std::array<Argus::UniqueObj<Argus::Buffer>, 10> buffers_;

  Argus::UniqueObj<Argus::Request> request_;
};

int Main() {
  std::this_thread::sleep_for(
      std::chrono::seconds(absl::GetFlag(FLAGS_camera) + 1));

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(55);
  event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({2, 3, 4}));

  aos::Sender<frc971::vision::CameraImage> sender =
      event_loop.MakeSender<frc971::vision::CameraImage>(
          absl::GetFlag(FLAGS_channel));

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
    Argus::ICameraProperties *i_camera_properties =
        Argus::interface_cast<Argus::ICameraProperties>(camera);
    LOG(INFO) << "Camera " << i_camera_properties->getModelName();
  }

  {
    ArgusCamera camera(i_camera_provider,
                       camera_devices[absl::GetFlag(FLAGS_camera)]);

    aos::monotonic_clock::time_point last_time = aos::monotonic_clock::epoch();

    aos::TimerHandler *timer = event_loop.AddTimer([&camera, &event_loop,
                                                    &sender, &last_time,
                                                    &timer]() {
      ArgusCamera::MappedBuffer buffer = camera.NextImageBlocking();

      if (buffer.nvbuf_surf() == nullptr) {
        event_loop.Exit();
        return;
      }

      const Argus::ICaptureMetadata *imetadata = buffer.imetadata();

      if (imetadata) {
        aos::Sender<frc971::vision::CameraImage>::Builder builder =
            sender.MakeBuilder();

        uint8_t *data_pointer = nullptr;
        builder.fbb()->StartIndeterminateVector(
            absl::GetFlag(FLAGS_width) * absl::GetFlag(FLAGS_height) * 2, 1, 64,
            &data_pointer);

        YCbCr422(buffer.nvbuf_surf(), data_pointer);
        flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
            builder.fbb()->EndIndeterminateVector(
                absl::GetFlag(FLAGS_width) * absl::GetFlag(FLAGS_height) * 2,
                1);

        auto image_builder = builder.MakeBuilder<frc971::vision::CameraImage>();
        image_builder.add_data(data_offset);
        image_builder.add_rows(absl::GetFlag(FLAGS_height));
        image_builder.add_cols(absl::GetFlag(FLAGS_width));
        {
          aos::ScopedNotRealtime nrt;
          image_builder.add_monotonic_timestamp_ns(
              imetadata->getSensorTimestamp());
        }
        builder.CheckOk(builder.Send(image_builder.Finish()));

        const aos::monotonic_clock::time_point after_send =
            aos::monotonic_clock::now();

        VLOG(1)
            << "Got " << imetadata->getCaptureId() << " delay "
            << chrono::duration<double>(
                   chrono::nanoseconds(
                       (buffer.start_time().time_since_epoch().count() -
                        (imetadata->getSensorTimestamp() +
                         imetadata->getFrameReadoutTime()))))
                   .count()
            << " mmap "
            << chrono::duration<double>(after_send - buffer.start_time())
                   .count()
            << "sec dt "
            << chrono::duration<double>(buffer.start_time() - last_time).count()
            << "sec, exposure " << imetadata->getSensorExposureTime();
      }

      last_time = buffer.start_time();
      timer->Schedule(event_loop.monotonic_now());
    });

    event_loop.OnRun([&event_loop, timer]() {
      timer->Schedule(event_loop.monotonic_now());
    });

    camera.Start();

    // Set the libargus threads which got spawned to RT priority.
    {
      DIR *dirp = opendir("/proc/self/task");
      PCHECK(dirp != nullptr);
      const int main_pid = getpid();
      struct dirent *directory_entry;
      while ((directory_entry = readdir(dirp)) != NULL) {
        const int thread_id = std::atoi(directory_entry->d_name);

        // ignore . and .. which are zeroes for some reason
        if (thread_id != 0 && thread_id != main_pid) {
          struct sched_param param;
          param.sched_priority = 56;
          sched_setscheduler(thread_id, SCHED_FIFO, &param);
        }
      }
      closedir(dirp);
    }

    event_loop.Run();
    LOG(INFO) << "Event loop shutting down";

    camera.Stop();
  }

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
