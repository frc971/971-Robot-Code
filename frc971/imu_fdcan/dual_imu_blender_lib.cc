#include "frc971/imu_fdcan/dual_imu_blender_lib.h"

#include "gflags/gflags.h"

DEFINE_bool(murata_only, false,
            "If true then only use the murata value and ignore the tdk.");

// Saturation for the gyro is measured in +- radians/s
static constexpr double kMurataGyroSaturation = (300.0 * M_PI) / 180;

// Measured in gs
static constexpr double kMurataAccelSaturation = 6.0;

// Coefficient to multiply the saturation values by to give some room on where
// we switch to tdk.
static constexpr double kSaturationCoeff = 0.9;
static constexpr int kSaturationCounterThreshold = 20;

using frc971::imu_fdcan::DualImuBlender;

DualImuBlender::DualImuBlender(aos::EventLoop *event_loop)
    : imu_values_batch_sender_(
          event_loop->MakeSender<frc971::IMUValuesBatchStatic>("/localizer")),
      dual_imu_blender_status_sender_(
          event_loop->MakeSender<frc971::imu::DualImuBlenderStatusStatic>(
              "/imu")) {
  // TODO(max): Give this a proper priority
  event_loop->SetRuntimeRealtimePriority(15);

  event_loop->MakeWatcher("/imu", [this](const frc971::imu::DualImu &dual_imu) {
    HandleDualImu(&dual_imu);
  });
}

void DualImuBlender::HandleDualImu(const frc971::imu::DualImu *dual_imu) {
  aos::Sender<frc971::IMUValuesBatchStatic>::StaticBuilder
      imu_values_batch_builder_ = imu_values_batch_sender_.MakeStaticBuilder();

  aos::Sender<frc971::imu::DualImuBlenderStatusStatic>::StaticBuilder
      dual_imu_blender_status_builder =
          dual_imu_blender_status_sender_.MakeStaticBuilder();

  frc971::IMUValuesStatic *imu_values =
      imu_values_batch_builder_->add_readings()->emplace_back();
  CHECK(imu_values != nullptr);

  imu_values->set_pico_timestamp_us(dual_imu->board_timestamp_us());
  imu_values->set_monotonic_timestamp_ns(dual_imu->kernel_timestamp());
  imu_values->set_data_counter(dual_imu->packet_counter());
  // Notes on saturation strategy:
  // We use the TDK to detect saturation because we presume that if the Murata
  // is saturated then it may produce poor or undefined behavior (including
  // potentially producing values that make it look like it is not saturated).
  // In practice, the Murata does seem to behave reasonably under saturation (it
  // just maxes out its outputs at the given value).

  if (std::abs(dual_imu->tdk()->gyro_x()) >=
      kSaturationCoeff * kMurataGyroSaturation) {
    dual_imu_blender_status_builder->set_gyro_x(imu::ImuType::TDK);
    imu_values->set_gyro_x(dual_imu->tdk()->gyro_x());
  } else {
    dual_imu_blender_status_builder->set_gyro_x(imu::ImuType::MURATA);
    imu_values->set_gyro_x(dual_imu->murata()->gyro_x());
  }

  if (std::abs(dual_imu->tdk()->gyro_y()) >=
      kSaturationCoeff * kMurataGyroSaturation) {
    dual_imu_blender_status_builder->set_gyro_y(imu::ImuType::TDK);
    imu_values->set_gyro_y(dual_imu->tdk()->gyro_y());
  } else {
    dual_imu_blender_status_builder->set_gyro_y(imu::ImuType::MURATA);
    imu_values->set_gyro_y(dual_imu->murata()->gyro_y());
  }

  // TODO(james): Currently we only do hysteresis for the gyro Z axis because
  // this is the only axis that is particularly critical. We should do something
  // like this for all axes.
  if (std::abs(dual_imu->tdk()->gyro_z()) >=
      kSaturationCoeff * kMurataGyroSaturation) {
    ++saturated_counter_;
  } else {
    --saturated_counter_;
  }
  if (saturated_counter_ <= -kSaturationCounterThreshold) {
    is_saturated_ = false;
    saturated_counter_ = -kSaturationCounterThreshold;
  } else if (saturated_counter_ >= kSaturationCounterThreshold) {
    is_saturated_ = true;
    saturated_counter_ = kSaturationCounterThreshold;
  }

  if (is_saturated_) {
    dual_imu_blender_status_builder->set_gyro_z(imu::ImuType::TDK);
    imu_values->set_gyro_z(dual_imu->tdk()->gyro_z());
  } else {
    dual_imu_blender_status_builder->set_gyro_z(imu::ImuType::MURATA);
    imu_values->set_gyro_z(dual_imu->murata()->gyro_z());
  }

  if (std::abs(dual_imu->tdk()->accelerometer_x()) >=
      kSaturationCoeff * kMurataAccelSaturation) {
    dual_imu_blender_status_builder->set_accelerometer_x(imu::ImuType::TDK);
    imu_values->set_accelerometer_x(dual_imu->tdk()->accelerometer_x());
  } else {
    dual_imu_blender_status_builder->set_accelerometer_x(imu::ImuType::MURATA);
    imu_values->set_accelerometer_x(dual_imu->murata()->accelerometer_x());
  }

  if (std::abs(dual_imu->tdk()->accelerometer_y()) >=
      kSaturationCoeff * kMurataAccelSaturation) {
    dual_imu_blender_status_builder->set_accelerometer_y(imu::ImuType::TDK);
    imu_values->set_accelerometer_y(dual_imu->tdk()->accelerometer_y());
  } else {
    dual_imu_blender_status_builder->set_accelerometer_y(imu::ImuType::MURATA);
    imu_values->set_accelerometer_y(dual_imu->murata()->accelerometer_y());
  }

  if (std::abs(dual_imu->tdk()->accelerometer_z()) >=
      kSaturationCoeff * kMurataAccelSaturation) {
    dual_imu_blender_status_builder->set_accelerometer_z(imu::ImuType::TDK);
    imu_values->set_accelerometer_z(dual_imu->tdk()->accelerometer_z());
  } else {
    dual_imu_blender_status_builder->set_accelerometer_z(imu::ImuType::MURATA);
    imu_values->set_accelerometer_z(dual_imu->murata()->accelerometer_z());
  }

  if (FLAGS_murata_only) {
    imu_values->set_gyro_x(dual_imu->murata()->gyro_x());
    imu_values->set_gyro_y(dual_imu->murata()->gyro_y());
    imu_values->set_gyro_z(dual_imu->murata()->gyro_z());

    imu_values->set_accelerometer_x(dual_imu->murata()->accelerometer_x());
    imu_values->set_accelerometer_y(dual_imu->murata()->accelerometer_y());
    imu_values->set_accelerometer_z(dual_imu->murata()->accelerometer_z());

    dual_imu_blender_status_builder->set_gyro_x(imu::ImuType::MURATA);
    dual_imu_blender_status_builder->set_gyro_y(imu::ImuType::MURATA);
    dual_imu_blender_status_builder->set_gyro_z(imu::ImuType::MURATA);

    dual_imu_blender_status_builder->set_accelerometer_x(imu::ImuType::MURATA);
    dual_imu_blender_status_builder->set_accelerometer_y(imu::ImuType::MURATA);
    dual_imu_blender_status_builder->set_accelerometer_z(imu::ImuType::MURATA);
  }

  dual_imu_blender_status_builder.CheckOk(
      dual_imu_blender_status_builder.Send());

  imu_values->set_temperature(
      dual_imu->murata()->chip_states()->Get(0)->temperature());

  imu_values_batch_builder_.CheckOk(imu_values_batch_builder_.Send());
}
