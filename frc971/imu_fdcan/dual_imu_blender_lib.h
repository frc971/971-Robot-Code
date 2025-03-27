#ifndef FRC971_IMU_FDCAN_DUAL_IMU_BLENDER_H_
#define FRC971_IMU_FDCAN_DUAL_IMU_BLENDER_H_

#include "absl/flags/declare.h"

#include "aos/events/event_loop.h"
#include "frc971/imu_fdcan/dual_imu_blender_status_static.h"
#include "frc971/imu_fdcan/dual_imu_generated.h"
#include "frc971/wpilib/imu_batch_static.h"

ABSL_DECLARE_FLAG(bool, use_one_orin);

namespace frc971::imu_fdcan {

// Takes in the values from the dual_imu and creates an IMUValuesBatch. Will use
// the murata until we've hit saturation according to the tdk, then we will
// switch to using tdk IMU values.
class DualImuBlender {
 public:
  DualImuBlender(aos::EventLoop *event_loop);

  void HandleDualImu(const frc971::imu::DualImu *dual_imu);

 private:
  aos::Sender<IMUValuesBatchStatic> imu_values_batch_sender_;
  aos::Sender<imu::DualImuBlenderStatusStatic> dual_imu_blender_status_sender_;
  int saturated_counter_ = 0;
  bool is_saturated_ = false;
};

}  // namespace frc971::imu_fdcan

#endif  // FRC971_IMU_FDCAN_DUAL_IMU_BLENDER_H_
