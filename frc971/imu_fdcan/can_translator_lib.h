#ifndef FRC971_IMU_FDCAN_CAN_TRANSLATOR_LIB_H_
#define FRC971_IMU_FDCAN_CAN_TRANSLATOR_LIB_H_
#include "absl/flags/flag.h"

#include "aos/events/event_loop.h"
#include "frc971/can_logger/can_logging_generated.h"
#include "frc971/imu_fdcan/can_translator_status_static.h"
#include "frc971/imu_fdcan/dual_imu_static.h"

ABSL_DECLARE_FLAG(bool, use_one_orin);

namespace frc971::imu_fdcan {

// Translates the CanFrames from the IMU into a DualIMU message based on the
// spec defined in this doc:
// https://docs.google.com/document/d/12AJUruW7DZ2pIrDzTyPC0qqFoia4QOSVlax6Jd7m4H0/edit?usp=sharing
class CANTranslator {
 public:
  CANTranslator(aos::EventLoop *event_loop, std::string_view canframe_channel);

 private:
  void HandleFrame(const can_logger::CanFrame *can_frame);

  aos::EventLoop *event_loop_;
  aos::Sender<imu::DualImuStatic> dual_imu_sender_;
  aos::Sender<imu::CanTranslatorStatusStatic> can_translator_status_sender_;

  std::array<uint8_t, 64> current_frame_;
  std::array<bool, 8> packets_arrived_;

  uint64_t valid_packet_count_ = 0;
  uint64_t invalid_packet_count_ = 0;
  uint64_t invalid_can_id_count_ = 0;
  uint64_t out_of_order_count_ = 0;
};

}  // namespace frc971::imu_fdcan

#endif  // FRC971_IMU_FDCAN_CAN_TRANSLATOR_LIB_H_
