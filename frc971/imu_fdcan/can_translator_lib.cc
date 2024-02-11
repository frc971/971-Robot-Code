#include "frc971/imu_fdcan/can_translator_lib.h"

using frc971::imu_fdcan::CANTranslator;

constexpr std::size_t kCanFrameSize = 64;
constexpr int kImuCanId = 1;

CANTranslator::CANTranslator(aos::EventLoop *event_loop,
                             std::string_view canframe_channel)
    : dual_imu_sender_(
          event_loop->MakeSender<frc971::imu::DualImuStatic>("/imu")),
      can_translator_status_sender_(
          event_loop->MakeSender<frc971::imu::CanTranslatorStatusStatic>(
              "/imu")) {
  // TODO(max): Update this with a proper priority
  event_loop->SetRuntimeRealtimePriority(15);

  event_loop->MakeWatcher(
      canframe_channel, [this](const frc971::can_logger::CanFrame &can_frame) {
        if (can_frame.data()->size() / sizeof(uint8_t) != 64) {
          invalid_packet_count_++;
        }

        if (can_frame.can_id() != kImuCanId) {
          invalid_can_id_count_++;
          return;
        }

        if (can_frame.data()->size() / sizeof(uint8_t) == 64) {
          valid_packet_count_++;
          HandleFrame(&can_frame);
        }
      });

  event_loop->AddPhasedLoop(
      [this](int) {
        aos::Sender<frc971::imu::CanTranslatorStatusStatic>::StaticBuilder
            status_builder = can_translator_status_sender_.MakeStaticBuilder();

        status_builder->set_valid_packet_count(valid_packet_count_);
        status_builder->set_invalid_packet_count(invalid_packet_count_);
        status_builder->set_invalid_can_id_count(invalid_can_id_count_);

        status_builder.CheckOk(status_builder.Send());
      },
      std::chrono::milliseconds(100));
}

// Gets the data from the span and iterates it to the next section of bytes.
template <typename T>
T GetAndPopDataFromBuffer(std::span<const uint8_t> &span) {
  T value = 0;

  std::memcpy(&value, span.data(), sizeof(T));
  span = span.subspan(sizeof(T));

  return value;
}

// Values from the data field mapping table in
// https://docs.google.com/document/d/12AJUruW7DZ2pIrDzTyPC0qqFoia4QOSVlax6Jd7m4H0/edit?usp=sharing
void CANTranslator::HandleFrame(const frc971::can_logger::CanFrame *can_frame) {
  aos::Sender<frc971::imu::DualImuStatic>::StaticBuilder dual_imu_builder =
      dual_imu_sender_.MakeStaticBuilder();

  std::span can_data(can_frame->data()->data(), kCanFrameSize);

  frc971::imu::SingleImuStatic *murata = dual_imu_builder->add_murata();

  auto *murata_chip_states = murata->add_chip_states();
  frc971::imu::ChipStateStatic *murata_uno_chip_state =
      murata_chip_states->emplace_back();
  frc971::imu::ChipStateStatic *murata_due_chip_state =
      murata_chip_states->emplace_back();

  frc971::imu::SingleImuStatic *tdk = dual_imu_builder->add_tdk();

  auto tdk_chip_states = tdk->add_chip_states();
  frc971::imu::ChipStateStatic *tdk_chip_state =
      tdk_chip_states->emplace_back();

  dual_imu_builder->set_board_timestamp_us(
      GetAndPopDataFromBuffer<uint32_t>(can_data));

  dual_imu_builder->set_packet_counter(
      GetAndPopDataFromBuffer<uint16_t>(can_data));

  tdk_chip_state->set_counter(GetAndPopDataFromBuffer<uint16_t>(can_data));
  murata_uno_chip_state->set_counter(
      GetAndPopDataFromBuffer<uint16_t>(can_data));
  murata_due_chip_state->set_counter(
      GetAndPopDataFromBuffer<uint16_t>(can_data));

  tdk->set_accelerometer_x(GetAndPopDataFromBuffer<float>(can_data));
  tdk->set_accelerometer_y(GetAndPopDataFromBuffer<float>(can_data));
  tdk->set_accelerometer_z(GetAndPopDataFromBuffer<float>(can_data));

  tdk->set_gyro_x(GetAndPopDataFromBuffer<float>(can_data));
  tdk->set_gyro_y(GetAndPopDataFromBuffer<float>(can_data));
  tdk->set_gyro_z(GetAndPopDataFromBuffer<float>(can_data));

  murata->set_accelerometer_x(GetAndPopDataFromBuffer<float>(can_data));
  murata->set_accelerometer_y(GetAndPopDataFromBuffer<float>(can_data));
  murata->set_accelerometer_z(GetAndPopDataFromBuffer<float>(can_data));

  murata->set_gyro_x(GetAndPopDataFromBuffer<float>(can_data));
  murata->set_gyro_y(GetAndPopDataFromBuffer<float>(can_data));
  murata->set_gyro_z(GetAndPopDataFromBuffer<float>(can_data));

  tdk_chip_state->set_temperature(GetAndPopDataFromBuffer<uint8_t>(can_data));
  murata_uno_chip_state->set_temperature(
      GetAndPopDataFromBuffer<uint8_t>(can_data));
  murata_due_chip_state->set_temperature(
      GetAndPopDataFromBuffer<uint8_t>(can_data));

  murata_uno_chip_state->set_max_counter(std::numeric_limits<uint16_t>::max());
  murata_due_chip_state->set_max_counter(std::numeric_limits<uint16_t>::max());
  tdk_chip_state->set_max_counter(std::numeric_limits<uint16_t>::max());
  dual_imu_builder->set_max_packet_counter(
      std::numeric_limits<uint16_t>::max());

  dual_imu_builder->set_kernel_timestamp(can_frame->monotonic_timestamp_ns());

  dual_imu_builder.CheckOk(dual_imu_builder.Send());
}
