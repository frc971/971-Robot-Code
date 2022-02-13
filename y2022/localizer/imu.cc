#include "y2022/localizer/imu.h"

#include "aos/util/crc32.h"
#include "glog/logging.h"
#include "y2022/constants.h"

namespace y2022::localizer {

namespace {

constexpr size_t kReadSize = 50;
constexpr double kGyroScale = 1 / 655360.0 / 360.0 * (2 * M_PI);
constexpr double kAccelScale = 1 / 26756268.0 / 9.80665;
constexpr double kTempScale = 0.1;

}  // namespace

Imu::Imu(aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      imu_sender_(
          event_loop_->MakeSender<frc971::IMUValuesBatch>("/drivetrain")) {
  event_loop->SetRuntimeRealtimePriority(30);
  imu_fd_ = open("/dev/adis16505", O_RDONLY | O_NONBLOCK);
  PCHECK(imu_fd_ != -1) << ": Failed to open SPI device for IMU.";
  aos::internal::EPoll *epoll = event_loop_->epoll();
  epoll->OnReadable(imu_fd_, [this]() {
    uint8_t buf[kReadSize];
    ssize_t read_len = read(imu_fd_, buf, kReadSize);
    // TODO: Do we care about gracefully handling EAGAIN or anything else?
    // This should only get called when there is data.
    PCHECK(read_len != -1);
    CHECK_EQ(read_len, static_cast<ssize_t>(kReadSize))
        << ": Read incorrect number of bytes.";

    auto sender = imu_sender_.MakeBuilder();

    const flatbuffers::Offset<frc971::IMUValues> values_offset =
        ProcessReading(sender.fbb(), absl::Span(buf, kReadSize));
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<frc971::IMUValues>>>
        readings_offset = sender.fbb()->CreateVector(&values_offset, 1);
    frc971::IMUValuesBatch::Builder batch_builder =
        sender.MakeBuilder<frc971::IMUValuesBatch>();
    batch_builder.add_readings(readings_offset);
    imu_sender_.CheckOk(sender.Send(batch_builder.Finish()));
  });
}

flatbuffers::Offset<frc971::IMUValues> Imu::ProcessReading(
    flatbuffers::FlatBufferBuilder *fbb, const absl::Span<uint8_t> message) {
  absl::Span<const uint8_t> buf = message;

  uint64_t driver_timestamp;
  memcpy(&driver_timestamp, buf.data(), sizeof(driver_timestamp));
  buf = buf.subspan(8);

  uint16_t diag_stat;
  memcpy(&diag_stat, buf.data(), sizeof(diag_stat));
  buf = buf.subspan(2);

  double x_gyro = ConvertValue32(buf, kGyroScale);
  buf = buf.subspan(4);
  double y_gyro = ConvertValue32(buf, kGyroScale);
  buf = buf.subspan(4);
  double z_gyro = ConvertValue32(buf, kGyroScale);
  buf = buf.subspan(4);
  double x_accel = ConvertValue32(buf, kAccelScale);
  buf = buf.subspan(4);
  double y_accel = ConvertValue32(buf, kAccelScale);
  buf = buf.subspan(4);
  double z_accel = ConvertValue32(buf, kAccelScale);
  buf = buf.subspan(4);
  double temp = ConvertValue16(buf, kTempScale);
  buf = buf.subspan(2);
  uint16_t data_counter;
  memcpy(&data_counter, buf.data(), sizeof(data_counter));
  buf = buf.subspan(2);
  uint32_t pico_timestamp;
  memcpy(&pico_timestamp, buf.data(), sizeof(pico_timestamp));
  buf = buf.subspan(4);
  int16_t encoder1_count;
  memcpy(&encoder1_count, buf.data(), sizeof(encoder1_count));
  buf = buf.subspan(2);
  int16_t encoder2_count;
  memcpy(&encoder2_count, buf.data(), sizeof(encoder2_count));
  buf = buf.subspan(2);
  uint32_t checksum;
  memcpy(&checksum, buf.data(), sizeof(checksum));
  buf = buf.subspan(4);

  CHECK(buf.empty()) << "Have leftover bytes: " << buf.size();

  u_int32_t calculated_checksum = aos::ComputeCrc32(message.subspan(8, 38));

  if (checksum != calculated_checksum) {
    this->failed_checksums_++;
  }

  const auto diag_stat_offset = PackDiagStat(fbb, diag_stat);

  frc971::IMUValues::Builder imu_builder(*fbb);

  if (checksum == calculated_checksum) {
    constexpr uint16_t kChecksumMismatch = 1 << 0;
    bool imu_checksum_matched = !(diag_stat & kChecksumMismatch);

    // data from the IMU packet
    if (imu_checksum_matched) {
      imu_builder.add_gyro_x(x_gyro);
      imu_builder.add_gyro_y(y_gyro);
      imu_builder.add_gyro_z(z_gyro);

      imu_builder.add_accelerometer_x(x_accel);
      imu_builder.add_accelerometer_y(y_accel);
      imu_builder.add_accelerometer_z(z_accel);

      imu_builder.add_temperature(temp);

      imu_builder.add_data_counter(data_counter);
    }

    // extra data from the pico
    imu_builder.add_pico_timestamp_us(pico_timestamp);
    imu_builder.add_left_encoder(
        constants::Values::DrivetrainEncoderToMeters(encoder1_count));
    imu_builder.add_right_encoder(
        constants::Values::DrivetrainEncoderToMeters(encoder2_count));
    imu_builder.add_previous_reading_diag_stat(diag_stat_offset);
  }

  // extra data from us
  imu_builder.add_monotonic_timestamp_ns(driver_timestamp);
  imu_builder.add_failed_checksums(failed_checksums_);
  imu_builder.add_checksum_failed(checksum != calculated_checksum);

  return imu_builder.Finish();
}

flatbuffers::Offset<frc971::ADIS16470DiagStat> Imu::PackDiagStat(
    flatbuffers::FlatBufferBuilder *fbb, uint16_t value) {
  frc971::ADIS16470DiagStat::Builder diag_stat_builder(*fbb);
  diag_stat_builder.add_clock_error(value & (1 << 7));
  diag_stat_builder.add_memory_failure(value & (1 << 6));
  diag_stat_builder.add_sensor_failure(value & (1 << 5));
  diag_stat_builder.add_standby_mode(value & (1 << 4));
  diag_stat_builder.add_spi_communication_error(value & (1 << 3));
  diag_stat_builder.add_flash_memory_update_error(value & (1 << 2));
  diag_stat_builder.add_data_path_overrun(value & (1 << 1));
  diag_stat_builder.add_checksum_mismatch(value & (1 << 0));
  return diag_stat_builder.Finish();
}

double Imu::ConvertValue32(absl::Span<const uint8_t> data,
                           double lsb_per_output) {
  int32_t value;
  memcpy(&value, data.data(), sizeof(value));
  return static_cast<double>(value) * lsb_per_output;
}

double Imu::ConvertValue16(absl::Span<const uint8_t> data,
                           double lsb_per_output) {
  int16_t value;
  memcpy(&value, data.data(), sizeof(value));
  return static_cast<double>(value) * lsb_per_output;
}

Imu::~Imu() { PCHECK(0 == close(imu_fd_)); }
}  // namespace y2022::localizer
