#include "frc971/wpilib/ADIS16470.h"

#include <cinttypes>

#include "glog/logging.h"

#include "aos/containers/sized_array.h"
#include "aos/time/time.h"
#include "hal/HAL.h"

namespace frc971 {
namespace wpilib {
namespace {
namespace chrono = std::chrono;
namespace registers {

// Flash memory write count
constexpr uint8_t FLASH_CNT = 0x00;
// Diagnostic and operational status
constexpr uint8_t DIAG_STAT = 0x02;
// X-axis gyroscope output, lower word
constexpr uint8_t X_GYRO_LOW = 0x04;
// X-axis gyroscope output, upper word
constexpr uint8_t X_GYRO_OUT = 0x06;
// Y-axis gyroscope output, lower word
constexpr uint8_t Y_GYRO_LOW = 0x08;
// Y-axis gyroscope output, upper word
constexpr uint8_t Y_GYRO_OUT = 0x0A;
// Z-axis gyroscope output, lower word
constexpr uint8_t Z_GYRO_LOW = 0x0C;
// Z-axis gyroscope output, upper word
constexpr uint8_t Z_GYRO_OUT = 0x0E;
// X-axis accelerometer output, lower word
constexpr uint8_t X_ACCL_LOW = 0x10;
// X-axis accelerometer output, upper word
constexpr uint8_t X_ACCL_OUT = 0x12;
// Y-axis accelerometer output, lower word
constexpr uint8_t Y_ACCL_OUT = 0x16;
// Y-axis accelerometer output, upper word
constexpr uint8_t Z_ACCL_LOW = 0x18;
// Z-axis accelerometer output, lower word
constexpr uint8_t Z_ACCL_OUT = 0x1A;
// Z-axis accelerometer output, upper word
constexpr uint8_t TEMP_OUT = 0x1C;
// Temperature output (internal, not calibrated)
constexpr uint8_t TIME_STAMP = 0x1E;
// PPS mode time stamp
constexpr uint8_t X_DELTANG_LOW = 0x24;
// X-axis delta angle output, lower word
constexpr uint8_t X_DELTANG_OUT = 0x26;
// X-axis delta angle output, upper word
constexpr uint8_t Y_DELTANG_LOW = 0x28;
// Y-axis delta angle output, lower word
constexpr uint8_t Y_DELTANG_OUT = 0x2A;
// Y-axis delta angle output, upper word
constexpr uint8_t Z_DELTANG_LOW = 0x2C;
// Z-axis delta angle output, lower word
constexpr uint8_t Z_DELTANG_OUT = 0x2E;
// Z-axis delta angle output, upper word
constexpr uint8_t X_DELTVEL_LOW = 0x30;
// X-axis delta velocity output, lower word
constexpr uint8_t X_DELTVEL_OUT = 0x32;
// X-axis delta velocity output, upper word
constexpr uint8_t Y_DELTVEL_LOW = 0x34;
// Y-axis delta velocity output, lower word
constexpr uint8_t Y_DELTVEL_OUT = 0x36;
// Y-axis delta velocity output, upper word
constexpr uint8_t Z_DELTVEL_LOW = 0x38;
// Z-axis delta velocity output, lower word
constexpr uint8_t Z_DELTVEL_OUT = 0x3A;
// Z-axis delta velocity output, upper word
constexpr uint8_t XG_BIAS_LOW = 0x40;
// X-axis gyroscope bias offset correction, lower word
constexpr uint8_t XG_BIAS_HIGH = 0x42;
// X-axis gyroscope bias offset correction, upper word
constexpr uint8_t YG_BIAS_LOW = 0x44;
// Y-axis gyroscope bias offset correction, lower word
constexpr uint8_t YG_BIAS_HIGH = 0x46;
// Y-axis gyroscope bias offset correction, upper word
constexpr uint8_t ZG_BIAS_LOW = 0x48;
// Z-axis gyroscope bias offset correction, lower word
constexpr uint8_t ZG_BIAS_HIGH = 0x4A;
// Z-axis gyroscope bias offset correction, upper word
constexpr uint8_t XA_BIAS_LOW = 0x4C;
// X-axis accelerometer bias offset correction, lower word
constexpr uint8_t XA_BIAS_HIGH = 0x4E;
// X-axis accelerometer bias offset correction, upper word
constexpr uint8_t YA_BIAS_LOW = 0x50;
// Y-axis accelerometer bias offset correction, lower word
constexpr uint8_t YA_BIAS_HIGH = 0x52;
// Y-axis accelerometer bias offset correction, upper word
constexpr uint8_t ZA_BIAS_LOW = 0x54;
// Z-axis accelerometer bias offset correction, lower word
constexpr uint8_t ZA_BIAS_HIGH = 0x56;
// Z-axis accelerometer bias offset correction, upper word
constexpr uint8_t FILT_CTRL = 0x5C;
// Filter control
constexpr uint8_t MSC_CTRL = 0x60;
// Miscellaneous control
constexpr uint8_t UP_SCALE = 0x62;
// Clock scale factor, PPS mode
constexpr uint8_t DEC_RATE = 0x64;
// Decimation rate control (output data rate)
constexpr uint8_t NULL_CNFG = 0x66;
// Auto-null configuration control
constexpr uint8_t GLOB_CMD = 0x68;
// Global commands
constexpr uint8_t FIRM_REV = 0x6C;
// Firmware revision
constexpr uint8_t FIRM_DM = 0x6E;
// Firmware revision date, month and day
constexpr uint8_t FIRM_Y = 0x70;
// Firmware revision date, year
constexpr uint8_t PROD_ID = 0x72;
// Product identification
constexpr uint8_t SERIAL_NUM = 0x74;
// Serial number (relative to assembly lot)
constexpr uint8_t USER_SCR1 = 0x76;
// User scratch register 1
constexpr uint8_t USER_SCR2 = 0x78;
// User scratch register 2
constexpr uint8_t USER_SCR3 = 0x7A;
// User scratch register 3
constexpr uint8_t FLSHCNT_LOW = 0x7C;
// Flash update count, lower word
constexpr uint8_t FLSHCNT_HIGH = 0x7E;
// Flash update count, upper word
constexpr uint8_t Y_ACCL_LOW = 0x14;

}  // namespace registers

// The complete automatic packet we will send. This needs to include the dummy 0
// bytes making up full 16-bit frames.
// Note that in addition to the 24-byte limit from the FPGA, this is also
// limited to 12 16-bit register reads by the IMU itself given that we're
// reading at the full 2kHz rate.
// We rotate the registers here by 1, such that the first thing we read is the
// last thing triggered by the previous reading. We put DIAG_STAT in this
// position because we don't care if it's one cycle stale.
constexpr uint8_t kAutospiPacket[] = {
    // X
    registers::X_GYRO_OUT,
    0,
    registers::X_ACCL_OUT,
    0,
    registers::X_ACCL_LOW,
    0,
    // Y
    registers::Y_GYRO_OUT,
    0,
    registers::Y_ACCL_OUT,
    0,
    registers::Y_ACCL_LOW,
    0,
    // Z
    registers::Z_GYRO_OUT,
    0,
    registers::Z_ACCL_OUT,
    0,
    registers::Z_ACCL_LOW,
    0,
    // Other
    registers::TEMP_OUT,
    0,
    registers::DIAG_STAT,
    0,
};
// clang-format on

static_assert((sizeof(kAutospiPacket) % 2) == 0,
              "Need a whole number of register reads");

static constexpr size_t kAutospiDataSize =
    sizeof(kAutospiPacket) + 1 /* timestamp */;

// radian/second/LSB for the gyros (for just the 16-bit value).
constexpr double kGyroLsbRadianSecond =
    1.0 / 10.0 * (2.0 * M_PI / 360.0) /* degrees -> radians */;
// G/LSB for the accelerometers (for the full 32-bit value).
constexpr double kAccelerometerLsbG = 1.0 / 52'428'800.0;
// C/LSB for the temperature.
constexpr double kTemperatureLsbDegree = 0.1;

// This is what the datasheet says PROD_ID should be.
constexpr uint16_t kExpectedProductId = 0x4056;
// This is the PROD_ID we observe.
constexpr uint16_t kObservedProductId = 0x4256;

}  // namespace

ADIS16470::ADIS16470(aos::EventLoop *event_loop, frc::SPI *spi,
                     frc::DigitalInput *data_ready, frc::DigitalOutput *reset)
    : event_loop_(event_loop),
      imu_values_sender_(
          event_loop_->MakeSender<::frc971::IMUValuesBatch>("/drivetrain")),
      initialize_timer_(
          event_loop_->AddTimer([this]() { DoInitializeStep(); })),
      spi_(spi),
      data_ready_(data_ready),
      reset_(reset) {
  // Rather than put the entire data packet into the header, just put a size
  // there and verify it matches here.
  CHECK_EQ(kAutospiDataSize, read_data_.size());

  // We're not doing burst mode, so this is the IMU's rated speed.
  spi_->SetClockRate(2'000'000);
  spi_->SetChipSelectActiveLow();
  spi_->SetMode(frc::SPI::Mode::kMode3);

  // NI's SPI driver defaults to SCHED_OTHER.  Find it's PID with ps, and change
  // it to a RT priority of 33.
  PCHECK(system("busybox ps -ef | grep '\\[spi0\\]' | awk '{print $1}' | xargs "
                "chrt -f -p "
                "33") == 0);
  PCHECK(system("busybox ps -ef | grep '\\[spi1\\]' | awk '{print $1}' | xargs "
                "chrt -f -p "
                "33") == 0);

  event_loop_->OnRun([this]() { BeginInitialization(); });
}

void ADIS16470::DoReads() {
  if (state_ != State::kRunning) {
    // Not sure how to interpret data received now, so ignore it.
    return;
  }

  auto builder = imu_values_sender_.MakeBuilder();

  int amount_to_read =
      spi_->ReadAutoReceivedData(to_read_.data(), 0, 0 /* don't block */);

  aos::SizedArray<flatbuffers::Offset<IMUValues>, 50> readings_offsets;
  while (true) {
    if (amount_to_read == 0) break;
    CHECK(!to_read_.empty());
    const int amount_read_now = std::min<int>(amount_to_read, to_read_.size());
    CHECK_GT(amount_read_now, 0) << "amount_to_read: " << amount_to_read
                                 << ", to_read_.size(): " << to_read_.size();
    spi_->ReadAutoReceivedData(to_read_.data(), amount_read_now,
                               0 /* don't block */);
    to_read_ = to_read_.subspan(amount_read_now);
    amount_to_read -= amount_read_now;

    if (to_read_.empty()) {
      flatbuffers::Offset<IMUValues> reading_offset =
          ProcessReading(builder.fbb());
      readings_offsets.push_back(reading_offset);

      // Reset for the next reading.
      to_read_ = absl::MakeSpan(read_data_);
    } else {
      CHECK_EQ(amount_to_read, 0);
      break;
    }
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<IMUValues>>>
      readings_offset = builder.fbb()->CreateVector(readings_offsets.data(),
                                                    readings_offsets.size());

  IMUValuesBatch::Builder imu_values_batch_builder =
      builder.MakeBuilder<IMUValuesBatch>();
  imu_values_batch_builder.add_readings(readings_offset);
  builder.CheckOk(builder.Send(imu_values_batch_builder.Finish()));
}

void ADIS16470::DoInitializeStep() {
  switch (state_) {
    case State::kUninitialized: {
      to_read_ = absl::MakeSpan(read_data_);

      // First, set the SPI to normal mode so it stops trying to talk
      // automatically.
      spi_->StopAuto();

      reset_->Set(false);
      // Datasheet says it needs a 1 us pulse, so make sure we do something in
      // between asserting and deasserting.
      std::this_thread::sleep_for(chrono::milliseconds(1));
      reset_->Set(true);

      state_ = State::kWaitForReset;
      // Datasheet says it takes 193 ms to come out of reset, so give it some
      // margin on top of that.
      initialize_timer_->Schedule(event_loop_->monotonic_now() +
                                  chrono::milliseconds(250));
    } break;

    case State::kWaitForReset: {
      flatbuffers::Offset<ADIS16470DiagStat> start_diag_stat;
      flatbuffers::Offset<ADIS16470DiagStat> self_test_diag_stat;
      bool success = false;
      auto builder = imu_values_sender_.MakeBuilder();

      // Configure the IMU the way we want it.
      const uint16_t product_id = ReadRegister(registers::PROD_ID, 0);
      if (product_id == kExpectedProductId ||
          product_id == kObservedProductId) {
        const uint16_t start_diag_stat_value =
            ReadRegister(registers::DIAG_STAT, 0);
        start_diag_stat = PackDiagStat(builder.fbb(), start_diag_stat_value);
        if (!DiagStatHasError(
                *GetTemporaryPointer(*builder.fbb(), start_diag_stat))) {
          WriteRegister(registers::FILT_CTRL, 0 /* no filtering */);
          WriteRegister(
              registers::MSC_CTRL,
              (1 << 7) /* enable gyro linear g compensation */ |
                  (1 << 6) /* enable point of percussion alignment */ |
                  (0 << 2) /* internal clock mode */ |
                  (0 << 1) /* sync polarity, doesn't matter */ |
                  (1 << 0) /* data ready is active high */);
          // Rate of the output will be 2000 / (DEC_RATE + 1) Hz.
          WriteRegister(registers::DEC_RATE,
                        1 /* Average every pair of values. */);

          // Start a sensor self test.
          WriteRegister(registers::GLOB_CMD, 1 << 2);
          // Datasheet says it takes 14ms, so give it some margin.
          std::this_thread::sleep_for(chrono::milliseconds(25));
          // Read DIAG_STAT again, and queue up a read of the first part of the
          // autospi data packet.
          const uint16_t self_test_diag_stat_value =
              ReadRegister(registers::DIAG_STAT, kAutospiPacket[0]);
          self_test_diag_stat =
              PackDiagStat(builder.fbb(), self_test_diag_stat_value);
          if (!DiagStatHasError(
                  *GetTemporaryPointer(*builder.fbb(), self_test_diag_stat))) {
            // Initialize automatic mode, but don't start it yet.
            spi_->InitAuto(kAutospiDataSize * 100);
            spi_->SetAutoTransmitData(kAutospiPacket,
                                      0 /* no extra 0s at the end */);
            // No idea what units the "stall period" is in. This value is just
            // bigger than the 16us min from the datasheet. It does not appear
            // to scale with SPICLK frequency. Empirically, this value comes out
            // to 16.7us.
            spi_->ConfigureAutoStall(
                0 /* the minimum CS delay is enough for this IMU */, 670,
                1 /* toggle CS every 2 8-bit bytes */);

            // Read any data queued up by the FPGA.
            while (true) {
              uint32_t buffer;
              if (spi_->ReadAutoReceivedData(&buffer, 1, 0 /* don't block */) ==
                  0) {
                break;
              }
            }

            // Finally, enable automatic mode so it starts reading data.
            spi_->StartAutoTrigger(*data_ready_, true, false);

            // We need a bit of time for the auto trigger to start up so we have
            // something to throw out.  1 khz trigger, so 2 ms gives us 2 cycles
            // to hit it worst case.
            std::this_thread::sleep_for(chrono::milliseconds(2));

            // Throw out the first sample.  It is almost always faulted due to
            // how we start up, and it isn't worth tracking for downstream users
            // to look at.
            to_read_ = absl::MakeSpan(read_data_);
            CHECK_EQ(spi_->ReadAutoReceivedData(
                         to_read_.data(), to_read_.size(),
                         1000.0 /* block for up to 1 second */),
                     static_cast<int>(to_read_.size()))
                << ": Failed to read first sample.";
            success = true;
          }
        }
      }

      IMUValues::Builder imu_builder = builder.MakeBuilder<IMUValues>();
      imu_builder.add_product_id(product_id);
      if (!start_diag_stat.IsNull()) {
        imu_builder.add_start_diag_stat(start_diag_stat);
      }
      if (!self_test_diag_stat.IsNull()) {
        imu_builder.add_self_test_diag_stat(self_test_diag_stat);
      }

      const flatbuffers::Offset<IMUValues> readings_offsets =
          imu_builder.Finish();
      const flatbuffers::Offset<
          flatbuffers::Vector<flatbuffers::Offset<IMUValues>>>
          readings_offset = builder.fbb()->CreateVector(&readings_offsets, 1);

      IMUValuesBatch::Builder imu_batch_builder =
          builder.MakeBuilder<IMUValuesBatch>();
      imu_batch_builder.add_readings(readings_offset);
      builder.CheckOk(builder.Send(imu_batch_builder.Finish()));
      if (success) {
        state_ = State::kRunning;
      } else {
        BeginInitialization();
      }
    } break;

    case State::kRunning:
      LOG(FATAL) << "Not a reset state";
  }
}

flatbuffers::Offset<IMUValues> ADIS16470::ProcessReading(
    flatbuffers::FlatBufferBuilder *fbb) {
  // If we ever see this, we'll need to decide how to handle it. Probably reset
  // everything and try again.
  CHECK_EQ(0, spi_->GetAutoDroppedCount());

  absl::Span<const uint32_t> to_process = read_data_;
  hal::fpga_clock::time_point fpga_time;
  {
    int32_t status = 0;
    const uint64_t fpga_expanded = HAL_ExpandFPGATime(to_process[0], &status);
    CHECK_EQ(0, status);
    fpga_time =
        hal::fpga_clock::time_point(hal::fpga_clock::duration(fpga_expanded));
  }
  to_process = to_process.subspan(1);

  const uint16_t diag_stat_value = (static_cast<uint16_t>(to_process[0]) << 8) |
                                   static_cast<uint16_t>(to_process[1]);
  const auto diag_stat = PackDiagStat(fbb, diag_stat_value);
  to_process = to_process.subspan(2);

  IMUValues::Builder imu_builder(*fbb);
  imu_builder.add_fpga_timestamp(
      aos::time::DurationInSeconds(fpga_time.time_since_epoch()));
  imu_builder.add_monotonic_timestamp_ns(
      time_converter_.FpgaToMonotonic(fpga_time).time_since_epoch().count());
  imu_builder.add_previous_reading_diag_stat(diag_stat);

  imu_builder.add_gyro_x(ConvertValue16(to_process, kGyroLsbRadianSecond));
  to_process = to_process.subspan(2);
  imu_builder.add_accelerometer_x(
      ConvertValue32(to_process, kAccelerometerLsbG));
  to_process = to_process.subspan(4);
  imu_builder.add_gyro_y(ConvertValue16(to_process, kGyroLsbRadianSecond));
  to_process = to_process.subspan(2);
  imu_builder.add_accelerometer_y(
      ConvertValue32(to_process, kAccelerometerLsbG));
  to_process = to_process.subspan(4);
  imu_builder.add_gyro_z(ConvertValue16(to_process, kGyroLsbRadianSecond));
  to_process = to_process.subspan(2);
  imu_builder.add_accelerometer_z(
      ConvertValue32(to_process, kAccelerometerLsbG));
  to_process = to_process.subspan(4);

  imu_builder.add_temperature(
      ConvertValue16(to_process, kTemperatureLsbDegree));
  to_process = to_process.subspan(2);

  CHECK(to_process.empty()) << "Have leftover bytes: " << to_process.size();

  return imu_builder.Finish();
}

double ADIS16470::ConvertValue32(absl::Span<const uint32_t> data,
                                 double lsb_per_output) {
  const uint32_t unsigned_value = (static_cast<uint32_t>(data[0]) << 24) |
                                  (static_cast<uint32_t>(data[1]) << 16) |
                                  (static_cast<uint32_t>(data[2]) << 8) |
                                  static_cast<uint32_t>(data[3]);
  int32_t signed_value;
  memcpy(&signed_value, &unsigned_value, sizeof(unsigned_value));
  return static_cast<double>(signed_value) * lsb_per_output;
}

double ADIS16470::ConvertValue16(absl::Span<const uint32_t> data,
                                 double lsb_per_output) {
  const uint16_t unsigned_value =
      (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
  int16_t signed_value;
  memcpy(&signed_value, &unsigned_value, sizeof(unsigned_value));
  return static_cast<double>(signed_value) * lsb_per_output;
}

flatbuffers::Offset<ADIS16470DiagStat> ADIS16470::PackDiagStat(
    flatbuffers::FlatBufferBuilder *fbb, uint16_t value) {
  ADIS16470DiagStat::Builder diag_stat_builder(*fbb);
  diag_stat_builder.add_clock_error(value & (1 << 7));
  diag_stat_builder.add_memory_failure(value & (1 << 6));
  diag_stat_builder.add_sensor_failure(value & (1 << 5));
  diag_stat_builder.add_standby_mode(value & (1 << 4));
  diag_stat_builder.add_spi_communication_error(value & (1 << 3));
  diag_stat_builder.add_flash_memory_update_error(value & (1 << 2));
  diag_stat_builder.add_data_path_overrun(value & (1 << 1));
  return diag_stat_builder.Finish();
}

bool ADIS16470::DiagStatHasError(const ADIS16470DiagStat &diag_stat) {
  return diag_stat.clock_error() || diag_stat.memory_failure() ||
         diag_stat.sensor_failure() || diag_stat.standby_mode() ||
         diag_stat.spi_communication_error() ||
         diag_stat.flash_memory_update_error() || diag_stat.data_path_overrun();
}

uint16_t ADIS16470::ReadRegister(uint8_t register_address,
                                 uint8_t next_register_address) {
  uint8_t send_buffer[2] = {static_cast<uint8_t>(register_address & 0x7f), 0};
  uint8_t dummy[2];
  spi_->Transaction(send_buffer, dummy, sizeof(send_buffer));
  uint8_t receive_buffer[2];
  uint8_t next_send_buffer[2] = {
      static_cast<uint8_t>(next_register_address & 0x7f), 0};
  spi_->Transaction(next_send_buffer, receive_buffer, sizeof(receive_buffer));
  return (static_cast<uint16_t>(receive_buffer[0]) << 8) |
         static_cast<uint16_t>(receive_buffer[1]);
}

void ADIS16470::WriteRegister(uint8_t register_address, uint16_t value) {
  uint8_t buffer1[2] = {static_cast<uint8_t>(register_address | 0x80),
                        static_cast<uint8_t>(value & 0xff)};
  uint8_t buffer2[2] = {static_cast<uint8_t>(register_address | 0x81),
                        static_cast<uint8_t>(value >> 8)};
  spi_->Write(buffer1, sizeof(buffer1));
  spi_->Write(buffer2, sizeof(buffer2));
}

}  // namespace wpilib
}  // namespace frc971
