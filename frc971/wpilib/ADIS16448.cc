#include "frc971/wpilib/ADIS16448.h"

#include "InterruptableSensorBase.h"
#undef ERROR

#include <inttypes.h>
#include <math.h>
#include <chrono>

#include "aos/common/logging/queue_logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/time.h"
#include "aos/linux_code/init.h"
#include "frc971/wpilib/imu.q.h"
#include "frc971/zeroing/averager.h"

namespace frc971 {
namespace wpilib {

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

template <uint8_t size>
bool ADIS16448::DoTransaction(uint8_t to_send[size], uint8_t to_receive[size]) {
  switch (spi_->Transaction(to_send, to_receive, size)) {
    case -1:
      LOG(INFO, "SPI::Transaction of %zd bytes failed\n", size);
      return false;
    case size:
      if (dummy_spi_) {
        uint8_t dummy_send, dummy_receive;
        dummy_spi_->Transaction(&dummy_send, &dummy_receive, 1);
      }
      return true;
    default:
      LOG(FATAL, "SPI::Transaction returned something weird\n");
  }
}

namespace {

// Addresses pulled out of the documentation.
constexpr uint8_t kMscCtrlAddress = 0x34;
constexpr uint8_t kSmplPrdAddress = 0x36;
constexpr uint8_t kDiagStatAddress = 0x3C;
constexpr uint8_t kGlobalReadAddress = 0x3E;
constexpr uint8_t kLotId1Address = 0x52;
constexpr uint8_t kLotId2Address = 0x54;
constexpr uint8_t kProdIdAddress = 0x56;
constexpr uint8_t kSerialNumberAddress = 0x58;

// degree/second/LSB for the gyros.
constexpr double kGyroLsbDegreeSecond = 1.0 / 25.0;
// G/LSB for the accelerometers.
constexpr double kAccelerometerLsbG = 1.0 / 1200.0;
// gauss/LSB for the magnetometers.
constexpr double kMagnetometerLsbGauss =
    1.0 / (7.0 / 1000.0) /* mgauss to gauss */;
// bar/LSB for the barometer.
constexpr double kBarometerLsbPascal = 0.02 * 100;
// degree/LSB C for the temperature sensor.
constexpr double kTemperatureLsbDegree = 0.07386;
// Degrees C corresponding to 0 for the temperature sensor.
constexpr double kTemperatureZero = 31;

// From somebody online who says this works with the sensor. I don't feel like
// re-deriving this, and I can't find what all the CRC parameters are supposed
// to be.
const uint16_t kCrcTable[256] = {
    0x0000, 0x17CE, 0x0FDF, 0x1811, 0x1FBE, 0x0870, 0x1061, 0x07AF, 0x1F3F,
    0x08F1, 0x10E0, 0x072E, 0x0081, 0x174F, 0x0F5E, 0x1890, 0x1E3D, 0x09F3,
    0x11E2, 0x062C, 0x0183, 0x164D, 0x0E5C, 0x1992, 0x0102, 0x16CC, 0x0EDD,
    0x1913, 0x1EBC, 0x0972, 0x1163, 0x06AD, 0x1C39, 0x0BF7, 0x13E6, 0x0428,
    0x0387, 0x1449, 0x0C58, 0x1B96, 0x0306, 0x14C8, 0x0CD9, 0x1B17, 0x1CB8,
    0x0B76, 0x1367, 0x04A9, 0x0204, 0x15CA, 0x0DDB, 0x1A15, 0x1DBA, 0x0A74,
    0x1265, 0x05AB, 0x1D3B, 0x0AF5, 0x12E4, 0x052A, 0x0285, 0x154B, 0x0D5A,
    0x1A94, 0x1831, 0x0FFF, 0x17EE, 0x0020, 0x078F, 0x1041, 0x0850, 0x1F9E,
    0x070E, 0x10C0, 0x08D1, 0x1F1F, 0x18B0, 0x0F7E, 0x176F, 0x00A1, 0x060C,
    0x11C2, 0x09D3, 0x1E1D, 0x19B2, 0x0E7C, 0x166D, 0x01A3, 0x1933, 0x0EFD,
    0x16EC, 0x0122, 0x068D, 0x1143, 0x0952, 0x1E9C, 0x0408, 0x13C6, 0x0BD7,
    0x1C19, 0x1BB6, 0x0C78, 0x1469, 0x03A7, 0x1B37, 0x0CF9, 0x14E8, 0x0326,
    0x0489, 0x1347, 0x0B56, 0x1C98, 0x1A35, 0x0DFB, 0x15EA, 0x0224, 0x058B,
    0x1245, 0x0A54, 0x1D9A, 0x050A, 0x12C4, 0x0AD5, 0x1D1B, 0x1AB4, 0x0D7A,
    0x156B, 0x02A5, 0x1021, 0x07EF, 0x1FFE, 0x0830, 0x0F9F, 0x1851, 0x0040,
    0x178E, 0x0F1E, 0x18D0, 0x00C1, 0x170F, 0x10A0, 0x076E, 0x1F7F, 0x08B1,
    0x0E1C, 0x19D2, 0x01C3, 0x160D, 0x11A2, 0x066C, 0x1E7D, 0x09B3, 0x1123,
    0x06ED, 0x1EFC, 0x0932, 0x0E9D, 0x1953, 0x0142, 0x168C, 0x0C18, 0x1BD6,
    0x03C7, 0x1409, 0x13A6, 0x0468, 0x1C79, 0x0BB7, 0x1327, 0x04E9, 0x1CF8,
    0x0B36, 0x0C99, 0x1B57, 0x0346, 0x1488, 0x1225, 0x05EB, 0x1DFA, 0x0A34,
    0x0D9B, 0x1A55, 0x0244, 0x158A, 0x0D1A, 0x1AD4, 0x02C5, 0x150B, 0x12A4,
    0x056A, 0x1D7B, 0x0AB5, 0x0810, 0x1FDE, 0x07CF, 0x1001, 0x17AE, 0x0060,
    0x1871, 0x0FBF, 0x172F, 0x00E1, 0x18F0, 0x0F3E, 0x0891, 0x1F5F, 0x074E,
    0x1080, 0x162D, 0x01E3, 0x19F2, 0x0E3C, 0x0993, 0x1E5D, 0x064C, 0x1182,
    0x0912, 0x1EDC, 0x06CD, 0x1103, 0x16AC, 0x0162, 0x1973, 0x0EBD, 0x1429,
    0x03E7, 0x1BF6, 0x0C38, 0x0B97, 0x1C59, 0x0448, 0x1386, 0x0B16, 0x1CD8,
    0x04C9, 0x1307, 0x14A8, 0x0366, 0x1B77, 0x0CB9, 0x0A14, 0x1DDA, 0x05CB,
    0x1205, 0x15AA, 0x0264, 0x1A75, 0x0DBB, 0x152B, 0x02E5, 0x1AF4, 0x0D3A,
    0x0A95, 0x1D5B, 0x054A, 0x1284};

uint16_t CalculateCrc(const uint8_t *data, size_t data_length) {
  uint16_t crc = 0xFFFF;
  uint16_t byte;

  while (data_length--) {
    // Compute lower byte CRC first.
    byte = data[1];
    crc = (crc >> 8) ^ kCrcTable[(crc & 0x00FF) ^ byte];
    // Compute upper byte of CRC.
    byte = data[0];
    crc = (crc >> 8) ^ kCrcTable[(crc & 0x00FF) ^ byte];
    data += 2;
  }
  crc = ~crc;  // Compute complement of CRC
  return static_cast<uint16_t>(
      (crc << 8) | (crc >> 8));  // Perform byte swap prior to returning CRC;
}

}  // namespace

ADIS16448::ADIS16448(SPI::Port port, DigitalInput *dio1)
    : spi_(new SPI(port)), dio1_(dio1) {
  // 1MHz is the maximum supported for burst reads, but we
  // want to go slower to hopefully make it more reliable.
  spi_->SetClockRate(1e5);
  spi_->SetChipSelectActiveLow();
  spi_->SetClockActiveLow();
  spi_->SetSampleDataOnFalling();
  spi_->SetMSBFirst();

  dio1_->RequestInterrupts();
  dio1_->SetUpSourceEdge(true, false);
}

void ADIS16448::SetDummySPI(SPI::Port port) {
  dummy_spi_.reset(new SPI(port));
  // Pick the same settings here in case the roboRIO decides to try something
  // stupid when switching.
  if (dummy_spi_) {
    dummy_spi_->SetClockRate(1e5);
    dummy_spi_->SetChipSelectActiveLow();
    dummy_spi_->SetClockActiveLow();
    dummy_spi_->SetSampleDataOnFalling();
    dummy_spi_->SetMSBFirst();
  }
}

void ADIS16448::InitializeUntilSuccessful() {
  while (run_ && !Initialize()) {
    if (reset_) {
      reset_->Set(false);
      // Datasheet says this needs to be at least 10 us long, so 10 ms is
      // plenty.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(10));
      reset_->Set(true);
      // Datasheet says this takes 90 ms typically, and we want to give it
      // plenty of margin.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(150));
    } else {
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(50));
    }
  }
  LOG(INFO, "IMU initialized successfully\n");

  ::aos::SetCurrentThreadRealtimePriority(33);
}

void ADIS16448::operator()() {
  // NI's SPI driver defaults to SCHED_OTHER.  Find it's PID with ps, and change
  // it to a RT priority of 33.
  PCHECK(system(
             "ps -ef | grep '\\[spi0\\]' | awk '{print $1}' | xargs chrt -f -p "
             "33") == 0);

  ::aos::SetCurrentThreadName("IMU");

  InitializeUntilSuccessful();

  // Rounded to approximate the 204.8 Hz.
  constexpr size_t kImuSendRate = 205;

  zeroing::Averager<double, 6 * kImuSendRate> average_gyro_x;
  zeroing::Averager<double, 6 * kImuSendRate> average_gyro_y;
  zeroing::Averager<double, 6 * kImuSendRate> average_gyro_z;

  bool got_an_interrupt = false;
  while (run_) {
    {
      const InterruptableSensorBase::WaitResult result =
          dio1_->WaitForInterrupt(0.1, !got_an_interrupt);
      if (result == InterruptableSensorBase::kTimeout) {
        LOG(WARNING, "IMU read timed out\n");
        InitializeUntilSuccessful();
        continue;
      }
    }
    got_an_interrupt = true;
    const monotonic_clock::time_point read_time = monotonic_clock::now();

    uint8_t to_send[2 * 14], to_receive[2 * 14];
    memset(&to_send[0], 0, sizeof(to_send));
    to_send[0] = kGlobalReadAddress;
    if (!DoTransaction<2 * 14>(to_send, to_receive)) continue;

    // If it's false now or another edge happened, then we're in trouble. This
    // won't catch all instances of being a little bit slow (because of the
    // interrupt delay among other things), but it will catch the code
    // constantly falling behind, which seems like the most likely failure
    // scenario.
    if (!dio1_->Get() ||
        dio1_->WaitForInterrupt(0, false) !=
            InterruptableSensorBase::kTimeout) {
      LOG(ERROR, "IMU read took too long\n");
      continue;
    }

    {
      const uint16_t calculated_crc = CalculateCrc(&to_receive[4], 11);
      uint16_t received_crc =
          to_receive[13 * 2 + 1] | (to_receive[13 * 2] << 8);
      if (received_crc != calculated_crc) {
        LOG(WARNING, "received CRC %" PRIx16 " but calculated %" PRIx16 "\n",
            received_crc, calculated_crc);
        InitializeUntilSuccessful();
        continue;
      }
    }

    {
      uint16_t diag_stat;
      memcpy(&diag_stat, &to_receive[2], 2);
      if (!CheckDiagStatValue(diag_stat)) {
        InitializeUntilSuccessful();
        continue;
      }
    }

    auto message = imu_values.MakeMessage();
    message->fpga_timestamp = dio1_->ReadRisingTimestamp();
    message->monotonic_timestamp_ns =
        chrono::duration_cast<chrono::nanoseconds>(read_time.time_since_epoch())
            .count();

    message->gyro_x =
        ConvertValue(&to_receive[4], kGyroLsbDegreeSecond * M_PI / 180.0);
    message->gyro_y =
        ConvertValue(&to_receive[6], kGyroLsbDegreeSecond * M_PI / 180.0);
    message->gyro_z =
        ConvertValue(&to_receive[8], kGyroLsbDegreeSecond * M_PI / 180.0);

    // The first few seconds of samples are averaged and subtracted from
    // subsequent samples for zeroing purposes.
    if (!gyros_are_zeroed_) {
      average_gyro_x.AddData(message->gyro_x);
      average_gyro_y.AddData(message->gyro_y);
      average_gyro_z.AddData(message->gyro_z);

      if (average_gyro_x.full() && average_gyro_y.full() &&
          average_gyro_z.full()) {
        ::aos::joystick_state.FetchLatest();
        if (::aos::joystick_state.get() && ::aos::joystick_state->enabled) {
          gyro_x_zeroed_offset_ = -average_gyro_x.GetAverage();
          gyro_y_zeroed_offset_ = -average_gyro_y.GetAverage();
          gyro_z_zeroed_offset_ = -average_gyro_z.GetAverage();
          LOG(INFO, "total gyro zero offset X:%f, Y:%f, Z:%f\n",
              gyro_x_zeroed_offset_, gyro_y_zeroed_offset_,
              gyro_z_zeroed_offset_);
          gyros_are_zeroed_ = true;
        }
      }
    }

    message->gyro_x += gyro_x_zeroed_offset_;
    message->gyro_y += gyro_y_zeroed_offset_;
    message->gyro_z += gyro_z_zeroed_offset_;

    message->accelerometer_x =
        ConvertValue(&to_receive[10], kAccelerometerLsbG);
    message->accelerometer_y =
        ConvertValue(&to_receive[12], kAccelerometerLsbG);
    message->accelerometer_z =
        ConvertValue(&to_receive[14], kAccelerometerLsbG);

    message->magnetometer_x =
        ConvertValue(&to_receive[16], kMagnetometerLsbGauss);
    message->magnetometer_y =
        ConvertValue(&to_receive[18], kMagnetometerLsbGauss);
    message->magnetometer_z =
        ConvertValue(&to_receive[20], kMagnetometerLsbGauss);

    message->barometer =
        ConvertValue(&to_receive[22], kBarometerLsbPascal, false);

    message->temperature =
        ConvertValue(&to_receive[24], kTemperatureLsbDegree) + kTemperatureZero;

    LOG_STRUCT(DEBUG, "sending", *message);
    if (!message.Send()) {
      LOG(WARNING, "sending queue message failed\n");
    }
  }
}

float ADIS16448::ConvertValue(uint8_t *data, double lsb_per_output, bool sign) {
  double value;
  if (sign) {
    int16_t raw_value = static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
    value = raw_value;
  } else {
    uint16_t raw_value =
        (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    value = raw_value;
  }
  return value * lsb_per_output;
}

bool ADIS16448::ReadRegister(uint8_t next_address, uint16_t *value) {
  uint8_t to_send[2], to_receive[2];
  to_send[0] = next_address;
  to_send[1] = 0;

  if (!DoTransaction<2>(to_send, to_receive)) return false;

  if (value) {
    memcpy(value, to_receive, 2);
  }
  return true;
}

bool ADIS16448::WriteRegister(uint8_t address, uint16_t value) {
  uint8_t to_send[4], to_receive[4];
  to_send[0] = address | 0x80;
  to_send[1] = value & 0xFF;
  to_send[2] = address | 0x81;
  to_send[3] = value >> 8;
  if (!DoTransaction<4>(to_send, to_receive)) return false;
  return true;
}

bool ADIS16448::CheckDiagStatValue(uint16_t value) const {
  bool r = true;
  if (value & (1 << 2)) {
    LOG(WARNING, "IMU gave flash update failure\n");
  }
  if (value & (1 << 3)) {
    LOG(WARNING, "IMU gave SPI communication failure\n");
  }
  if (value & (1 << 4)) {
    LOG(WARNING, "IMU gave sensor overrange\n");
  }
  if (value & (1 << 5)) {
    LOG(WARNING, "IMU gave self-test failure\n");
    r = false;
    if (value & (1 << 10)) {
      LOG(WARNING, "IMU gave X-axis gyro self-test failure\n");
    }
    if (value & (1 << 11)) {
      LOG(WARNING, "IMU gave Y-axis gyro self-test failure\n");
    }
    if (value & (1 << 12)) {
      LOG(WARNING, "IMU gave Z-axis gyro self-test failure\n");
    }
    if (value & (1 << 13)) {
      LOG(WARNING, "IMU gave X-axis accelerometer self-test failure\n");
    }
    if (value & (1 << 14)) {
      LOG(WARNING, "IMU gave Y-axis accelerometer self-test failure\n");
    }
    if (value & (1 << 15)) {
      LOG(WARNING, "IMU gave Z-axis accelerometer self-test failure, %x\n",
          value);
    }
    if (value & (1 << 0)) {
      LOG(WARNING, "IMU gave magnetometer functional test failure\n");
    }
    if (value & (1 << 1)) {
      LOG(WARNING, "IMU gave barometer functional test failure\n");
    }
  }
  if (value & (1 << 6)) {
    LOG(WARNING, "IMU gave flash test checksum failure\n");
  }
  if (value & (1 << 8)) {
    LOG(WARNING, "IMU says alarm 1 is active\n");
  }
  if (value & (1 << 9)) {
    LOG(WARNING, "IMU says alarm 2 is active\n");
  }
  return r;
}

bool ADIS16448::Initialize() {
  if (!ReadRegister(kProdIdAddress, nullptr)) return false;
  uint16_t product_id;
  if (!ReadRegister(kLotId1Address, &product_id)) return false;
  if (product_id != 0x4040) {
    LOG(ERROR, "product ID is %" PRIx16 " instead of 0x4040\n", product_id);
    return false;
  }

  uint16_t lot_id1, lot_id2, serial_number;
  if (!ReadRegister(kLotId2Address, &lot_id1)) return false;
  if (!ReadRegister(kSerialNumberAddress, &lot_id2)) return false;
  if (!ReadRegister(0, &serial_number)) return false;
  LOG(INFO, "have IMU %" PRIx16 "%" PRIx16 ": %" PRIx16 "\n", lot_id1, lot_id2,
      serial_number);

  // Divide the sampling by 2^2 = 4 to get 819.2 / 4 = 204.8 Hz.
  if (!WriteRegister(kSmplPrdAddress, (2 << 8) | 1)) return false;

  // Start a self test.
  if (!WriteRegister(kMscCtrlAddress, 1 << 10)) return false;
  // Wait for the self test to finish.
  {
    uint16_t value;
    do {
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(10));
      if (!ReadRegister(kMscCtrlAddress, &value)) return false;
    } while ((value & (1 << 10)) != 0);
  }

  if (!ReadRegister(kDiagStatAddress, nullptr)) return false;
  uint16_t diag_stat;
  if (!ReadRegister(0, &diag_stat)) return false;
  if (!CheckDiagStatValue(diag_stat)) return false;

  if (!WriteRegister(kMscCtrlAddress,
                     ((0 << 0) |  // DIO1
                      (1 << 1) |  // DIO goes high when data is valid
                      (1 << 2) |  // enable DIO changing when data is vald
                      (1 << 4) |  // enable CRC16 for burst mode
                      (1 << 6)))) {
    return false;
  }
  return true;
}

}  // namespace wpilib
}  // namespace frc971
