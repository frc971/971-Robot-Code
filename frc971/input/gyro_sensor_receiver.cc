#include <memory>

#include "aos/common/inttypes.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/sensors/sensor_unpacker.h"
#include "aos/common/sensors/sensor_receiver.h"
#include "aos/common/glibusb/glibusb.h"
#include "aos/common/glibusb/gbuffer.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/input/gyro_board_data.h"
#include "frc971/queues/GyroAngle.q.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::angle_adjust;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::index_loop;
using ::frc971::sensors::gyro;

namespace frc971 {
namespace {

inline double drivetrain_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (19.0 / 50.0) /*output reduction*/ * (64.0 / 24.0) /*encoder gears*/ *
      (3.5 /*wheel diameter*/ * 2.54 / 100.0 * M_PI);
}

inline double wrist_translate(int32_t in) {
  return static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      (14.0 / 50.0 * 20.0 / 84.0) /*gears*/ * (2 * M_PI);
}

inline double angle_adjust_translate(int32_t in) {
  static const double kCableDiameter = 0.060;
  return -static_cast<double>(in) / (256.0 /*cpr*/ * 4.0 /*quad*/) *
      ((0.75 + kCableDiameter) / (16.61125 + kCableDiameter)) /*pulleys*/ *
      (2 * M_PI);
}

inline double shooter_translate(int32_t in) {
 return static_cast<double>(in) / (32.0 /*cpr*/ * 4.0 /*quad*/) *
      (15.0 / 34.0) /*gears*/ * (2 * M_PI);
}

inline double index_translate(int32_t in) {
  return -static_cast<double>(in) / (128.0 /*cpr*/ * 4.0 /*quad*/) *
      (1.0) /*gears*/ * (2 * M_PI);
}

}  // namespace

class GyroSensorUnpacker :
    public ::aos::sensors::SensorUnpackerInterface<GyroBoardData> {
 public:
  GyroSensorUnpacker()
      : top_rise_count_(0),
        last_top_rise_count_(0),
        top_fall_count_(0),
        last_top_fall_count_(0),
        bottom_rise_count_(0),
        last_bottom_rise_count_(0),
        bottom_fall_delay_count_(0),
        last_bottom_fall_delay_count_(0),
        bottom_fall_count_(0),
        last_bottom_fall_count_(0),
        wrist_rise_count_(0),
        last_wrist_rise_count_(0),
        shooter_angle_rise_count_(0),
        last_shooter_angle_rise_count_(0) {
  }

  void UnpackFrom(GyroBoardData *data) {
    if (data->robot_id != 0) {
      LOG(ERROR, "gyro board sent data for robot id %hhd!"
          " dip switches are %x\n", data->robot_id, data->base_status & 0xF);
      return;
    } else {
      LOG(DEBUG, "processing a packet dip switches %x\n",
          data->base_status & 0xF);
    }

    static ::aos::time::Time last_time = ::aos::time::Time::Now();
    if ((last_time - ::aos::time::Time::Now()) >
        ::aos::time::Time::InMS(0.00205)) {
      LOG(INFO, "missed one\n");
    }

    gyro.MakeWithBuilder()
        .angle(data->gyro_angle / 16.0 / 1000.0 / 180.0 * M_PI)
        .Send();

    UpdateWrappingCounter(data->main.top_rise_count,
        &last_top_rise_count_, &top_rise_count_);
    UpdateWrappingCounter(data->main.top_fall_count,
        &last_top_fall_count_, &top_fall_count_);
    UpdateWrappingCounter(data->main.bottom_rise_count,
        &last_bottom_rise_count_, &bottom_rise_count_);
    UpdateWrappingCounter(data->main.bottom_fall_delay_count,
        &last_bottom_fall_delay_count_, &bottom_fall_delay_count_);
    UpdateWrappingCounter(data->main.bottom_fall_count,
        &last_bottom_fall_count_, &bottom_fall_count_);
    UpdateWrappingCounter(data->main.wrist_rise_count,
        &last_wrist_rise_count_, &wrist_rise_count_);
    UpdateWrappingCounter(data->main.shooter_angle_rise_count,
        &last_shooter_angle_rise_count_, &shooter_angle_rise_count_);

    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(data->main.right_drive))
        .left_encoder(-drivetrain_translate(data->main.left_drive))
        .Send();

    wrist.position.MakeWithBuilder()
        .pos(wrist_translate(data->main.wrist))
        .hall_effect(!data->main.wrist_hall_effect)
        .calibration(wrist_translate(data->main.capture_wrist_rise))
        .Send();

    angle_adjust.position.MakeWithBuilder()
        .angle(angle_adjust_translate(data->main.shooter_angle))
        .bottom_hall_effect(!data->main.angle_adjust_bottom_hall_effect)
        .middle_hall_effect(false)
        .bottom_calibration(angle_adjust_translate(
                data->main.capture_shooter_angle_rise))
        .middle_calibration(angle_adjust_translate(
                0))
        .Send();

    shooter.position.MakeWithBuilder()
        .position(shooter_translate(data->main.shooter))
        .Send();

    index_loop.position.MakeWithBuilder()
        .index_position(index_translate(data->main.indexer))
        .top_disc_detect(!data->main.top_disc)
        .top_disc_posedge_count(top_rise_count_)
        .top_disc_posedge_position(index_translate(data->main.capture_top_rise))
        .top_disc_negedge_count(top_fall_count_)
        .top_disc_negedge_position(index_translate(data->main.capture_top_fall))
        .bottom_disc_detect(!data->main.bottom_disc)
        .bottom_disc_posedge_count(bottom_rise_count_)
        .bottom_disc_negedge_count(bottom_fall_count_)
        .bottom_disc_negedge_wait_position(index_translate(
                data->main.capture_bottom_fall_delay))
        .bottom_disc_negedge_wait_count(bottom_fall_delay_count_)
        .loader_top(data->main.loader_top)
        .loader_bottom(data->main.loader_bottom)
        .Send();
  }

 private:
  void UpdateWrappingCounter(
      uint8_t current, uint8_t *last, int32_t *counter) {
    if (*last > current) {
      *counter += 0x100;
    }
    *counter = (*counter & 0xffffff00) | current;
    *last = current;
  }

  int32_t top_rise_count_;
  uint8_t last_top_rise_count_;
  int32_t top_fall_count_;
  uint8_t last_top_fall_count_;
  int32_t bottom_rise_count_;
  uint8_t last_bottom_rise_count_;
  int32_t bottom_fall_delay_count_;
  uint8_t last_bottom_fall_delay_count_;
  int32_t bottom_fall_count_;
  uint8_t last_bottom_fall_count_;
  int32_t wrist_rise_count_;
  uint8_t last_wrist_rise_count_;
  int32_t shooter_angle_rise_count_;
  uint8_t last_shooter_angle_rise_count_;
};

class GyroSensorReceiver :
    public ::aos::sensors::SensorReceiver<GyroBoardData> {
 public:
  GyroSensorReceiver(
      ::aos::sensors::SensorUnpackerInterface<GyroBoardData> *unpacker)
      : ::aos::sensors::SensorReceiver<GyroBoardData>(unpacker),
        start_time_(0, 0) {
    static_assert(sizeof(GyroBoardData) <= kDataLength,
                  "the buffer will be too small");
  }

 private:
  static const unsigned char kEndpoint = 0x81;
  // in ms
  // 0 is unlimited
  static const unsigned int kReadTimeout = 1000;

  static constexpr glibusb::VendorProductId kDeviceId = 
      glibusb::VendorProductId(0x1424  /* vendor ID */,
                               0xd243  /* product ID */);

  // How big of a buffer to give the function.
  static const size_t kDataLength = 64;

  virtual bool DoReceiveData() {
    // Loop and then return once we get a good one.
    while (true) {
      using glibusb::UsbEndpoint;
      UsbEndpoint::IoStatus result =
          endpoint_->ReadAtMostWithTimeout(sizeof(GyroBoardData),
                                           kReadTimeout,
                                           &buffer_);
      switch (result) {
        case UsbEndpoint::kSuccess:
          break;
        case UsbEndpoint::kTimeout:
          LOG(WARNING, "read timed out\n");
          continue;
        case UsbEndpoint::kNoDevice:
          LOG(ERROR, "no device\n");
          return true;
        case UsbEndpoint::kUnknown:
        case UsbEndpoint::kFail:
        case UsbEndpoint::kAbort:
          LOG(ERROR, "read failed\n");
          continue;
      }
      if (buffer_.Length() < sizeof(GyroBoardData)) {
        LOG(ERROR, "read %zd bytes instead of at least %zd\n",
            buffer_.Length(), sizeof(GyroBoardData));
        continue;
      }

      memcpy(&data()->values, buffer_.GetBufferPointer(sizeof(GyroBoardData)),
             sizeof(GyroBoardData));
      if (data()->count == 0) {
        start_time_ = ::aos::time::Time::Now();
        data()->count = 1;
      } else {
        ::aos::time::Time delta_time = ::aos::time::Time::Now() - start_time_;
        data()->count = static_cast<int32_t>(
            (delta_time / ::aos::sensors::kSensorSendFrequency) + 0.5);
      }
      return false;
    }
  }

  virtual void Reset() {
    device_ = ::std::unique_ptr<glibusb::UsbDevice>(
        libusb_.FindSingleMatchingDeviceOrLose(kDeviceId));
    CHECK(device_);
    endpoint_ = ::std::unique_ptr<glibusb::UsbInEndpoint>(
        device_->InEndpoint(kEndpoint));

    data()->count = 0;
  }

  virtual void Synchronized(::aos::time::Time start_time) {
    // Subtract off how many packets it read while synchronizing from the time.
    start_time_ = start_time -
        ::aos::sensors::kSensorSendFrequency * data()->count;
  }

  ::std::unique_ptr<glibusb::UsbDevice> device_;
  ::std::unique_ptr<glibusb::UsbInEndpoint> endpoint_;
  glibusb::Buffer buffer_;

  ::aos::time::Time start_time_;

  glibusb::Libusb libusb_;
};
constexpr glibusb::VendorProductId GyroSensorReceiver::kDeviceId;

}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::GyroSensorUnpacker unpacker;
  ::frc971::GyroSensorReceiver receiver(&unpacker);
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
