#include <string.h>

#include <memory>

#include "aos/common/inttypes.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/glibusb/glibusb.h"
#include "aos/common/glibusb/gbuffer.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/control_loop/ControlLoop.h"

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
using ::aos::util::WrappingCounter;

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

class GyroSensorReceiver {
 public:
  GyroSensorReceiver() {
    Reset();
  }

  void RunIteration() {
    if (ReceiveData()) {
      Reset();
    } else {
      const ::aos::time::Time received_time = ::aos::time::Time::Now();
      if (phase_locker_.IsCurrentPacketGood(received_time, sequence_.count())) {
        LOG(DEBUG, "processing data\n");
        ProcessData();
      } else {
        LOG(DEBUG, "not processing\n");
      }
    }
  }

 private:
  static const unsigned char kEndpoint = 0x3;
  // 0 is unlimited
  static constexpr ::aos::time::Time kReadTimeout =
      ::aos::time::Time::InSeconds(1.5);
  static constexpr ::glibusb::VendorProductId kDeviceId = 
      ::glibusb::VendorProductId(0x1424  /* vendor ID */,
                                 0xd243  /* product ID */);

  static const int kPacketsPerLoopCycle = 10;

  // How long before the control loops run we want to use a packet.
  static constexpr ::aos::time::Time kDesiredOffset =
      ::aos::time::Time::InSeconds(-0.0025);

  // Contains all of the complicated state and logic for locking onto the the
  // correct phase.
  class {
   public:
    void Reset() {
      LOG(INFO, "resetting\n");
      last_guessed_time_ = ::aos::time::Time(0, 0);
      good_phase_ = guess_phase_ = kUnknownPhase;
      guess_phase_good_ = guess_phase_bad_ = 0;
      good_phase_early_ = good_phase_late_ = 0;
    }

    // Gets called for every packet received.
    // Returns whether or not to process the values from this packet.
    bool IsCurrentPacketGood(const ::aos::time::Time &received_time,
                             int32_t sequence) {
      // How often we (should) receive packets.
      static const ::aos::time::Time kPacketFrequency =
          ::aos::control_loops::kLoopFrequency / kPacketsPerLoopCycle;
      static const ::aos::time::Time kPacketClose =
          kPacketFrequency * 65 / 100;
      static const ::aos::time::Time kSwitchOffset =
          kPacketFrequency * 6 / 10;

      // When we want to receive a packet for the next cycle of control loops.
      const ::aos::time::Time next_desired =
          ::aos::control_loops::NextLoopTime(received_time + kDesiredOffset);
      // How far off of when we want the next packet this one is.
      const ::aos::time::Time offset = next_desired - received_time;

      const int received_phase = sequence % kPacketsPerLoopCycle;

      assert(!(good_phase_early_ != 0 && good_phase_late_ != 0));

      if (guess_phase_good_ > kMinGoodGuessCycles) {
        good_phase_ = guess_phase_;
        if (guess_phase_offset_ < kPacketFrequency / -2) {
          ++good_phase_;
        } else if (guess_phase_offset_ > kPacketFrequency / 2) {
          --good_phase_;
        }
      } else if (guess_phase_bad_ > kMaxBadGuessCycles) {
        Reset();
      }
      if (good_phase_early_ > kSwitchCycles) {
        good_phase_early_ = 0;
        LOG(INFO, "switching to 1 phase earlier\n");
        --good_phase_;
      } else if (good_phase_late_ > kSwitchCycles) {
        good_phase_late_ = 0;
        LOG(INFO, "switching to 1 phase later\n");
        ++good_phase_;
      }
      if (good_phase_ == kUnknownPhase) {
        LOG(INFO, "guessing which packet is good\n");

        // If we're going to call this packet a good guess.
        bool guess_is_good = false;
        // If it's close to the right time.
        if (offset.abs() < kPacketClose) {
          // If we didn't (also) guess that the previous one was good.
          if (received_time - last_guessed_time_ > kPacketFrequency * 2) {
            LOG(DEBUG, "guessing this one is good\n");
            guess_is_good = true;
          } else {
            LOG(DEBUG, "just guessed\n");
          }
          if (guess_phase_ == kUnknownPhase) {
            if (offset.abs() < kPacketFrequency * 55 / 100) {
              guess_phase_ = received_phase;
              guess_phase_offset_ = offset;
            }
          } else if (received_phase == guess_phase_) {
            ++guess_phase_good_;
            guess_phase_bad_ = 0;
            guess_phase_offset_ = (guess_phase_offset_ * 9 + offset) / 10;
          }
        } else if (guess_phase_ != kUnknownPhase &&
                   received_phase == guess_phase_) {
          ++guess_phase_bad_;
          guess_phase_good_ = ::std::max(0, guess_phase_good_ -
                                         (kMinGoodGuessCycles / 10));
        }
        if (guess_is_good) {
          last_guessed_time_ = received_time;
          return true;
        } else {
          return false;
        }
      } else {  // we know what phase we're looking for
        // Deal with it if the above logic for tweaking the phase that we're
        // using wrapped it around.
        if (good_phase_ == -1) {
          good_phase_ = kPacketsPerLoopCycle;
        } else if (good_phase_ == kPacketsPerLoopCycle) {
          good_phase_ = 0;
        }
        assert(good_phase_ >= 0);
        assert(good_phase_ < kPacketsPerLoopCycle);

        if (received_phase == good_phase_) {
          if (offset < -kSwitchOffset) {
            ++good_phase_early_;
            good_phase_late_ = 0;
          } else if (offset > kSwitchOffset) {
            ++good_phase_late_;
            good_phase_early_ = 0;
          } else {
            good_phase_early_ = good_phase_late_ = 0;
          }
          return true;
        } else {
          return false;
        }
      }
    }

   private:
    // How many times the packet we guessed has to be close to right to use the
    // guess.
    static const int kMinGoodGuessCycles = 30;
    // How many times in a row we have to guess the wrong packet before trying
    // again.
    static const int kMaxBadGuessCycles = 3;

    // How many times in a row a different packet has to be better than the one
    // that we're using befor switching to it.
    static const int kSwitchCycles = 15;

    ::aos::time::Time last_guessed_time_{0, 0};

    const int kUnknownPhase = -11;
    // kUnknownPhase or the sequence number (%kPacketsPerLoopCycle) to
    // use or think about using.
    // If not kUnknownPhase, 0 <= these < kPacketsPerLoopCycle.
    int good_phase_, guess_phase_;
    int guess_phase_good_, guess_phase_bad_;
    ::aos::time::Time guess_phase_offset_{0, 0};
    int good_phase_early_, good_phase_late_;
  } phase_locker_;

  // Returns true if receiving failed and we should try a Reset().
  bool ReceiveData() {
    // Loop and then return once we get a good one.
    while (true) {
      using ::glibusb::UsbEndpoint;
      UsbEndpoint::IoStatus result =
          endpoint_->ReadAtMostWithTimeout(sizeof(GyroBoardData),
                                           kReadTimeout.ToMSec(),
                                           &buffer_);
      switch (result) {
        case UsbEndpoint::kSuccess:
          sequence_.Update(data()->sequence);
          return false;
        case UsbEndpoint::kTimeout:
          LOG(WARNING, "read timed out\n");
          return true;
        case UsbEndpoint::kNoDevice:
          LOG(ERROR, "no device\n");
          return true;
        case UsbEndpoint::kUnknown:
        case UsbEndpoint::kFail:
        case UsbEndpoint::kAbort:
          LOG(ERROR, "read failed\n");
          continue;
      }
    }
  }

  GyroBoardData *data() {
    return static_cast<GyroBoardData *>(
        buffer_.GetBufferPointer(sizeof(GyroBoardData)));
  }

  void Reset() {
    // Make sure to delete the endpoint before its device.
    endpoint_.reset();
    device_ = ::std::unique_ptr< ::glibusb::UsbDevice>(
        libusb_.FindSingleMatchingDeviceOrLose(kDeviceId));
    CHECK(device_);
    endpoint_ = ::std::unique_ptr< ::glibusb::UsbInEndpoint>(
        device_->InEndpoint(kEndpoint));
    sequence_.Reset();
    phase_locker_.Reset();
  }

  void ProcessData() {
    if (data()->robot_id != 0) {
      LOG(ERROR, "gyro board sent data for robot id %hhd!"
          " dip switches are %x\n",
          data()->robot_id, data()->base_status & 0xF);
      return;
    } else {
      LOG(DEBUG, "processing a packet dip switches %x\n",
          data()->base_status & 0xF);
    }

    static ::aos::time::Time last_time = ::aos::time::Time::Now();
    if ((last_time - ::aos::time::Time::Now()) >
        ::aos::time::Time::InMS(0.0011)) {
      LOG(INFO, "missed one\n");
    }

    gyro.MakeWithBuilder()
        .angle(data()->gyro_angle / 16.0 / 1000.0 / 180.0 * M_PI)
        .Send();

    drivetrain.position.MakeWithBuilder()
        .right_encoder(drivetrain_translate(data()->main.right_drive))
        .left_encoder(-drivetrain_translate(data()->main.left_drive))
        .Send();

    wrist.position.MakeWithBuilder()
        .pos(wrist_translate(data()->main.wrist))
        .hall_effect(!data()->main.wrist_hall_effect)
        .calibration(wrist_translate(data()->main.capture_wrist_rise))
        .Send();

    angle_adjust.position.MakeWithBuilder()
        .angle(angle_adjust_translate(data()->main.shooter_angle))
        .bottom_hall_effect(!data()->main.angle_adjust_bottom_hall_effect)
        .middle_hall_effect(false)
        .bottom_calibration(angle_adjust_translate(
                data()->main.capture_shooter_angle_rise))
        .middle_calibration(angle_adjust_translate(
                0))
        .Send();

    shooter.position.MakeWithBuilder()
        .position(shooter_translate(data()->main.shooter))
        .Send();

    index_loop.position.MakeWithBuilder()
        .index_position(index_translate(data()->main.indexer))
        .top_disc_detect(!data()->main.top_disc)
        .top_disc_posedge_count(top_rise_.Update(data()->main.top_rise_count))
        .top_disc_posedge_position(
            index_translate(data()->main.capture_top_rise))
        .top_disc_negedge_count(top_fall_.Update(data()->main.top_fall_count))
        .top_disc_negedge_position(
            index_translate(data()->main.capture_top_fall))
        .bottom_disc_detect(!data()->main.bottom_disc)
        .bottom_disc_posedge_count(
            bottom_rise_.Update(data()->main.bottom_rise_count))
        .bottom_disc_negedge_count(
            bottom_fall_.Update(data()->main.bottom_fall_count))
        .bottom_disc_negedge_wait_position(index_translate(
                data()->main.capture_bottom_fall_delay))
        .bottom_disc_negedge_wait_count(
            bottom_fall_delay_.Update(data()->main.bottom_fall_delay_count))
        .loader_top(data()->main.loader_top)
        .loader_bottom(data()->main.loader_bottom)
        .Send();
  }

  ::std::unique_ptr< ::glibusb::UsbDevice> device_;
  ::std::unique_ptr< ::glibusb::UsbInEndpoint> endpoint_;
  ::glibusb::Buffer buffer_;

  WrappingCounter sequence_;

  ::glibusb::Libusb libusb_;

  WrappingCounter top_rise_;
  WrappingCounter top_fall_;
  WrappingCounter bottom_rise_;
  WrappingCounter bottom_fall_delay_;
  WrappingCounter bottom_fall_;
};
constexpr ::glibusb::VendorProductId GyroSensorReceiver::kDeviceId;
constexpr ::aos::time::Time GyroSensorReceiver::kReadTimeout;
constexpr ::aos::time::Time GyroSensorReceiver::kDesiredOffset;

}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::GyroSensorReceiver receiver;
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
