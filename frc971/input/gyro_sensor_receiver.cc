#include <string.h>

#include <memory>

#include "aos/common/inttypes.h"
#include "aos/atom_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/wrapping_counter.h"
#include "aos/common/control_loop/ControlLoop.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/input/gyro_board_data.h"
#include "frc971/queues/GyroAngle.q.h"
#include "gyro_board/src/libusb-driver/libusb_wrap.h"

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
      }
    }
  }

 private:
  static const unsigned char kEndpoint = 0x83;
  // 0 is unlimited
  static constexpr ::aos::time::Time kReadTimeout =
      ::aos::time::Time::InSeconds(1.5);
  // vendor ID
  static const int32_t kVid = 0x1424;
  // product ID
  static const int32_t kPid = 0xd243;

  // A value to put into completed_transfer_ to indicate that it failed.
  static constexpr libusb::Transfer *kTransferFailed =
      reinterpret_cast<libusb::Transfer *>(-1);
  // The kernel on the fitpc seems to miss ~11-15 packets in a row if it misses
  // any with just 2, so 25 should be enough to ride over any holes.
  static const int kNumTransfers = 25;

  // How big of a buffer we're going to give the usb transfer stuff.
  static const size_t kDataLength = 128;
  static_assert(kDataLength >= sizeof(GyroBoardData), "buffer is too small");

  static const int kPacketsPerLoopCycle = 10;

  // How long "after" the control loops run we want to use a packet.
  static constexpr ::aos::time::Time kDesiredOffset =
      ::aos::time::Time::InSeconds(-0.0025);

  // How long without a good packet until we give up and Reset().
  static constexpr ::aos::time::Time kResetTime =
      ::aos::time::Time::InSeconds(0.75);
  // How old of a packet we log about.
  static constexpr ::aos::time::Time kStaleTime =
      ::aos::time::Time::InSeconds(0.005);

  // Contains all of the complicated state and logic for locking onto the the
  // correct phase.
  class {
   public:
    void Reset() {
      LOG(INFO, "resetting\n");
      last_good_packet_time_ = ::aos::time::Time(0, 0);
      good_phase_ = guess_phase_ = kUnknownPhase;
      guess_phase_good_ = guess_phase_bad_ = 0;
      good_phase_early_ = good_phase_late_ = 0;
    }

    // Gets called for every packet received.
    // Returns whether or not to process the values from this packet.
    bool IsCurrentPacketGood(const ::aos::time::Time &received_time,
                             int32_t sequence) {
      if (last_good_packet_time_ != ::aos::time::Time(0, 0) &&
          received_time - last_good_packet_time_ > kResetTime) {
        LOG(WARNING, "no good packet received in too long\n");
        Reset();
        return false;
      }

      using ::aos::control_loops::kLoopFrequency;
      // How often we (should) receive packets.
      static const ::aos::time::Time kPacketFrequency =
          kLoopFrequency / kPacketsPerLoopCycle;
      static const ::aos::time::Time kPacketClose =
          kPacketFrequency * 65 / 100;
      static const ::aos::time::Time kSwitchOffset =
          kPacketFrequency * 6 / 10;

      // When we want to receive a packet for the next cycle of control loops.
      ::aos::time::Time next_desired =
          ::aos::control_loops::NextLoopTime(received_time) + kDesiredOffset;
      // If we came up with something more than 1 packet in the past.
      if (next_desired - received_time < -kPacketFrequency) {
        next_desired += kLoopFrequency;
      }
      // How far off of when we want the next packet this one is.
      const ::aos::time::Time offset = next_desired - received_time;

      const int received_phase = sequence % kPacketsPerLoopCycle;

      assert(!(good_phase_early_ != 0 && good_phase_late_ != 0));

      if (good_phase_ == kUnknownPhase &&
          guess_phase_good_ > kMinGoodGuessCycles) {
        good_phase_ = guess_phase_;
        if (guess_phase_offset_ < kPacketFrequency / -2) {
          ++good_phase_;
        } else if (guess_phase_offset_ > kPacketFrequency / 2) {
          --good_phase_;
        }
        LOG(INFO, "locked on to phase %d\n", good_phase_);
      } else if (guess_phase_bad_ > kMaxBadGuessCycles) {
        LOG(INFO, "guessed wrong phase too many times\n");
        Reset();
      }
      if (good_phase_early_ > kSwitchCycles) {
        good_phase_early_ = 0;
        LOG(INFO, "switching from phase %d to %d-1\n",
            good_phase_, good_phase_);
        --good_phase_;
      } else if (good_phase_late_ > kSwitchCycles) {
        good_phase_late_ = 0;
        LOG(INFO, "switching from phase %d to %d+1\n",
            good_phase_, good_phase_);
        ++good_phase_;
      }
      if (good_phase_ == kUnknownPhase) {
        LOG(INFO, "guessing which packet is good\n");

        // If it's close to the right time.
        if (offset.abs() < kPacketClose) {
          if (guess_phase_ == kUnknownPhase) {
            if (offset.abs() < kPacketFrequency * 55 / 100) {
              guess_phase_ = received_phase;
              guess_phase_offset_ = offset;
            }
          } else if (received_phase == guess_phase_) {
            LOG(DEBUG, "guessed right phase %d\n", received_phase);
            ++guess_phase_good_;
            guess_phase_bad_ = 0;
            guess_phase_offset_ = (guess_phase_offset_ * 9 + offset) / 10;
          }
        } else if (guess_phase_ != kUnknownPhase &&
                   received_phase == guess_phase_) {
          LOG(DEBUG, "guessed wrong phase %d\n", received_phase);
          ++guess_phase_bad_;
          guess_phase_good_ = ::std::max(0, guess_phase_good_ -
                                         (kMinGoodGuessCycles / 10));
        }
        return false;
      } else {  // we know what phase we're looking for
        // Deal with it if the above logic for tweaking the phase that we're
        // using wrapped it around.
        if (good_phase_ == -1) {
          good_phase_ = kPacketsPerLoopCycle;
        } else if (good_phase_ == kPacketsPerLoopCycle) {
          LOG(DEBUG, "dewrapping\n");
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
          last_good_packet_time_ = received_time;

          if (::aos::time::Time::Now() - received_time > kStaleTime) {
            // TODO(brians): Do we actually want to use this one or what?
            LOG(WARNING, "received a stale packet\n");
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

    ::aos::time::Time last_good_packet_time_{0, 0};

    const int kUnknownPhase = -11;
    // kUnknownPhase or the sequence number (%kPacketsPerLoopCycle) to
    // use or think about using.
    // If not kUnknownPhase, 0 <= these < kPacketsPerLoopCycle.
    int good_phase_, guess_phase_;
    int guess_phase_good_, guess_phase_bad_;
    ::aos::time::Time guess_phase_offset_{0, 0};
    int good_phase_early_, good_phase_late_;
  } phase_locker_;

  static void StaticTransferCallback(libusb::Transfer *transfer, void *self) {
    static_cast<GyroSensorReceiver *>(self)->TransferCallback(transfer);
  }
  void TransferCallback(libusb::Transfer *transfer) {
    if (transfer->status() == LIBUSB_TRANSFER_COMPLETED) {
      LOG(DEBUG, "transfer %p completed\n", transfer);
      completed_transfer_ = transfer;
    } else if (transfer->status() == LIBUSB_TRANSFER_TIMED_OUT) {
      LOG(WARNING, "transfer %p timed out\n", transfer);
      completed_transfer_ = kTransferFailed;
    } else if (transfer->status() == LIBUSB_TRANSFER_CANCELLED) {
      LOG(DEBUG, "transfer %p cancelled\n", transfer);
    } else {
      LOG(FATAL, "transfer %p has status %d\n", transfer, transfer->status());
    }
    transfer->Submit();
  }

  // Returns true if receiving failed and we should try a Reset().
  bool ReceiveData() {
    // Loop and then return once we get a good one.
    while (true) {
      completed_transfer_ = NULL;
      while (completed_transfer_ == NULL) {
        libusb_.HandleEvents();
      }
      if (completed_transfer_ == kTransferFailed) {
        LOG(WARNING, "transfer failed\n");
        return true;
      }

      if (completed_transfer_->read_bytes() <
          static_cast<ssize_t>(sizeof(GyroBoardData))) {
        LOG(ERROR, "read %d bytes instead of at least %zd\n",
            completed_transfer_->read_bytes(), sizeof(GyroBoardData));
        continue;
      }

      memcpy(data(), completed_transfer_->data(),
             sizeof(GyroBoardData));

      int32_t count_before = sequence_.count();
      sequence_.Update(data()->sequence);
      if (sequence_.count() - count_before != 1) {
        LOG(WARNING, "count went from %" PRId32" to %" PRId32 "\n",
            count_before, sequence_.count());
      }

      return false;
    }
  }

  GyroBoardData *data() {
    return &data_;
  }

  void Reset() {
    typedef ::std::unique_ptr<libusb::IsochronousTransfer> TransferType;
    for (TransferType &c : transfers_) {
      c.reset();
    }
    dev_handle_ = ::std::unique_ptr<LibUSBDeviceHandle>(
        libusb_.FindDeviceWithVIDPID(kVid, kPid));
    if (!dev_handle_) {
      LOG(ERROR, "couldn't find device. exiting\n");
      exit(1);
    }
    for (TransferType &c : transfers_) {
      c.reset(new libusb::IsochronousTransfer(kDataLength, 1,
                                              StaticTransferCallback, this));
      c->FillIsochronous(dev_handle_.get(), kEndpoint, kReadTimeout);
      c->Submit();
    }

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

  GyroBoardData data_;

  WrappingCounter sequence_;

  LibUSB libusb_;
  ::std::unique_ptr<LibUSBDeviceHandle> dev_handle_;
  ::std::unique_ptr<libusb::IsochronousTransfer> transfers_[kNumTransfers];
  // Temporary variable for holding a completed transfer to communicate that
  // information from the callback to the code that wants it.
  libusb::Transfer *completed_transfer_;

  WrappingCounter top_rise_;
  WrappingCounter top_fall_;
  WrappingCounter bottom_rise_;
  WrappingCounter bottom_fall_delay_;
  WrappingCounter bottom_fall_;
};
constexpr ::aos::time::Time GyroSensorReceiver::kReadTimeout;
constexpr ::aos::time::Time GyroSensorReceiver::kDesiredOffset;
constexpr ::aos::time::Time GyroSensorReceiver::kResetTime;
constexpr ::aos::time::Time GyroSensorReceiver::kStaleTime;

}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::GyroSensorReceiver receiver;
  while (true) {
    receiver.RunIteration();
  }
  ::aos::Cleanup();
}
