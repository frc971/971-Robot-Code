#include <string.h>
#include <errno.h>
#include <inttypes.h>

#include "frc971/input/usb_receiver.h"

#include "aos/common/logging/logging.h"
#include "aos/common/control_loop/ControlLoop.h"

namespace frc971 {

USBReceiver::USBReceiver(uint8_t expected_robot_id)
    : expected_robot_id_(expected_robot_id) {
  Reset();
}

void USBReceiver::RunIteration() {
  if (ReceiveData()) {
    Reset();
  } else {
    static const int kCountsPerSecond = 100000;
    const ::aos::time::Time timestamp =
        ::aos::time::Time(data()->timestamp / kCountsPerSecond,
                          (data()->timestamp * ::aos::time::Time::kNSecInSec /
                           kCountsPerSecond) %
                          ::aos::time::Time::kNSecInSec);

    if (data()->robot_id != expected_robot_id_) {
      LOG(ERROR, "gyro board sent data for robot id %hhd instead of %hhd!"
          " dip switches are %hhx\n",
          data()->robot_id, expected_robot_id_, data()->dip_switches);
      return;
    }
    if (data()->checksum != GYRO_BOARD_DATA_CHECKSUM) {
      LOG(ERROR,
          "gyro board sent checksum %" PRIu16 " instead of %" PRIu16 "!\n",
          data()->checksum, GYRO_BOARD_DATA_CHECKSUM);
      return;
    }

    PacketReceived(timestamp);

    if (phase_locker_.IsCurrentPacketGood(transfer_received_time_, frame_number_)) {
      LOG(DEBUG, "processing dips %hhx frame %" PRId32 " at %f\n",
          data()->dip_switches, data()->frame_number, timestamp.ToSeconds());

      ProcessData(timestamp);
    }
  }
}

void USBReceiver::PhaseLocker::Reset() {
  LOG(INFO, "resetting\n");
  last_good_packet_time_ = ::aos::time::Time(0, 0);
  last_good_sequence_ = 0;
  good_phase_ = guess_phase_ = kUnknownPhase;
  guess_phase_good_ = guess_phase_bad_ = 0;
  good_phase_early_ = good_phase_late_ = 0;
}

void USBReceiver::PacketReceived(const ::aos::time::Time &/*timestamp*/) {}
void USBReceiver::ProcessData(const ::aos::time::Time &/*timestamp*/) {}

bool USBReceiver::PhaseLocker::IsCurrentPacketGood(
    const ::aos::time::Time &received_time,
    uint32_t sequence) {
  if (last_good_packet_time_ != ::aos::time::Time(0, 0) &&
      received_time - last_good_packet_time_ > kResetTime) {
    LOG(WARNING, "no good packet received in too long\n");
    Reset();
    return false;
  }
  if (last_good_sequence_ != 0 && sequence - last_good_sequence_ > 100) {
    LOG(WARNING, "skipped too many packets\n");
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
    LOG(DEBUG, "guessing which packet is good\n");

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
      last_good_sequence_ = sequence;

      return true;
    } else {
      return false;
    }
  }
}

void USBReceiver::StaticTransferCallback(libusb::Transfer *transfer,
                                         void *self) {
  static_cast<USBReceiver *>(self)->TransferCallback(transfer);
}

void USBReceiver::TransferCallback(libusb::Transfer *transfer) {
  transfer_received_time_ = ::aos::time::Time::Now();
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

bool USBReceiver::ReceiveData() {
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

    if (data()->unknown_frame) {
      LOG(WARNING, "unknown frame number\n");
      return true;
    }
    uint32_t frame_number_before = frame_number_;
    frame_number_ = data()->frame_number;
    if (frame_number_ < 0) {
      LOG(WARNING, "negative frame number %" PRId32 "\n", frame_number_);
      return true;
    }
    if (frame_number_before == 0) {
      LOG(INFO, "frame number starting at %" PRId32 "\n", frame_number_);
    } else if (frame_number_ - frame_number_before != 1) {
      LOG(WARNING, "frame number went from %" PRId32" to %" PRId32 "\n",
          frame_number_before, frame_number_);
    }
    if (frame_number_ < last_frame_number_) {
      LOG(WARNING, "frame number went down\n");
      return true;
    }
    last_frame_number_ = frame_number_;

    return false;
  }
}

void USBReceiver::Reset() {
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

  last_frame_number_ = frame_number_ = 0;
  phase_locker_.Reset();
}

constexpr ::aos::time::Time USBReceiver::kReadTimeout;
constexpr ::aos::time::Time USBReceiver::kDesiredOffset;
constexpr ::aos::time::Time USBReceiver::kResetTime;

}  // namespace frc971
