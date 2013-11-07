#ifndef FRC971_INPUT_USB_RECEIVER_H_
#define FRC971_INPUT_USB_RECEIVER_H_

#include <memory>

#include "aos/common/time.h"
#include "aos/common/macros.h"

#include "gyro_board/src/libusb-driver/libusb_wrap.h"
#include "frc971/input/gyro_board_data.h"

namespace frc971 {

// TODO(brians): Figure out how to deal with the kernel bunching packets up on
// us.
class USBReceiver {
 public:
  USBReceiver(uint8_t expected_robot_id);

  void RunIteration();

  // The relative priority that tasks doing this should get run at (ie what to
  // pass to ::aos::Init(int)).
  static const int kRelativePriority = 5;

 protected:
  GyroBoardData *data() { return &data_; }

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
      ::aos::time::Time::InSeconds(-0.003);

  // How long without a good packet until we give up and Reset().
  static constexpr ::aos::time::Time kResetTime =
      ::aos::time::Time::InSeconds(0.25);

  // Contains all of the complicated state and logic for locking onto the the
  // correct phase.
  class PhaseLocker {
   public:
    void Reset();

    // Gets called for every packet received.
    // Returns whether or not to process the values from this packet.
    bool IsCurrentPacketGood(const ::aos::time::Time &received_time,
                             uint32_t sequence);

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

    uint32_t last_good_sequence_;

    const int kUnknownPhase = -11;
    // kUnknownPhase or the sequence number (%kPacketsPerLoopCycle) to
    // use or think about using.
    // If not kUnknownPhase, 0 <= these < kPacketsPerLoopCycle.
    int good_phase_, guess_phase_;
    int guess_phase_good_, guess_phase_bad_;
    ::aos::time::Time guess_phase_offset_{0, 0};
    int good_phase_early_, good_phase_late_;
  } phase_locker_;

  static void StaticTransferCallback(libusb::Transfer *transfer, void *self);
  void TransferCallback(libusb::Transfer *transfer);

  // Returns true if receiving failed and we should try a Reset().
  bool ReceiveData();

  void Reset();

  // These 2 are the functions for subclasses to override and do stuff in.
  // timestamp for both of them is the time (as best as this code can determine)
  // that the values in the packet were captured.
  // They both have empty implementations here for subclasses that don't want to
  // do anything in one of them.

  // Gets called after each packet is received (possibly before ProcessData for
  // the same packet).
  virtual void PacketReceived(const ::aos::time::Time &timestamp);
  // Gets called every 10th packet (or so) (at the right time for data for
  // control loops to get read). PacketReceived will always be called right
  // before this.
  virtual void ProcessData(const ::aos::time::Time &timestamp);

  const uint8_t expected_robot_id_;

  GyroBoardData data_;

  int32_t last_frame_number_, frame_number_;

  LibUSB libusb_;
  ::std::unique_ptr<LibUSBDeviceHandle> dev_handle_;
  ::std::unique_ptr<libusb::IsochronousTransfer> transfers_[kNumTransfers];

  // "Temporary" variables for communicating information about a transfer that
  // finished from the callback to the rest of the code.
  libusb::Transfer *completed_transfer_;
  ::aos::time::Time transfer_received_time_{0, 0};

  DISALLOW_COPY_AND_ASSIGN(USBReceiver);
};

}  // namespace frc971

#endif  // FRC971_INPUT_USB_RECEIVER_H_
