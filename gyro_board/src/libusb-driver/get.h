#ifndef GET_H_
#define GET_H_

#include <sys/socket.h>
#include <linux/can.h>
#include <memory>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>

#include "thread.h"
#include "libusb_wrap.h"

// Use a packed version of can_frame.
// This allows for us to just transfer the data over the wire trivially.
struct packed_can_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t can_dlc; /* data length code: 0 .. 8 */
	uint8_t data[8];
} __attribute__((packed));

class DbgReader;

// Compare function for CAN IDs.
struct can_frame_priority_comparator
    : std::binary_function <struct can_frame,
                            struct can_frame, bool> {
  bool operator() (const struct can_frame& x,
                   const struct can_frame& y) const;
};

class GyroDriver {
 public:
  // Constructs a GyroDriver.
  // GyroDriver takes ownership of the device handle.
  explicit GyroDriver(LibUSBDeviceHandle *dev_handle);
  virtual ~GyroDriver();

  // Queues a CAN frame.  The driver will copy the frame to reduce memory churn
  // and non-realtime behavior associated with allocating and deallocating the
  // messages.
  // Returns true if the message was queued and false otherwise.
  bool QueueCanFrame(const struct can_frame &msg);

  // Returns 1 packet worth of debug data from the bulk debug channel.
  std::string GetDebugData();

  // Waits until a CAN frame is available and returns it.
  // Returns true if we got a frame and false if we were interrupted.
  bool GetCanFrame(struct can_frame *msg);

  // Terminates the rx and tx threads in preperation for shutdown.
  // Not safe for use in signal handlers.
  void Terminate();

 private:
  // Class that runs in a seperate thread and receives and queues all messages.
  class PacketReceiver;

  // USB Device handle.
  std::unique_ptr<LibUSBDeviceHandle> dev_handle_;
  // Handle for the object run in the debug thread which prints out debug
  // information.
  std::unique_ptr<DbgReader> dbg_;
  // Thread handle for the debug thread.
  std::unique_ptr<boost::thread> debug_thread_;

  // Handle for the transmit object which runs in the transmit thread and also
  // handles priority queueing of packets.
  //std::unique_ptr<PacketTransmitter> tx_;
  // Thread handle for the transmit thread.
  //std::unique_ptr<boost::thread> tx_thread_;

  // Handle for the rx object which runs in the rx thread and also
  // handles priority queueing of packets.
  std::unique_ptr<PacketReceiver> rx_;
  // Thread handle for the receive thread.
  std::unique_ptr<boost::thread> rx_thread_;
};

class DbgReader : public Thread {
 public:
  explicit DbgReader(GyroDriver *dev_handle);

  // Serve.
  void operator()();

 private:
  GyroDriver *gyro_;
};

#endif  // GET_H_
