// Copyright 2012 Google Inc. All Rights Reserved.
//
// Usb Endpoint Interfaces.

#ifndef _GLIBUSB_GLIBUSB_ENDPOINT_H_
#define _GLIBUSB_GLIBUSB_ENDPOINT_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <libusb.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include "gbuffer.h"

#ifndef MAX_ISO_BUFFER_LENGTH
#define MAX_ISO_BUFFER_LENGTH 32768
#endif

#ifndef MAX_BULK_BUFFER_LENGTH
#define MAX_BULK_BUFFER_LENGTH 16384
#endif

#ifndef MAX_CTRL_BUFFER_LENGTH
#define MAX_CTRL_BUFFER_LENGTH 4096
#endif

namespace glibusb {

// Provides a base-class for all endpoints.
class Buffer;

class Notification {
 public:
  explicit Notification(bool prenotify = false);
  bool HasBeenNotified() const;
  void WaitForNotification() const;
  bool WaitForNotificationWithTimeout(int64_t milliseconds) const;
  void Notify();

 private:
  bool HasBeenNotifiedUnlocked() const;

  mutable boost::mutex mutex_;
  mutable boost::condition notified_changed_;
  bool notified_;

  Notification(const Notification &) = delete;
  void operator=(const Notification &) = delete;
};


class UsbEndpoint {
 public:
  // The values for Direction are defined by the USB spec and must not be
  // modified.
  enum DirectionType { kIn = 0x80, kOut = 0x00 };
  enum TransferType { kControl, kBulk, kInterrupt, kIsochronous };
  enum IoStatus { kSuccess, kFail, kTimeout, kAbort, kNoDevice, kUnknown };

  // The max transfer (in bytes) that the kernel supports.
  static const int32_t kMaxBulkTransferBytes = MAX_BULK_BUFFER_LENGTH;

  virtual ~UsbEndpoint() {}

  // Returns the endpoint number.
  int endpoint_address() const {
    return endpoint_address_and_direction_ & LIBUSB_ENDPOINT_ADDRESS_MASK;
  }

  // Returns the direction.
  DirectionType direction() const { return direction_; }

  // Returns the transfer type.
  TransferType transfer() const { return transfer_; }

  // Returns the wMaxPacketSize value for this endpoint in the
  // active device configuration.
  int GetMaxPacketSize() { return DoGetMaxPacketSize(); }

  // Returns the maximum packet size for this endpoint that
  // can be sent or received during one microframe.
  int GetMaxIsoPacketSize() { return DoGetMaxIsoPacketSize(); }

 protected:
  // Constructs an endpoint with the provided tranfser type and address.
  UsbEndpoint(DirectionType direction, TransferType transfer,
              int endpoint_address);

  // Returns the address that libusb uses to refer to the endpoint.  This
  // includes the direction as the highest bit.
  int endpoint_address_and_direction() const {
    return endpoint_address_and_direction_;
  }

 private:
  virtual int DoGetMaxPacketSize() = 0;
  virtual int DoGetMaxIsoPacketSize() = 0;

  // Endpoint type and direction.
  DirectionType direction_;
  TransferType transfer_;

  // Endpoint address.
  int endpoint_address_and_direction_;

  UsbEndpoint(const UsbEndpoint &) = delete;
  void operator=(const UsbEndpoint &) = delete;
};

// Provides an interface to allow reading from a USB endpoint.
class UsbInEndpoint : public UsbEndpoint {
 public:
  // Constructs an endpoint with the provided transfer type and address.
  UsbInEndpoint(TransferType transfer, int endpoint_address);
  virtual ~UsbInEndpoint() {}

  // Reads into the buffer from the endpoint until the transfer limit is
  // reached, or a zero length packet/non full packet is received.
  // Returns true on success.  Waits forever.
  bool Read(Buffer *out);

  // Reads into the buffer from the endpoint until the transfer limit is
  // reached, or a zero length packet/non full packet is received.
  IoStatus ReadWithTimeout(int32_t timeout_milliseconds, Buffer *out);

  // Reads into the buffer from the endpoint until the length is
  // reached, or a zero length packet/non full packet is received.
  // Returns true on success.
  bool ReadAtMost(uint32_t length, Buffer *out);

  // Reads into the buffer from the endpoint until the length is
  // reached, or a zero length packet/non full packet is received.
  IoStatus ReadAtMostWithTimeout(
      uint32_t length, int32_t timeout_milliseconds, Buffer *out);

  // Reads into the buffer from the endpoint until the length is
  // reached, or a zero length packet/non full packet is received.
  // Cancels the request when the notification is notified.
  IoStatus ReadAtMostWithTimeoutAndNotification(
      uint32_t length, int32_t timeout_milliseconds, Buffer *out,
      Notification *quit);

 private:
  // Actually executes the read, with the length, timeout, buffer, and
  // notification.
  virtual IoStatus DoRead(
      uint32_t length, int32_t timeout_milliseconds, Buffer *out,
      Notification *quit) = 0;

  UsbInEndpoint(const UsbInEndpoint &) = delete;
  void operator=(const UsbInEndpoint &) = delete;
};

// Provides an interface to allow writing to a USB endpoint.
class UsbOutEndpoint : public UsbEndpoint {
 public:
  // Constructs an endpoint with the provided tranfser type and address.
  UsbOutEndpoint(TransferType transfer, int endpoint_address);
  virtual ~UsbOutEndpoint() {}

  // Writes the buffer to the endpoint.  Returns true on success.
  bool Write(const Buffer &buffer);

  // Writes the buffer to the endpoint with timeout.
  IoStatus WriteWithTimeout(const Buffer &buffer, int timeout_milliseconds);

 private:
  // Implements the actual write.
  virtual IoStatus DoWrite(const Buffer &buffer,
			   int32_t timeout_milliseconds) = 0;

  UsbOutEndpoint(const UsbOutEndpoint &) = delete;
  void operator=(const UsbOutEndpoint &) = delete;
};

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_ENDPOINT_H_
