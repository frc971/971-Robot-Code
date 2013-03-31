#ifndef LIBUSB_H_
#define LIBUSB_H_

#include <libusb-1.0/libusb.h>

#include "aos/common/macros.h"

class LibUSBDeviceHandle;
namespace libusb {
class Transfer;
}

class LibUSB {
 public:
  explicit LibUSB();
  ~LibUSB();

  // Return a device handle or NULL with the correct VID and PID.
  LibUSBDeviceHandle *FindDeviceWithVIDPID(
      int vid, int pid);

  void HandleEvents();

 private:
  libusb_context *ctx_;

  DISALLOW_COPY_AND_ASSIGN(LibUSB);
};

class LibUSBDeviceHandle {
 public:
  virtual ~LibUSBDeviceHandle();
  // Transfers data using an interrupt transfer.
  int interrupt_transfer(unsigned char endpoint, unsigned char *data,
                         int length, int *transferred, unsigned int timeout);

  // Transfers data using a bulk transfer.
  int bulk_transfer(unsigned char endpoint, unsigned char *data,
                    int length, int *transferred, unsigned int timeout);

 private:
  friend class LibUSB; // For constructor
  friend class libusb::Transfer;  // for access to dev_handle_
  // Takes ownership of the device handle and frees it when destructed.
  explicit LibUSBDeviceHandle(libusb_device_handle *dev_handle);
  libusb_device_handle *dev_handle_;

 DISALLOW_COPY_AND_ASSIGN(LibUSBDeviceHandle);
};

// TODO(brians): move everything in here
namespace libusb {

// Wraps a libusb_transfer*.
// Represents an asynchronous transfer.
// Only designed to support interrupt and bulk transfers because they are very
// similar and simple.
class Transfer {
 public:
  Transfer(size_t data_length,
           void (*callback)(Transfer *, void *),
           void *user_data);
  ~Transfer();

  void FillInterrupt(LibUSBDeviceHandle *device,
                     unsigned char endpoint,
                     unsigned int timeout);

  void Submit();
  void Cancel();

  libusb_transfer_status status() { return transfer_->status; }
  int read_bytes() { return transfer_->actual_length; }

  const uint8_t *data() { return data_; }

 private:
  static void StaticTransferCallback(libusb_transfer *self) {
    static_cast<Transfer *>(self->user_data)->TransferCallback();
  }
  void TransferCallback();

  libusb_transfer *const transfer_;

  uint8_t *const data_;
  size_t data_length_;

  void (*const callback_)(Transfer *, void *);
  void *const user_data_;

  DISALLOW_COPY_AND_ASSIGN(Transfer);
};

}  // namespace libusb

#endif  // LIBUSB_H_
