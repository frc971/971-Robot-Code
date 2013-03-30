#ifndef LIBUSB_H_
#define LIBUSB_H_

#include <libusb-1.0/libusb.h>

class LibUSBDeviceHandle;

class LibUSB {
 public:
  explicit LibUSB();
  virtual ~LibUSB();
  // Return a device handle or NULL with the correct VID and PID.
  LibUSBDeviceHandle *FindDeviceWithVIDPID(
      int vid, int pid);

 private:
  libusb_context *ctx_;
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
  // Takes ownership of the device handle and frees it when destructed.
  explicit LibUSBDeviceHandle(libusb_device_handle *dev_handle);
 libusb_device_handle *dev_handle_;
};

#endif  // LIBUSB_H_
