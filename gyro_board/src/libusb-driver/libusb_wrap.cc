#include <libusb.h>
#include <google/gflags.h>
#include <glog/logging.h>
#include <iostream>

#include "libusb_wrap.h"

LibUSB::LibUSB() {
  CHECK_GE(libusb_init(&ctx_), 0) << ": LibUSB failed to initialize.";
  libusb_set_debug(ctx_, 3);
}

LibUSBDeviceHandle *LibUSB::FindDeviceWithVIDPID(int vid, int pid) {
  int r;
  libusb_device **devs;
  libusb_device_handle *dev_handle;

  ssize_t cnt;
  cnt = libusb_get_device_list(ctx_, &devs);
  if(cnt < 0) {
    LOG(ERROR) << "Get Device Error";
    return NULL;
  }
  LOG(INFO) << cnt << " Devices in list.";
  bool found = false;
  for (int i = 0; i < cnt; ++i) {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(devs[i], &desc);
    CHECK_GE(r, 0) << ": Couldn't get device descriptor, error " << r;
    if (desc.idVendor == vid && desc.idProduct == pid) {
      LOG(INFO) << "Device " << (int)libusb_get_bus_number(devs[i]) << ":"
                << (int)libusb_get_device_address(devs[i]) << " matches";
      r = libusb_open(devs[i], &dev_handle);
      if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        LOG(INFO) << "Device already in use, trying next one.";
        continue;
      }
      if (r < 0) {
        LOG(WARNING) << "Failed to open device.";
      } else {
        found = true;
        break;
      }
    }
  }
  libusb_free_device_list(devs, 1);
  if (!found) {
    LOG(ERROR) << "Couldn't open device.";
    return NULL;
  }

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    LOG(INFO) << "Kernel Driver Active";
    if (libusb_detach_kernel_driver(dev_handle, 0) == 0) {
      LOG(INFO) << "Kernel Driver Detached!";
    } else {
      LOG(ERROR) << "Couldn't detach kernel driver.";
      return NULL;
    }
  }

  r = libusb_claim_interface(dev_handle, 0);
  if (r < 0) {
    LOG(ERROR) << "Cannot Claim Interface";
    return 0;
  }
  LOG(INFO) << "Claimed Interface";
  return new LibUSBDeviceHandle(dev_handle);
}

LibUSB::~LibUSB() {
  libusb_exit(ctx_);
}

LibUSBDeviceHandle::LibUSBDeviceHandle(
    libusb_device_handle *dev_handle) : dev_handle_(dev_handle) { }

LibUSBDeviceHandle::~LibUSBDeviceHandle() {
  int r;
  r = libusb_release_interface(dev_handle_, 0);
  if (r != 0) {
    LOG(FATAL) << "Cannot Release Interface";
  }
  LOG(INFO) << "Released Interface";

  libusb_close(dev_handle_);
}

int LibUSBDeviceHandle::interrupt_transfer(
    unsigned char endpoint, unsigned char *data, int length,
    int *transferred, unsigned int timeout) {
  return libusb_interrupt_transfer(dev_handle_, endpoint, data, length,
                                   transferred, timeout);
}

int LibUSBDeviceHandle::bulk_transfer(
    unsigned char endpoint, unsigned char *data,
    int length, int *transferred, unsigned int timeout) {
  return libusb_bulk_transfer(dev_handle_, endpoint, data, length,
                              transferred, timeout);
}
