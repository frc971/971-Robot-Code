#include "libusb_wrap.h"

#include <string.h>

#include <iostream>

#include "aos/common/logging/logging.h"

LibUSB::LibUSB() {
  if (libusb_init(&ctx_) < 0) {
    LOG(FATAL, "libusb_init(%p) failed\n", &ctx_);
  }
  libusb_set_debug(ctx_, 3);
}

LibUSBDeviceHandle *LibUSB::FindDeviceWithVIDPID(int vid, int pid) {
  int r;
  libusb_device **devs;
  libusb_device_handle *dev_handle;

  ssize_t cnt;
  cnt = libusb_get_device_list(ctx_, &devs);
  if (cnt < 0) {
    LOG(ERROR, "Get Device Error\n");
    return NULL;
  }
  LOG(INFO, "%d Devices in list.\n", cnt);
  bool found = false;
  for (int i = 0; i < cnt; ++i) {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(devs[i], &desc);
    if (r < 0) {
      LOG(FATAL, "lib_usb_get_device_descriptor(%p, %p) failed\n",
          devs[i], &desc);
    }
    if (desc.idVendor == vid && desc.idProduct == pid) {
      LOG(INFO, "Device %d:%d matches\n",
          (int)libusb_get_bus_number(devs[i]),
          (int)libusb_get_device_address(devs[i]));
      r = libusb_open(devs[i], &dev_handle);
      if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
        LOG(INFO, "Device already in use, trying next one.\n");
        continue;
      }
      if (r < 0) {
        LOG(WARNING, "Failed to open device.\n");
      } else {
        found = true;
        break;
      }
    }
  }
  libusb_free_device_list(devs, 1);
  if (!found) {
    LOG(ERROR, "Couldn't open device.\n");
    return NULL;
  }

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    LOG(INFO, "Kernel Driver Active\n");
    if (libusb_detach_kernel_driver(dev_handle, 0) == 0) {
      LOG(INFO, "Kernel Driver Detached!\n");
    } else {
      LOG(ERROR, "Couldn't detach kernel driver.\n");
      return NULL;
    }
  }

  r = libusb_claim_interface(dev_handle, 0);
  if (r < 0) {
    LOG(ERROR, "Cannot Claim Interface\n");
    return 0;
  }
  LOG(INFO, "Claimed Interface\n");
  return new LibUSBDeviceHandle(dev_handle);
}

LibUSB::~LibUSB() {
  libusb_exit(ctx_);
}

void LibUSB::HandleEvents() {
  int ret = libusb_handle_events(ctx_);
  if (ret != 0) {
    LOG(FATAL, "libusb_handle_events(%p) returned %d\n", ctx_, ret);
  }
}

LibUSBDeviceHandle::LibUSBDeviceHandle(
    libusb_device_handle *dev_handle) : dev_handle_(dev_handle) { }

LibUSBDeviceHandle::~LibUSBDeviceHandle() {
  int r = libusb_release_interface(dev_handle_, 0);
  if (r != 0) {
    LOG(FATAL, "Cannot Release Interface\n");
  }
  LOG(INFO, "Released Interface\n");

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

namespace libusb {

Transfer::Transfer(size_t data_length,
                   void (*callback)(Transfer *, void *),
                   void *user_data,
                   int num_iso_packets)
    : transfer_(libusb_alloc_transfer(num_iso_packets)),
      data_(new uint8_t[data_length]),
      data_length_(data_length),
      callback_(callback),
      user_data_(user_data) {
}
Transfer::~Transfer() {
  libusb_free_transfer(transfer_);
  delete data_;
}

void Transfer::FillInterrupt(LibUSBDeviceHandle *device,
                             unsigned char endpoint,
                             unsigned int timeout) {
  libusb_fill_interrupt_transfer(transfer_,
                                 device->dev_handle_,
                                 endpoint,
                                 data_,
                                 data_length_,
                                 StaticTransferCallback,
                                 this,
                                 timeout);
}

void Transfer::Submit() {
  int ret = libusb_submit_transfer(transfer_);
  if (ret != 0) {
    if (ret == LIBUSB_ERROR_BUSY) {
      LOG(FATAL, "transfer %p already submitted\n", this);
    }
    LOG(FATAL, "libusb error %d submitting transfer %p. errno %d: %s\n",
        ret, this, errno, strerror(errno));
  }
}

void Transfer::Cancel() {
  int ret = libusb_cancel_transfer(transfer_);
  if (ret != 0) {
    LOG(FATAL, "libusb error %d cancelling transfer %p. errno %d: %s\n",
        ret, this, errno, strerror(errno));
  }
}

void Transfer::TransferCallback() {
  callback_(this, user_data_);
}

IsochronousTransfer::IsochronousTransfer(size_t packet_length,
                                         int num_packets,
                                         void (*callback)(Transfer *, void *),
                                         void *user_data)
    : Transfer(packet_length * num_packets, callback, user_data, num_packets),
      num_packets_(num_packets) {
}

void IsochronousTransfer::FillIsochronous(LibUSBDeviceHandle *device,
                               unsigned char endpoint,
                               const ::aos::time::Time &timeout) {
  libusb_fill_iso_transfer(transfer_,
                           device->dev_handle_,
                           endpoint,
                           data_,
                           data_length_,
                           num_packets_,
                           StaticTransferCallback,
                           this,
                           timeout.ToMSec());
  transfer_->iso_packet_desc[0].length = data_length_;
}

}  // namespace libusb
