// Copyright 2012 Google Inc. All Rights Reserved.

#include "glibusb_internal.h"

#include <libusb.h>

#include "glog/logging.h"
#include "glibusb_endpoint.h"

namespace glibusb {

// Converts libusb endpoint address to integer
int DescriptorToAddress(const struct libusb_endpoint_descriptor *descriptor) {
  return descriptor->bEndpointAddress & LIBUSB_ENDPOINT_ADDRESS_MASK;
}

// Converts libusb direction to UsbEndpoint direction.
UsbEndpoint::DirectionType DescriptorToDirection(
    const struct libusb_endpoint_descriptor *descriptor) {
  if ((descriptor->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
      LIBUSB_ENDPOINT_IN) {
    return UsbEndpoint::kIn;
  } else {
    return UsbEndpoint::kOut;
  }
}

// Converts libusb transfer type to UsbEndpoint transfer type.
UsbEndpoint::TransferType DescriptorToTransfer(
    const struct libusb_endpoint_descriptor *descriptor) {
  switch (descriptor->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) {
    case LIBUSB_TRANSFER_TYPE_CONTROL:
      return UsbEndpoint::kControl;
      break;
    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
      return UsbEndpoint::kIsochronous;
      break;
    case LIBUSB_TRANSFER_TYPE_BULK:
      return UsbEndpoint::kBulk;
      break;
    case LIBUSB_TRANSFER_TYPE_INTERRUPT:
      return UsbEndpoint::kInterrupt;
      break;
    default:
      LOG(FATAL) << "bogus transfer type";
  }
}

}  // namespace glibusb
