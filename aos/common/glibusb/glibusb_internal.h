// Copyright 2012 Google Inc. All Rights Reserved.
//
// Internal conversions between libusb descriptors and enums.

#ifndef _GLIBUSB_GLIBUSB_INTERNAL_H_
#define _GLIBUSB_GLIBUSB_INTERNAL_H_

#include "glibusb_endpoint.h"

namespace glibusb {

// Converts libusb endpoint address to integer
int DescriptorToAddress(const struct libusb_endpoint_descriptor *descriptor);

// Converts libusb direction to UsbEndpoint direction.
UsbEndpoint::DirectionType DescriptorToDirection(
    const struct libusb_endpoint_descriptor *descriptor);

// Converts libusb transfer type to UsbEndpoint transfer type.
UsbEndpoint::TransferType DescriptorToTransfer(
    const struct libusb_endpoint_descriptor *descriptor);

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_INTERNAL_H_
