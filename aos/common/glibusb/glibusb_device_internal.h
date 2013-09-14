// Copyright 2012 Google Inc. All Rights Reserved.
//
// Modified by FRC Team 971.
//
// Wrapper for libusb's device that implements the UsbDevice interface.

#ifndef _GLIBUSB_GLIBUSB_DEVICE_INTERNAL_H_
#define _GLIBUSB_GLIBUSB_DEVICE_INTERNAL_H_

#include <stdint.h>
#include <utility>
#include <functional>

#include "glibusb.h"
#include "glibusb_endpoint.h"

namespace glibusb {

// Provides an interface to an individual USB device.
class PhysicalUsbDevice : public UsbDevice {
 public:
  virtual ~PhysicalUsbDevice();

 private:
  friend class Libusb;  // For private constructor.
  // Constructs a device given the context and handle.
  // Frees the handle on destruction.
  PhysicalUsbDevice(struct libusb_context *context,
                    struct libusb_device_handle *handle);

  typedef ::std::function<bool(const struct libusb_endpoint_descriptor *)>
    EndpointMatcher;

  // Iterates through all the endpoint descriptors for this device
  // and allocates and returns a UsbEndpointType for the first
  // endpoint for which the matcher returns true or NULL.
  template <class UsbEndpointType>
    UsbEndpointType *MatchEndpoint(EndpointMatcher matcher);

  virtual bool DoSetAlternateSetting(int setting);
  virtual UsbInEndpoint *DoFindInEndpoint(UsbEndpoint::TransferType endpoint);
  virtual UsbOutEndpoint *DoFindOutEndpoint(UsbEndpoint::TransferType endpoint);
  virtual UsbInEndpoint *DoInEndpoint(int number);
  virtual UsbOutEndpoint *DoOutEndpoint(int number);
  virtual struct DeviceLocationAndId DoDeviceLocationAndId();

  // Libusb context and handle used to interact with libusb.
  struct libusb_context *libusb_context_;
  struct libusb_device_handle *device_handle_;

  PhysicalUsbDevice(const PhysicalUsbDevice &) = delete;
  void operator=(const PhysicalUsbDevice &) = delete;
};

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_DEVICE_INTERNAL_H_
