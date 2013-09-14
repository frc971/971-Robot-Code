// Copyright 2012 Google Inc. All Rights Reserved.
//
// Wrapper for libusb's endpoints.

#ifndef _GLIBUSB_GLIBUSB_ENDPOINT_INTERNAL_H_
#define _GLIBUSB_GLIBUSB_ENDPOINT_INTERNAL_H_

#include <stddef.h>
#include <stdint.h>
#include <libusb.h>

#include "glibusb_endpoint.h"

class Notification;

namespace glibusb {

class Buffer;


// Provides an interface to allow reading from a USB endpoint.
class PhysicalUsbInEndpoint : public UsbInEndpoint {
 public:
  virtual ~PhysicalUsbInEndpoint();

 private:
  friend class PhysicalUsbDevice;  // For constructor
  // Constructs an endpoint given the context, handle, and a descriptor of the
  // endpoint.  The context and handle must remain valid throughout the
  // lifetime of this object.
  PhysicalUsbInEndpoint(struct libusb_context *context,
                        struct libusb_device_handle *handle,
                        const struct libusb_endpoint_descriptor *descriptor);

  virtual int DoGetMaxPacketSize();
  virtual int DoGetMaxIsoPacketSize();

  // Actually executes the read, with the length, timeout, buffer, and
  // notification.
  virtual IoStatus DoRead(
      uint32_t length, int32_t timeout_milliseconds, Buffer *out,
      Notification *quit);

  // Libusb handles and endpoint information.
  struct libusb_context *libusb_context_;
  struct libusb_device_handle *handle_;

  PhysicalUsbInEndpoint(const PhysicalUsbInEndpoint &) = delete;
  void operator=(const PhysicalUsbInEndpoint &) = delete;
};

// Provides an interface to allow writing to a USB endpoint.
class PhysicalUsbOutEndpoint : public UsbOutEndpoint {
 public:
  virtual ~PhysicalUsbOutEndpoint();

 private:
  friend class PhysicalUsbDevice;  // For constructor
  // Constructs an endpoint given the context, handle, and a descriptor of the
  // endpoint.  The context and handle must remain valid throughout the
  // lifetime of this object.
  PhysicalUsbOutEndpoint(struct libusb_context *context,
                         struct libusb_device_handle *handle,
                         const struct libusb_endpoint_descriptor *descriptor);

  virtual int DoGetMaxPacketSize();
  virtual int DoGetMaxIsoPacketSize();

  // Implements the actual write.
  virtual IoStatus DoWrite(const Buffer &buffer,
			   int32_t timeout_milliseconds);

  // Libusb handles and endpoint information.
  struct libusb_context *libusb_context_;
  struct libusb_device_handle *handle_;

  PhysicalUsbOutEndpoint(const PhysicalUsbOutEndpoint &) = delete;
  void operator=(const PhysicalUsbOutEndpoint &) = delete;
};

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_ENDPOINT_INTERNAL_H_
