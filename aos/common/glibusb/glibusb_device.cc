// Copyright 2012 Google Inc. All Rights Reserved.

#include <stddef.h>
#include <glog/logging.h>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <libusb.h>

#include "glibusb.h"
#include "glibusb_device_internal.h"
#include "glibusb_endpoint.h"
#include "glibusb_endpoint_internal.h"
#include "glibusb_internal.h"

namespace glibusb {

namespace {
VendorProductId GetVendorAndProductIdInternal(
  struct libusb_device_handle *device_handle) {
  CHECK_NOTNULL(device_handle);
  libusb_device_descriptor desc;
  libusb_device *dev = libusb_get_device(device_handle);
  libusb_get_device_descriptor(dev, &desc);
  return VendorProductId(desc.idVendor, desc.idProduct);
}
}  // namespace

PhysicalUsbDevice::PhysicalUsbDevice(struct libusb_context *context,
                                     struct libusb_device_handle *handle)
    : UsbDevice(GetVendorAndProductIdInternal(handle)),
      libusb_context_(CHECK_NOTNULL(context)),
      device_handle_(CHECK_NOTNULL(handle)) {
  int r = libusb_claim_interface(device_handle_, 0);
  // TODO(charliehotel): this must not be FATAL.
  CHECK_GE(r, 0) << ": libusb_claim_interface failed, r=" << std::dec << r;

  struct libusb_device *dev = libusb_get_device(device_handle_);
  // TODO(charliehotel): this must not be FATAL.
  CHECK(dev != NULL) << ": libusb_get_device failed";

  struct libusb_device_descriptor desc;
  r = libusb_get_device_descriptor(dev, &desc);
  // TODO(charliehotel): this must not be FATAL.
  CHECK_GE(r, 0) << ": libusb_get_device_descriptor failed";

  VLOG(2) << "vid=0x" << std::hex << desc.idVendor
	  << ", pid=0x" << desc.idProduct;
  VLOG(2) << "  # of configurations = "
	  << static_cast<int>(desc.bNumConfigurations);

  struct libusb_config_descriptor *config;
  r = libusb_get_active_config_descriptor(dev, &config);
  // TODO(charliehotel): this must not be FATAL.
  CHECK_GE(r, 0) << ": libusb_get_active_config_descriptor failed";
  // TODO(charliehotel): this must not be FATAL.
  CHECK_NOTNULL(config);

  if (config->bNumInterfaces != 1) {
    VLOG(2) << "config->bNumInterfaces="
	    << static_cast<int>(config->bNumInterfaces)
	    << ", expected ony one";
  }

  if (VLOG_IS_ON(2)) {
    for (int i = 0; i < config->bNumInterfaces; ++i) {
      const struct libusb_interface *interface = config->interface + i;
      const struct libusb_interface_descriptor *setting = interface->altsetting;

      VLOG(2) << "bInterfaceNumber="
              << static_cast<int>(setting->bInterfaceNumber);
      VLOG(2) << "bAlternateSetting="
              << static_cast<int>(setting->bAlternateSetting);
      VLOG(2) << "bNumEndpoints="
              << static_cast<int>(setting->bNumEndpoints);

      for (int j = 0; j < setting->bNumEndpoints; ++j) {
        const struct libusb_endpoint_descriptor *endpoint =
            setting->endpoint + j;
        switch (endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) {
          case LIBUSB_TRANSFER_TYPE_CONTROL:
            VLOG(2) << "control";
            break;
          case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
            VLOG(2) << "iso";
            break;
          case LIBUSB_TRANSFER_TYPE_BULK:
            VLOG(2) << "bulk";
            break;
          case LIBUSB_TRANSFER_TYPE_INTERRUPT:
            VLOG(2) << "interrupt";
            break;
          default:
            LOG(FATAL) << "unknown transfer type";
        }
        if ((endpoint->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
            LIBUSB_ENDPOINT_IN) {
          VLOG(2) << " ep: 0x"
                  << std::hex << static_cast<int>(endpoint->bEndpointAddress)
                  << " (in)";
        } else {
          VLOG(2) << " ep: 0x"
                  << std::hex << static_cast<int>(endpoint->bEndpointAddress)
                  << " (out)";
        }
        VLOG(2) << "   packet size="
                << std::dec << static_cast<int>(endpoint->wMaxPacketSize);
        VLOG(2) << "   interval="
                << std::dec << static_cast<int>(endpoint->bInterval);
      }
    }
  }
  libusb_free_config_descriptor(config);
}

PhysicalUsbDevice::~PhysicalUsbDevice() {
  CHECK_NOTNULL(device_handle_);
  libusb_close(device_handle_);
  device_handle_ = nullptr;
}

bool PhysicalUsbDevice::DoSetAlternateSetting(int setting) {
  CHECK_NOTNULL(device_handle_);
  int r = libusb_set_interface_alt_setting(device_handle_, 0, setting);
  return r == 0;
}

template <class UsbEndpointType>
UsbEndpointType *PhysicalUsbDevice::MatchEndpoint(EndpointMatcher matcher) {
  CHECK_NOTNULL(device_handle_);
  struct libusb_config_descriptor *config;
  libusb_device *dev = libusb_get_device(device_handle_);
  const int r = libusb_get_active_config_descriptor(dev, &config);
  // TODO(charliehotel): this must not be FATAL.
  CHECK_GE(r, 0) << ": libusb_get_active_config_descriptor failed";
  // TODO(charliehotel): this must not be FATAL.
  CHECK_NOTNULL(config);
  const struct libusb_interface *interface = config->interface;
  const struct libusb_interface_descriptor *setting = interface->altsetting;
  for (int j = 0; j < setting->bNumEndpoints; ++j) {
    const struct libusb_endpoint_descriptor *descriptor = setting->endpoint + j;
    if (matcher(descriptor)) {
      UsbEndpointType *ans = new UsbEndpointType(
          libusb_context_, device_handle_, descriptor);
      libusb_free_config_descriptor(config);
      return ans;
    }
  }
  libusb_free_config_descriptor(config);
  return NULL;
}

namespace {

struct DescriptorHasAddressAndDirection {
  DescriptorHasAddressAndDirection(int address, UsbEndpoint::DirectionType direction)
    : address_(address), direction_(direction) {}

  // Returns true if the descriptor provided has an address equal to the number
  bool operator()(const struct libusb_endpoint_descriptor *descriptor) {
    return (DescriptorToAddress(descriptor) == address_ &&
	    DescriptorToDirection(descriptor) == direction_);
  }

  int address_;
  UsbEndpoint::DirectionType direction_;
};

// Returns true if the descriptor has a transfer type and direction equal to the
// provided type and direction.
struct DescriptorIsOfTypeAndDirection {
  DescriptorIsOfTypeAndDirection(UsbEndpoint::TransferType transfer_type,
				 UsbEndpoint::DirectionType direction)
    : transfer_type_(transfer_type), direction_(direction) {}

  bool operator()(const struct libusb_endpoint_descriptor *descriptor) {
    return (DescriptorToTransfer(descriptor) == transfer_type_ &&
	    DescriptorToDirection(descriptor) == direction_);
  }

  UsbEndpoint::TransferType transfer_type_;
  UsbEndpoint::DirectionType direction_;
};

}  // namespace

UsbInEndpoint *PhysicalUsbDevice::DoInEndpoint(int number) {
  CHECK_EQ(number & LIBUSB_ENDPOINT_ADDRESS_MASK, number)
      << ": Endpoint out of range.";

  DescriptorHasAddressAndDirection matcher(number, UsbEndpoint::kIn);
  return MatchEndpoint<PhysicalUsbInEndpoint>(matcher);
}

UsbOutEndpoint *PhysicalUsbDevice::DoOutEndpoint(int number) {
  CHECK_EQ(number & LIBUSB_ENDPOINT_ADDRESS_MASK, number)
      << ": Endpoint out of range.";

  DescriptorHasAddressAndDirection matcher(number, UsbEndpoint::kOut);
  return MatchEndpoint<PhysicalUsbOutEndpoint>(matcher);
}

UsbInEndpoint *PhysicalUsbDevice::DoFindInEndpoint(
    UsbEndpoint::TransferType endpoint) {

  DescriptorIsOfTypeAndDirection matcher(endpoint, UsbEndpoint::kIn);
  return MatchEndpoint<PhysicalUsbInEndpoint>(matcher);
}

UsbOutEndpoint *PhysicalUsbDevice::DoFindOutEndpoint(
    UsbEndpoint::TransferType endpoint) {

  DescriptorIsOfTypeAndDirection matcher(endpoint, UsbEndpoint::kOut);
  return MatchEndpoint<PhysicalUsbOutEndpoint>(matcher);
}

struct DeviceLocationAndId PhysicalUsbDevice::DoDeviceLocationAndId() {
  CHECK_NOTNULL(device_handle_);
  libusb_device *device = ::libusb_get_device(device_handle_);
  CHECK_NOTNULL(device);
  struct DeviceLocationAndId dlid;
  dlid.location.bus_number = ::libusb_get_bus_number(device);
  dlid.location.device_address = ::libusb_get_device_address(device);
  dlid.id = vendor_product_id_;
  return dlid;
}

}  // namespace glibusb
