// Copyright 2012 Google Inc. All Rights Reserved.
//
// Modified by FRC Team 971.

#include <stddef.h>
#include <inttypes.h>
#include <libusb-1.0/libusb.h>

#include "aos/common/logging/logging.h"
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
  if (r < 0) {
    // TODO(charliehotel): this must not be FATAL.
    LOG(FATAL, "libusb_claim_interface failed with %d: %s\n",
        r, libusb_error_name(r));
  }

  struct libusb_device *dev = libusb_get_device(device_handle_);
  if (dev == NULL) {
    // TODO(charliehotel): this must not be FATAL.
    LOG(FATAL, "libusb_get_device failed\n");
  }

  struct libusb_device_descriptor desc;
  r = libusb_get_device_descriptor(dev, &desc);
  if (r < 0) {
    // TODO(charliehotel): this must not be FATAL.
    LOG(FATAL, "libusb_get_device_descriptor failed with %d: %s\n",
        r, libusb_error_name(r));
  }

  LOG(DEBUG, "vid=0x%" PRIx16 ", pid=0x%" PRIx16 ", # of configurations = %d\n",
      desc.idVendor, desc.idProduct, static_cast<int>(desc.bNumConfigurations));

  struct libusb_config_descriptor *config;
  r = libusb_get_active_config_descriptor(dev, &config);
  if (r < 0) {
    // TODO(charliehotel): this must not be FATAL.
    LOG(FATAL, "libusb_get_active_config_descriptor failed with %d: %s\n",
        r, libusb_error_name(r));
  }
  // TODO(charliehotel): this must not be FATAL.
  CHECK_NOTNULL(config);

  if (config->bNumInterfaces != 1) {
    LOG(DEBUG, "config->bNumInterfaces=%d, expected only one\n",
        static_cast<int>(config->bNumInterfaces));
  }

  // TODO(brians): Make this enableable through the logging stuff.
  if (false) {
    for (int i = 0; i < config->bNumInterfaces; ++i) {
      const struct libusb_interface *interface = config->interface + i;
      const struct libusb_interface_descriptor *setting = interface->altsetting;

      LOG(DEBUG, "bInterfaceNumber=%d bAlternateSetting=%d bNumEndpoints=%d\n",
          static_cast<int>(setting->bInterfaceNumber),
          static_cast<int>(setting->bAlternateSetting),
          static_cast<int>(setting->bNumEndpoints));

      for (int j = 0; j < setting->bNumEndpoints; ++j) {
        const struct libusb_endpoint_descriptor *endpoint =
            setting->endpoint + j;
        switch (endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) {
          case LIBUSB_TRANSFER_TYPE_CONTROL:
            LOG(DEBUG, "control\n");
            break;
          case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
            LOG(DEBUG, "iso\n");
            break;
          case LIBUSB_TRANSFER_TYPE_BULK:
            LOG(DEBUG, "bulk\n");
            break;
          case LIBUSB_TRANSFER_TYPE_INTERRUPT:
            LOG(DEBUG, "interrupt\n");
            break;
          default:
            LOG(FATAL, "unknown transfer type\n");
        }
        if ((endpoint->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
            LIBUSB_ENDPOINT_IN) {
          LOG(DEBUG, " ep: 0x%x (in)\n",
              static_cast<int>(endpoint->bEndpointAddress));
        } else {
          LOG(DEBUG, " ep: 0x%x (out)\n",
              static_cast<int>(endpoint->bEndpointAddress));
        }
        LOG(DEBUG, "  packet size=%d interval=%d\n",
            static_cast<int>(endpoint->wMaxPacketSize),
            static_cast<int>(endpoint->bInterval));
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
  if (r < 0) {
    // TODO(charliehotel): this must not be FATAL.
    LOG(FATAL, "libusb_get_active_config_descriptor failed with %d: %s\n",
        r, libusb_error_name(r));
  }
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
  if ((number & LIBUSB_ENDPOINT_ADDRESS_MASK) != number) {
    LOG(FATAL, "Endpoint %d out of range.\n", number);
  }

  DescriptorHasAddressAndDirection matcher(number, UsbEndpoint::kIn);
  return MatchEndpoint<PhysicalUsbInEndpoint>(matcher);
}

UsbOutEndpoint *PhysicalUsbDevice::DoOutEndpoint(int number) {
  if ((number & LIBUSB_ENDPOINT_ADDRESS_MASK) != number) {
    LOG(FATAL, "Endpoint %d out of range.\n", number);
  }

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
