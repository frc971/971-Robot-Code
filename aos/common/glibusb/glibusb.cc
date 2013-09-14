// Copyright 2012 Google Inc. All Rights Reserved.

#include "glibusb.h"

#include <cstdio>
#include <sstream>
#include <iomanip>
#include <string>
#include <glog/logging.h>
#include <libusb.h>

#include "glibusb_device_internal.h"

namespace glibusb {

namespace {
bool safe_strtou32(const std::string &s, uint32_t *value) {
  CHECK_NOTNULL(value);
  std::stringstream stream(s);
  stream >> std::dec;
  return stream >> *value;
}

bool safe_strtou32_hex(const std::string &s, uint32_t *value) {
  CHECK_NOTNULL(value);
  std::stringstream stream(s);
  stream >> std::hex;
  return stream >> *value;
}
}

std::string DeviceLocation::ToString() const 
{
  return DeviceLocationToString(*this);
}

std::string DeviceLocationToString(const DeviceLocation &location) {
  std::stringstream stream;
  stream 
    << std::dec << std::setw(3) << std::setfill('0')
    << static_cast<int>(location.bus_number)
    << ":"
    << std::dec << std::setw(3) << std::setfill('0')
    << static_cast<int>(location.device_address);
  return stream.str();
}

std::ostream &operator <<(std::ostream &out,
			  const DeviceLocation &location) {
  out << DeviceLocationToString(location);
  return out;
}

std::string VendorProductIdToString(const VendorProductId &vendor_product_id) {
  std::stringstream stream;
  stream 
    << std::hex << std::setw(4) << std::setfill('0')
    << static_cast<int>(vendor_product_id.vendor_id)
    << ":"
    << std::hex << std::setw(4) << std::setfill('0')
    << static_cast<int>(vendor_product_id.product_id);
  return stream.str();
}

std::string DeviceLocationAndId::ToString() const 
{
  return DeviceLocationAndIdToString(*this);
}

std::string DeviceLocationAndIdToString(const DeviceLocationAndId &location_and_id) {
  return DeviceLocationToString(location_and_id.location) + " " +
      VendorProductIdToString(location_and_id.id);
}

std::ostream &operator <<(std::ostream &out,
			  const DeviceLocationAndId &location_and_id) {
  out << DeviceLocationAndIdToString(location_and_id);
  return out;
}

//////////////////////////////////////////////////////////////////////

Libusb::Libusb() {
  CHECK_EQ(libusb_init(&libusb_context_), 0);
}

Libusb::~Libusb() {
  libusb_exit(libusb_context_);
}

void Libusb::SetDebug(int level) {
  libusb_set_debug(libusb_context_, level);
}

void Libusb::FindDeviceLocationAndId(std::vector<DeviceLocationAndId> *result) {
  CHECK_NOTNULL(result);
  struct libusb_device **devices_head;
  CHECK_GE(libusb_get_device_list(libusb_context_, &devices_head), 0);
  CHECK_NOTNULL(devices_head);

  for (struct libusb_device **devices = devices_head;
       *devices != NULL; ++devices) {
    struct libusb_device_descriptor descriptor;
    CHECK_GE(libusb_get_device_descriptor(*devices, &descriptor), 0);
    VLOG(2) << "idVendor = 0x" << std::hex << descriptor.idVendor
            << " idProduct = 0x" << std::hex << descriptor.idProduct;
    DeviceLocationAndId dev_location_id;
    dev_location_id.location.bus_number = libusb_get_bus_number(*devices);
    dev_location_id.location.device_address =
        libusb_get_device_address(*devices);
    dev_location_id.id.vendor_id = descriptor.idVendor;
    dev_location_id.id.product_id = descriptor.idProduct;
    result->push_back(dev_location_id);
  }
  libusb_free_device_list(devices_head, /*unref_devices=*/ 1);
}

// Find a single device that matches the vendor_id and product_id, and
// optionally bus_number and device_address.  CHECK if more than one device is
// found or no devices are found.
//
// This is the implementation behind FindSingleMatchingDeviceAtLocationOrLose()
// and FindSingleMatchingDeviceOrLose().
static libusb_device_handle *FindSingleDevice(
    struct libusb_context *context,
    bool match_location,
    const DeviceLocationAndId &dev_location_id) {
  struct libusb_device **devices_head;
  CHECK_GE(libusb_get_device_list(context, &devices_head), 0);
  CHECK_NOTNULL(devices_head);

  struct libusb_device *matching_device = NULL;
  for (struct libusb_device **devices = devices_head;
       *devices != NULL; ++devices) {
    if (match_location) {
      uint8_t device_bus_number = libusb_get_bus_number(*devices);
      uint8_t device_device_address = libusb_get_device_address(*devices);
      if (device_bus_number != dev_location_id.location.bus_number ||
          device_device_address != dev_location_id.location.device_address)
        continue;
    }

    struct libusb_device_descriptor descriptor;
    CHECK_GE(libusb_get_device_descriptor(*devices, &descriptor), 0);
    VLOG(2) << "idVendor = 0x" << std::hex << descriptor.idVendor
            << " idProduct = 0x" << std::hex << descriptor.idProduct;
    if (descriptor.idVendor == dev_location_id.id.vendor_id &&
        descriptor.idProduct == dev_location_id.id.product_id) {
      CHECK(matching_device == NULL) << ": found multiple matching devices";
      matching_device = *devices;
    }
  }
  if (match_location) {
    int bus_number = static_cast<int>(dev_location_id.location.bus_number);
    int device_address =
        static_cast<int>(dev_location_id.location.device_address);
    CHECK(matching_device != NULL)
        << ": no matching device found for "
        << "vid=" << std::hex << dev_location_id.id.vendor_id << ", "
        << "pid=" << std::hex << dev_location_id.id.product_id << ", "
        << "bus_number=" << std::dec << bus_number << ", "
        << "device_address=" << std::dec << device_address;
  } else {
    const int vendor_id = dev_location_id.id.vendor_id;
    const int product_id = dev_location_id.id.product_id;
    CHECK(matching_device != NULL) << ": no matching device found for "
                                   << "vid=" << std::hex << vendor_id << ", "
                                   << "pid=" << std::hex << product_id;
  }

  struct libusb_device_handle *handle = NULL;
  if (matching_device != NULL) {
    int return_value = libusb_open(matching_device, &handle);
    if (return_value < 0) {
      // TODO(charliehotel): this must not be FATAL.
      LOG(FATAL) << "Failed to open device: "
                 << libusb_error_name(return_value);
    }
    CHECK_NOTNULL(handle);           // should never happen
  }
  libusb_free_device_list(devices_head, /*unref_devices=*/ 1);
  return handle;
}

UsbDevice *Libusb::FindSingleMatchingDeviceAtLocationOrLose(
    const DeviceLocationAndId &dev_location_id) {
  return CHECK_NOTNULL(FindSingleMatchingDeviceAtLocation(dev_location_id));
}

UsbDevice *Libusb::FindSingleMatchingDeviceAtLocation(
    const DeviceLocationAndId &dev_location_id) {
  auto handle = FindSingleDevice(libusb_context_,
				 /* match_location= */ true,
				 dev_location_id);
  if (handle == NULL) {
    return NULL;
  } else {
    return new PhysicalUsbDevice(libusb_context_, handle);
  }
}

UsbDevice *Libusb::FindSingleMatchingDeviceOrLose(
    const VendorProductId &id) {
  return CHECK_NOTNULL(FindSingleMatchingDeviceOrLose(id));
}

UsbDevice *Libusb::FindSingleMatchingDevice(
    const VendorProductId &id) {
  DeviceLocationAndId dev_location_id;
  dev_location_id.id = id;
  auto handle = FindSingleDevice(libusb_context_,
				 /* match_location= */ false,
				 dev_location_id);
  if (handle == NULL) {
    return NULL;
  } else {
    return new PhysicalUsbDevice(libusb_context_, handle);
  }
}

void Libusb::FindDeviceBySpecification(
    const std::string &target_vendor_product_id,
    const std::string &target_device_location,
    DeviceLocationAndId *dev_location_id) {
  std::vector<VendorProductId> target_ids;
  ParseProductVendorString(target_vendor_product_id, &target_ids);
  FindSingleDeviceMatchingTargetIds(target_ids, target_device_location,
                                    dev_location_id);
}

/*static*/ void Libusb::ParseProductVendorString(
    const std::string &target_vendor_product_id,
    std::vector<VendorProductId> *target_ids) {
  CHECK_NE(target_vendor_product_id, "");
  CHECK_EQ(target_vendor_product_id.size(), 9);
  CHECK_EQ(target_vendor_product_id[4], ':');
  uint32_t vendor_id;
  CHECK(safe_strtou32_hex(
      target_vendor_product_id.substr(0, 4), &vendor_id));
  uint32_t product_id;
  CHECK(safe_strtou32_hex(
      target_vendor_product_id.substr(5, 4), &product_id));
  VendorProductId temp;
  temp.vendor_id = vendor_id;
  temp.product_id = product_id;
  target_ids->push_back(temp);
}

/*static*/ void Libusb::ParseDeviceLocationString(
    const std::string &target_device_location, DeviceLocation *location) {
  CHECK_EQ(target_device_location.size(), 7);
  CHECK_EQ(target_device_location[3], ':');
  uint32_t parsed_bus_number;
  CHECK(safe_strtou32(
      target_device_location.substr(0, 3), &parsed_bus_number));
  uint32_t parsed_device_address;
  CHECK(safe_strtou32(
      target_device_location.substr(4, 3), &parsed_device_address));
  location->bus_number = parsed_bus_number;
  location->device_address = parsed_device_address;
}

/*static*/ void Libusb::FindSingleDeviceMatchingTargetId(
    const VendorProductId &target_id,
    const std::string &target_device_location,
    DeviceLocationAndId *dev_location_id) {
  std::vector<VendorProductId> target_ids;
  target_ids.push_back(target_id);
  FindSingleDeviceMatchingTargetIds(
      target_ids, target_device_location, dev_location_id);
}

void Libusb::FindSingleDeviceMatchingTargetIds(
    const std::vector<VendorProductId> &target_ids,
    const std::string &target_device_location,
    DeviceLocationAndId *result_dev_location_id) {

  bool have_target_device_location = (target_device_location != "");
  DeviceLocation location;
  if (have_target_device_location) {
    ParseDeviceLocationString(target_device_location, &location);
  }

  // Get the location and vendor/product IDs for all attached devices.
  std::vector<DeviceLocationAndId> dev_location_ids;
  FindDeviceLocationAndId(&dev_location_ids);

  // Filter the list by target parameters.  Make sure that exactly one device
  // is found.
  bool found_exactly_one_device = false;
  for (const auto &dev_location_id : dev_location_ids) {
    if (have_target_device_location) {
      if (dev_location_id.location.bus_number != location.bus_number ||
          dev_location_id.location.device_address != location.device_address)
        continue;
    }

    bool found_matching_product_vendor_id = false;
    for (const auto &target : target_ids) {
      if (target.vendor_id == dev_location_id.id.vendor_id &&
          target.product_id == dev_location_id.id.product_id) {
        found_matching_product_vendor_id = true;
        break;
      }
    }
    if (!found_matching_product_vendor_id)
      continue;

    CHECK(!found_exactly_one_device);
    found_exactly_one_device = true;
    *result_dev_location_id = dev_location_id;
  }
  CHECK(found_exactly_one_device);
}

}  // namespace glibusb
