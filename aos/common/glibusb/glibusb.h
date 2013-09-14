// Copyright 2012 Google Inc. All Rights Reserved.
//
// Wrapper for libusb.

#ifndef _GLIBUSB_GLIBUSB_H_
#define _GLIBUSB_GLIBUSB_H_

#include <stdint.h>
#include <iosfwd>
#include <string>
#include <vector>

#include "glibusb_endpoint.h"

extern "C" {
struct libusb_context;
}

namespace glibusb {

// Structure to hold the physical location on the USB bus of the device.
struct DeviceLocation {
  DeviceLocation() : bus_number(0), device_address(0) {}
  DeviceLocation(uint8_t new_bus_number, uint8_t new_device_address) 
   : bus_number(new_bus_number), device_address(new_device_address) {}

  bool operator==(const struct DeviceLocation &rhs) const {
    return ((bus_number == rhs.bus_number) &&
	    (device_address == rhs.device_address));
  }

  std::string ToString() const;

  uint8_t bus_number;
  uint8_t device_address;
};

// Returns a string representing the DeviceLocation.
std::string DeviceLocationToString(const DeviceLocation &location);
std::ostream &operator <<(std::ostream &out,
			  const DeviceLocation &location);


// Structure to hold the USB vendor and product ids for a device.
struct VendorProductId {
  VendorProductId() : vendor_id(0), product_id(0) {}
  VendorProductId(uint16_t new_vendor_id, uint16_t new_product_id)
      : vendor_id(new_vendor_id), product_id(new_product_id) {}

  bool operator==(const struct VendorProductId &rhs) const {
    return ((vendor_id == rhs.vendor_id) &&
	    (product_id == rhs.product_id));
  }

  std::string ToString() const;

  uint16_t vendor_id;
  uint16_t product_id;
};

// Returns a string representing the VendorProductId.
std::string VendorProductIdToString(const VendorProductId &vendor_product_id);

// Structure to hold the location and id of a device.  This is enough to
// uniquely identify the device redundantly.
struct DeviceLocationAndId {
  DeviceLocationAndId() {}
  DeviceLocationAndId(const DeviceLocation &new_location,
		      const VendorProductId &new_id)
    : location(new_location), id(new_id)
  {}

  bool operator==(const struct DeviceLocationAndId &rhs) const {
    return ((location == rhs.location) &&
	    (id == rhs.id));
  }

  std::string ToString() const;

  DeviceLocation location;
  VendorProductId id;
};

// Returns a string representing the DeviceLocation and provides a stream
// operator for logging.
std::string DeviceLocationAndIdToString(const DeviceLocationAndId &location_and_id);
std::ostream &operator <<(std::ostream &out,
			  const DeviceLocationAndId &location_and_id);



// Provides an interface to an individual USB device.
class UsbDevice {
 public:
  explicit UsbDevice(VendorProductId vendor_product_id)
      : vendor_product_id_(vendor_product_id) {}
  virtual ~UsbDevice() {}

  // Activates an alternate setting; returns true on success.
  bool SetAlternateSetting(int setting) {
    return DoSetAlternateSetting(setting);
  }

  // Returns the first endpoint to match the direction and transfer types.
  // Caller is responsible for freeing the endpoint.
  UsbInEndpoint *FindInEndpoint(UsbEndpoint::TransferType endpoint) {
    return DoFindInEndpoint(endpoint);
  }
  UsbOutEndpoint *FindOutEndpoint(UsbEndpoint::TransferType endpoint) {
    return DoFindOutEndpoint(endpoint);
  }

  // Returns the endpoint at the specified address.
  // Caller is responsible for freeing the endpoint.
  // Virtual for testing.
  UsbInEndpoint *InEndpoint(int number) { return DoInEndpoint(number); }
  UsbOutEndpoint *OutEndpoint(int number) { return DoOutEndpoint(number); }

  // Returns the vendor and product ids.
  VendorProductId GetVendorAndProductId() { return vendor_product_id_; }

  struct DeviceLocationAndId Id() { return DoDeviceLocationAndId(); }

 protected:
  VendorProductId vendor_product_id_;

 private:
  friend class Libusb;  // For private constructor.

  virtual bool DoSetAlternateSetting(int setting) = 0;
  virtual UsbInEndpoint *DoFindInEndpoint(
      UsbEndpoint::TransferType endpoint) = 0;
  virtual UsbOutEndpoint *DoFindOutEndpoint(
      UsbEndpoint::TransferType endpoint) = 0;
  virtual UsbInEndpoint *DoInEndpoint(int number) = 0;
  virtual UsbOutEndpoint *DoOutEndpoint(int number) = 0;
  virtual struct DeviceLocationAndId DoDeviceLocationAndId() = 0;

  UsbDevice(const UsbDevice &) = delete;
  void operator=(const UsbDevice &) = delete;
};


// Provides RAII-style libusb initialization.
class Libusb {
 public:
  Libusb();
  ~Libusb();

  // Sets the debug level.
  void SetDebug(int level);

  // Returns the locations, vendor ids, and product ids of all attached devices.
  void FindDeviceLocationAndId(std::vector<DeviceLocationAndId> *result);

  // Finds and returns exactly one device with the matching vendor and
  // product ID or CHECKs.
  UsbDevice *FindSingleMatchingDeviceOrLose(const VendorProductId &id);
  // Does the same,  but returns NULL on failure rather than CHECKing.
  UsbDevice *FindSingleMatchingDevice(const VendorProductId &id);

  // Finds and returns exactly one device with the matching vendor and
  // product ID, at the specified bus number and device address, or CHECKs.
  UsbDevice *FindSingleMatchingDeviceAtLocationOrLose(
      const DeviceLocationAndId &dev_location_id);
  // Does the same, but returns NULL on failure rather than CHECKing.
  UsbDevice *FindSingleMatchingDeviceAtLocation(
      const DeviceLocationAndId &dev_location_id);

  // Finds exactly one device that matches whatever parameters were specified,
  // or CHECKs.
  //
  // Args:
  //   target_vendor_product_id: a vendor/product ID pair to search for,
  //       or empty string.
  //   target_device_location: a device location to search for, or empty string.
  //   [out] dev_location_id: The location and Ids of the chosen device.
  void FindDeviceBySpecification(
      const std::string &target_vendor_product_id,
      const std::string &target_device_location,
      DeviceLocationAndId *dev_location_id);

  // Finds exactly one device matching the specified parameters or checks.
  //
  // Args:
  //   target_ids: a list of product and vendor ids that match.
  //   target_device_location: a string with the device location, or an empty
  //       string to search all locations.
  //   [out] dev_location_id: The location and Ids of the chosen device.
  void FindSingleDeviceMatchingTargetId(
      const VendorProductId &target_id,
      const std::string &target_device_location,
      DeviceLocationAndId *dev_location_id);

  // Finds exactly one device matching the specified parameters or checks.
  //
  // Args:
  //   target_ids: a list of product and vendor ids that match.
  //   target_device_location: a string with the device location, or an empty
  //       string to search all locations.
  //   [out] dev_location_id: The location and Ids of the chosen device.
  void FindSingleDeviceMatchingTargetIds(
      const std::vector<VendorProductId> &target_ids,
      const std::string &target_device_location,
      DeviceLocationAndId *dev_location_id);

  // Parses a vendor id and product id string, and appends the result on
  // target_ids.
  // The string must be in the form VVVV:PPPP, where VVVV is a 4-digit hex
  // vendor id and PPPP is a 4-digit hex product id.
  static void ParseProductVendorString(const std::string &target_vendor_product_id,
                                       std::vector<VendorProductId> *target_ids);

  static void ParseDeviceLocationString(const std::string &target_device_location,
                                        DeviceLocation *location);

  // TODO(charliehotel): add richer ways to retrieve device handles.

 private:
  // Libusb handle
  struct libusb_context *libusb_context_;

  Libusb(const Libusb &) = delete;
  void operator=(const Libusb &) = delete;
};

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_H_
