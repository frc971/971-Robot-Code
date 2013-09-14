// Copyright 2012 Google Inc. All Rights Reserved.
// Author: charliehotel@google.com (Christopher Hoover)

#include "../glibusb.h"

#include <sstream>
#include <gtest/gtest.h>

namespace glibusb {
namespace testing {

// Tests that DeviceLocationAndId gets streamed correctly.
TEST(LibusbTest, LogsLocationAndID) {
  DeviceLocationAndId location_and_id;
  location_and_id.location.bus_number = 10;
  location_and_id.location.device_address = 20;
  location_and_id.id.vendor_id = 0x30;
  location_and_id.id.product_id = 0x40;

  std::stringstream out;
  out << location_and_id;
  EXPECT_EQ("010:020 0030:0040", out.str());
}

// Tests setup and teardown.
TEST(LibusbTest, SetupTeardown) {
  Libusb libusb;
}

// Tests SetDebug
TEST(LibusbTest, SetDebug) {
  Libusb libusb;
  libusb.SetDebug(1);
}

}  // namespace testing
}  // namespace glibusb
