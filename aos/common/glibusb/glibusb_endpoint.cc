// Copyright 2012 Google Inc. All Rights Reserved.

#include "glibusb_endpoint.h"

#include <stddef.h>
#include <glog/logging.h>
#include <boost/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "gbuffer.h"
#include "glibusb_endpoint_internal.h"
#include "glibusb_internal.h"
#include "glibusb_transfer.h"

namespace glibusb {

namespace {
int LibusbGetMaxPacketSize(libusb_device_handle *handle,
		     unsigned char endpoint) 
{
  libusb_device *device = CHECK_NOTNULL(libusb_get_device(handle));
  return libusb_get_max_packet_size(device, endpoint);
}

int LibusbGetMaxIsoPacketSize(libusb_device_handle *handle,
		     unsigned char endpoint) 
{
  libusb_device *device = CHECK_NOTNULL(libusb_get_device(handle));
  return libusb_get_max_iso_packet_size(device, endpoint);
}
}  // namespace

Notification::Notification(bool prenotify)
  : notified_(prenotify) {}

bool Notification::HasBeenNotified() const {
  boost::lock_guard<boost::mutex> lock(mutex_);
  return notified_;
}

void Notification::WaitForNotification() const {
  boost::unique_lock<boost::mutex> lock(mutex_);
  notified_changed_.wait(
      lock, 
      boost::bind(&Notification::HasBeenNotifiedUnlocked, this));
}

bool Notification::WaitForNotificationWithTimeout(int64_t milliseconds) const {
  boost::unique_lock<boost::mutex> lock(mutex_);
  auto timeout = boost::posix_time::milliseconds(milliseconds);
  notified_changed_.timed_wait(
      lock, timeout,
      boost::bind(&Notification::HasBeenNotifiedUnlocked, this));
  return notified_;
}

void Notification::Notify() {
  boost::lock_guard<boost::mutex> lock(mutex_);
  CHECK(!notified_) << ": already notified";
  notified_ = true;
  notified_changed_.notify_all();
}

bool Notification::HasBeenNotifiedUnlocked() const {
  return notified_;
}

////////////////////////////////////////////////////////////////////////

const int32_t UsbEndpoint::kMaxBulkTransferBytes;

UsbEndpoint::UsbEndpoint(DirectionType direction, TransferType transfer,
			 int endpoint_address)
    : direction_(direction),
      transfer_(transfer),
      endpoint_address_and_direction_(endpoint_address | direction) {
  CHECK_EQ(endpoint_address_and_direction_ & 0x80, direction)
      << ": Direction in address doesn't match specified direction.";
  CHECK_EQ(endpoint_address_and_direction_ & 0x8f,
           endpoint_address_and_direction_)
      << ": Invalid endpoint address.";
}

////////////////////////////////////////////////////////////////////////

UsbInEndpoint::UsbInEndpoint(TransferType transfer, int endpoint_address)
    : UsbEndpoint(UsbEndpoint::kIn, transfer, endpoint_address) {
}

bool UsbInEndpoint::Read(Buffer *out) {
  IoStatus status = ReadAtMostWithTimeout(kMaxBulkTransferBytes, 0, out);
  return status == kSuccess;
}

UsbEndpoint::IoStatus
UsbInEndpoint::ReadWithTimeout(int32_t timeout_milliseconds,
			       Buffer *out) {
  return ReadAtMostWithTimeout(kMaxBulkTransferBytes,
                               timeout_milliseconds, out);
}

bool UsbInEndpoint::ReadAtMost(uint32_t length, Buffer *out) {
  IoStatus status = ReadAtMostWithTimeout(length, 0, out);
  return status == kSuccess;
}

UsbEndpoint::IoStatus UsbInEndpoint::ReadAtMostWithTimeout(
    uint32_t length, int32_t timeout_milliseconds, Buffer *out) {
  return DoRead(length, timeout_milliseconds, out, NULL);
}

UsbEndpoint::IoStatus UsbInEndpoint::ReadAtMostWithTimeoutAndNotification(
    uint32_t length, int32_t timeout_milliseconds, Buffer *out,
    Notification *quit) {
  CHECK_NOTNULL(quit);
  return DoRead(length, timeout_milliseconds, out, quit);
}

////////////////////////////////////////////////////////////////////////

UsbOutEndpoint::UsbOutEndpoint(TransferType transfer, int endpoint_address)
    : UsbEndpoint(UsbEndpoint::kOut, transfer, endpoint_address) {
}

bool UsbOutEndpoint::Write(const Buffer &buffer) {
  IoStatus status = DoWrite(buffer, 0);
  return status == kSuccess;
}

UsbEndpoint::IoStatus UsbOutEndpoint::WriteWithTimeout(const Buffer &buffer,
				      int32_t timeout_milliseconds) {
  return DoWrite(buffer, timeout_milliseconds);
}

////////////////////////////////////////////////////////////////////////

PhysicalUsbInEndpoint::PhysicalUsbInEndpoint(
    struct libusb_context *context,
    struct libusb_device_handle *handle,
    const struct libusb_endpoint_descriptor *descriptor)
    : UsbInEndpoint(DescriptorToTransfer(descriptor),
                    DescriptorToAddress(descriptor)),
      libusb_context_(CHECK_NOTNULL(context)),
      handle_(CHECK_NOTNULL(handle)) {
  VLOG(1) << "0x" << std::hex << static_cast<int>(endpoint_address_and_direction())
          << ", max_packet_size " << std::dec << descriptor->wMaxPacketSize;
  CHECK_EQ(DescriptorToDirection(descriptor), UsbEndpoint::kIn);
}

PhysicalUsbInEndpoint::~PhysicalUsbInEndpoint() {
  CHECK_NOTNULL(handle_);
  handle_ = nullptr;
  libusb_context_ = nullptr;
}

int PhysicalUsbInEndpoint::DoGetMaxPacketSize() {
  CHECK_NOTNULL(handle_);
  return LibusbGetMaxPacketSize(handle_, endpoint_address());
}

int PhysicalUsbInEndpoint::DoGetMaxIsoPacketSize() {
  CHECK_NOTNULL(handle_);
  return LibusbGetMaxIsoPacketSize(handle_, endpoint_address());
}

namespace {
unsigned char LibUsbTransferType(int transfer_type) {
  switch (transfer_type) {
  case UsbEndpoint::kControl:
    return LIBUSB_TRANSFER_TYPE_CONTROL;
  case UsbEndpoint::kBulk:
    return LIBUSB_TRANSFER_TYPE_BULK;
  case UsbEndpoint::kInterrupt:
    return LIBUSB_TRANSFER_TYPE_INTERRUPT;
  case UsbEndpoint::kIsochronous:
    return LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
  default:
    LOG(FATAL) << "transfer_type " << transfer_type << " is bogus";
  }
}

const char kTransferTypeNameControl[] = "control";
const char kTransferTypeNameBulk[] = "bulk";
const char kTransferTypeNameInterrupt[] = "interrupt";
const char kTransferTypeNameIsochronous[] = "isochronous";

const char *TransferTypeName(int transfer_type) {
  switch (transfer_type) {
  case UsbEndpoint::kControl:
    return kTransferTypeNameControl;
  case UsbEndpoint::kBulk:
    return kTransferTypeNameBulk;
  case UsbEndpoint::kInterrupt:
    return kTransferTypeNameInterrupt;
  case UsbEndpoint::kIsochronous:
    return kTransferTypeNameIsochronous;
  default:
    LOG(FATAL) << "transfer_type " << transfer_type << " is bogus";
  }
}
}  // namespace

UsbEndpoint::IoStatus PhysicalUsbInEndpoint::DoRead(
    uint32_t length, int32_t timeout_milliseconds, Buffer *out,
    Notification *quit) {
  CHECK_NOTNULL(handle_);
  CHECK_GE(timeout_milliseconds, 0);
  CHECK_NOTNULL(out);

  VLOG(2) << "read on 0x" << std::hex << endpoint_address_and_direction()
          <<  ", size 0x" << std::hex << length
          << ", timeout " << std::dec << timeout_milliseconds << " [ms]";

  boost::scoped_ptr<Buffer> whole_buffer(new Buffer());
  whole_buffer->Resize(length);
  void *p = whole_buffer->GetBufferPointer(length);
  int transferred;
  const unsigned int timeout = static_cast<unsigned int>(timeout_milliseconds);
  int r;

  unsigned char transfer_type = LibUsbTransferType(transfer());
  r = do_sync_bulk_transfer(libusb_context_,
                            handle_, endpoint_address_and_direction(),
                            static_cast<unsigned char *>(p), length,
                            &transferred, timeout, transfer_type,
                            quit);

  switch (r) {
  case LIBUSB_SUCCESS:
    {
      size_t size_transferred = static_cast<size_t>(transferred);
      whole_buffer->Resize(size_transferred);

      VLOG(2)
        << "read on 0x"
        << std::hex << static_cast<int>(endpoint_address_and_direction())
        << ", size_transferred=0x" << std::hex << size_transferred;
      out->Copy(*whole_buffer);
      return kSuccess;
    }
  case LIBUSB_ERROR_TIMEOUT:
    VLOG(2) << "libusb_" << TransferTypeName(transfer())
	    << "_transfer timeout";
    return kTimeout;
  case LIBUSB_ERROR_IO:
    VLOG(1) << "Device i/o error.";
    return kFail;
  case LIBUSB_ERROR_NO_DEVICE:
    VLOG(1) << "Device disconnected.";
    return kNoDevice;
  case LIBUSB_ERROR_OTHER:
    LOG(INFO) << "libusb_" << TransferTypeName(transfer())
	      << "_transfer failed with r=" << std::dec << r;
    return kUnknown;
  default:
    // Most of these are more esoteric.
    LOG(INFO) << "libusb_" << TransferTypeName(transfer())
	      << "_transfer failed with r=" << std::dec << r;
    return kFail;
  }
}

////////////////////////////////////////////////////////////////////////

PhysicalUsbOutEndpoint::PhysicalUsbOutEndpoint(
    struct libusb_context *context,
    struct libusb_device_handle *handle,
    const struct libusb_endpoint_descriptor *descriptor)
    : UsbOutEndpoint(DescriptorToTransfer(descriptor),
                     DescriptorToAddress(descriptor)),
      libusb_context_(CHECK_NOTNULL(context)),
      handle_(CHECK_NOTNULL(handle)) {
  VLOG(1) << "0x" << std::hex << static_cast<int>(endpoint_address_and_direction())
          << ", max_packet_size " << std::dec << descriptor->wMaxPacketSize;
  CHECK_EQ(DescriptorToDirection(descriptor), UsbEndpoint::kOut);
}

PhysicalUsbOutEndpoint::~PhysicalUsbOutEndpoint() {
  CHECK_NOTNULL(handle_);
  handle_ = nullptr;
  libusb_context_ = nullptr;
}

int PhysicalUsbOutEndpoint::DoGetMaxPacketSize() {
  CHECK_NOTNULL(handle_);
  return LibusbGetMaxPacketSize(handle_, endpoint_address());
}

int PhysicalUsbOutEndpoint::DoGetMaxIsoPacketSize() {
  CHECK_NOTNULL(handle_);
  return LibusbGetMaxIsoPacketSize(handle_, endpoint_address());
}

UsbEndpoint::IoStatus PhysicalUsbOutEndpoint::DoWrite(
    const Buffer &buffer, int32_t timeout_milliseconds) {
  CHECK_NOTNULL(handle_);
  CHECK_EQ(direction(), kOut);

  VLOG(2) << "writing on 0x" << std::hex << endpoint_address_and_direction()
          << ", length=" << std::dec << buffer.Length()
	  << ", timeout " << std::dec << timeout_milliseconds << " [ms]";

  size_t length = buffer.Length();
  const unsigned char *p =
      static_cast<const unsigned char *>(buffer.GetBufferPointer(length));
  const unsigned int timeout = static_cast<unsigned int>(timeout_milliseconds);

  int transferred;
  int r;

  switch (transfer()) {
    case kBulk:
      VLOG(2) << "libusb_bulk_transfer, length=" << std::dec << length;
      r = libusb_bulk_transfer(handle_, endpoint_address_and_direction(),
                               const_cast<unsigned char *>(p),
                               length, &transferred,
                               timeout);
      VLOG(2) << "libusb_bulk_transfer, r=" << std::dec << r
              << ", transferred=" << std::dec << transferred;
      break;
    case kInterrupt:
      VLOG(2) << "libusb_interrupt_transfer, length="
              << std::dec << length;
      r = libusb_interrupt_transfer(handle_, endpoint_address_and_direction(),
                                    const_cast<unsigned char *>(p),
                                    length, &transferred,
                                    timeout);
      VLOG(2) << "libusb_interrupt_transfer, r=" << std::dec << r
              << ", transferred=" << std::dec << transferred;
      break;
    default:
      LOG(FATAL) << "bogus transfer() value";
  }

  size_t size_transferred;

  switch (r) {
  case LIBUSB_SUCCESS:
    size_transferred = static_cast<size_t>(transferred);
    CHECK_EQ(size_transferred, length);
    return kSuccess;
  case LIBUSB_ERROR_TIMEOUT:
    VLOG(2) << "libusb_" << TransferTypeName(transfer()) << "_transfer timeout";
    return kTimeout;
  case LIBUSB_ERROR_IO:
    VLOG(1) << "Device i/o error.";
    return kFail;
  case LIBUSB_ERROR_NO_DEVICE:
    VLOG(1) << "Device disconnected.";
    return kNoDevice;
  case LIBUSB_ERROR_OTHER:
    LOG(INFO) << "libusb_" << TransferTypeName(transfer())
	      << "_transfer failed with r=" << std::dec << r;
    return kUnknown;
  default:
    VLOG(1) << "libusb_" << TransferTypeName(transfer())
	    << "_transfer failed, r=" << std::dec << r;
    return kFail;
  }
}

}  // namespace glibusb
