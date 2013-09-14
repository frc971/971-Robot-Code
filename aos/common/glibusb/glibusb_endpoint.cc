// Copyright 2012 Google Inc. All Rights Reserved.
//
// Modified by FRC Team 971.

#include "glibusb_endpoint.h"

#include <stddef.h>
#include <inttypes.h>
#include <memory>

#include "aos/common/logging/logging.h"
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
  : notified_changed_(&mutex_), notified_(prenotify) {}

bool Notification::HasBeenNotified() const {
  ::aos::MutexLocker lock(&mutex_);
  return notified_;
}

void Notification::WaitForNotification() const {
  ::aos::MutexLocker lock(&mutex_);
  while (HasBeenNotifiedUnlocked()) notified_changed_.Wait();
}

void Notification::Notify() {
  ::aos::MutexLocker lock(&mutex_);
  if (notified_) {
    LOG(FATAL, "already notified\n");
  }
  notified_ = true;
  notified_changed_.Broadcast();
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
  if ((endpoint_address_and_direction_ & 0x80) != direction) {
    LOG(FATAL, "Direction in address %x doesn't match direction %x.\n",
        endpoint_address_and_direction_, direction);
  }
  if ((endpoint_address_and_direction_ & 0x8f) !=
      endpoint_address_and_direction_) {
    LOG(FATAL, "Invalid endpoint address %x.\n",
        endpoint_address_and_direction_);
  }
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
  LOG(DEBUG, "0x%x, max_packet_size=%" PRId16 "\n",
      static_cast<int>(endpoint_address_and_direction()),
      descriptor->wMaxPacketSize);
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
    LOG(FATAL, "transfer_type %d is bogus", transfer_type);
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
    LOG(FATAL, "transfer_type %d is bogus", transfer_type);
  }
}
}  // namespace

UsbEndpoint::IoStatus PhysicalUsbInEndpoint::DoRead(
    uint32_t length, int32_t timeout_milliseconds, Buffer *out,
    Notification *quit) {
  CHECK_NOTNULL(handle_);
  CHECK_GE(timeout_milliseconds, 0);
  CHECK_NOTNULL(out);

  // TODO(brians): Conditionally enable this.
  LOG(DEBUG, "read on 0x%x, size 0x%x, timeout %" PRId32 " [ms]\n",
      endpoint_address_and_direction(), length, timeout_milliseconds);

  ::std::unique_ptr<Buffer> whole_buffer(new Buffer());
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

      // TODO(brians): Conditionally enable this.
      LOG(DEBUG, "read on 0x%x, size_transferred=%zx\n",
          endpoint_address_and_direction(), size_transferred);
      out->Copy(*whole_buffer);
      return kSuccess;
    }
  case LIBUSB_ERROR_TIMEOUT:
    LOG(DEBUG, "libusb_%s_transfer timeout\n",
        TransferTypeName(transfer()));
    return kTimeout;
  case LIBUSB_ERROR_IO:
    LOG(DEBUG, "device I/O error\n");
    return kFail;
  case LIBUSB_ERROR_NO_DEVICE:
    LOG(DEBUG, "device disconnected\n");
    return kNoDevice;
  case LIBUSB_ERROR_OTHER:
    LOG(INFO, "libusb_%s_transfer other error\n", TransferTypeName(transfer()));
    return kUnknown;
  default:
    // Most of these are more esoteric.
    LOG(INFO, "libusb_%s_transfer failed with %d: %s\n",
        TransferTypeName(transfer()), r, libusb_error_name(r));
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
  LOG(DEBUG, "0x%x, max_packet_size=%" PRId16 "\n",
      static_cast<int>(endpoint_address_and_direction()),
      descriptor->wMaxPacketSize);
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

  LOG(DEBUG, "writing on 0x%x, length=%zd, timeout %d [ms]\n",
      endpoint_address_and_direction(), buffer.Length(), timeout_milliseconds);

  size_t length = buffer.Length();
  const unsigned char *p =
      static_cast<const unsigned char *>(buffer.GetBufferPointer(length));
  const unsigned int timeout = static_cast<unsigned int>(timeout_milliseconds);

  int transferred;
  int r;

  // TODO(brians): Conditionally enable this.
  LOG(DEBUG, "libusb_%s_transfer, length=%d\n",
      TransferTypeName(transfer()), length);
  switch (transfer()) {
    case kBulk:
      r = libusb_bulk_transfer(handle_, endpoint_address_and_direction(),
                               const_cast<unsigned char *>(p),
                               length, &transferred,
                               timeout);
      break;
    case kInterrupt:
      r = libusb_interrupt_transfer(handle_, endpoint_address_and_direction(),
                                    const_cast<unsigned char *>(p),
                                    length, &transferred,
                                    timeout);
      break;
    case kControl:
    case kIsochronous:
    default:
      LOG(FATAL, "bogus transfer() value\n");
  }
  // TODO(brians): Conditionally enable this.
  LOG(DEBUG, "libusb_%s_transfer, r=%d (%s), transferred=%d\n",
      TransferTypeName(transfer()), r, libusb_error_name(r), transferred);

  size_t size_transferred;

  switch (r) {
  case LIBUSB_SUCCESS:
    size_transferred = static_cast<size_t>(transferred);
    CHECK_EQ(size_transferred, length);
    return kSuccess;
  case LIBUSB_ERROR_TIMEOUT:
    LOG(DEBUG, "libusb_%s_transfer timeout\n",
        TransferTypeName(transfer()));
    return kTimeout;
  case LIBUSB_ERROR_IO:
    LOG(DEBUG, "device I/O error\n");
    return kFail;
  case LIBUSB_ERROR_NO_DEVICE:
    LOG(DEBUG, "device disconnected\n");
    return kNoDevice;
  case LIBUSB_ERROR_OTHER:
    LOG(INFO, "libusb_%s_transfer other error\n", TransferTypeName(transfer()));
    return kUnknown;
  default:
    LOG(INFO, "libusb_%s_transfer failed with %d: %s\n",
        TransferTypeName(transfer()), r, libusb_error_name(r));
    return kFail;
  }
}

}  // namespace glibusb
