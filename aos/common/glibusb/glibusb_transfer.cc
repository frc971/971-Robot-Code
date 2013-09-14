// Copyright 2012 Google Inc. All Rights Reserved.
//
// Alternative libusb call to do transfers that quits when
// a notification is notified.
//
// This code was originally from third_party/libusb/libusb1/sync.c
// and has been slightly modified to poll the notification.

#include "glibusb_transfer.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <libusb.h>

#include "glibusb_endpoint.h"


DEFINE_int32(notification_poll,
             60,
             "Time in seconds to wait between checking the quit notification "
             "when waiting for USB messages.");

namespace {

static bool GEQZero(const char* flagname, int32_t value) {
  if (value < 0) {
    printf("Invalid value for --%s: %d\n", flagname, value);
    return false;
  }
  return true;
}

static const bool notification_poll_dummy = google::RegisterFlagValidator(
  &FLAGS_notification_poll, &GEQZero);

}  // namespace


namespace glibusb {

namespace {
// Static code from libusb1/sync.c
void bulk_transfer_cb(struct libusb_transfer *transfer) {
  int *completed = static_cast<int*>(transfer->user_data);
  *completed = 1;
  VLOG(3) << "actual_length=" << transfer->actual_length;
  /* caller interprets results and frees transfer */
}
}  // namespace

int do_sync_bulk_transfer(
    struct libusb_context *context,
    struct libusb_device_handle *dev_handle,
    unsigned char endpoint, unsigned char *buffer, int length,
    int *transferred, unsigned int timeout, unsigned char type,
    Notification *quit) {
  struct libusb_transfer *transfer = libusb_alloc_transfer(0);
  int completed = 0;
  int r;

  if (!transfer)
    return LIBUSB_ERROR_NO_MEM;

  libusb_fill_bulk_transfer(transfer, dev_handle, endpoint, buffer, length,
    bulk_transfer_cb, &completed, timeout);
  transfer->type = type;

  r = libusb_submit_transfer(transfer);
  if (r < 0) {
    libusb_free_transfer(transfer);
    return r;
  }

  while (!completed) {
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r = libusb_handle_events_timeout_completed(context, &tv, &completed);
    if (quit != NULL && quit->HasBeenNotified()) {
      LOG(WARNING) << "Caught quit notification.  Canceling transfer.";
      r = LIBUSB_ERROR_TIMEOUT;
    }
    if (r < 0) {
      if (r == LIBUSB_ERROR_INTERRUPTED) {
        continue;
      }
      libusb_cancel_transfer(transfer);
      while (!completed) {
        struct timeval cancel_tv;
        cancel_tv.tv_sec = (quit == NULL ? FLAGS_notification_poll : 60);
        cancel_tv.tv_usec = 0;
        if (libusb_handle_events_timeout_completed(context, &cancel_tv,
                                                   &completed) < 0) {
          break;
        }
      }
      libusb_free_transfer(transfer);
      return r;
    }
  }

  *transferred = transfer->actual_length;
  switch (transfer->status) {
    case LIBUSB_TRANSFER_COMPLETED:
      r = 0;
      break;
    case LIBUSB_TRANSFER_TIMED_OUT:
      r = LIBUSB_ERROR_TIMEOUT;
      break;
    case LIBUSB_TRANSFER_STALL:
      r = LIBUSB_ERROR_PIPE;
      break;
    case LIBUSB_TRANSFER_OVERFLOW:
      r = LIBUSB_ERROR_OVERFLOW;
      break;
    case LIBUSB_TRANSFER_NO_DEVICE:
      r = LIBUSB_ERROR_NO_DEVICE;
      break;
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_CANCELLED:
      r = LIBUSB_ERROR_IO;
      break;
    default:
      LOG(WARNING) << "unrecognised status code " << transfer->status;
      r = LIBUSB_ERROR_OTHER;
  }

  libusb_free_transfer(transfer);
  return r;
}

}  // namespace glibusb
