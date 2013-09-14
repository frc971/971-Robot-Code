// Copyright 2012 Google Inc. All Rights Reserved.
//
// Alternative libusb call to do transfers that quits when
// a notification is notified.

#ifndef _GLIBUSB_GLIBUSB_TRANSFER_H_
#define _GLIBUSB_GLIBUSB_TRANSFER_H_

extern "C" {
struct libusb_context;
struct libusb_device_handle;
}

class Notification;

namespace glibusb {

// Bulk transfer code cribbed from libusb1/sync.c
// The difference between this and the original code is that the
// transfer now accepts a notification to poll for the quit message.
// When it receives a quit message on the notification, it cancels the transfer.
// The provided API's don't support better reuse of the existing code from what
// I can tell.
int do_sync_bulk_transfer(
    struct libusb_context *context,
    struct libusb_device_handle *dev_handle,
    unsigned char endpoint, unsigned char *buffer, int length,
    int *transferred, unsigned int timeout, unsigned char type,
    Notification *quit);

}  // namespace glibusb

#endif  // _GLIBUSB_GLIBUSB_TRANSFER_H_
