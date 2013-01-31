#ifndef __USB_ASCHUH_CAN_DRIVER__H_
#define __USB_ASCHUH_CAN_DRIVER__H_

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>

struct aschuh_can_device {
	struct usb_device	*udev;
	struct usb_interface	*interface;
};
#define VENDOR_ID		0x1424
#define PRODUCT_ID		0xd243

#endif
