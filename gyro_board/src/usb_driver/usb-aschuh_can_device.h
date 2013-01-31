#ifndef _USB_ASCHUH_CAN_DEVICE__H_
#define _USB_ASCHUH_CAN_DEVICE__H_
struct aschuh_dev;
struct aschuh_chan{
    struct urb              *in_urb;           /* the urb to read data with */
    unsigned char           *in_buffer;        /* the buffer to receive data */
    size_t                  in_size;           /* the size of the receive buffer */
    size_t                  in_filled;         /* number of bytes in the buffer */
    size_t                  in_copied;         /* already copied to user space */
    __u8                    in_endpointAddr;   /* the address of the in endpoint */
    __u8                    out_endpointAddr;  /* the address of the out endpoint */
    int                     errors;            /* the last request tanked */
    spinlock_t              err_lock;          /* lock for errors */
    struct mutex            io_mutex;          /* synchronize I/O with disconnect */
    struct semaphore        limit_sem;              /* limiting the number of writes in progress */
    struct completion       in_completion;     /* to wait for an ongoing read */
    bool                    ongoing_read;      /* a read is going on */
    bool                    processed_urb;     /* indicates we haven't processed the urb */
	__u8					bEndpointInterval; /* used in fill_int_urb */
	struct aschuh_dev		*owner;
};

struct aschuh_dev{
    struct usb_device       *udev;                  /* the usb device for this device */
    struct usb_interface    *interface;             /* the interface for this device */
    struct usb_anchor       submitted;              /* in case we need to retract our submissions */
	struct aschuh_chan		bulk;					/* channel for handling bulk requests */
	struct aschuh_chan		intr;
    int                     open_count;             /* count the number of openers */
    struct kref             kref;
};
static void aschuh_can_driver_disconnect(struct usb_interface *interface);
static int aschuh_can_driver_probe(struct usb_interface *interface,
        const struct usb_device_id *id);


#endif
