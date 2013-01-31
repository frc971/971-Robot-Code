#include "usb-aschuh_can_driver.h"
#include "usb-aschuh_can_device.h"

#define to_aschuh_dev(d) container_of(d, struct aschuh_dev, kref)
static void aschuh_dev_delete(struct kref *kref)
{
    struct aschuh_dev *dev = to_aschuh_dev(kref);

    usb_free_urb(dev->bulk.in_urb);
	usb_free_urb(dev->intr.in_urb);
    usb_put_dev(dev->udev);
    kfree(dev->bulk.in_buffer);
    kfree(dev->intr.in_buffer);
    kfree(dev);
}

#include "usb-aschuh_fops_read.c"
#include "usb-aschuh_fops_write.c"
#include "usb-aschuh_fops_open.c"
#include "usb-aschuh_fops_ioctl.c"
#include "usb-aschuh_fops_release.c"

static const struct file_operations aschuh_dev_fops = {
	.owner =	THIS_MODULE,
	.read  = 	aschuh_dev_read,
	.write = 	aschuh_dev_write,
	.open = 	aschuh_dev_open,
	.release = 	aschuh_dev_release,
	.flush = 	NULL,
	.llseek = 	NULL,
	.unlocked_ioctl = aschuh_dev_ioctl,
};

static int init_in_channel(struct aschuh_chan *chan,
		struct usb_endpoint_descriptor *endpoint){
    size_t buffer_size;
    buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
    chan->in_size = buffer_size;
    chan->in_endpointAddr = endpoint->bEndpointAddress;
    chan->in_buffer = kmalloc(buffer_size, GFP_KERNEL);
    if (!chan->in_buffer) {
        err("Could not allocate bulk_in_buffer");
		return -1;
    }
    chan->in_urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!chan->in_urb) {
        err("Could not allocate bulk_in_urb");
		return -1;
    }
	return 0;
}
int aschuh_dev_connect_endpoints(struct aschuh_dev *dev,
	    struct usb_host_interface *iface_desc){
    struct usb_endpoint_descriptor *endpoint;
    int i,rv;
    for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
        endpoint = &iface_desc->endpoint[i].desc;

        if (!dev->bulk.in_endpointAddr &&
                usb_endpoint_is_bulk_in(endpoint)) {
    		/* we found a bulk in endpoint */
    		printk("ASCHUH: found_bulk_in : 0x%x \n",endpoint->bEndpointAddress);
			if((rv = init_in_channel(&dev->bulk,endpoint)))
				return rv;
        }
        if (!dev->bulk.out_endpointAddr &&
                usb_endpoint_is_bulk_out(endpoint)) {
            /* we found a bulk out endpoint */
            printk("ASCHUH: found_bulk_out : 0x%x \n",endpoint->bEndpointAddress);
            dev->bulk.out_endpointAddr = endpoint->bEndpointAddress;
        }
        if (!dev->intr.in_endpointAddr &&
                usb_endpoint_is_int_in(endpoint)) {
    		/* we found a bulk in endpoint */
    		printk("ASCHUH: found_intr_in : 0x%x \n",endpoint->bEndpointAddress);
			if((rv = init_in_channel(&dev->intr,endpoint)))
				return rv;
			dev->intr.bEndpointInterval = endpoint->bInterval;
        }
        if (!dev->intr.out_endpointAddr &&
                usb_endpoint_is_int_out(endpoint)) {
            /* we found a intr out endpoint */
            printk("ASCHUH: found_intr_out : 0x%x \n",endpoint->bEndpointAddress);
            dev->intr.out_endpointAddr = endpoint->bEndpointAddress;
        }
    }
    if (!(dev->bulk.in_endpointAddr && dev->bulk.out_endpointAddr)) {
        err("Could not find both bulk-in and bulk-out endpoints");
		return -1;
    }
    if (!(dev->intr.in_endpointAddr && dev->intr.out_endpointAddr)) {
        err("Could not find both intr-in and intr-out endpoints");
		return -1;
    }
	return 0;
}

#define ASCHUH_DEV_MINOR_BASE 192
//used in getting the file in /dev/
static struct usb_class_driver aschuh_dev_class = {
    .name =         "aschuh%d",
    .fops =         &aschuh_dev_fops,
    .minor_base =   ASCHUH_DEV_MINOR_BASE,
};
static void init_aschuh_chan(struct aschuh_chan *chan,struct aschuh_dev *dev){
	sema_init(&chan->limit_sem, WRITES_IN_FLIGHT);
    mutex_init(&chan->io_mutex);
    spin_lock_init(&chan->err_lock);
    init_completion(&chan->in_completion);
	chan->owner = dev;
}
static int aschuh_can_driver_probe(struct usb_interface *interface, 
		const struct usb_device_id *id)
{
	// New Drivers Show up Here!
	struct aschuh_dev *dev;
    int retval = -ENOMEM;

    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        err("Out of memory");
        goto error;
    }
    kref_init(&dev->kref);
	
    init_usb_anchor(&dev->submitted);
	init_aschuh_chan(&dev->bulk,dev);
	init_aschuh_chan(&dev->intr,dev);


    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    /* set up the endpoint information */
    /* use only the first bulk-in and bulk-out endpoints */
	if(aschuh_dev_connect_endpoints(dev,interface->cur_altsetting))
		goto error;

    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    /* we can register the device now, as it is ready */
    retval = usb_register_dev(interface, &aschuh_dev_class);
    if (retval) {
        /* something prevented us from registering this driver */
        err("Not able to get a minor for this device.");
        usb_set_intfdata(interface, NULL);
        goto error;
    }

	printk("CAN driver has been probed - %d\n",interface->minor);
    /* let the user know what node this device is now attached to */
    dev_info(&interface->dev,
            "USB aschuh_dev device now attached to USB_ASCHUH-%d",
            interface->minor);
    return 0;

error:
    if (dev)
        /* this frees allocated memory */
        kref_put(&dev->kref, aschuh_dev_delete);
	return -ENOMEM;
}
static void aschuh_can_driver_disconnect(struct usb_interface *interface)
{
    struct aschuh_dev *dev;
    int minor = interface->minor;

    dev = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);

    /* give back our minor */
    usb_deregister_dev(interface, &aschuh_dev_class);

    /* prevent more I/O from starting */
	mutex_lock(&dev->bulk.io_mutex);
	mutex_lock(&dev->intr.io_mutex);
    dev->interface = NULL;
    mutex_unlock(&dev->bulk.io_mutex);
    mutex_unlock(&dev->intr.io_mutex);

    usb_kill_anchored_urbs(&dev->submitted);

    /* decrement our usage count */
    kref_put(&dev->kref, aschuh_dev_delete);

    dev_info(&interface->dev, "USB aschuh_dev #%d now disconnected", minor);
}

