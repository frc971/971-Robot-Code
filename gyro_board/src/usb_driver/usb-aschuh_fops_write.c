#include <linux/usb.h>
static void aschuh_dev_write_callback(struct urb *urb)
{
    struct aschuh_chan *chan;

    chan = urb->context;
    /* sync/async unlink faults aren't errors */
    if (urb->status) {
        if (!(urb->status == -ENOENT ||
                    urb->status == -ECONNRESET ||
                    urb->status == -ESHUTDOWN))
            err("%s - nonzero write bulk status received: %d",
                    __func__, urb->status);

        spin_lock(&chan->err_lock);
        chan->errors = urb->status;
        spin_unlock(&chan->err_lock);
    }

    /* free up our allocated buffer */
    usb_free_coherent(urb->dev, urb->transfer_buffer_length,
            urb->transfer_buffer, urb->transfer_dma);
    up(&chan->limit_sem);
}
//#define WRITES_IN_FLIGHT        8
#define MAX_TRANSFER	64

static ssize_t aschuh_dev_write(struct file *file, const char *user_buffer,
        size_t count, loff_t *ppos)
{
    struct aschuh_chan *chan;
	struct aschuh_dev *dev;
    int retval = 0;
    struct urb *urb = NULL;
    char *buf = NULL;
    size_t writesize = min(count, (size_t)MAX_TRANSFER);

    chan = file->private_data;
	dev = chan->owner;

    /* verify that we actually have some data to write */
    if (count == 0)
        goto exit;

    /*
     * limit the number of URBs in flight to stop a user from using up all
     * RAM
     */
    if (!(file->f_flags & O_NONBLOCK)) {
        if (down_interruptible(&chan->limit_sem)) {
            retval = -ERESTARTSYS;
            goto exit;
        }
    } else {
        if (down_trylock(&chan->limit_sem)) {
            retval = -EAGAIN;
            goto exit;
        }
    }

    spin_lock_irq(&chan->err_lock);
    retval = chan->errors;
    if (retval < 0) {
        /* any error is reported once */
        chan->errors = 0;
        /* to preserve notifications about reset */
        retval = (retval == -EPIPE) ? retval : -EIO;
    }
    spin_unlock_irq(&chan->err_lock);
    if (retval < 0)
        goto error;

    /* create a urb, and a buffer for it, and copy the data to the urb */
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb) {
        retval = -ENOMEM;
        goto error;
    }

    buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
            &urb->transfer_dma);
    if (!buf) {
        retval = -ENOMEM;
        goto error;
    }

    if (copy_from_user(buf, user_buffer, writesize)) {
        retval = -EFAULT;
        goto error;
    }

    /* this lock makes sure we don't submit URBs to gone devices */
    mutex_lock(&chan->io_mutex);
    if (!dev->interface) {          /* disconnect() was called */
        mutex_unlock(&chan->io_mutex);
		printk("Should be exiting shortly.... stay tuned for details\n");
        retval = -ENODEV;
        goto error;
    }

    /* initialize the urb properly */
	if(chan == &dev->bulk){
    	usb_fill_bulk_urb(urb, dev->udev,
            usb_sndbulkpipe(dev->udev, chan->out_endpointAddr),
            buf, writesize, aschuh_dev_write_callback, chan);
	}else{
		usb_fill_int_urb(urb, dev->udev,
            usb_sndintpipe(dev->udev, chan->out_endpointAddr),
            buf, writesize, aschuh_dev_write_callback, chan,
			chan->bEndpointInterval);
	}
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    usb_anchor_urb(urb, &dev->submitted);
    /* send the data out the bulk port */
    retval = usb_submit_urb(urb, GFP_KERNEL);
    mutex_unlock(&chan->io_mutex);
    if (retval) {
        err("%s - failed submitting write urb, error %d", __func__,
                retval);
    	usb_unanchor_urb(urb);
		goto error;
    }

    /*
     * release our reference to this urb, the USB core will eventually free
     * it entirely
     */
    usb_free_urb(urb);


    return writesize;

error:
    if (urb) {
        usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
        usb_free_urb(urb);
    }
    up(&chan->limit_sem);

exit:
    return retval;
}

