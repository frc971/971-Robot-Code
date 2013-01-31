static void aschuh_chan_read_callback(struct urb *urb)
{
    struct aschuh_chan *chan;

    chan = urb->context;

    spin_lock(&chan->err_lock);
    /* sync/async unlink faults aren't errors */
    if (urb->status) {
        if (!(urb->status == -ENOENT ||
                    urb->status == -ECONNRESET ||
                    urb->status == -ESHUTDOWN))
            err("%s - nonzero write bulk status received: %d",
                    __func__, urb->status);

        chan->errors = urb->status;
    } else {
        chan->in_filled = urb->actual_length;
    }
    chan->ongoing_read = 0;
    spin_unlock(&chan->err_lock);

    complete(&chan->in_completion);
}

static int aschuh_chan_submit(struct aschuh_chan *chan){
    int rv;
    /* tell everybody to leave the URB alone */
    spin_lock_irq(&chan->err_lock);
    chan->ongoing_read = 1;
    spin_unlock_irq(&chan->err_lock);
	

    /* do it */
    rv = usb_submit_urb(chan->in_urb, GFP_KERNEL);
    if (rv < 0) {
        err("%s - failed submitting read urb, error %d",
                __func__, rv);
        chan->in_filled = 0;
        rv = (rv == -ENOMEM) ? rv : -EIO;
        spin_lock_irq(&chan->err_lock);
        chan->ongoing_read = 0;
        spin_unlock_irq(&chan->err_lock);
    }

    return rv;
}


static int aschuh_dev_bulk_do_read_io(struct aschuh_dev *dev,size_t count)
{
	struct aschuh_chan *chan = &dev->bulk;
    /* prepare a read */
    usb_fill_bulk_urb(chan->in_urb, dev->udev,
            usb_rcvbulkpipe(dev->udev, chan->in_endpointAddr),
            chan->in_buffer,
            min(chan->in_size, count),
            aschuh_chan_read_callback,
            chan);
	return aschuh_chan_submit(chan);
}

static int aschuh_dev_intr_do_read_io(struct aschuh_dev *dev,size_t count)
{
	struct aschuh_chan *chan = &dev->intr;
    /* prepare a read */
    usb_fill_int_urb(chan->in_urb, dev->udev,
            usb_rcvintpipe(dev->udev, chan->in_endpointAddr),
            chan->in_buffer,
            chan->in_size,
            aschuh_chan_read_callback,
            chan,chan->bEndpointInterval);
	return aschuh_chan_submit(chan);
}

static ssize_t aschuh_dev_read(struct file *file, char *buffer, size_t count,
        loff_t *ppos)
{
	int rv;
	bool ongoing_io;
	struct aschuh_chan *chan = file->private_data;
	struct aschuh_dev *dev = chan->owner; 
	enum chan_types{
		BULK_CHAN, INTR_CHAN
	} chan_type = INTR_CHAN;
    size_t available;
	if(chan == &dev->intr)
		chan_type = INTR_CHAN;
	else if(chan == &dev->bulk){
		chan_type = BULK_CHAN;
	}else{
		printk("Chan type not debug or data!!!");
		return 0;
	}
	

	    /* if we cannot read at all, return EOF */
    if (!chan->in_urb || !count)
        return 0;

	rv = mutex_lock_interruptible(&chan->io_mutex);
    if (rv < 0)
        return rv;

	if (!dev->interface) {          /* disconnect() was called */
        rv = -ENODEV;
        goto exit;
    }

	    /* if IO is under way, we must not touch things */
	while(true){
    	spin_lock_irq(&chan->err_lock);
    	ongoing_io = chan->ongoing_read;
    	spin_unlock_irq(&chan->err_lock);

    	if (ongoing_io) {
        	/* nonblocking IO shall not wait */
        	if (file->f_flags & O_NONBLOCK) {
            	rv = -EAGAIN;
            	goto exit;
        	}
        	/*
         	 * IO may take forever
         	 * hence wait in an interruptible state
         	 */
			mutex_unlock(&chan->io_mutex);
        	rv = wait_for_completion_interruptible(&chan->in_completion);
        	if (rv < 0)
            	return rv;

			rv = mutex_lock_interruptible(&chan->io_mutex);
    		if (rv < 0)
        		return rv;
        	/*
         	 * by waiting we also semiprocessed the urb
         	 * we must finish now
         	 */
        	chan->in_copied = 0;
    	}


    	/* errors must be reported */
    	rv = chan->errors;
    	if (rv < 0) {
        	/* any error is reported once */
        	chan->errors = 0;
        	/* to preserve notifications about reset */
        	rv = (rv == -EPIPE) ? rv : -EIO;
        	/* no data to deliver */
        	chan->in_filled = 0;
        	/* report it */
        	goto exit;
    	}
		/*
     	 * if the buffer is filled we may satisfy the read
     	 * else we need to start IO
     	 */

    	/* we had read data */
    	available = chan->in_filled - chan->in_copied;

    	if (!chan->in_filled || !available) {
        	/*
         	 * all data has been used or no data is avaiable.
         	 * actual IO needs to be done
         	 */
			if(chan_type == INTR_CHAN)
        		rv = aschuh_dev_intr_do_read_io(dev, count);
			else if(chan_type == BULK_CHAN){
        		rv = aschuh_dev_bulk_do_read_io(dev, count);
			}
        	if (rv < 0)
            	goto exit;
    	}else{
    		size_t chunk = min(available, count);
    		/*
     	 	 * data is available
     	 	 * chunk tells us how much shall be copied
     	 	 */

    		if (copy_to_user(buffer,
                		chan->in_buffer + chan->in_copied,
                		chunk))
        		rv = -EFAULT;
    		else
        		rv = chunk;

    		chan->in_copied += chunk;

    		/*
     	 	 * if we are asked for more than we have,
     	 	 * we start IO but don't wait
     	 	 */
    		//if (available < count){
			//	if(chan_type == INTR_CHAN)
        	//		aschuh_dev_intr_do_read_io(dev, count);
			//	else if(chan_type == BULK_CHAN){
        	//		aschuh_dev_bulk_do_read_io(dev, count);
			//	}
			//}
			goto exit;
		}
	}
exit:
	mutex_unlock(&chan->io_mutex);
	return rv;
}
