static int aschuh_dev_release(struct inode *inode, struct file *file)
{
    struct aschuh_dev *dev;
	printk("closing device!!!\n");

    dev = ((struct aschuh_chan *)file->private_data)->owner;
    if (dev == NULL)
        return -ENODEV;

    /* allow the device to be autosuspended */
   	//mutex_lock(&dev->bulk.io_mutex);
    //mutex_lock(&dev->intr.io_mutex);
    //if (!--dev->open_count && dev->interface)
    //    usb_autopm_put_interface(dev->interface);
    //mutex_unlock(&dev->bulk.io_mutex);
    //mutex_unlock(&dev->intr.io_mutex);

    /* decrement the count on our device */
    kref_put(&dev->kref, aschuh_dev_delete);
    return 0;
}

