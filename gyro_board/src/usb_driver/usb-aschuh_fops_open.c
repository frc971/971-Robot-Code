static struct usb_driver aschuh_can_driver;
static int aschuh_dev_open(struct inode *inode, struct file *file)
{
    struct aschuh_dev *dev;
    struct usb_interface *interface;
    int subminor;

    subminor = iminor(inode);
	printk("opening device!!!\n");
    interface = usb_find_interface(&aschuh_can_driver, subminor);
    if (!interface) {
        err("%s - error, can't find device for minor %d",
                __func__, subminor);
        return( -ENODEV);
    }

    dev = usb_get_intfdata(interface);
    if (!dev) {
        return( -ENODEV);
    }

    /* increment our usage count for the device */
    kref_get(&dev->kref);

    /* lock the device to allow correctly handling errors
     * in resumption */
    mutex_lock(&dev->bulk.io_mutex);
    mutex_lock(&dev->intr.io_mutex);
	//TODO(parker): Hey folks! I just commented this out.
	// it would be a good thing to check what usb_autopm_get_interface
	// actually does :P. Good thing to check if you find errors.
	/*
    if (!dev->open_count++) {
        int retval = usb_autopm_get_interface(interface);
        if (retval) {
            dev->open_count--;
            mutex_unlock(&dev->bulk.io_mutex);
            mutex_unlock(&dev->intr.io_mutex);
            kref_put(&dev->kref, aschuh_dev_delete);
            return retval;
        }
    }
	*/
	/* else { //uncomment this block if you want exclusive open
         retval = -EBUSY;
         dev->open_count--;
         mutex_unlock(&dev->io_mutex);
         kref_put(&dev->kref, aschuh_dev_delete);
         goto exit;
         } */
    /* prevent the device from being autosuspended */

    /* save our object in the file's private structure */
    file->private_data = &dev->intr;
    mutex_unlock(&dev->bulk.io_mutex);
    mutex_unlock(&dev->intr.io_mutex);

    return 0;
}

