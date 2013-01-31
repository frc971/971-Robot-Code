static long aschuh_dev_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct aschuh_chan *new_chan;
	struct aschuh_chan *chan = file->private_data;
	struct aschuh_dev *dev = chan->owner;
	printk("ioctl(%d,%d)\n",cmd,(int)arg);
	if(cmd == 1){
		if(arg == 254){
			new_chan = &dev->bulk;
		}else{
			new_chan = &dev->intr;
		}
		file->private_data = new_chan;
		return 0;
	}else{
		return 0;
	}
}
