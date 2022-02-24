/*
 * adis16505.c - Driver for the adis16505 IMU used by 971.
 */
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h> /* Needed for pr_info() */
#include <linux/kfifo.h>
#include <linux/module.h> /* Needed by all modules */
#include <linux/poll.h>

#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_platform.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("frc971");
MODULE_DESCRIPTION("adis16505 rp2040 driver");

#define MODULE_NAME "adis16505"

//! filter for the device tree class
static struct of_device_id adis16505_match[] = {
    {.compatible = "frc971,adis16505"}, {}};

MODULE_DEVICE_TABLE(of, adis16505_match);

#define TRANSFER_SIZE 42
struct imu_sample {
  char time[8];
  char d[TRANSFER_SIZE];
};

struct adis16505_state {
  dev_t character_device;
  struct class *device_class;
  struct cdev handle_cdev;

  struct spi_device *spi;

  struct spi_message spi_msg;

  struct spi_transfer spi_xfer;

  char tx_buff[128];
  char rx_buff[128];

  int count;

  spinlock_t lock;

  wait_queue_head_t wq;

  spinlock_t fifo_read_lock;
  DECLARE_KFIFO(fifo, struct imu_sample, 32);
};

static int adis16505_dev_open(struct inode *in, struct file *f) {
  struct adis16505_state *ts =
      container_of(in->i_cdev, struct adis16505_state, handle_cdev);
  int count;

  f->private_data = ts;

  spin_lock(&ts->lock);
  count = ts->count;
  if (count == 0) {
    ++(ts->count);
  }
  spin_unlock(&ts->lock);

  printk("open %p, count %d\n", ts, count);
  if (count > 0) {
    return -EBUSY;
  }
  return 0;
}

static int adis16505_dev_release(struct inode *in, struct file *f) {
  struct adis16505_state *ts;
  ts = container_of(in->i_cdev, struct adis16505_state, handle_cdev);

  printk("release %p\n", ts);
  spin_lock(&ts->lock);
  --(ts->count);
  spin_unlock(&ts->lock);

  return 0;
}

static ssize_t adis16505_dev_read(struct file *f, char *d, size_t s,
                                  loff_t *of) {
  struct adis16505_state *ts = f->private_data;
  int err;

  if (s != sizeof(struct imu_sample)) {
    return -EINVAL;
  }

  while (true) {
    struct imu_sample sample;
    int elements;

    spin_lock(&ts->fifo_read_lock);
    elements = kfifo_get(&ts->fifo, &sample);
    spin_unlock(&ts->fifo_read_lock);

    if (elements == 0) {
      bool empty;
      if (f->f_flags & O_NONBLOCK) {
        return -EAGAIN;
      }

      err = wait_event_interruptible(ts->wq,
                                     (spin_lock(&ts->fifo_read_lock),
                                      empty = !kfifo_is_empty(&ts->fifo),
                                      spin_unlock(&ts->fifo_read_lock), empty));
      if (err != 0) {
        return err;
      }
      continue;
    }

    memcpy(d, &sample, sizeof(sample));
    return sizeof(sample);
  }
}

static unsigned int adis16505_dev_poll(struct file *f,
                                       struct poll_table_struct *wait) {
  struct adis16505_state *ts = f->private_data;
  __poll_t mask = 0;

  poll_wait(f, &ts->wq, wait);

  spin_lock(&ts->fifo_read_lock);
  if (!kfifo_is_empty(&ts->fifo)) {
    mask |= (POLLIN | POLLRDNORM);
  }
  spin_unlock(&ts->fifo_read_lock);

  return mask;
}

static const struct file_operations adis16505_cdev_opps = {
    .read = adis16505_dev_read,
    .open = adis16505_dev_open,
    .release = adis16505_dev_release,
    .poll = adis16505_dev_poll,
};

static void all_done(void *ts_ptr) {
  // struct adis16505_state *ts = ts_ptr;
  // printk("All done %x %x\n", ts->rx_buff[0], ts->rx_buff[1]);
}

static irqreturn_t adis16505_irq(int irq, void *handle) {
  struct adis16505_state *ts = handle;
  struct imu_sample s;
  int err;
  int i;

  u64 time = ktime_get_ns();
  memcpy(&s.time, &time, sizeof(time));

  spi_message_init(&ts->spi_msg);
  for (i = 0; i < TRANSFER_SIZE; ++i) {
    ts->tx_buff[i] = i;
  }

  ts->spi_xfer.tx_buf = ts->tx_buff;
  ts->spi_xfer.rx_buf = ts->rx_buff;
  ts->spi_xfer.len = TRANSFER_SIZE;

  spi_message_add_tail(&ts->spi_xfer, &ts->spi_msg);

  ts->spi_msg.complete = all_done;
  ts->spi_msg.context = ts;

  err = spi_sync(ts->spi, &ts->spi_msg);

  // TODO(austin): Timestamp.  Also decode the packet for real.
  for (i = 0; i < TRANSFER_SIZE; ++i) {
    s.d[i] = ts->rx_buff[i];
  }

  // Attempt to emplace.  If it fails, just drop the data.
  kfifo_put(&ts->fifo, s);

  wake_up_interruptible(&ts->wq);

  return IRQ_HANDLED;
}

static int adis16505_probe(struct spi_device *spi) {
  int err;
  struct adis16505_state *ts;

  if (!spi->irq) {
    dev_dbg(&spi->dev, "no IRQ?\n");
    return -EINVAL;
  }

  if (spi->max_speed_hz > 10000000) {
    dev_err(&spi->dev, "f(sample) %d KHz?\n", spi->max_speed_hz / 1000);
    return -EINVAL;
  }

  spi->bits_per_word = 8;
  spi->mode = SPI_MODE_3;
  spi->max_speed_hz = 2000000;

  err = spi_setup(spi);
  if (err < 0) {
    return err;
  }

  ts = kzalloc(sizeof(struct adis16505_state), GFP_KERNEL);

  if (!ts) {
    err = -ENOMEM;
    goto err_free_mem;
  }

  printk("Ts allocated %p\n", ts);

  spin_lock_init(&ts->lock);
  spin_lock_init(&ts->fifo_read_lock);
  ts->count = 0;
  INIT_KFIFO(ts->fifo);
  init_waitqueue_head(&ts->wq);

  spi_set_drvdata(spi, ts);
  ts->spi = spi;

  // Flags are sourced from the device tree.
  err = request_threaded_irq(spi->irq, NULL, adis16505_irq, IRQF_ONESHOT,
                             spi->dev.driver->name, ts);

  if (!ts) {
    dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
    goto err_free_mem;
  }

  err = alloc_chrdev_region(&ts->character_device, 0, 1, "adis16505");
  if (err < 0) {
    dev_dbg(&spi->dev, "alloc_chrdev_region error %i", err);
    goto err_free_irq;
  }

  // create device class
  if ((ts->device_class = class_create(THIS_MODULE, "adis16505_class")) ==
      NULL) {
    dev_dbg(&spi->dev, "class_create error");
    goto error_classCreate;
  }

  if (NULL == device_create(ts->device_class, NULL, ts->character_device, NULL,
                            "adis16505")) {
    dev_dbg(&spi->dev, "device_create error");
    goto error_deviceCreate;
  }

  cdev_init(&ts->handle_cdev, &adis16505_cdev_opps);
  err = cdev_add(&ts->handle_cdev, ts->character_device, 1);
  if (-1 == err) {
    dev_dbg(&spi->dev, "cdev_add error %i", err);
    goto error_device_add;
    return -1;
  }

  dev_dbg(&spi->dev, "Probed adis16505\n");

  if (err < 0) {
    goto err_free_mem;
  }

  return 0;

error_device_add:
  device_destroy(ts->device_class, ts->character_device);
error_deviceCreate:
  class_destroy(ts->device_class);
error_classCreate:
  unregister_chrdev_region(ts->character_device, 1);
err_free_irq:
  free_irq(spi->irq, ts);

err_free_mem:
  kfree(ts);
  return err;
}

static int adis16505_remove(struct spi_device *spi) {
  struct adis16505_state *ts = spi_get_drvdata(spi);

  device_destroy(ts->device_class, ts->character_device);

  class_destroy(ts->device_class);

  unregister_chrdev_region(ts->character_device, 1);

  free_irq(spi->irq, ts);

  kfree(ts);

  dev_dbg(&spi->dev, "unregistered adis16505\n");
  return 0;
}

static struct spi_driver adis16505_driver = {
    .driver =
        {
            .name = "adis16505",
            .of_match_table = of_match_ptr(adis16505_match),
        },
    .probe = adis16505_probe,
    .remove = adis16505_remove,
};

module_spi_driver(adis16505_driver); 
