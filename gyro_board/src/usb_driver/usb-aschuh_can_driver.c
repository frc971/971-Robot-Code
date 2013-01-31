#include "usb-aschuh_can_driver.h"
#include "usb-aschuh_can_device.h"
#define WRITES_IN_FLIGHT        8
static const struct usb_device_id aschuh_can_device_table[] = {
	{ USB_DEVICE(VENDOR_ID,	PRODUCT_ID) },
	{ } /*terminating entry */
};
MODULE_DEVICE_TABLE(usb, aschuh_can_device_table);


#include "usb-aschuh_can_device.c"
static struct usb_driver aschuh_can_driver = {
	.name = 	"aschuh_can_driver",
	.probe =  	aschuh_can_driver_probe,
	.disconnect = 	aschuh_can_driver_disconnect,
	.suspend =	NULL,
	.resume =	NULL,
	.pre_reset = 	NULL,
	.post_reset = 	NULL,
	.id_table = 	aschuh_can_device_table,
	.supports_autosuspend = 0,
};

static int __init aschuh_can_driver_init(void)
{
	int result;

	result = usb_register(&aschuh_can_driver);
	if(result)
		err("usb_register failed. Error number %d",result);

	printk("INIT CAN driver\n");
	return result;
}
static void __exit aschuh_can_driver_exit(void)
{
	usb_deregister(&aschuh_can_driver);
	printk("Closing CAN driver\n");
}


module_init(aschuh_can_driver_init);
module_exit(aschuh_can_driver_exit);


MODULE_LICENSE("GPL"); //not sure about this.
