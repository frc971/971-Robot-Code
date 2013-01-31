obj-m := deploy-module.o
deploy-module-y := usb-aschuh_can_driver.o

all:
	@make -C /lib/modules/`uname -r | tr -d '\n'`/build M=`pwd` modules

clean:
	@make -C /lib/modules/`uname -r | tr -d '\n'`/build M=`pwd` clean
