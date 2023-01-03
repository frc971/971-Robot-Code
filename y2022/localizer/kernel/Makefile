obj-m += adis16505.o 
 
PWD := $(CURDIR) 
 
rockpi:
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make -C ../../../frc971/rockpi/linux M=$(PWD) modules

all: 
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make -C ../../../../linux M=$(PWD) modules 

clean: 
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make -C ../../../../linux M=$(PWD) clean
