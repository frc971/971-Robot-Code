obj-m += adis16505.o 
 
PWD := $(CURDIR) 
 
all: 
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make -C ../../../../linux M=$(PWD) modules 

clean: 
	ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make -C ../../../../linux M=$(PWD) clean
