NAME=main

LDSCRIPT=lpc1758_Debug.ld

CSRC=main.c

GCC_PATH=/usr/local/cortex-m3/bin

PORT=/dev/ttyS1

CC=$(GCC_PATH)/arm-eabi-gcc
LD=$(GCC_PATH)/arm-eabi-ld
OBJCOPY=$(GCC_PATH)/arm-eabi-objcopy
OBJDUMP=$(GCC_PATH)/arm-eabi-objdump
AS=$(GCC_PATH)/arm-eabi-as
FLASHER=lpc21isp

CFLAGS=-I. -I./FreeRTOS/include -I./FreeRTOS/portable/GCC/ARM_CM3/ -I./CommonDemoTasks/include -O3 -mcpu=cortex-m3 -mthumb -Wl,--gc-sections -ffunction-sections -Wl,-static -Werror

SPEED=38400
OSC=12000

SOURCES=main.c \
	ParTest.c \
	printf-stdarg.c \
	cr_startup_lpc17.c \
	CommonDemoTasks/flash.c \
	FreeRTOS/portable/MemMang/heap_2.c \
	alloc.c \
	analog.c \
	FreeRTOS/portable/GCC/ARM_CM3/port.c \
	FreeRTOS/tasks.c \
	FreeRTOS/list.c \
	FreeRTOS/queue.c \
	CAN.c \
	LPCUSB/usbinit.c \
	LPCUSB/usbcontrol.c \
	LPCUSB/USB_SENSOR_STREAM.c \
	LPCUSB/usbhw_lpc.c \
	spi.c \
	LPCUSB/usbstdreq.c

all: $(NAME).hex

$(NAME).elf: Makefile $(SOURCES:.c=.o) $(LDSCRIPT)
	$(CC) $(CFLAGS) -nostartfiles -nostdlib -T $(LDSCRIPT) -o $@ -L/usr/local/cortex-m3/lib/gcc/arm-eabi/4.5.1/ $(SOURCES:.c=.o) -Wa,-Map -Wa,main.map -lgcc

%.o: %.c Makefile
	$(CC) $(CFLAGS) -nostartfiles -nostdlib -c -o $@ $< -Wall -std=gnu99

run: deploy
	$(FLASHER) -termonly $(NAME).hex $(PORT) $(SPEED) $(OSC)

deploy: all $(NAME).hex
	$(FLASHER) -hex -verify $(NAME).hex $(PORT) $(SPEED) $(OSC)

cat:
	@cd ../../bin; python serial_looper.py

assm.S: $(NAME).elf
	$(OBJDUMP) -D -S $(NAME).elf > $@

%.hex: %.elf Makefile
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(SOURCES:.c=.o) $(NAME).hex $(NAME).elf
