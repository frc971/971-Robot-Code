NAME=main

LDSCRIPT=lpc1758_Debug.ld

CSRC=main.c

# See if /opt/cortex-m3 exists first, because that's the preferred location. If
# not, fall back to /usr/local/cortex-m3.
ifneq ($(wildcard /opt/cortex-m3),)
	TOOLS_PREFIX=/opt/cortex-m3
else
	TOOLS_PREFIX=/usr/local/cortex-m3
endif
GCC_PATH=$(TOOLS_PREFIX)/bin

PORT=/dev/ttyUSB0

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
	digital.c \
	encoder.c \
	FreeRTOS/portable/GCC/ARM_CM3/port.c \
	FreeRTOS/tasks.c \
	FreeRTOS/list.c \
	FreeRTOS/queue.c \
	CAN.c \
	LPCUSB/usbinit.c \
	LPCUSB/usbcontrol.c \
	usb_device.c \
	LPCUSB/usbhw_lpc.c \
	gyro.c \
	LPCUSB/usbstdreq.c

DATA_STRUCT_CHECKSUM=$(shell ./data_struct_checksum.sh)

all: $(NAME).hex

$(NAME).elf: Makefile $(SOURCES:.c=.o) $(LDSCRIPT)
	$(CC) $(CFLAGS) -nostartfiles -nostdlib -T $(LDSCRIPT) -o $@ -L$(TOOLS_PREFIX)/lib/gcc/arm-eabi/4.5.1/ $(SOURCES:.c=.o) -Wa,-Map -Wa,main.map -lgcc

%.o: %.c Makefile data_struct_checksum.sh
	$(CC) $(CFLAGS) -nostartfiles -nostdlib -c -o $@ $< -Wall -std=gnu99 -DDATA_STRUCT_CHECKSUM=$(DATA_STRUCT_CHECKSUM)

run: deploy
	$(FLASHER) -termonly -control $(NAME).hex $(PORT) $(SPEED) $(OSC)

deploy: all $(NAME).hex
	# TODO(aschuh): Figure out why the verify fails (or remove it) and then
	# remove the -.
	-$(FLASHER) -hex -verify -control $(NAME).hex $(PORT) $(SPEED) $(OSC)

reset: deploy
	# Echo an ESC into it to immediately exit the terminal.
	`which echo` -e '\e' | $(FLASHER) -termonly -control $(PORT) $(SPEED) $(OSC)

cat:
	@cd ../../bin; python serial_looper.py

assm.S: $(NAME).elf
	$(OBJDUMP) -D -S $(NAME).elf > $@
# So that you can see the assembly for an individual file with any comments etc.
%.s: %.c Makefile
	$(CC) $(CFLAGS) -nostartfiles -nostdlib -S -o $@ $< -Wall -std=gnu99

%.hex: %.elf Makefile
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(SOURCES:.c=.o) $(NAME).hex $(NAME).elf
