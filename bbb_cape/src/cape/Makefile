OBJDIR := .obj/
$(shell mkdir -p $(OBJDIR))

ifeq ($(OS),Windows_NT)
  CROSS_COMPILE := arm-none-eabi-
else
  CROSS_COMPILE := /opt/cortex-m3/bin/arm-eabi-
endif

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump
AS := $(CROSS_COMPILE)as

CPPFLAGS := -I.. -ICMSIS \

CFLAGS := -nostartfiles -nostdlib -ffreestanding -fbuiltin \
	-O3 -mcpu=cortex-m3 \
	-mthumb -Wl,--gc-sections -ffunction-sections -Wl,-static \
	-Wall -Werror --std=gnu99 \
	-Wstrict-aliasing=2 -Wcast-qual -Wpointer-arith \

LDFLAGS := -O3 -mcpu=cortex-m3 \
	-nostartfiles -nostdlib \
	-T gcc_arm.ld \

ASFLAGS := -O3 -mcpu=cortex-m3 \
	-nostartfiles -nostdlib \
	-mthumb \

OBJECTS_main_common := main \
	uart_common \
	uart_dma \
	uart \
	fill_packet \
	cows \
	encoder \
	crc \
	gyro \
	led \
	analog \
	digital \

OBJECTS_bootloader := bootloader \
	uart_common \
	uart_byte \
	led \

OBJECTS_main_test := $(OBJECTS_main_common) \
	robot_test \

OUTPUTS := main_test bootloader

# The sort is to remove duplicates because Make warns about those.
OBJECTS := $(sort $(foreach output,$(OUTPUTS),$(OBJECTS_$(output))))

OUTPUTS_elf := $(OUTPUTS:%=$(OBJDIR)%.elf)
OUTPUTS_hex := $(OUTPUTS:%=$(OBJDIR)%.hex)

.PHONY: all
all: $(OUTPUTS_hex)

-include $(OBJECTS:%=$(OBJDIR)%.d)

.SECONDEXPANSION:
PERCENT := %

# "$(OBJDIR)%.elf: $(OBJECTS_%:%=$(OBJDIR)%.o)" with the % signs meaning the
# right thing in the right places.
$(OUTPUTS_elf): $(OBJDIR)%.elf: $$(OBJECTS_$$*:$$(PERCENT)=$(OBJDIR)$$(PERCENT).o)
$(OUTPUTS_elf): $(OBJDIR)%.elf:	gcc_arm.ld %.ld $(OBJDIR)STM32F2XX_startup.o
	$(CC) $(CPPFLAGS) -T $*.ld $(LDFLAGS) -o $@ \
		$(OBJDIR)STM32F2XX_startup.o \
		$(OBJECTS_$*:%=$(OBJDIR)%.o) \
		-Wa,-Map -Wa,$(OBJDIR)$*.map

$(OBJECTS:%=$(OBJDIR)%.o): $(OBJDIR)%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -MD -MP -MT '$@ $*.s' -c -o $@ $<
$(OBJDIR)%.o: %.S
	$(CC) $(CPPFLAGS) $(ASFLAGS) -MD -MP -MT '$@ $*.s' -c -o $@ $<

# So that you can see the assembly for an individual file with any comments etc.
$(OBJECTS:%=%.s): %.s: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -S -o $@ $<
%.s: %.S
	$(CC) $(CPPFLAGS) $(ASFLAGS) -S -o $@ $<

# So that you can see the assembly of the whole .elf file.
$(OUTPUTS:%=elf_%.s): elf_%.s: $(OBJDIR)%.elf
	$(OBJDUMP) -d -S $< > $@

$(OUTPUTS_hex): $(OBJDIR)%.hex: $(OBJDIR)%.elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(OBJDIR)
