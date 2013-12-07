OBJDIR := .obj/
$(shell mkdir -p $(OBJDIR))

CROSS_COMPILE := /opt/cortex-m3/bin/arm-eabi-

CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump
AS := $(CROSS_COMPILE)as

CPPFLAGS := -I.. -ICMSIS \

CFLAGS := -nostartfiles -nostdlib \
	-O3 -mcpu=cortex-m3 \
	-mthumb -Wl,--gc-sections -ffunction-sections -Wl,-static \
	-Wall -Werror --std=gnu99 \

LDFLAGS := -O3 -mcpu-cortex-m3 \
	-nostartfiles -nostdlib \
	-L$(TOOLS_PREFIX)/lib/gcc/arm-eabi/4.5.1/ \
	-T gcc_arm.ld \

ASFLAGS := -O3 -mcpu=cortex-m3 \
	-nostartfiles -nostdlib \
	-mthumb \

OBJECTS_main := main \

OBJECTS_bootloader := bootloader \

OUTPUTS := main bootloader

OBJECTS := $(foreach output,$(OUTPUTS),$(OBJECTS_$(output)))

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
	$(CC) $(CPPFLAGS) $(CFLAGS) -MD -MP -c -o $@ $<
$(OBJDIR)%.o: %.S
	$(CC) $(CPPFLAGS) $(ASFLAGS) -MD -MP -c -o $@ $<

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
