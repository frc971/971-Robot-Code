cc_binary(
  name = 'medium_salsa.elf',
  srcs = [
    'medium_salsa.cc',
  ],
  deps = [
    ':util',
    '//motors/core',
    '//motors/usb',
  ],
  restricted_to = ['//tools:cortex-m4f'],
)

genrule(
  name = 'medium_salsa',
  srcs = [
    'medium_salsa.elf',
  ],
  outs = [
    'medium_salsa.hex',
  ],
  cmd = '$(OBJCOPY) -O ihex $<  $@',
  executable = True,
  output_to_bindir = True,
  restricted_to = ['//tools:cortex-m4f'],
)

cc_library(
  name = 'util',
  hdrs = [
    'util.h',
  ],
  restricted_to = ['//tools:cortex-m4f'],
)
