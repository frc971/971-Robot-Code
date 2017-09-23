load('//motors:macros.bzl', 'hex_from_elf')
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
  name = 'medium_salsa.elf',
  srcs = [
    'medium_salsa.cc',
  ],
  deps = [
    ':util',
    '//motors/core',
    '//motors/usb:legacy',
  ],
  restricted_to = mcu_cpus,
)

hex_from_elf(
  name = 'medium_salsa',
  restricted_to = mcu_cpus,
)

cc_library(
  name = 'util',
  visibility = ['//visibility:public'],
  hdrs = [
    'util.h',
  ],
  restricted_to = mcu_cpus,
)
