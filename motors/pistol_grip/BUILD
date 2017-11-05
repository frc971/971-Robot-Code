load('//motors:macros.bzl', 'hex_from_elf')
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
  name = 'drivers_station.elf',
  srcs = [
    'drivers_station.cc',
  ],
  deps = [
    '//motors:util',
    '//motors/core',
    '//motors/usb',
    '//motors/usb:cdc',
    '//motors/usb:hid',
  ],
  restricted_to = mcu_cpus,
)

hex_from_elf(
  name = 'drivers_station',
  restricted_to = mcu_cpus,
)
