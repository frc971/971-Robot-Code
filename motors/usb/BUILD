load("//tools:environments.bzl", "mcu_cpus")

cc_library(
  name = 'legacy',
  visibility = ['//visibility:public'],
  hdrs = [
    'usb_desc.h',
    'usb_serial.h',
    'usb_dev.h',
  ],
  srcs = [
    'usb_mem.h',
    'usb_names.h',

    'usb_desc.c',
    'usb_serial.c',
    'usb_dev.c',
    'usb_mem.c',
  ],
  defines = [
    'USB_SERIAL=1',
  ],
  deps = [
    '//motors/core',
  ],
  restricted_to = mcu_cpus,
)

cc_library(
  name = 'usb',
  visibility = ['//visibility:public'],
  hdrs = [
    'usb.h',
  ],
  srcs = [
    'usb.cc',
  ],
  deps = [
    ':constants',
    '//aos/common:macros',
    '//motors/core',
    '//motors:util',
  ],
  restricted_to = mcu_cpus,
)

cc_library(
  name = 'cdc',
  visibility = ['//visibility:public'],
  hdrs = [
    'cdc.h',
  ],
  srcs = [
    'cdc.cc',
  ],
  deps = [
    ':usb',
    ':queue',
    '//motors/core',
    '//motors:util',
  ],
  restricted_to = mcu_cpus,
)

cc_library(
  name = 'queue',
  hdrs = [
    'queue.h',
  ],
  srcs = [
    'queue.cc',
  ],
  compatible_with = mcu_cpus,
)

cc_test(
  name = 'queue_test',
  srcs = [
    'queue_test.cc',
  ],
  deps = [
    ':queue',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'constants',
  hdrs = [
    'constants.h',
  ],
  compatible_with = mcu_cpus,
)

cc_test(
  name = 'constants_test',
  srcs = [
    'constants_test.cc',
  ],
  deps = [
    ':constants',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'hid',
  visibility = ['//visibility:public'],
  hdrs = [
    'hid.h',
  ],
  srcs = [
    'hid.cc',
  ],
  deps = [
    ':usb',
    '//motors:util',
  ],
  restricted_to = mcu_cpus,
)
