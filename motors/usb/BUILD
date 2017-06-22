cc_library(
  name = 'usb',
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
)
