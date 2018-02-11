load('//motors:macros.bzl', 'hex_from_elf')
load("//tools:environments.bzl", "mcu_cpus")

cc_binary(
  name = 'drivers_station.elf',
  srcs = [
    'drivers_station.cc',
  ],
  deps = [
    '//motors:util',
    '//motors/peripheral:can',
    '//motors/core',
    '//motors/usb',
    '//motors/usb:cdc',
    '//motors/usb:hid',
    '//motors/usb:interrupt_out',
  ],
  restricted_to = mcu_cpus,
)

hex_from_elf(
  name = 'drivers_station',
  restricted_to = mcu_cpus,
)

cc_binary(
  name = 'controller.elf',
  srcs = [
    'vtable_wheel.cc',
    'vtable_trigger.cc',
    'controller.cc',
  ],
  deps = [
    ':motor_controls',
    '//motors:util',
    '//motors:motor',
    '//motors/core',
    '//motors/peripheral:can',
    '//motors/peripheral:adc',
    '//motors/usb',
    '//motors/usb:cdc',
    '//frc971/control_loops/drivetrain:haptic_input_uc',
  ],
  restricted_to = mcu_cpus,
)

hex_from_elf(
  name = 'controller',
  restricted_to = mcu_cpus,
)

cc_binary(
  name = 'usb_forward_linux',
  srcs = [
    'usb_forward.cc',
  ],
  deps = [
    # Don't add anything else here. :usb_forward_windows still has to build it
    # without any other dependencies.
    '@libusb_1_0',
  ],
  restricted_to = ['//tools:k8'],
)

genrule(
  name = 'usb_forward_windows',
  outs = [
    'usb_forward.exe',
  ],
  srcs = [
    'usb_forward.cc',
    '@libusb_1_0_windows//file',
  ],
  tools = [
    'usb_forward_windows_build.sh',
  ],
  cmd = ' '.join([
    '$(location usb_forward_windows_build.sh)',
    '$(location usb_forward.cc)',
    '$(location @libusb_1_0_windows//file)',
    '$@',
  ]),
  output_to_bindir = True,
)

cc_library(
  name = 'motor_controls',
  visibility = ['//visibility:public'],
  hdrs = [
    'motor_controls.h',
  ],
  srcs = [
    'motor_controls.cc',
  ],
  deps = [
    '//motors:math',
    '//motors:motor',
    '//motors/peripheral:configuration',
    '//third_party/eigen',
  ],
  restricted_to = mcu_cpus,
)
