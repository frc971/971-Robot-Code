# This file is needed because gyp wants to drag in all of the targets in a file
# which is a problem because the main and bot3 code contain some executables
# with the same names.
{
  'targets': [
    {
      'target_name': 'usb_receiver',
      'type': 'static_library',
      'sources': [
        'usb_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:controls',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/common/common.gyp:time',
      ],
      'variables': {
        # TODO(brians): Add dependency on this file too (or something).
        'checksum': '<!(<(DEPTH)/gyro_board/src/usb/data_struct_checksum.sh)',
      },
      'defines': [
        'GYRO_BOARD_DATA_CHECKSUM=<(checksum)',
      ],
    },
  ],
}
