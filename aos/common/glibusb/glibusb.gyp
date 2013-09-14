{
  'targets': [
    {
      'target_name': 'gbuffer',
      'type': 'static_library',
      'sources': [
        'gbuffer.cc',
        'ghexdump.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'gbuffer_test',
      'type': 'executable',
      'sources': [
        'gbuffer_test.cc',
      ],
      'dependencies': [
        'gbuffer',
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
    {
      'target_name': 'glibusb',
      'type': 'static_library',
      'sources': [
        # These all have circularish dependencies so it's not really possible to
        # split them out.
        'glibusb.cc',
        'glibusb_device.cc',
        'glibusb_internal.cc',
        'glibusb_transfer.cc',
        'glibusb_endpoint.cc',
      ],
      'dependencies': [
        'gbuffer',
        '<(AOS)/build/aos.gyp:logging',
        '<(EXTERNALS):libusb',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:condition',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):libusb',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:condition',
      ],
    },
    {
      'target_name': 'glibusb_test',
      'type': 'executable',
      'sources': [
        'glibusb_test.cc',
       ],
      'dependencies': [
        'glibusb',
        '<(EXTERNALS):gtest',
      ],
    },
  ],
}
