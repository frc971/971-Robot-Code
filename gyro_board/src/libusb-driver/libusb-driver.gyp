{
  'targets': [
    {
      'target_name': 'get',
      'type': 'executable',
      'sources': [
        'get.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gflags',
        '<(AOS)/common/util/util.gyp:thread',
        'libusb_wrap',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'libusb_wrap',
      'type': 'static_library',
      'sources': [
        'libusb_wrap.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):libusb',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):libusb',
      ],
    },
  ],
}
