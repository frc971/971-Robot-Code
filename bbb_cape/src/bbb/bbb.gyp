{
  'targets': [
    {
      'target_name': 'crc',
      'type': 'static_library',
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
      ],
      'sources': [
        'crc.cc',
      ],
    },
  ],
}
