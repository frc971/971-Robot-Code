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
    {
      'target_name': 'uart_receiver',
      'type': 'static_library',
      'dependencies': [
        'crc',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'sources': [
        'uart_receiver.cc',
      ],
    },
  ],
}
