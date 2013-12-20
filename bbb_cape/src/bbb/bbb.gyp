{
  'target_defaults': {
    'include_dirs': [
      '..',
    ],
  },
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
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
      ],
      'sources': [
        'uart_receiver.cc',
      ],
    },
    {
      'target_name': 'uart_receiver_test',
      'type': 'executable',
      'dependencies': [
        'uart_receiver',
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'sources': [
        'uart_receiver_test.cc',
      ],
    },
    {
      'target_name': 'uart_receiver_main',
      'type': 'executable',
      'dependencies': [
        'uart_receiver',
        'gpios',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
      ],
      'sources': [
        'uart_receiver_main.cc',
      ],
    },
    {
      'target_name': 'gpios',
      'type': 'static_library',
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'sources': [
        'gpios.cc',
      ],
    },
  ],
}
