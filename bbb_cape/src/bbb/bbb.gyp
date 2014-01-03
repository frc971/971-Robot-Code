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
      'target_name': 'uart_reader',
      'type': 'static_library',
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'sources': [
        'uart_reader.cc',
        'uart_reader_termios2.c',
      ],
    },
    {
      'target_name': 'cows_test',
      'type': 'executable',
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
      ],
      'sources': [
        'cows_test.cc',
      ],
    },
    {
      'target_name': 'packet_finder_test',
      'type': 'executable',
      'dependencies': [
        '<(EXTERNALS):gtest',
        'packet_finder',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
      'sources': [
        'packet_finder_test.cc',
      ],
    },
    {
      'target_name': 'packet_finder',
      'type': 'static_library',
      'sources': [
        'packet_finder.cc',
      ],
      'dependencies': [
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
        '<(AOS)/build/aos.gyp:logging',
        'crc',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'data_struct',
      'type': 'static_library',
      'sources': [
        # 'data_struct.h',
      ],
      'dependencies': [
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
      ],
    },
    {
      'target_name': 'uart_reader_main',
      'type': 'executable',
      'dependencies': [
        'uart_reader',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
        'packet_finder',
        'data_struct',
      ],
      'sources': [
        'uart_reader_main.cc',
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
