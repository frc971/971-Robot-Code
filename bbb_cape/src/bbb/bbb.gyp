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
      'sources': [
        'crc.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
      ],
    },
    {
      'target_name': 'byte_reader',
      'type': 'static_library',
      'sources': [
        # 'byte_reader.h',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'uart_reader',
      'type': 'static_library',
      'sources': [
        'uart_reader.cc',
        'uart_reader_termios2.c',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        'byte_reader',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
        'byte_reader',
      ],
    },
    {
      'target_name': 'cows_test',
      'type': 'executable',
      'sources': [
        'cows_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
      ],
    },
    {
      'target_name': 'packet_finder_test',
      'type': 'executable',
      'sources': [
        'packet_finder_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'packet_finder',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/common.gyp:time',
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
        '<(AOS)/common/common.gyp:time',
        'byte_reader',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        'byte_reader',
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
      'sources': [
        'uart_reader_main.cc',
      ],
      'dependencies': [
        'uart_reader',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
        'packet_finder',
        'data_struct',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'gpios',
      'type': 'static_library',
      'sources': [
        'gpios.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
