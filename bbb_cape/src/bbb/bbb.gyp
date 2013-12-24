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
        'crc',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:data_struct',
      ],
      'sources': [
        'uart_reader.cc',
        'packet_finder.cc',
        'uart_reader_termios2.c',
      ],
    },
    #{
    #  'target_name': 'uart_reader_test',
    #  'type': 'executable',
    #  'dependencies': [
    #    'uart_reader',
    #    '<(EXTERNALS):gtest',
    #    '<(AOS)/build/aos.gyp:logging',
    #  ],
    #  'sources': [
    #    'uart_reader_test.cc',
    #  ],
    #},
    {
      'target_name': 'uart_reader_main',
      'type': 'executable',
      'dependencies': [
        'uart_reader',
        'gpios',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:init',
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
