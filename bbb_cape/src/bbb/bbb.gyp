{
  'target_defaults': {
    'include_dirs': [
      '..',
    ],
    'direct_dependent_settings': {
      'include_dirs': [
        '..',
      ],
    },
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
      'target_name': 'sensor_generation',
      'type': 'static_library',
      'sources': [
        'sensor_generation.q',
      ],
      'variables': {
        'header_path': 'bbb',
      },
      'includes': ['../../../aos/build/queues.gypi'],
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
        '<(AOS)/linux_code/linux_code.gyp:init',
        'packet_finder',
        'data_struct',
        '<(AOS)/common/common.gyp:time',
        'gpios',
      ],
    },
    {
      'target_name': 'gpios',
      'type': 'static_library',
      'sources': [
        'gpios.cc',
        'gpi.cc',
        'gpo.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'sensor_reader',
      'type': 'static_library',
      'sources': [
        'sensor_reader.cc',
      ],
      'dependencies': [
        'uart_reader',
        'packet_finder',
        'data_struct',
        '<(AOS)/common/common.gyp:time',
        'sensor_generation',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
        'crc',
        '<(EXTERNALS):stm32flash',
      ],
      'export_dependent_settings': [
        'uart_reader',
        'packet_finder',
        'data_struct',
        '<(AOS)/common/common.gyp:time',
      ],
    },
  ],
}
