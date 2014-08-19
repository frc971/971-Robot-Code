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
      'target_name': 'led',
      'type': 'static_library',
      'sources': [
        'led.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'crc',
      'type': 'static_library',
      'sources': [
        'crc.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/build/aos.gyp:logging',
        'byte_io',
      ],
    },
    {
      'target_name': 'byte_io',
      'type': 'static_library',
      'sources': [
        # 'byte_io.h',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'export_uart',
      'type': 'static_library',
      'sources': [
        'export_uart.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:run_command',
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
        'byte_io',
        'export_uart',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
        'byte_io',
      ],
    },
    {
      'target_name': 'all_tests',
      'type': 'none',
      'variables': {
        'no_rsync': 1,
      },
      'dependencies': [
        'cows_test',
        'packet_finder_test',
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
        'byte_io',
      ],
    },
    {
      'target_name': 'packet_finder',
      'type': 'static_library',
      'sources': [
        'packet_finder.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(DEPTH)/bbb_cape/src/cape/cape.gyp:cows',
        'crc',
        'byte_io',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/util/util.gyp:run_command',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
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
        'packet_finder',
        'data_struct',
        'cape_manager',
        '<(AOS)/common/common.gyp:time',
        'hex_byte_reader',
        'crc',
        '<(AOS)/common/controls/controls.gyp:sensor_generation',
        '<(AOS)/linux_code/linux_code.gyp:configuration',
        '<(AOS)/common/util/util.gyp:log_interval',
      ],
      'export_dependent_settings': [
        'packet_finder',
        'data_struct',
        'cape_manager',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',
      ],
    },
    {
      'target_name': 'cape_flasher',
      'type': 'static_library',
      'sources': [
        'cape_flasher.cc',
      ],
      'dependencies': [
        'byte_io',
        'crc',
      ],
    },
    {
      'target_name': 'hex_byte_reader',
      'type': 'static_library',
      'sources': [
        'hex_byte_reader.cc',
      ],
      'dependencies': [
        'byte_io',
        '<(AOS)/common/common.gyp:time',
        '<(EXTERNALS):stm32flash',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        'byte_io',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'cape_manager',
      'type': 'static_library',
      'sources': [
        'cape_manager.cc',
      ],
      'dependencies': [
        'gpios',
        'uart_reader',
        'cape_flasher',
        '<(AOS)/common/common.gyp:time',
        'hex_byte_reader',
      ],
      'export_dependent_settings': [
        'gpios',
        'uart_reader',
      ],
    },
    {
      'target_name': 'test_sensor_receiver',
      'type': 'executable',
      'sources': [
        'test_sensor_receiver.cc',
      ],
      'dependencies': [
        'sensor_reader',
        '<(AOS)/linux_code/linux_code.gyp:init',
      ],
    },
  ],
}
