{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        'stm32_flasher',
      ],
    },
    {
      'target_name': 'stm32_flasher',
      'type': 'executable',
      'sources': [
        'stm32_flasher.cc',
      ],
      'dependencies': [
        'stm32flash',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'stm32flash',
      'type': 'static_library',
      'sources': [
        'stm32flash/init.c',
        'stm32flash/parsers/hex.c',
        'stm32flash/serial_common.c',
        'stm32flash/serial_platform.c',
        'stm32flash/utils.c',
        'stm32flash/stm32.c',
      ],
      'cflags': [
        '-Wno-error',
      ],
    },
  ],
}
