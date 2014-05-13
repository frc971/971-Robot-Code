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
        '<(EXTERNALS):stm32flash',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:gpios',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:export_uart',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/libc/libc.gyp:dirname',
      ],
    },
  ],
}
