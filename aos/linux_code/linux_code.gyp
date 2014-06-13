{
  'targets': [
    {
      'target_name': 'init',
      'type': 'static_library',
      'sources': [
        'init.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'configuration',
      'type': 'static_library',
      'sources': [
        'configuration.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'core',
      'type': 'executable',
      'sources': [
        'core.cc',
      ],
      'dependencies': [
        'init',
        '<(AOS)/common/util/util.gyp:run_command',
      ],
    },
  ],
}
