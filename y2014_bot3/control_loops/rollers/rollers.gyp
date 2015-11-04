{
  'targets': [
    {
      'target_name': 'rollers_loop',
      'type': 'static_library',
      'sources': ['rollers.q'],
      'variables': {
        'header_path': 'bot3/control_loops/rollers',
      },
      'dependencies': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'rollers_lib',
      'type': 'static_library',
      'sources': [
        'rollers.cc',
      ],
      'dependencies': [
        'rollers_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
      'export_dependent_settings': [
        'rollers_loop',
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
    },
    {
      'target_name': 'rollers',
      'type': 'executable',
      'sources': [
        'rollers_main.cc',
      ],
      'dependencies': [
        'rollers_lib',
        '<(AOS)/linux_code/linux_code.gyp:init'
      ],
    },
  ],
}
