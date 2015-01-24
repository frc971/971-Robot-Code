{
  'targets': [
    {
      'target_name': 'fridge_queues',
      'type': 'static_library',
      'sources': ['fridge.q'],
      'variables': {
        'header_path': 'frc971/control_loops/fridge',
      },
      'dependencies': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop_queues',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',

      ],
      'includes': ['../../../aos/build/queues.gypi'],
    },
  ],
}
