{
  'targets': [
    {
      'target_name': 'claw_queues',
      'type': 'static_library',
      'sources': ['claw.q'],
      'variables': {
        'header_path': 'frc971/control_loops/claw',
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
