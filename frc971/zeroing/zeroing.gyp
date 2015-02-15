{
  'targets': [
    {
      'target_name': 'zeroing',
      'type': 'static_library',
      'sources': [
        'zeroing.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
        '<(DEPTH)/frc971/frc971.gyp:constants',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
    },
    {
      'target_name': 'zeroing_test',
      'type': 'executable',
      'sources': [
        'zeroing_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
        'zeroing',
        '<(AOS)/common/util/util.gyp:thread',
        '<(AOS)/common/common.gyp:die',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:position_sensor_sim',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
      ],
    },
  ],
}
