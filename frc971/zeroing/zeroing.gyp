{
  'targets': [
    {
      'target_name': 'zeroing_queue',
      'type': 'static_library',
      'sources': [
        '<(DEPTH)/frc971/zeroing/zeroing_queue.q',
      ],
      'variables': {
        'header_path': 'frc971/zeroing',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'zeroing',
      'type': 'static_library',
      'sources': [
        'zeroing.cc',
      ],
      'dependencies': [
        'zeroing_queue',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:queues',
        '<(DEPTH)/frc971/frc971.gyp:constants',
      ],
      'export_dependent_settings': [
        'zeroing_queue',
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
        'zeroing_queue',
        '<(AOS)/common/util/util.gyp:thread',
        '<(AOS)/common/common.gyp:die',
      ],
    },
  ],
}
