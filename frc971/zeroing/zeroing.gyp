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
