{
  'targets': [
    {
      # This test runs on the atom to verify that the cRIO version of the queues
      # works.
      'target_name': 'unsafe_queue_test',
      'type': '<(aos_target)',
      'sources': [
        'queue_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_test_queue',
      ],
    },
  ],
}
