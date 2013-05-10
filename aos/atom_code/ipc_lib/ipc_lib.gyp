{
  'targets': [
    {
      'target_name': 'ipc_lib',
      'type': 'static_library',
      'sources': [
        'aos_sync.c',
        'binheap.c',
        'core_lib.c',
        'queue.c',
        'shared_mem.c',
      ],
      'dependencies': [
        # TODO(brians): fix this once there's a nice logging interface to use
        # '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'binheap_test',
      'type': 'executable',
      'sources': [
        'binheap_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'ipc_lib',
      ],
    },
    {
      'target_name': 'ipc_queue_test',
      'type': 'executable',
      'sources': [
        'queue_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'ipc_lib',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
