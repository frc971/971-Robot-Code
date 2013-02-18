{
  'targets': [
    {
      'target_name': 'init',
      'type': 'static_library',
      'sources': [
        '<(AOS)/atom_code/init.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
  ],
}
