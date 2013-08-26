{
  'targets': [
    {
      'target_name': 'netconsole',
      'type': 'executable',
      'sources': [
        'netconsole.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/atom_code/atom_code.gyp:configuration',
        '<(AOS)/common/common.gyp:util',
      ],
    },
    {
      'target_name': 'starter_exe',
      'type': 'executable',
      'sources': [
        'starter.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(EXTERNALS):libevent',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/common.gyp:time',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'starter.sh',
            'starter_loop.sh',
          ],
        },
      ],
    },
  ],
}
