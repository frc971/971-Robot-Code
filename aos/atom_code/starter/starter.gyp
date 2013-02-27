{
  'targets': [
    {
      'target_name': 'starter_exe',
      'type': 'executable',
      'sources': [
        'starter.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(EXTERNALS):libevent',
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
