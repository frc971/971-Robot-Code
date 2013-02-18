{
  'targets': [
    {
      'target_name': 'starter_exe',
      'type': 'executable',
      'sources': [
        'starter.cpp',
      ],
      'dependencies': [
        '<(AOS)/atom_code/atom_code.gyp:init',
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
