{
  'targets': [
    {
      'target_name': 'starter_exe',
      'type': 'executable',
      'sources': [
        'starter.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
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
