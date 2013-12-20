{
  'target_defaults': {
    'include_dirs': [
      '..',
    ],
  },
  'targets': [
    {
      'target_name': 'cows',
      'type': 'static_library',
      'sources': [
        'cows.c',
      ],
    },
    {
      'target_name': 'data_struct',
      'type': 'static_library',
    },
  ],
}
