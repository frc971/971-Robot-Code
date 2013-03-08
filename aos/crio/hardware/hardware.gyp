{
  'targets': [
    {
      'target_name': 'digital_source',
      'type': 'static_library',
      'sources': [
        'digital_source.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
    {
      'target_name': 'counter',
      'type': 'static_library',
      'sources': [
        'counter.cc',
      ],
      'dependencies': [
        'digital_source',
        '<(EXTERNALS):WPILib',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
  ],
}
