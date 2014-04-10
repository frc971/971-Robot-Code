{
  'targets': [
    {
      'target_name': 'polytope',
      'type': 'static_library',
      'sources': [
        #'polytope.h',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(EXTERNALS):libcdd',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(EXTERNALS):libcdd',
      ],
    },
    {
      'target_name': 'sensor_generation',
      'type': 'static_library',
      'sources': [
        'sensor_generation.q',
      ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'output_check',
      'type': 'static_library',
      'sources': [
        'output_check.q',
      ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'includes': ['../../../aos/build/queues.gypi'],
    },
  ],
}
