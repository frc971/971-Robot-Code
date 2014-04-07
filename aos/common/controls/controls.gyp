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
  ],
}
