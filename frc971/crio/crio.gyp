{
  'targets': [
    {
      'target_name': 'dumb_main',
      'type': 'static_library',
      'sources': [
        'dumb_main.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/crio/crio.gyp:ip',
        '<(AOS)/common/util/util.gyp:inet_addr',
      ],
    },
    {
      'target_name': 'FRC_UserProgram',
      'type': 'shared_library',
      'dependencies': [
        '<(EXTERNALS):libgcc-4.5.2',
        'dumb_main',
      ],
    },
  ],
}
