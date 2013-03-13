{
  'targets': [
    {
      'target_name': 'ControlsManager',
      'type': 'static_library',
      'sources': [
        'ControlsManager.cpp',
        'JoyStickRead.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/crio/motor_server/motor_server.gyp:CRIOControlLoopRunner',
        '<(AOS)/crio/motor_server/motor_server.gyp:MotorServer',
        '<(AOS)/common/network/network.gyp:socket',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
      ],
    },
  ],
}
