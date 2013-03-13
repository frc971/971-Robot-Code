{
  'targets': [
    {
      'target_name': 'CRIOControlLoopRunner',
      'type': 'static_library',
      'sources': [
        # 'ControlLoopGoals.h'
        'CRIOControlLoopRunner.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
        'output',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/common.gyp:controls',
      ],
    },
    {
      'target_name': 'output',
      'type': 'static_library',
      'sources': [
        # 'OutputDevice.h',
        # 'SensorSender-tmpl.h',
        'SensorOutput.cc',
        # 'SolenoidOutput.h'
        'MotorControllerOutput.cc',
        'MotorOutput.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/common/input/input.gyp:sensor_input',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/common/input/input.gyp:sensor_input',
      ],
    },
    {
      'target_name': 'MotorServer',
      'type': 'static_library',
      'sources': [
        'MotorServer.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/messages/messages.gyp:QueueHolder',
        '<(AOS)/common/network/network.gyp:socket',
        'CRIOControlLoopRunner',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:ByteBuffer',
        'output',
        '<(AOS)/common/network/network.gyp:socket',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/common/common.gyp:mutex',
        '<(AOS)/common/messages/messages.gyp:QueueHolder',
        '<(AOS)/common/network/network.gyp:socket',
        'CRIOControlLoopRunner',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:ByteBuffer',
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
  ],
}
