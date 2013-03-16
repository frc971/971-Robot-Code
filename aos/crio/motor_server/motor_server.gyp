{
  'targets': [
    {
      'target_name': 'crio_control_loop_runner',
      'type': 'static_library',
      'sources': [
        #'ControlLoopGoals.h'
        #'crio_control_loop_runner-tmpl.h',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/sensors/sensors.gyp:sensor_broadcaster',
        '<(AOS)/common/sensors/sensors.gyp:sensor_sink',
        'output',
        'MotorServer',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/common.gyp:controls',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/sensors/sensors.gyp:sensor_broadcaster',
        '<(AOS)/common/sensors/sensors.gyp:sensor_sink',
        'output',
        'MotorServer',
      ],
    },
    {
      'target_name': 'output',
      'type': 'static_library',
      'sources': [
        # 'OutputDevice.h',
        # 'SolenoidOutput.h'
        'MotorControllerOutput.cc',
        'MotorOutput.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/common/input/input.gyp:sensor_input',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/common/input/input.gyp:sensor_input',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
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
        '<(AOS)/crio/shared_libs/shared_libs.gyp:ByteBuffer',
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
  ],
}
