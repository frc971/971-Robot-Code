{
  'targets': [
    {
# The WPILib code that we've modified.
      'target_name': 'WPILib_changes',
      'type': 'static_library',
      'sources': [
        '<!@(find <(AOS)/externals/WPILib/WPILib/ -name *.cpp)',
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
        '<(EXTERNALS):libgcc-4.5.2',
      ],
      'cflags!': ['-Werror', '-ggdb3', '-O0'],
      'cflags': ['-ggdb1', '-O3'],
    },
    {
      'target_name': 'user_program',
      'type': 'static_library',
      'sources': [
        'main.cc',
      ],
      'dependencies': [
        '<(AOS)/crio/motor_server/motor_server.gyp:MotorServer',
        '../output/output.gyp:MotorWriter',
        'WPILib_changes',
        '<(EXTERNALS):WPILib',
        '<(AOS)/crio/controls/controls.gyp:ControlsManager',
        '<(AOS)/crio/shared_libs/shared_libs.gyp:interrupt_notifier',
        #'<(AOS)/crio/motor_server/motor_server.gyp:crio_control_loop_runner',
        #'<(AOS)/common/sensors/sensors.gyp:sensor_broadcaster',
        #'<(DEPTH)/frc971/input/input.gyp:sensor_packer',
        #'<(DEPTH)/frc971/input/input.gyp:sensor_unpacker',
      ],
    },
    {
      'target_name': 'FRC_UserProgram',
      'type': 'shared_library',
      'dependencies': [
        'user_program'
      ],
    },
    {
      'target_name': 'FRC_UserProgram_WithTests',
      'type': 'shared_library',
      'dependencies': [
        # For testing.
        '<(AOS)/build/aos_all.gyp:Crio',
      ],
    },
  ],
}
