{
  'targets': [
    {
# The WPILib code that we've modified.
      'target_name': 'WPILib_changes',
      'type': 'static_library',
      'sources': [
      ],
      'dependencies': [
        '<(EXTERNALS):WPILib',
      ],
      'cflags!': ['-Werror'],
    },
    {
      'target_name': 'user_program',
      'type': 'static_library',
      'sources': [
        'main.cc',
      ],
      'dependencies': [
        '../input/input.gyp:SensorReader',
        '../input/input.gyp:SensorWriter',
        '../output/output.gyp:MotorWriter',
        '../output/output.gyp:SensorSender',
        'WPILib_changes',
        '<(EXTERNALS):WPILib',
        '<(AOS)/common/messages/messages.gyp:aos_queues',
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
        'user_program',
        # For testing.
        '<(AOS)/build/aos_all.gyp:Crio',
      ],
    },
  ],
}
