{
  'targets': [
    {
      'target_name': 'constants',
      'type': 'static_library',
      'sources': [
        'constants.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/network/network.gyp:team_number',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:polydrivetrain_plants',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:state_feedback_loop',
      ],
    },
    {
      'target_name': 'joystick_reader',
      'type': 'executable',
      'sources': [
        'joystick_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/prime/input/input.gyp:joystick_input',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:log_interval',

        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        '<(DEPTH)/frc971/actions/actions.gyp:action_client',
      ],
    },
  ],
}
