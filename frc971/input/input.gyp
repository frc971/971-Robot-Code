{
  'targets': [
    {
      'target_name': 'hot_goal_reader',
      'type': 'executable',
      'sources': [
        'hot_goal_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(DEPTH)/frc971/queues/queues.gyp:hot_goal',
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
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/actions/actions.gyp:shoot_action_queue',
        '<(DEPTH)/frc971/actions/actions.gyp:action_client',
        '<(DEPTH)/frc971/actions/actions.gyp:catch_action_queue',
        '<(DEPTH)/frc971/actions/actions.gyp:shoot_action_lib',
      ],
    },
    {
      'target_name': 'sensor_receiver',
      'type': 'executable',
      'sources': [
        'sensor_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:sensor_reader',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(DEPTH)/frc971/control_loops/claw/claw.gyp:claw_loop',
        '<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/common/controls/controls.gyp:output_check',
        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
      ],
    },
  ],
}
