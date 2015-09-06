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
        '<(DEPTH)/y2015/control_loops/drivetrain/drivetrain.gyp:polydrivetrain_plants',
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
        '<(AOS)/common/actions/actions.gyp:action_lib',

        '<(DEPTH)/frc971/queues/queues.gyp:gyro',
        '<(DEPTH)/y2015/control_loops/claw/claw.gyp:claw_queue',
        '<(DEPTH)/y2015/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/y2015/control_loops/fridge/fridge.gyp:fridge_queue',
        '<(DEPTH)/y2015/y2015.gyp:constants',
        '<(DEPTH)/y2015/autonomous/autonomous.gyp:auto_queue',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        '<(DEPTH)/y2015/actors/actors.gyp:stack_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:stack_and_lift_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:stack_and_hold_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:pickup_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:lift_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:held_to_lift_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:can_pickup_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:score_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:horizontal_can_pickup_action_lib',
        '<(DEPTH)/y2015/actors/actors.gyp:fridge_profile_lib',
      ],
    },
  ],
}
