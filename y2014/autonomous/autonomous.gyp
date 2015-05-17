{
  'targets': [
    {
      'target_name': 'auto_lib',
      'type': 'static_library',
      'sources': [
        'auto.cc',
      ],
      'dependencies': [
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(DEPTH)/y2014/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/y2014/control_loops/shooter/shooter.gyp:shooter_queue',
        '<(DEPTH)/y2014/control_loops/claw/claw.gyp:claw_queue',
        '<(DEPTH)/y2014/y2014.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/y2014/actors/actors.gyp:shoot_action_lib',
        '<(DEPTH)/y2014/actors/actors.gyp:drivetrain_action_lib',
        '<(DEPTH)/y2014/queues/queues.gyp:hot_goal',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(DEPTH)/y2014/queues/queues.gyp:profile_params',
        '<(DEPTH)/y2014/queues/queues.gyp:auto_mode',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/controls/controls.gyp:control_loop',
      ],
    },
    {
      'target_name': 'auto',
      'type': 'executable',
      'sources': [
        'auto_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(DEPTH)/frc971/autonomous/autonomous.gyp:auto_queue',
        'auto_lib',
      ],
    },
  ],
}
