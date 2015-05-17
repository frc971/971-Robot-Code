{
  'targets': [
    {
      'target_name': 'binaries',
      'type': 'none',
      'dependencies': [
        'drivetrain_action',
        'shoot_action',
      ],
    },
    {
      'target_name': 'shoot_action_queue',
      'type': 'static_library',
      'sources': ['shoot_action.q'],
      'variables': {
        'header_path': 'y2014/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'shoot_action_lib',
      'type': 'static_library',
      'sources': [
        'shoot_actor.cc',
      ],
      'dependencies': [
        'shoot_action_queue',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/y2014/queues/queues.gyp:profile_params',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/y2014/control_loops/shooter/shooter.gyp:shooter_queue',
        '<(DEPTH)/y2014/control_loops/claw/claw.gyp:claw_queue',
        '<(DEPTH)/y2014/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/y2014/y2014.gyp:constants',
      ],
      'export_dependent_settings': [
        'shoot_action_queue',
        '<(AOS)/common/actions/actions.gyp:action_lib',
      ],
    },
    {
      'target_name': 'shoot_action',
      'type': 'executable',
      'sources': [
        'shoot_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'shoot_action_lib',
        'shoot_action_queue',
      ],
    },
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'y2014/actors',
      },
      'dependencies': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_queue',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'drivetrain_action_lib',
      'type': 'static_library',
      'sources': [
        'drivetrain_actor.cc',
      ],
      'dependencies': [
        'drivetrain_action_queue',
        '<(DEPTH)/y2014/y2014.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/y2014/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
      ],
    },
    {
      'target_name': 'drivetrain_action',
      'type': 'executable',
      'sources': [
        'drivetrain_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        'drivetrain_action_lib',
        'drivetrain_action_queue',
      ],
    },
  ],
}
