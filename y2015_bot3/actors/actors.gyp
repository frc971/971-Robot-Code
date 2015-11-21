{
  'targets': [
    {
      'target_name': 'binaries',
      'type': 'none',
      'dependencies': [
        'drivetrain_action_bot3',
      ],
    },
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'bot3/actors',
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
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_lib',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
      ],
    },
    {
      'target_name': 'drivetrain_action_bot3',
      'type': 'executable',
      'sources': [
        'drivetrain_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
        'drivetrain_action_lib',
      ],
    },
  ],
}
