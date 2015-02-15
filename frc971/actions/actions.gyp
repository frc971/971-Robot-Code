{
  'targets': [
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'frc971/actions',
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
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        'drivetrain_action_queue',
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
        'drivetrain_action_queue',
        'drivetrain_action_lib',
      ],
    },
  ],
}
