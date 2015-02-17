{
  'targets': [
    {
      'target_name': 'drivetrain_action_queue',
      'type': 'static_library',
      'sources': ['drivetrain_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
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
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/frc971/control_loops/drivetrain/drivetrain.gyp:drivetrain_queue',
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
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'drivetrain_action_queue',
        'drivetrain_action_lib',
      ],
    },

    {
      'target_name': 'fridge_profile_action_queue',
      'type': 'static_library',
      'sources': ['fridge_profile_action.q'],
      'variables': {
        'header_path': 'frc971/actors',
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
      'target_name': 'fridge_profile_action_lib',
      'type': 'static_library',
      'sources': [
        'fridge_profile_actor.cc',
      ],
      'dependencies': [
        'fridge_profile_action_queue',
        '<(DEPTH)/frc971/frc971.gyp:constants',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(EXTERNALS):eigen',
        '<(AOS)/common/util/util.gyp:trapezoid_profile',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'fridge_profile_action_queue',
      ],
    },
    {
      'target_name': 'fridge_profile_action',
      'type': 'executable',
      'sources': [
        'fridge_profile_actor_main.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        'fridge_profile_action_queue',
        'fridge_profile_action_lib',
      ],
    },
    {
      'target_name': 'fridge_profile_action_test',
      'type': 'executable',
      'sources': [
        'fridge_profile_actor_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/linux_code/linux_code.gyp:init',
        '<(AOS)/common/actions/actions.gyp:action_lib',
        '<(DEPTH)/frc971/control_loops/fridge/fridge.gyp:fridge_queue',
        'fridge_profile_action_queue',
        'fridge_profile_action_lib',
      ],
    },
  ],
}
