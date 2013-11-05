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
    }
  ],
}
