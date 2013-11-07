{
  'targets': [
    {
      'target_name': 'MotorWriter',
      'type': '<(aos_target)',
      'sources': [
        'atom_motor_writer.cc'
      ],
      'dependencies': [
        '<(AOS)/atom_code/output/output.gyp:motor_output',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/bot3/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/common/common.gyp:controls',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
