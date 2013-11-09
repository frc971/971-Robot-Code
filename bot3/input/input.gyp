{
  'targets': [
    {
      'target_name': 'joystick_reader',
      'type': 'executable',
      'sources': [
        'joystick_reader.cc',
      ],
      'dependencies': [
        '<(AOS)/atom_code/input/input.gyp:joystick_input',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',

        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/bot3/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(DEPTH)/bot3/autonomous/autonomous.gyp:auto_queue',
      ],
    },
    {
      'target_name': 'gyro_sensor_receiver',
      'type': 'executable',
      'sources': [
        'gyro_sensor_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_loop',
        '<(DEPTH)/bot3/control_loops/shooter/shooter.gyp:shooter_loop',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(DEPTH)/frc971/input/usb.gyp:usb_receiver',
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
      ],
    },
  ],
}
