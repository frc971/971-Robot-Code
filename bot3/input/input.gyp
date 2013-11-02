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
        '<(DEPTH)/frc971/queues/queues.gyp:queues',
        '<(AOS)/atom_code/atom_code.gyp:init',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        'usb_receiver',
      ],
    },
    {
      'target_name': 'usb_receiver',
      'type': 'static_library',
      'sources': [
        '<(DEPTH)/frc971/input/usb_receiver.cc',
      ],
      'dependencies': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:controls',
      ],
      'export_dependent_settings': [
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:libusb_wrap',
        '<(AOS)/common/util/util.gyp:wrapping_counter',
        '<(AOS)/common/common.gyp:time',
      ],
    },
  ],
}
