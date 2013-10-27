{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Atom',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '<(DEPTH)/bot3/control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        #'<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter_lib_test',
        #'<(DEPTH)/frc971/control_loops/shooter/shooter.gyp:shooter',
        '<(DEPTH)/bot3/autonomous/autonomous.gyp:auto',
        '<(DEPTH)/bot3/input/input.gyp:joystick_reader',
        #'../input/input.gyp:AutoMode',
        '<(DEPTH)/bot3/output/output.gyp:MotorWriter',
        '<(DEPTH)/bot3/output/output.gyp:CameraServer',
        #'camera/camera.gyp:frc971',
        '<(DEPTH)/gyro_board/src/libusb-driver/libusb-driver.gyp:get',
        '<(DEPTH)/bot3/input/input.gyp:gyro_sensor_receiver',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'scripts/start_list.txt',
          ],
        },
      ],
    },
  ],
}
