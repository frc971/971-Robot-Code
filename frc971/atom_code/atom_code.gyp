{
  'targets': [
    {
      'target_name': 'All',
      'type': 'none',
      'dependencies': [
        '<(AOS)/build/aos_all.gyp:Atom',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain',
        '../control_loops/drivetrain/drivetrain.gyp:drivetrain_lib_test',
        '../control_loops/wrist/wrist.gyp:wrist',
        '../control_loops/wrist/wrist.gyp:wrist_lib_test',
        '../control_loops/index/index.gyp:index',
        '../control_loops/index/index.gyp:index_lib_test',
        '../control_loops/angle_adjust/angle_adjust.gyp:angle_adjust',
        '../control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_lib_test',
        '../control_loops/angle_adjust/angle_adjust.gyp:angle_adjust_csv',
        '../control_loops/shooter/shooter.gyp:shooter_lib_test',
        '../control_loops/shooter/shooter.gyp:shooter',
        '../input/input.gyp:JoystickReader',
        '../input/input.gyp:SensorReader',
        '../input/input.gyp:GyroReader',
        '../input/input.gyp:AutoMode',
        '../output/output.gyp:MotorWriter',
        '../output/output.gyp:CameraServer',
        'camera/camera.gyp:frc971',
      ],
      'copies': [
        {
          'destination': '<(rsync_dir)',
          'files': [
            'scripts/aos_module.ko',
            'scripts/start_list.txt',
          ],
        },
      ],
    },
  ],
}
