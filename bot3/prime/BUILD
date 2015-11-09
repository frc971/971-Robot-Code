package(default_visibility = ['//visibility:public'])

load('/aos/prime/downloader/downloader', 'aos_downloader')

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//bot3/control_loops/drivetrain:drivetrain_bot3',
    '//bot3/control_loops/intake:intake',
    '//bot3:joystick_reader_bot3',
    '//bot3/control_loops/elevator:elevator',
    '//bot3/autonomous:auto_bot3',
    '//bot3/actors:binaries',
    '//bot3/wpilib:wpilib_interface',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
