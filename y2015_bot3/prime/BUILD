package(default_visibility = ['//visibility:public'])

load('/aos/prime/downloader/downloader', 'aos_downloader')

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//y2015_bot3/control_loops/drivetrain:drivetrain',
    '//y2015_bot3/control_loops/intake:intake',
    '//y2015_bot3:joystick_reader',
    '//y2015_bot3/control_loops/elevator:elevator',
    '//y2015_bot3/autonomous:auto',
    '//y2015_bot3/actors:binaries',
    '//y2015_bot3/wpilib:wpilib_interface',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
