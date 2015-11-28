package(default_visibility = ['//visibility:public'])

load('/aos/downloader/downloader', 'aos_downloader')

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//y2014_bot3/control_loops/drivetrain:drivetrain_y2014_bot3',
    '//y2014_bot3/control_loops/rollers:rollers',
    '//y2014_bot3/autonomous:auto_y2014_bot3',
    '//y2014_bot3:joystick_reader_y2014_bot3',
    '//y2014_bot3/wpilib:wpilib_interface_y2014_bot3',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
