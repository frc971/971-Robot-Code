package(default_visibility = ['//visibility:public'])

load('/aos/prime/downloader/downloader', 'aos_downloader')

aos_downloader(
  name = 'download',
  srcs = [
    '//aos:prime_binaries',
    '//y2015/control_loops/drivetrain:drivetrain',
    '//y2015/control_loops/fridge:fridge',
    '//y2015/control_loops/claw:claw',
    '//y2015/autonomous:auto',
    '//y2015:joystick_reader',
    '//y2015/http_status:http_status',
    '//y2015/actors:binaries',
    ':start_list.txt',
  ],
)
