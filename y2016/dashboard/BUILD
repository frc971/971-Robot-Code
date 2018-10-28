load('//aos/seasocks:gen_embedded.bzl', 'gen_embedded')
load('//aos/downloader:downloader.bzl', 'aos_downloader_dir')

gen_embedded(
  name = 'gen_embedded',
  srcs = glob(['www_defaults/**/*'], exclude=['www/**/*']),
)

aos_downloader_dir(
  name = 'www_files',
  visibility = ['//visibility:public'],
  srcs = glob([
    'www/**/*',
  ]),
  dir = "www",
)

cc_binary(
  name = 'dashboard',
  visibility = ['//visibility:public'],
  srcs = [
    'dashboard.cc',
    'dashboard.h',
  ],
  deps = [
    ':gen_embedded',
    '//aos:init',
    '//aos/logging',
    '//aos/util:phased_loop',
    '//aos/time:time',
    '//frc971/autonomous:auto_queue',
    '//third_party/seasocks',
    '//y2016/vision:vision_queue',
    '//y2016/control_loops/superstructure:superstructure_queue',
    '//y2016/queues:ball_detector',
  ],
)
