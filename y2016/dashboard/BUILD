load('/aos/externals/seasocks/gen_embedded', 'gen_embedded')
load('/aos/downloader/downloader', 'aos_downloader_dir')

gen_embedded(
  name = 'gen_embedded',
  srcs = glob(['www_defaults/**/*'], ['www/**/*']),
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
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common:time',
    '//third_party/seasocks',
    '//y2016/actors:autonomous_action_queue',
    '//y2016/vision:vision_queue',
    '//y2016/control_loops/superstructure:superstructure_queue',
    '//y2016/queues:ball_detector',
  ],
)
