load('/tools/build_rules/protobuf', 'proto_cc_library')
load('/aos/build/queues', 'queue_library')
load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')

queue_library(
  name = 'vision_queue',
  visibility = ['//visibility:public'],
  srcs = [
    'vision.q',
  ],
)

proto_cc_library(
  name = 'vision_data',
  src = 'vision_data.proto',
)

proto_cc_library(
  name = 'calibration',
  src = 'calibration.proto',
)

_calibration_values_embedded_before = '''
#include "y2016/vision/stereo_geometry.h"

#include <string>

#include "third_party/protobuf/src/google/protobuf/text_format.h"

namespace y2016 {
namespace vision {
namespace {

const ::std::string kEmbeddedContents ='''

_calibration_values_embedded_after = ''';

}  // namespace

CalibrationFile EmbeddedCalibrationFile() {
  CalibrationFile r;
  ::google::protobuf::TextFormat::ParseFromString(kEmbeddedContents, &r);
  return r;
}

}  // namespace vision
}  // namespace y2016'''

genrule(
  name = 'embed_calibration_pb',
  srcs = [
    'calibration.pb',
  ],
  outs = [
    'stereo_geometry_embedded.cc',
  ],
  cmd = '\n'.join([
    'echo \'%s\' > $@' % _calibration_values_embedded_before,
    'cat $(location :calibration.pb) | sed \'s/"/\\\\"/g ; s/^/"/g ; s/$$/\\\\n"/g\' >> $@',
    'echo \'%s\' >> $@' % _calibration_values_embedded_after,
  ]),
)

cc_library(
  name = 'stereo_geometry',
  hdrs = [
    'stereo_geometry.h',
  ],
  srcs = [
    'stereo_geometry.cc',
    'stereo_geometry_embedded.cc',
  ],
  deps = [
    ':calibration',
    '//aos/common/logging',
    '//aos/vision/math:vector',
  ],
)

cc_binary(name = 'target_sender',
  srcs = ['target_sender.cc'],
  deps = [
    '//aos/common:time',
    '//aos/common/logging:logging',
    '//aos/common/logging:implementations',
    '//aos/vision/image:reader',
    '//aos/vision/image:jpeg_routines',
    '//aos/vision/image:image_stream',
    '//y2016/vision:blob_filters',
    '//aos/vision/events:udp',
    '//aos/vision/events:epoll_events',
    '//aos/vision/events:socket_types',
    ':stereo_geometry',
    ':vision_data',
    ':calibration',
  ]
)

cc_library(name = "blob_filters",
    srcs = ["blob_filters.cc"],
    hdrs = ["blob_filters.h"],
    deps = [
      "//aos/vision/math:vector",
      "//aos/vision/math:segment",
      "//aos/vision/blob:range_image",
      "//aos/vision/blob:threshold",
      "//aos/vision/blob:find_blob",
      "//aos/vision/blob:hierarchical_contour_merge",
      "//aos/vision/blob:codec",
        ],
  visibility = ['//visibility:public'],
)

cc_binary(
  name = 'target_receiver',
  srcs = [
    'target_receiver.cc',
  ],
  visibility = ['//visibility:public'],
  deps = [
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/linux_code:init',
    '//aos/common:time',
    '//aos/vision/events:udp',
    ':vision_queue',
    ':vision_data',
    ':stereo_geometry',
    '//y2016:constants',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//aos/common:mutex',
  ],
)

gtk_dependent_cc_binary(
  name = "debug_receiver",
  srcs = ["debug_receiver.cc"],
  visibility = ['//visibility:public'],
  deps = [
    "//aos/vision/image:image_types",
    "//aos/vision/image:jpeg_routines",
    "//aos/vision/events:socket_types",
    "//aos/vision/events:tcp_client",
    "//aos/vision/events:epoll_events",
    "//aos/vision/events:gtk_event",
    "//aos/vision/debug:debug_window",
    "//aos/vision/blob:range_image",
    "//aos/vision/blob:codec",
    "//aos/vision/blob:stream_view",
    ":stereo_geometry",
    ":blob_filters",
    ":vision_data",
    ":calibration",
  ],
)
