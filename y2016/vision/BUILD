load('/tools/build_rules/protobuf', 'proto_cc_library')
load('/aos/build/queues', 'queue_library')

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
