load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary", "gtk_dependent_cc_library")

flatbuffer_cc_library(
    name = "vision_fbs",
    srcs = [
        "vision.fbs",
    ],
    gen_reflections = 1,
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "vision_data",
    srcs = ["vision_data.proto"],
)

cc_proto_library(
    name = "calibration",
    srcs = ["calibration.proto"],
)

_calibration_values_embedded_before = """
#include "y2016/vision/stereo_geometry.h"

#include <string>

#include "external/com_google_protobuf/src/google/protobuf/text_format.h"

namespace y2016 {
namespace vision {
namespace {

const ::std::string kEmbeddedContents ="""

_calibration_values_embedded_after = """;

}  // namespace

CalibrationFile EmbeddedCalibrationFile() {
  CalibrationFile r;
  ::google::protobuf::TextFormat::ParseFromString(kEmbeddedContents, &r);
  return r;
}

}  // namespace vision
}  // namespace y2016"""

genrule(
    name = "embed_calibration_pb",
    srcs = [
        "calibration.pb",
    ],
    outs = [
        "stereo_geometry_embedded.cc",
    ],
    cmd = "\n".join([
        "echo '%s' > $@" % _calibration_values_embedded_before,
        "cat $(location :calibration.pb) | sed 's/\"/\\\\\"/g ; s/^/\"/g ; s/$$/\\\\n\"/g' >> $@",
        "echo '%s' >> $@" % _calibration_values_embedded_after,
    ]),
)

cc_library(
    name = "stereo_geometry",
    srcs = [
        "stereo_geometry.cc",
        "stereo_geometry_embedded.cc",
    ],
    hdrs = [
        "stereo_geometry.h",
    ],
    deps = [
        ":calibration",
        "//aos/logging",
        "//aos/vision/math:vector",
    ],
)

cc_binary(
    name = "target_sender",
    srcs = ["target_sender.cc"],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    deps = [
        ":calibration",
        ":stereo_geometry",
        ":vision_data",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/time",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:jpeg_routines",
        "//aos/vision/image:reader",
        "//y2016/vision:blob_filters",
    ],
)

cc_library(
    name = "blob_filters",
    srcs = ["blob_filters.cc"],
    hdrs = ["blob_filters.h"],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/blob:hierarchical_contour_merge",
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:threshold",
        "//aos/vision/math:segment",
        "//aos/vision/math:vector",
    ],
)

cc_binary(
    name = "target_receiver",
    srcs = [
        "target_receiver.cc",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":stereo_geometry",
        ":vision_data",
        ":vision_fbs",
        "//aos:init",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/mutex",
        "//aos/time",
        "//aos/vision/events:udp",
        "//frc971/control_loops:control_loops_fbs",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//y2016:constants",
    ],
)

gtk_dependent_cc_binary(
    name = "debug_receiver",
    srcs = ["debug_receiver.cc"],
    restricted_to = [
        "//tools:k8",
        "//tools:armhf-debian",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":blob_filters",
        ":calibration",
        ":stereo_geometry",
        ":vision_data",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:range_image",
        "//aos/vision/blob:stream_view",
        "//aos/vision/debug:debug_window",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:gtk_event",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:tcp_client",
        "//aos/vision/image:image_types",
        "//aos/vision/image:jpeg_routines",
    ],
)
