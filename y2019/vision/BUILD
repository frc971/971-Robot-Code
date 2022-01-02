load("//tools:platforms.bzl", "platforms")
load("//tools/build_rules:gtk_dependent.bzl", "gtk_dependent_cc_binary")

package(default_visibility = ["//visibility:public"])

VISION_TARGETS = platforms.any_of([
    "@platforms//cpu:x86_64",
    "//tools/platforms/hardware:raspberry_pi",
])

cc_library(
    name = "constants",
    srcs = [
        "constants.cc",
        "constants_formatting.cc",
    ],
    hdrs = ["constants.h"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "constants_formatting",
    srcs = ["constants_formatting_main.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [":constants"],
)

genrule(
    name = "generate_constants",
    outs = ["validate_constants.cc"],
    cmd = "$(location :constants_formatting) $(OUTS)",
    target_compatible_with = ["@platforms//os:linux"],
    tools = [":constants_formatting"],
)

cc_library(
    name = "image_writer",
    srcs = ["image_writer.cc"],
    hdrs = ["image_writer.h"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/vision/image:image_types",
        "@com_github_google_glog//:glog",
    ],
)

sh_test(
    name = "constants_formatting_test",
    srcs = ["constants_formatting_test.sh"],
    args = [
        "$(location :constants.cc)",
        "$(location :validate_constants.cc)",
    ],
    data = [
        ":constants.cc",
        ":validate_constants.cc",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_library(
    name = "target_finder",
    srcs = [
        "target_finder.cc",
        "target_geometry.cc",
    ],
    hdrs = [
        "target_finder.h",
        "target_types.h",
    ],
    target_compatible_with = VISION_TARGETS,
    deps = [
        ":constants",
        "//aos/util:math",
        "//aos/vision/blob:contour",
        "//aos/vision/blob:hierarchical_contour_merge",
        "//aos/vision/blob:region_alloc",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:overlay",
        "//aos/vision/math:vector",
        "@com_google_ceres_solver//:ceres",
    ],
)

gtk_dependent_cc_binary(
    name = "debug_viewer",
    srcs = ["debug_viewer.cc"],
    target_compatible_with = VISION_TARGETS,
    deps = [
        ":target_finder",
        "//aos/vision/blob:move_scale",
        "//aos/vision/blob:threshold",
        "//aos/vision/blob:transpose",
        "//aos/vision/debug:debug_framework",
        "//aos/vision/math:vector",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "target_sender",
    srcs = ["target_sender.cc"],
    target_compatible_with = VISION_TARGETS,
    deps = [
        ":image_writer",
        ":target_finder",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//y2019/jevois:serial",
        "//y2019/jevois:structures",
        "//y2019/jevois:uart",
        "//y2019/jevois/camera:image_stream",
        "//y2019/jevois/camera:reader",
        "@com_github_google_glog//:glog",
        "@com_google_ceres_solver//:ceres",
    ],
)

cc_binary(
    name = "serial_waiter",
    srcs = ["serial_waiter.cc"],
    target_compatible_with = VISION_TARGETS,
    deps = [
        "//aos/time",
        "//y2019/jevois:serial",
    ],
)

cc_binary(
    name = "debug_serial",
    srcs = ["debug_serial.cc"],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos/logging",
        "//aos/logging:implementations",
        "//y2019/jevois:serial",
        "//y2019/jevois:structures",
        "//y2019/jevois:uart",
    ],
)

cc_binary(
    name = "global_calibration",
    srcs = [
        "global_calibration.cc",
    ],
    target_compatible_with = VISION_TARGETS,
    deps = [
        ":target_finder",
        "//aos/logging",
        "//aos/logging:implementations",
        "//aos/vision/blob:codec",
        "//aos/vision/blob:find_blob",
        "//aos/vision/events:epoll_events",
        "//aos/vision/events:socket_types",
        "//aos/vision/events:udp",
        "//aos/vision/image:image_dataset",
        "//aos/vision/image:image_stream",
        "//aos/vision/image:reader",
        "@com_google_ceres_solver//:ceres",
    ],
)
