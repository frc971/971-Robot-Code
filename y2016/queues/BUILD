load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "ball_detector_fbs",
    srcs = [
        "ball_detector.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
