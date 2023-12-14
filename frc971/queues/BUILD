load("//aos/flatbuffers:generate.bzl", "static_flatbuffer")

package(default_visibility = ["//visibility:public"])

static_flatbuffer(
    name = "gyro_fbs",
    srcs = [
        "gyro.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

static_flatbuffer(
    name = "gyro_uid_fbs",
    srcs = [
        "gyro_uid.fbs",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)
