load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")
load("//aos:flatbuffers.bzl", "cc_static_flatbuffer")

flatbuffer_cc_library(
    name = "localizer_output_fbs",
    srcs = [
        "localizer_output.fbs",
    ],
    gen_reflections = True,
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

flatbuffer_cc_library(
    name = "localizer_status_fbs",
    srcs = [
        "localizer_status.fbs",
    ],
    gen_reflections = True,
    includes = [
        "//frc971/control_loops:control_loops_fbs_includes",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs_includes",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

cc_static_flatbuffer(
    name = "localizer_schema",
    function = "frc971::controls::LocalizerStatusSchema",
    target = ":localizer_status_fbs_reflection_out",
    visibility = ["//visibility:public"],
)
