load("@com_github_google_flatbuffers//:build_defs.bzl", "flatbuffer_go_library", "flatbuffer_ts_library")

FILE_NAMES = (
    "error_response",
    "submit_data_scouting",
    "submit_data_scouting_response",
)

filegroup(
    name = "fbs_files",
    srcs = ["%s.fbs" % name for name in FILE_NAMES],
    visibility = ["//visibility:public"],
)

[(
    flatbuffer_go_library(
        name = name + "_go_fbs",
        srcs = [name + ".fbs"],
        importpath = "github.com/frc971/971-Robot-Code/scouting/webserver/requests/messages/" + name,
        target_compatible_with = ["@platforms//cpu:x86_64"],
        visibility = ["//visibility:public"],
    ),
    flatbuffer_ts_library(
        name = name + "_ts_fbs",
        srcs = [name + ".fbs"],
        target_compatible_with = ["@platforms//cpu:x86_64"],
        visibility = ["//visibility:public"],
    ),
) for name in FILE_NAMES]
