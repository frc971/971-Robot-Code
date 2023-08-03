load("//tools/build_rules:js.bzl", "ts_project")
load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//frc971/downloader:downloader.bzl", "aos_downloader_dir")

ts_project(
    name = "demo",
    srcs = [
        "demo.ts",
    ],
    target_compatible_with = ["@platforms//os:linux"],
)

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
    target_compatible_with = ["@platforms//os:linux"],
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        "//y2019/vision/server/www:files",
        "//y2019/vision/server/www:visualizer_bundle",
    ],
    dir = "www",
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "server_data_proto_proto",
    srcs = ["server_data.proto"],
    target_compatible_with = ["@platforms//os:linux"],
)

cc_proto_library(
    name = "server_data_proto",
    target_compatible_with = ["@platforms//os:linux"],
    deps = ["server_data_proto_proto"],
)

cc_binary(
    name = "server",
    srcs = [
        "server.cc",
    ],
    data = [
        "//y2019/vision/server/www:files",
        "//y2019/vision/server/www:visualizer_bundle",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        ":server_data_proto",
        "//aos:init",
        "//aos/containers:ring_buffer",
        "//aos/events:shm_event_loop",
        "//aos/logging",
        "//aos/seasocks:seasocks_logger",
        "//aos/time",
        "//frc971/control_loops:pose",
        "//frc971/control_loops/drivetrain:drivetrain_status_fbs",
        "//third_party/seasocks",
        "//y2019:constants",
        "//y2019/control_loops/drivetrain:camera_fbs",
        "//y2019/control_loops/superstructure:superstructure_status_fbs",
        "@com_google_protobuf//:protobuf",
    ],
)
