load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")
load("@com_google_protobuf//:protobuf.bzl", "cc_proto_library")
load("//aos/downloader:downloader.bzl", "aos_downloader_dir")
load("@build_bazel_rules_typescript//:defs.bzl", "ts_library")
load("@build_bazel_rules_nodejs//:defs.bzl", "rollup_bundle")

ts_library(
    name = "demo",
    srcs = [
        "demo.ts",
    ],
)

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
)

aos_downloader_dir(
    name = "www_files",
    srcs = [
        "//y2019/vision/server/www:files",
        "//y2019/vision/server/www:visualizer_bundle",
    ],
    dir = "www",
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "server_data_proto",
    srcs = ["server_data.proto"],
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
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        ":server_data_proto",
        "//aos:init",
        "//aos/logging",
        "//aos/time",
        "//frc971/control_loops/drivetrain:drivetrain_queue",
        "//frc971/control_loops:pose",
        "//third_party/seasocks",
        "//y2019:constants",
        "//y2019/control_loops/drivetrain:camera_queue",
    ],
)
