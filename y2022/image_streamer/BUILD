load("//aos/seasocks:gen_embedded.bzl", "gen_embedded")

gen_embedded(
    name = "gen_embedded",
    srcs = glob(
        include = ["www_defaults/**/*"],
        exclude = ["www/**/*"],
    ),
)

filegroup(
    name = "image_streamer_start",
    srcs = ["image_streamer_start.sh"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "image_streamer",
    srcs = ["image_streamer.cc"],
    args = [
        "--data_dir=y2022/image_streamer/www",
    ],
    copts = [
        "-Wno-cast-align",
    ],
    data = [
        "//y2022:aos_config",
        "//y2022/image_streamer/www:files",
        "//y2022/image_streamer/www:main_bundle.min.js",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:aarch64": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }) + [
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gen_embedded",
        "//aos:configuration_fbs",
        "//aos:init",
        "//aos/events:glib_main_loop",
        "//aos/events:shm_event_loop",
        "//aos/network:connect_fbs",
        "//aos/network:web_proxy_fbs",
        "//aos/seasocks:seasocks_logger",
        "//frc971/vision:vision_fbs",
        "//third_party:gstreamer",
        "//third_party/seasocks",
        "@com_github_google_flatbuffers//:flatbuffers",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
    ],
)
