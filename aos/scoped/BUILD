package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scoped_fd",
    srcs = [
        "scoped_fd.cc",
    ],
    hdrs = [
        "scoped_fd.h",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    deps = [
        "//aos:macros",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/log:check",
    ],
)
