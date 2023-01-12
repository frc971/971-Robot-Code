load(":defs.bzl", "extract_numpy_headers")

cc_library(
    name = "python",
    deps = [
        "@python3_9_x86_64-unknown-linux-gnu//:python_headers",
        "@python3_9_x86_64-unknown-linux-gnu//:libpython",
    ],
    defines = [
        "FRC971_UPSTREAM_BUNDLED_PYTHON",
        "FRC971_PYTHON_HOME=../python3_9_x86_64-unknown-linux-gnu/",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "python_runtime",
    data = [
        "@python3_9_x86_64-unknown-linux-gnu//:files",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    visibility = ["//visibility:public"],
)

extract_numpy_headers(
    name = "numpy_headers",
    numpy = "@pip//numpy",
    header_prefix = "numpy_headers",
    visibility = ["//visibility:private"],
)

cc_library(
    name = "numpy_cc",
    hdrs = [
        ":numpy_headers",
    ],
    includes = [
        "numpy_headers",
    ],
    deps = [
        ":python",
    ],
    visibility = ["//visibility:public"],
)
