load(":defs.bzl", "extract_numpy_headers")

cc_library(
    name = "python",
    deps = select({
        "//tools/platforms/python:debian_bundled_python": [
            "@python_repo//:python3.9_lib",
        ],
        "//tools/platforms/python:upstream_bundled_python": [
            "@python3_9_x86_64-unknown-linux-gnu//:python_headers",
            "@python3_9_x86_64-unknown-linux-gnu//:libpython",
        ],
    }),
    defines = select({
        "//tools/platforms/python:debian_bundled_python": [
            "FRC971_DEBIAN_BUNDLED_PYTHON",
        ],
        "//tools/platforms/python:upstream_bundled_python": [
            "FRC971_UPSTREAM_BUNDLED_PYTHON",
            "FRC971_PYTHON_HOME=../python3_9_x86_64-unknown-linux-gnu/",
        ],
    }),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "python_runtime",
    data = select({
        "//tools/platforms/python:debian_bundled_python": [
            "@python_repo//:all_files",
        ],
        "//tools/platforms/python:upstream_bundled_python": [
            "@python3_9_x86_64-unknown-linux-gnu//:files",
        ],
    }),
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
