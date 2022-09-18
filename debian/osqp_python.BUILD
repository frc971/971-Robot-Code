py_library(
    name = "python_osqp",
    srcs = glob(["**/*.py"]),
    data = glob(
        include = ["**/*"],
        exclude = ["**/*.py"],
    ),
    imports = ["."],
    target_compatible_with = [
        "@//tools/platforms/python:debian_bundled_python",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@pip//numpy",
        "@pip//scipy",
        "@qdldl_amd64//:python_qdldl",
    ],
)
