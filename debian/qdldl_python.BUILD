py_library(
    name = "python_qdldl",
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
    ],
)
