py_library(
    name = "python_opencv",
    srcs = glob(["**/*.py"]),
    data = glob(
        include = ["**/*"],
        exclude = ["**/*.py"],
    ),
    deps = [
        "@python_repo//:numpy",
    ],
    imports = ["."],
    visibility = ["//visibility:public"],
)
