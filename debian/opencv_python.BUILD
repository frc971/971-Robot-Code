py_library(
    name = "python_opencv",
    srcs = glob(["**/*.py"]),
    data = glob(
        include = ["**/*"],
        exclude = ["**/*.py"],
    ),
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@python_repo//:numpy",
    ],
)
