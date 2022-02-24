py_library(
    name = "python_qdldl",
    srcs = glob(["**/*.py"]),
    data = glob(
        include = ["**/*"],
        exclude = ["**/*.py"],
    ),
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@python_repo//:numpy",
        "@python_repo//:scipy",
    ],
)
