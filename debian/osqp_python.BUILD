py_library(
    name = "python_osqp",
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
        "@qdldl_amd64//:python_qdldl",
    ],
)
