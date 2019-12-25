package(default_visibility = ["//visibility:public"])

py_binary(
    name = "plot_action",
    srcs = [
        "logentry.py",
        "logreader.py",
        "plot_action.py",
        "plotter.py",
    ],
    legacy_create_init = False,
    restricted_to = ["//tools:k8"],
    deps = [
        ":python_init",
        "@matplotlib",
    ],
)

py_library(
    name = "python_init",
    srcs = ["__init__.py"],
    deps = ["//frc971:python_init"],
)
