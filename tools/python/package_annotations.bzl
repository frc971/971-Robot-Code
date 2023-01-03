load("@rules_python//python:pip.bzl", "package_annotation")

ANNOTATIONS = {
    "matplotlib": package_annotation(
        data = ["@gtk_runtime//:gtk_runtime"],
        deps = ["@bazel_tools//tools/python/runfiles"],
    ),
    "pygobject": package_annotation(
        data = ["@gtk_runtime//:gtk_runtime"],
        deps = ["@bazel_tools//tools/python/runfiles"],
    ),
    "python-gflags": package_annotation(
        deps = ["@pip_deps_six//:pkg"],
    ),
}
