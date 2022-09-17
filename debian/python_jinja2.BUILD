py_library(
    name = "python_jinja2",
    srcs = glob(["src/jinja2/*.py"]),
    imports = ["src/"],
    target_compatible_with = [
        "@//tools/platforms/python:debian_bundled_python",
    ],
    visibility = ["//visibility:public"],
    deps = ["@python_markupsafe"],
)
