py_binary(
    name = "python_yapf",
    srcs = glob(["yapf/**/*.py"]),
    main = "yapf/__main__.py",
    target_compatible_with = [
        "@//tools/platforms/python:debian_bundled_python",
    ],
    visibility = ["//visibility:public"],
)
