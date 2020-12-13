sh_binary(
    name = "quiet_success",
    srcs = ["quiet_success.sh"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "jinja2_generator",
    srcs = ["jinja2_generator.py"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["@python_jinja2"],
)
