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
    deps = ["@pip//jinja2"],
)

py_binary(
    name = "apache_runner",
    srcs = ["apache_runner.py"],
    data = [
        "apache_template.conf",
        "@apache2//:all_files",
    ],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = ["@pip//jinja2"],
)
