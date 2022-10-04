load("//tools/build_rules:pandoc.bzl", "pandoc_html")

exports_files(["mkdocs_bin.py"])

pandoc_html(
    name = "index",
    src = "README.md",
    target_compatible_with = ["@platforms//os:linux"],
)

py_binary(
    name = "mkdocs_bin",
    srcs = ["mkdocs_bin.py"],
    visibility = ["//visibility:public"],
    deps = ["@pip//mkdocs"],
)
