load("@aspect_rules_js//js:defs.bzl", "js_binary")
load("@npm//:create-foxglove-extension/package_json.bzl", "bin")

bin.create_foxglove_extension_binary(
    name = "create_foxglove_extension",
)

bin.foxglove_extension_binary(
    name = "foxglove_extension",
    data = [
        # This upstream binary needs the dummy npm binary in its runfiles since
        # it will invoke this dummy npm binary.
        ":foxglove_extension_wrapper_npm",
    ],
)

js_binary(
    name = "foxglove_extension_wrapper",
    data = [
        ":foxglove_extension",
        # This binary needs the dummy npm binary in its runfiles since it needs
        # to point the `foxglove_extension` binary above to it.
        ":foxglove_extension_wrapper_npm",
        "//:node_modules/create-foxglove-extension",
    ],
    entry_point = "foxglove_extension_wrapper.js",
    visibility = ["//visibility:public"],
)

js_binary(
    name = "foxglove_extension_wrapper_npm",
    entry_point = "foxglove_extension_wrapper_npm.js",
)

py_binary(
    name = "creation_wrapper",
    srcs = ["creation_wrapper.py"],
    data = [
        "BUILD.bazel.tmpl",
        ":creation_wrapper_npm",
        "@com_github_bazelbuild_buildtools//buildozer",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
    ],
    deps = [
        "@pip//pyyaml",
        "@rules_python//python/runfiles",
    ],
)

py_binary(
    name = "creation_wrapper_npm",
    srcs = ["creation_wrapper_npm.py"],
)
