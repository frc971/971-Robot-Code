load("@aspect_bazel_lib//lib:run_binary.bzl", "run_binary")

# Validates the constants.json file and outputs a formatted version.
# TODO(max): Make this generic/template it out into frc971
def constants_json(name, src, out):
    run_binary(
        name = name,
        tool = "//y2024_bot3/constants:constants_formatter",
        srcs = [src],
        outs = [out],
        args = ["$(location %s)" % (src)] + ["$(location %s)" % (out)],
        visibility = ["//visibility:public"],
    )
