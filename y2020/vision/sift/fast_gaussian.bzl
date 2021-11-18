load("//tools:platforms.bzl", "platforms")
load("//tools/build_rules:select.bzl", "cpu_select")

def fast_gaussian(sigmas, sizes):
    files = []
    for _, sigma_name, _ in sigmas:
        for cols, rows in sizes:
            files.append("fast_gaussian_%dx%d_%s" % (cols, rows, sigma_name))
    for _, sigma_name, _ in sigmas:
        for cols, rows in sizes:
            files.append("fast_gaussian_subtract_%dx%d_%s" % (cols, rows, sigma_name))
    for cols, rows in sizes:
        files.append("fast_subtract_%dx%d" % (cols, rows))

    params = struct(
        sigmas = sigmas,
        sizes = sizes,
    )

    headers = [f + ".h" for f in files] + [
        "fast_gaussian_all.h",
    ]
    objects = [f + ".o" for f in files] + [
        "fast_gaussian_runtime.o",
    ]
    htmls = [f + ".html" for f in files]

    native.genrule(
        name = "generate_fast_gaussian",
        tools = [
            ":fast_gaussian_runner",
        ],
        cmd = " ".join([
            "$(location :fast_gaussian_runner)",
            "'" + params.to_json() + "'",
            "$(RULEDIR)",
        ]) + " " + cpu_select({
            "amd64": "k8",
            "roborio": "roborio",
            "armhf": "armv7",
            "cortex-m": "cortex-m",
            "cortex-m0plus": "cortex-m0plus",
        }),
        outs = headers + objects + htmls,
        # The tool doesn't support anything other than k8 and armv7.
        # right now.
        target_compatible_with = platforms.any_of([
            "@platforms//cpu:x86_64",
            "//tools/platforms/hardware:raspberry_pi",
        ]),
    )

    native.cc_library(
        name = "fast_gaussian_all",
        hdrs = ["fast_gaussian_all.h"],
        srcs = headers + objects,
        deps = [
            "//third_party:halide_runtime",
        ],
    )
