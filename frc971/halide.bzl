def halide_library(name, src, function, args, visibility = None):
    native.genrule(
        name = name + "_build_generator",
        outs = [
            name + "_generator",
        ],
        srcs = [
            src,
        ],
        cmd = "$(location //frc971:halide_generator_compile_script) $(OUTS) $(location " + src + ")",
        tools = [
            "//frc971:halide_generator_compile_script",
        ],
    )
    native.genrule(
        name = "generate_" + name,
        srcs = [
            ":" + name + "_generator",
        ],
        outs = [
            name + ".h",
            name + ".o",
            name + ".stmt.html",
        ],
        # TODO(austin): Upgrade halide...
        cmd = "$(location :" + name + "_generator) -g '" + function + "' -o $(RULEDIR) -f " + name + " -e 'o,h,html' " + select({
            "@platforms//cpu:x86_64": "target=host ",
            "@platforms//cpu:aarch64": "target=arm-64-linux ",
            "//conditions:default": "",
        }) + args,
        target_compatible_with = select({
            "@platforms//cpu:x86_64": [],
            "@platforms//cpu:arm64": [],
            "//conditions:default": ["@platforms//:incompatible"],
        }) + ["@platforms//os:linux"],
    )

    native.cc_library(
        name = name,
        srcs = [name + ".o"],
        hdrs = [name + ".h"],
        visibility = visibility,
        target_compatible_with = select({
            "@platforms//cpu:x86_64": [],
            "@platforms//cpu:arm64": [],
            "//conditions:default": ["@platforms//:incompatible"],
        }) + ["@platforms//os:linux"],
        deps = [
            "//third_party:halide_runtime",
        ],
    )
