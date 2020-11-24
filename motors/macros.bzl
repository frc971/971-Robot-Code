def hex_from_elf(name, target_compatible_with = None):
    native.genrule(
        name = name,
        srcs = ["%s.elf" % name],
        outs = ["%s.hex" % name],
        cmd = "$(OBJCOPY) -O ihex $< $@",
        executable = True,
        output_to_bindir = True,
        target_compatible_with = target_compatible_with,
        toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
    )
