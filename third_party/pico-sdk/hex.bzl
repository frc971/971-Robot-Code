def bin_from_elf(name, target_compatible_with = None):
    native.genrule(
        name = name,
        srcs = ["%s.elf" % name],
        outs = ["%s.bin" % name],
        cmd = "$(OBJCOPY) -Obinary $< $@",
        executable = True,
        output_to_bindir = True,
        target_compatible_with = target_compatible_with,
        toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
    )

def uf2_from_elf(name, target_compatible_with = None):
    native.genrule(
        name = name,
        srcs = ["%s.elf" % name],
        outs = ["%s.uf2" % name],
        cmd = "$(location //third_party/pico-sdk/tools/elf2uf2) $< $@",
        executable = True,
        output_to_bindir = True,
        target_compatible_with = target_compatible_with,
        tools = [
            "//third_party/pico-sdk/tools/elf2uf2",
        ],
    )
