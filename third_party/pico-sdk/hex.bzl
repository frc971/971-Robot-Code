def bin_from_elf(name, target_compatible_with = None):
    native.genrule(
        name = name,
        srcs = ["%s.elf" % name],
        outs = ["%s.bin" % name],
        cmd = "$(OBJCOPY) -Obinary $< $@",
        executable = True,
        output_to_bindir = True,
        target_compatible_with = target_compatible_with,
        toolchains = ["//tools/cpp:cc_toolchain_make_variables"],
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

def pioasm(name, src, out, target_compatible_with = None):
    native.genrule(
        name = name,
        srcs = [src],
        outs = [out],
        cmd = "$(location //third_party/pico-sdk/tools/pioasm) -o c-sdk $< $@",
        executable = True,
        output_to_bindir = True,
        target_compatible_with = target_compatible_with,
        tools = [
            "//third_party/pico-sdk/tools/pioasm",
        ],
    )
