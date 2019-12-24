def hex_from_elf(name, restricted_to = None):
    native.genrule(
        name = name,
        srcs = ["%s.elf" % name],
        outs = ["%s.hex" % name],
        cmd = "$(OBJCOPY) -O ihex $< $@",
        executable = True,
        output_to_bindir = True,
        restricted_to = restricted_to,
    )
