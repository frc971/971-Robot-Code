genrule(
    name = "gen_wrapper",
    outs = ["pandoc_wrapper.sh"],
    cmd = "\n".join([
        "cat > $@ <<END",
        "#!/bin/bash",
        "export LD_LIBRARY_PATH=external/pandoc/usr/lib/x86_64-linux-gnu:external/pandoc/lib/x86_64-linux-gnu",
        "exec external/pandoc/usr/bin/pandoc --data-dir=external/pandoc/usr/share/pandoc/data \"\\$$@\"",
        "END",
    ]),
    executable = True,
)

sh_binary(
    name = "pandoc_wrapper",
    srcs = [":pandoc_wrapper.sh"],
    data = glob(["**"]),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "pandoc",
    srcs = ["pandoc_wrapper.sh"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "all_files",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)
