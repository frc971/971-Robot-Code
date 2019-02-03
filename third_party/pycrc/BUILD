licenses(["notice"])

genrule(
    name = "copy_main",
    srcs = [
        "pycrc.py",
    ],
    outs = [
        "pycrc_main.py",
    ],
    cmd = "cp $< $@",
)

py_binary(
    name = "pycrc_main",
    srcs = glob([
        "pycrc/*.py",
    ]) + [
        "pycrc_main.py",
    ],
    imports = ["."],
    legacy_create_init = False,
    main = "pycrc_main.py",
    visibility = ["//visibility:public"],
)
