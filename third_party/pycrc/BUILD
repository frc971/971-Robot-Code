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
    target_compatible_with = ["@platforms//os:linux"],
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
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
)
