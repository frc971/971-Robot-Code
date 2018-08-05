genrule(
    name = "patch_init",
    srcs = [
        "usr/lib/python2.7/dist-packages/matplotlib/__init__.py",
        "@//debian:matplotlib_patches",
    ],
    outs = ["matplotlib/__init__.py"],
    cmd = " && ".join([
        "cp $(location usr/lib/python2.7/dist-packages/matplotlib/__init__.py) $@",
        "readonly PATCH=\"$$(readlink -f $(location @patch))\"",
        "readonly FILE=\"$$(readlink -f $(location @//debian:matplotlib_patches))\"",
        "(cd $(@D) && \"$${PATCH}\" -p1 < \"$${FILE}\") > /dev/null",
    ]),
    tools = [
        "@patch",
    ],
)

_src_files = glob(
    include = ["usr/lib/python2.7/dist-packages/**/*.py"],
    exclude = [
        "usr/lib/python2.7/dist-packages/matplotlib/__init__.py",
    ],
)

_data_files = glob([
    "usr/share/matplotlib/mpl-data/**",
])

_src_copied = ["/".join(f.split("/")[4:]) for f in _src_files]

_builtin_so_files = glob([
    "usr/lib/python2.7/dist-packages/**/*.x86_64-linux-gnu.so",
])

_system_so_files = glob([
    "usr/lib/x86_64-linux-gnu/**/*.so*",
    "lib/x86_64-linux-gnu/**/*.so*",
])

_builtin_so_copied = ["/".join(f.split("/")[4:]) for f in _builtin_so_files]

_system_so_copied = ["rpathed/" + f for f in _system_so_files]

_builtin_rpaths = [":".join([
    "\\$$ORIGIN/%s" % rel,
    "\\$$ORIGIN/%s/rpathed/usr/lib/x86_64-linux-gnu" % rel,
    "\\$$ORIGIN/%s/rpathed/lib/x86_64-linux-gnu" % rel,
]) for rel in ["/".join([".." for _ in so.split("/")[1:]]) for so in _builtin_so_copied]]

_system_rpaths = [":".join([
    "\\$$ORIGIN/%s/rpathed/usr/lib/x86_64-linux-gnu" % rel,
    "\\$$ORIGIN/%s/rpathed/lib/x86_64-linux-gnu" % rel,
]) for rel in ["/".join([".." for _ in so.split("/")[1:]]) for so in _system_so_copied]]

genrule(
    name = "run_patchelf_builtin",
    srcs = _builtin_so_files,
    outs = _builtin_so_copied,
    cmd = "\n".join(
        [
            "cp $(location %s) $(location %s)" % (src, dest)
            for src, dest in zip(_builtin_so_files, _builtin_so_copied)
        ] +
        ["$(location @patchelf) --set-rpath %s $(location %s)" % (rpath, so) for rpath, so in zip(_builtin_rpaths, _builtin_so_copied)],
    ),
    tools = [
        "@patchelf",
    ],
)

genrule(
    name = "run_patchelf_system",
    srcs = _system_so_files,
    outs = _system_so_copied,
    cmd = "\n".join(
        [
            "cp $(location %s) $(location %s)" % (src, dest)
            for src, dest in zip(_system_so_files, _system_so_copied)
        ] +
        ["$(location @patchelf) --set-rpath %s $(location %s)" % (rpath, so) for rpath, so in zip(_system_rpaths, _system_so_copied)],
    ),
    tools = [
        "@patchelf",
    ],
)

genrule(
    name = "copy_files",
    srcs = _src_files,
    outs = _src_copied,
    cmd = " && ".join(["cp $(location %s) $(location %s)" % (src, dest) for src, dest in zip(
        _src_files,
        _src_copied,
    )]),
)

genrule(
    name = "create_empty_rc",
    outs = ["usr/share/matplotlib/mpl-data/matplotlibrc"],
    cmd = "touch $@",
)

py_library(
    name = "matplotlib",
    srcs = _src_copied + [
        "matplotlib/__init__.py",
    ],
    data = _data_files + _builtin_so_copied + _system_so_copied + [
        ":usr/share/matplotlib/mpl-data/matplotlibrc",
    ],
    imports = ["usr/lib/python2.7/dist-packages"],
    restricted_to = ["@//tools:k8"],
    visibility = ["//visibility:public"],
)
