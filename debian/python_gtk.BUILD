genrule(
    name = "patch_gi_init",
    srcs = [
        "usr/lib/python3/dist-packages/gi/__init__.py",
        "@//debian:python_gi_patches",
    ],
    outs = ["gi/__init__.py"],
    cmd = " && ".join([
        "cp $(location usr/lib/python3/dist-packages/gi/__init__.py) $@",
        "readonly PATCH=\"$$(readlink -f $(location @patch))\"",
        "readonly FILE=\"$$(readlink -f $(location @//debian:python_gi_patches))\"",
        "(cd $(@D) && \"$${PATCH}\" -p1 < \"$${FILE}\") > /dev/null",
    ]),
    tools = [
        "@patch",
    ],
)

genrule(
    name = "patch_shapely_init",
    srcs = [
        "usr/lib/python3/dist-packages/shapely/__init__.py",
        "@//debian:python_shapely_patches",
    ],
    outs = ["shapely/__init__.py"],
    cmd = " && ".join([
        "cp $(location usr/lib/python3/dist-packages/shapely/__init__.py) $@",
        "readonly PATCH=\"$$(readlink -f $(location @patch))\"",
        "readonly FILE=\"$$(readlink -f $(location @//debian:python_shapely_patches))\"",
        "(cd $(@D) && \"$${PATCH}\" -p1 < \"$${FILE}\") > /dev/null",
    ]),
    tools = [
        "@patch",
    ],
)

genrule(
    name = "patch_geos",
    srcs = [
        "usr/lib/python3/dist-packages/shapely/geos.py",
        "@//debian:python_geos_patches",
    ],
    outs = ["shapely/geos.py"],
    cmd = " && ".join([
        "cp $(location usr/lib/python3/dist-packages/shapely/geos.py) $@",
        "readonly PATCH=\"$$(readlink -f $(location @patch))\"",
        "readonly FILE=\"$$(readlink -f $(location @//debian:python_geos_patches))\"",
        "(cd $(@D) && \"$${PATCH}\" -p1 < \"$${FILE}\") > /dev/null",
    ]),
    tools = [
        "@patch",
    ],
)

_src_files = glob(
    include = ["usr/lib/python3/dist-packages/**/*.py"],
    exclude = [
        "usr/lib/python3/dist-packages/gi/__init__.py",
        "usr/lib/python3/dist-packages/shapely/__init__.py",
        "usr/lib/python3/dist-packages/shapely/geos.py",
    ],
)

_data_files = glob([
    "usr/lib/x86_64-linux-gnu/girepository-1.0/**/*",
])

_src_copied = ["/".join(f.split("/")[4:]) for f in _src_files]

_builtin_so_files = glob([
    "usr/lib/python3/dist-packages/**/*.cpython-34m-x86_64-linux-gnu.so",
])

_system_so_files = glob(
    include = [
        "lib/x86_64-linux-gnu/**/*.so*",
        "usr/lib/**/*.so*",
    ],
    exclude = [
        "usr/lib/**/*.cpython-34m-x86_64-linux-gnu.so",
    ],
)

_builtin_so_copied = ["/".join(f.split("/")[4:]) for f in _builtin_so_files]

_system_so_copied = ["rpathed/" + f for f in _system_so_files]

_builtin_rpaths = [":".join([
    "\\$$ORIGIN/%s" % rel,
    "\\$$ORIGIN/%s/rpathed/usr/lib/x86_64-linux-gnu" % rel,
    "\\$$ORIGIN/%s/rpathed/usr/lib" % rel,
    "\\$$ORIGIN/%s/rpathed/lib/x86_64-linux-gnu" % rel,
]) for rel in ["/".join([".." for _ in so.split("/")[1:]]) for so in _builtin_so_copied]]

_system_rpaths = [":".join([
    "\\$$ORIGIN/%s/rpathed/usr/lib/x86_64-linux-gnu" % rel,
    "\\$$ORIGIN/%s/rpathed/usr/lib" % rel,
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
    name = "copy_libgeos_c",
    srcs = ["rpathed/usr/lib/libgeos_c.so.1"],
    outs = ["rpathed/usr/lib/libgeos_c.so"],
    cmd = "cp $< $@",
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

py_library(
    name = "python_gtk",
    srcs = _src_copied + [
        "gi/__init__.py",
        "shapely/__init__.py",
        "shapely/geos.py",
    ],
    data = _data_files + _builtin_so_copied + _system_so_copied + [
        "rpathed/usr/lib/libgeos_c.so",
    ],
    imports = ["usr/lib/python3/dist-packages"],
    restricted_to = ["@//tools:k8"],
    visibility = ["//visibility:public"],
)
