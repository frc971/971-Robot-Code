# TODO(austin): I bet this is wrong.
licenses(["restricted"])

load("@//tools/build_rules:fortran.bzl", "f2c_library")
load("@//tools/build_rules:select.bzl", "compiler_select")

# We can't create _wrapper.so in the slycot folder, and can't move it.
# The best way I found to do this is to modify _wrapper.pyf to instead generate
# a _fortranwrapper.so library, and then place a _wrapper.py file in slycot/
# which loads _fortranwrapper from the correct location.  This means that I
# don't need to modify the repository.
genrule(
    name = "_fortranwrapper_pyf",
    srcs = ["slycot/src/_wrapper.pyf"],
    outs = ["slycot/src/_fortranwrapper.pyf"],
    cmd = "cat $(SRCS) | sed 's/_wrapper/_fortranwrapper/' > $(OUTS)",
    restricted_to = ["@//tools:k8"],
)

# The contents of the file telling f2py how to translate various types. The
# format doesn't seem to be very well-documented, but this seems to make all the
# argument types match up.
_f2py_f2cmap_contents = """{
"integer": {
  "check m>=0": "long",
  "check n>=0": "long",
  "check p>=0": "long",
  "": "long",
},
"logical": {
  "": "long",
},
}"""

# Now generate the module wrapper.
genrule(
    name = "_fortranwrappermodule",
    srcs = [
        "slycot/src/analysis.pyf",
        "slycot/src/synthesis.pyf",
        "slycot/src/_fortranwrapper.pyf",
        "slycot/src/math.pyf",
        "slycot/src/transform.pyf",
    ],
    outs = ["_fortranwrappermodule.c"],
    cmd = "\n".join([
        "cat > .f2py_f2cmap <<END",
        _f2py_f2cmap_contents,
        "END",
        "readlink -f .f2py_f2cmap",
        " ".join([
            "$(location @python_repo//:f2py)",
            "$(location :slycot/src/_fortranwrapper.pyf)",
            "--include-paths external/slycot_repo/slycot/src/",
            "--coutput $(OUTS)",
        ]),
        " ".join([
            "sed",
            "\"s/Generation date.*/Generation date: redacted/\"",
            "-i $(OUTS)",
        ]),
    ]),
    restricted_to = ["@//tools:k8"],
    tools = [
        "@python_repo//:f2py",
    ],
)

# Build it.
cc_library(
    name = "slycot_c",
    srcs = [
        ":_fortranwrappermodule",
    ],
    copts = [
        "-Wno-error",
        "-Wno-incompatible-pointer-types-discards-qualifiers",
        "-Wno-cast-align",
        "-Wno-unused-parameter",
        "-Wno-missing-field-initializers",
        "-Wno-unused-function",
        "-Wno-unused-but-set-variable",
    ],
    restricted_to = ["@//tools:k8"],
    deps = [
        ":slicot",
        "@python_repo//:python2.7_f2py",
        "@python_repo//:python2.7_lib",
    ],
)

# Link it all together.  Make sure all the deps get static linked into a single
# shared object, which will then be loaded by the Python interpreter.
cc_binary(
    name = "slycot/_fortranwrapper.so",
    linkshared = True,
    linkstatic = True,
    restricted_to = ["@//tools:k8"],
    deps = [
        ":slicot",
        ":slycot_c",
    ],
)

# Generate the _wrapper file which loads _fortranwrapper and pretends.
genrule(
    name = "_wrapper",
    outs = ["slycot/_wrapper.py"],
    cmd = "echo \"from slycot._fortranwrapper import *\" > $(OUTS)",
    output_to_bindir = True,
)

# Now present a python library for slycot
py_library(
    name = "slycot",
    srcs = [
        "slycot/__init__.py",
        "slycot/analysis.py",
        "slycot/examples.py",
        "slycot/math.py",
        "slycot/synthesis.py",
        "slycot/transform.py",
        ":_wrapper",
    ],
    data = [
        ":slycot/_fortranwrapper.so",
    ],
    imports = ["."],
    restricted_to = ["@//tools:k8"],
    visibility = ["//visibility:public"],
)

f2c_library(
    name = "slicot",
    srcs = glob(["slycot/src/*.f"]),
    copts = [
        # This gets triggered because it doesn't realize xerbla doesn't return.
        # TODO(Brian): Try and get __attribute__((noreturn)) on xerbla somehow.
        "-Wno-uninitialized",
    ] + compiler_select({
        "clang": [
        ],
        "gcc": [
            "-Wno-unused-but-set-variable",
            "-Wno-discarded-qualifiers",
        ],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "@clapack",
    ],
)
