load("@//tools/build_rules:select.bzl", "compiler_select")

f2c_copts = compiler_select({
    "clang": [
        "-Wno-incompatible-pointer-types-discards-qualifiers",
        # Clang appears to be a bit over-eager about this and the comma operator.
        "-Wno-sometimes-uninitialized",
    ],
    "gcc": [
        # TODO(Brian): Remove this once we can actually disable all the warnings.
        # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=43245 isn't fixed in our
        # roborio toolchain yet, so we can't for now.
        "-Wno-error",
    ],
}) + [
    # f2c appears to know what it's doing without adding extra ().
    "-Wno-parentheses",
    "-Wno-unused-parameter",
    "-Wno-missing-field-initializers",
    "-Wno-unused-variable",
]

"""Copts to use when compiling f2c-generated files.

This is useful when building externally-f2ced files."""

def f2c_library(name, srcs, copts = [], **kwargs):
    """Converts Fortran code to C and then compiles it.

    Attrs:
      srcs: .f source files
      **kwargs: passed to native.cc_library
    """
    c_srcs = [f[:-2] + ".c" for f in srcs]

    out_dir = c_srcs[0].split("/")[:-1]
    for c_src in c_srcs:
        if c_src.split("/")[:-1] != out_dir:
            # Need to figure out how to make multiple f2c calls or something to
            # support this, and we haven't had a use case yet.
            fail("Multiple output directories not supported", "srcs")

    native.genrule(
        name = "_%s_f2c" % name,
        visibility = ["//visibility:private"],
        srcs = srcs,
        outs = c_srcs,
        tools = [
            "@f2c",
            "@//tools/build_rules:quiet_success",
        ],
        cmd = " ".join([
            "$(location @//tools/build_rules:quiet_success)",
            "$(location @f2c)",
            "-d$(@D)/%s" % ("/".join(out_dir),),
            "$(SRCS)",
        ]),
    )
    native.cc_library(
        name = name,
        srcs = c_srcs,
        copts = f2c_copts + copts,
        **kwargs
    )
