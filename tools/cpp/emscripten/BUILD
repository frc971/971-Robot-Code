package(default_visibility = ["//visibility:public"])

_minimum_fileset = [
    "emar.sh",
    "emcc.sh",
    "@emscripten_clang//:all",
    "@emscripten_toolchain//:all",
    "@nodejs//:bin/node",
]

filegroup(
    name = "minimum_files",
    srcs = _minimum_fileset,
)

filegroup(
    name = "all",
    srcs = _minimum_fileset + [
        ":emscripten_cache_content",
    ],
)

# A list of all the cached libraries generad and used by emscripten.
_libs = \
    ["emscripten_cache/asmjs/" + lib for lib in [
        "generated_struct_info.json",
        "libc.bc",
        "libcompiler_rt.a",
        "libc-wasm.bc",
        "libpthreads_stub.bc",
        "libhtml5.bc",
        "libdlmalloc.bc",
        "libal.bc",
        "libc++_noexcept.a",
        "libc++abi.bc",
        "libgl-webgl2.bc",
        "libgl.bc",
        "libc-extras.bc",
    ]]

filegroup(
    name = "emscripten_cache_content",
    srcs = glob(["emscripten_cache/**/*"]) + _libs,
)

cc_toolchain(
    name = "cc_emscripten",
    all_files = ":all",
    compiler_files = ":all",
    cpu = "web",
    dwp_files = ":empty",
    dynamic_runtime_libs = [":empty"],
    linker_files = ":all",
    objcopy_files = ":empty",
    static_runtime_libs = [":empty"],
    strip_files = ":empty",
    supports_param_files = 0,
)

# TODO(james): Currently, this gets built with the host configuration.
# Currently, that doesn't actually impact the build since there's nothing that
# affects how the genrule is run. However, that also means that changing
# the configuration (and thus the flags that may be passed to the C++
# compiler) will not change how these cache files are generated.
genrule(
    name = "gencache",
    # Note that foo.o is just some arbitrary .o file. I had trouble getting
    # emscripten to work properly when pointed at a literally empty file, but
    # the exact contents of the .o aren't particularly important.
    srcs = [":foo.o"],
    outs = _libs,
    cmd = "$(location gencache.sh) $(OUTS)",
    tools = [
        ":gencache.sh",
        ":minimum_files",
    ],
)
