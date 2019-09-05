package(default_visibility = ["//visibility:public"])

filegroup(
    name = "all",
    srcs = [
        "emar.sh",
        "emcc.sh",
        ":emscripten_cache_content",
        "@emscripten_clang//:all",
        "@emscripten_toolchain//:all",
        "@nodejs//:bin/node",
    ],
)

# TODO(james): There is a set of static files that emscripten will always
# attempt to either rebuild or source from a cache. Ideally, we would make
# the build of these files part of the normal build process; however, the
# emscripten compiler handles sandboxing poorly for these files and so it
# is simplest to just manually run emscripten outside of the sandbox and
# copy over the cache folder for now.
filegroup(
    name = "emscripten_cache_content",
    srcs = glob(["emscripten_cache/**/*"]),
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
