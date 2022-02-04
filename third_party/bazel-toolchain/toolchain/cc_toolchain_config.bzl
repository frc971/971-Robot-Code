# Copyright 2021 The Bazel Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load(
    "//bazel_tools_changes/tools/cpp:unix_cc_toolchain_config.bzl",
    unix_cc_toolchain_config = "cc_toolchain_config",
)
load(
    "//toolchain/internal:common.bzl",
    _check_os_arch_keys = "check_os_arch_keys",
    _host_tool_features = "host_tool_features",
    _host_tools = "host_tools",
    _os_arch_pair = "os_arch_pair",
)

# Macro for calling cc_toolchain_config from @bazel_tools with setting the
# right paths and flags for the tools.
def cc_toolchain_config(
        name,
        host_arch,
        host_os,
        target_arch,
        target_os,
        toolchain_path_prefix,
        target_toolchain_path_prefix,
        tools_path_prefix,
        wrapper_bin_prefix,
        sysroot_path,
        additional_include_dirs,
        llvm_version,
        standard_library,
        static_libstdcxx,
        conlyopts,
        cxxopts,
        copts,
        opt_copts,
        dbg_copts,
        fastbuild_copts,
        linkopts,
        host_tools_info = {}):
    host_os_arch_key = _os_arch_pair(host_os, host_arch)
    target_os_arch_key = _os_arch_pair(target_os, target_arch)
    _check_os_arch_keys([host_os_arch_key, target_os_arch_key])

    # A bunch of variables that get passed straight through to
    # `create_cc_toolchain_config_info`.
    # TODO: What do these values mean, and are they actually all correct?
    host_system_name = host_arch
    (
        toolchain_identifier,
        target_system_name,
        target_cpu,
        target_libc,
        compiler,
        abi_version,
        abi_libc_version,
        multiarch,
    ) = {
        "darwin-x86_64": (
            "clang-x86_64-darwin",
            "x86_64-apple-macosx",
            "darwin",
            "macosx",
            "clang",
            "darwin_x86_64",
            "darwin_x86_64",
            None,
        ),
        "linux-x86_64": (
            "clang-x86_64-linux",
            "x86_64-unknown-linux-gnu",
            "k8",
            "glibc_unknown",
            "clang",
            "clang",
            "glibc_unknown",
            "x86_64-linux-gnu",
        ),
        "linux-aarch64": (
            "clang-aarch64-linux",
            "aarch64-unknown-linux-gnu",
            "aarch64",
            "glibc_unknown",
            "clang",
            "clang",
            "glibc_unknown",
            "aarch64-linux-gnu",
        ),
        "linux-armv7": (
            "clang-armv7-linux",
            "armv7a-unknown-linux-gnueabihf",
            "armv7",
            "glibc_unknown",
            "clang",
            "clang",
            "glibc_unknown",
            "arm-linux-gnueabihf",
        ),
    }[target_os_arch_key]

    # Unfiltered compiler flags:
    unfiltered_compile_flags = [
        # Do not resolve our symlinked resource prefixes to real paths.
        "-no-canonical-prefixes",
        # Reproducibility
        "-Wno-builtin-macro-redefined",
        "-D__DATE__=\"redacted\"",
        "-D__TIMESTAMP__=\"redacted\"",
        "-D__TIME__=\"redacted\"",
        "-fdebug-prefix-map={}=__bazel_toolchain_llvm_repo__/".format(toolchain_path_prefix),
    ]
    if target_toolchain_path_prefix != toolchain_path_prefix:
        unfiltered_compile_flags.extend([
            "-fdebug-prefix-map={}=__bazel_target_toolchain_llvm_repo__/".format(target_toolchain_path_prefix),
        ])

    is_xcompile = not (host_os == target_os and host_arch == target_arch)

    resource_dir = [
        "-resource-dir",
        "{}lib/clang/{}".format(target_toolchain_path_prefix, llvm_version),
    ]

    # Default compiler flags:
    compile_flags = [
        "--target=" + target_system_name,
        # Security
        "-U_FORTIFY_SOURCE",  # https://github.com/google/sanitizers/issues/247
        "-D_FORTIFY_SOURCE=2",
        "-ggdb3",
        "-fstack-protector",
        "-fno-omit-frame-pointer",
        # Diagnostics
        "-fcolor-diagnostics",
        "-Wall",
        "-Wthread-safety",
        "-Wself-assign",
        "-B{}bin/".format(toolchain_path_prefix),
    ] + resource_dir

    dbg_compile_flags = ["-fstandalone-debug"]

    opt_compile_flags = [
        "-O2",
        "-DNDEBUG",
        "-ffunction-sections",
        "-fdata-sections",
    ]

    fastbuild_compile_flags = [
    ]

    link_flags = [
        "--target=" + target_system_name,
        "-lm",
        "-no-canonical-prefixes",
    ] + resource_dir
    link_libs = []

    # Linker flags:
    if host_os == "darwin" and not is_xcompile:
        # lld is experimental for Mach-O, so we use the native ld64 linker.
        use_lld = False
        link_flags.extend([
            "-headerpad_max_install_names",
            "-undefined",
            "dynamic_lookup",
        ])
    else:
        # Note that for xcompiling from darwin to linux, the native ld64 is
        # not an option because it is not a cross-linker, so lld is the
        # only option.
        use_lld = True
        link_flags.extend([
            "-fuse-ld=lld",
            "-Wl,--build-id=md5",
            "-Wl,--hash-style=gnu",
            "-Wl,-z,relro,-z,now",
        ])

    # Flags related to C++ standard.
    cxx_flags = [
        "-std=c++17",
    ]
    compile_not_cxx_flags = []

    # We only support getting libc++ from the toolchain for now. Is it worth
    # supporting libc++ from the sysroot? Or maybe just part of a LLVM distribution
    # that's built for the target?
    if not standard_library and is_xcompile:
        print("WARNING: Using libc++ for host architecture while cross compiling, this is " +
              "probably not what you want. Explicitly set standard_libraries to libc++ to silence.")

    # The linker has no way of knowing if there are C++ objects; so we
    # always link C++ libraries.
    if not standard_library or standard_library == "libc++":
        cxx_flags.extend([
            "-stdlib=libc++",
        ])
        if use_lld:
            # For single-platform builds, we can statically link the bundled
            # libraries.
            link_flags.extend([
                "-L{}lib".format(target_toolchain_path_prefix),
            ])
            if static_libstdcxx:
                link_flags.extend([
                    "-l:libc++.a",
                    "-l:libc++abi.a",
                ])
            else:
                link_flags.extend([
                    "-l:libc++.so",
                    "-l:libc++abi.so",
                ])
            link_flags.extend([
                "-l:libunwind.a",
                # Compiler runtime features.
                "-rtlib=compiler-rt",
            ])
        else:
            if not static_libstdcxx:
                # TODO: Not sure how to achieve static linking of bundled libraries
                # with ld64; maybe we don't really need it.
                print("WARNING: static libc++ with non-lld linker not supported, ignoring")
            link_flags.extend([
                "-lc++",
                "-lc++abi",
            ])
    elif standard_library.startswith("libstdc++"):
        if not use_lld:
            fail("libstdc++ only supported with lld")

        # We use libgcc when using libstdc++ from a sysroot. Most libstdc++
        # builds link to libgcc, which means we need to use libgcc's exception
        # handling implementation, not the separate one in compiler-rt.
        # Unfortunately, clang sometimes emits code incompatible with libgcc,
        # see <https://bugs.llvm.org/show_bug.cgi?id=27455> and
        # <https://lists.llvm.org/pipermail/cfe-dev/2016-April/048466.html> for
        # example. This seems to be a commonly-used configuration with clang
        # though, so it's probably good enough for most people.

        link_flags.extend([
            "-L{}lib".format(target_toolchain_path_prefix),
        ])

        # We expect to pick up these libraries from the sysroot.
        if static_libstdcxx:
            link_flags.extend([
                "-l:libstdc++.a",
            ])
        else:
            link_flags.extend([
                "-l:libstdc++.so",
            ])

        if standard_library == "libstdc++":
            cxx_flags.extend([
                "-stdlib=libstdc++",
            ])
        elif standard_library.startswith("libstdc++-"):
            # -stdlib does nothing when using -nostdinc besides produce a warning
            # that it's unused, so don't use it here.

            libstdcxx_version = standard_library[len("libstdc++-"):]

            common_include_flags = [
                "-nostdinc",
                "-isystem",
                target_toolchain_path_prefix + "lib/clang/{}/include".format(llvm_version),
                "-isystem",
                sysroot_path + "/usr/local/include",
                "-isystem",
                sysroot_path + "/usr/" + multiarch + "/include",
                "-isystem",
                sysroot_path + "/usr/include/" + multiarch,
                "-isystem",
                sysroot_path + "/usr/include",
                "-isystem",
                sysroot_path + "/include",
                "-isystem",
                sysroot_path + "/usr/include",
            ]
            compile_not_cxx_flags.extend(common_include_flags)
            cxx_flags.extend([
                "-nostdinc++",
                "-isystem",
                sysroot_path + "/usr/include/c++/" + libstdcxx_version,
                "-isystem",
                sysroot_path + "/usr/include/" + multiarch + "/c++/" + libstdcxx_version,
                "-isystem",
                sysroot_path + "/usr/include/c++/" + libstdcxx_version + "/backward",
            ] + common_include_flags)
        else:
            fail("Invalid standard_libary: " + standard_library)
    else:
        fail("Invalid standard_libary: " + standard_library)

    link_libs.extend([
        # To support libunwind. We do this even if not using libunwind explicitly
        # to keep the resulting toolchains more similar.
        "-lpthread",
        "-ldl",
    ])

    opt_link_flags = ["-Wl,--gc-sections"] if target_os == "linux" else []

    # Coverage flags:
    coverage_compile_flags = ["-fprofile-instr-generate", "-fcoverage-mapping"]
    coverage_link_flags = ["-fprofile-instr-generate"]

    ## NOTE: framework paths is missing here; unix_cc_toolchain_config
    ## doesn't seem to have a feature for this.

    # C++ built-in include directories:
    cxx_builtin_include_directories = [
        target_toolchain_path_prefix + "include/c++/v1",
        target_toolchain_path_prefix + "lib/clang/{}/include".format(llvm_version),
        target_toolchain_path_prefix + "lib64/clang/{}/include".format(llvm_version),
    ]

    sysroot_prefix = ""
    if sysroot_path:
        sysroot_prefix = "%sysroot%"
    if target_os == "linux":
        cxx_builtin_include_directories.extend([
            sysroot_prefix + "/include",
            sysroot_prefix + "/usr/include",
            sysroot_prefix + "/usr/local/include",
        ])
    elif target_os == "darwin":
        cxx_builtin_include_directories.extend([
            sysroot_prefix + "/usr/include",
            sysroot_prefix + "/System/Library/Frameworks",
        ])
    else:
        fail("Unreachable")

    cxx_builtin_include_directories.extend(additional_include_dirs)

    ## NOTE: make variables are missing here; unix_cc_toolchain_config doesn't
    ## pass these to `create_cc_toolchain_config_info`.

    # Tool paths:
    # `llvm-strip` was introduced in V7 (https://reviews.llvm.org/D46407):
    llvm_version = llvm_version.split(".")
    llvm_major_ver = int(llvm_version[0]) if len(llvm_version) else 0
    strip_binary = (tools_path_prefix + "bin/llvm-strip") if llvm_major_ver >= 7 else _host_tools.get_and_assert(host_tools_info, "strip")

    # TODO: The command line formed on darwin does not work with llvm-ar.
    ar_binary = tools_path_prefix + "bin/llvm-ar"
    if host_os == "darwin":
        # Bazel uses arg files for longer commands; some old macOS `libtool`
        # versions do not support this.
        #
        # In these cases we want to use `libtool_wrapper.sh` which translates
        # the arg file back into command line arguments.
        if not _host_tools.tool_supports(host_tools_info, "libtool", features = [_host_tool_features.SUPPORTS_ARG_FILE]):
            ar_binary = wrapper_bin_prefix + "bin/host_libtool_wrapper.sh"
        else:
            ar_binary = host_tools_info["libtool"]["path"]

    # The tool names come from [here](https://github.com/bazelbuild/bazel/blob/c7e58e6ce0a78fdaff2d716b4864a5ace8917626/src/main/java/com/google/devtools/build/lib/rules/cpp/CppConfiguration.java#L76-L90):
    tool_paths = {
        "ar": ar_binary,
        "cpp": tools_path_prefix + "bin/clang-cpp",
        "gcc": wrapper_bin_prefix + "bin/cc_wrapper.sh",
        "gcov": tools_path_prefix + "bin/llvm-profdata",
        "ld": tools_path_prefix + "bin/ld.lld" if use_lld else _host_tools.get_and_assert(host_tools_info, "ld"),
        "llvm-cov": tools_path_prefix + "bin/llvm-cov",
        "nm": tools_path_prefix + "bin/llvm-nm",
        "objcopy": tools_path_prefix + "bin/llvm-objcopy",
        "objdump": tools_path_prefix + "bin/llvm-objdump",
        "strip": strip_binary,
        "dwp": tools_path_prefix + "bin/llvm-dwp",
        "llvm-profdata": tools_path_prefix + "bin/llvm-profdata",
    }

    # Start-end group linker support:
    # This was added to `lld` in this patch: http://reviews.llvm.org/D18814
    #
    # The oldest version of LLVM that we support is 6.0.0 which was released
    # after the above patch was merged, so we just set this to `True` when
    # `lld` is being used as the linker.
    supports_start_end_lib = use_lld

    # Add extra flags at the end so they can override anything from this file if desired.
    cxx_flags.extend(cxxopts)
    compile_flags.extend(copts)
    dbg_compile_flags.extend(dbg_copts)
    opt_compile_flags.extend(opt_copts)
    fastbuild_compile_flags.extend(fastbuild_copts)
    link_flags.extend(linkopts)

    # Source: https://cs.opensource.google/bazel/bazel/+/master:tools/cpp/unix_cc_toolchain_config.bzl
    unix_cc_toolchain_config(
        name = name,
        cpu = target_cpu,
        compiler = compiler,
        toolchain_identifier = toolchain_identifier,
        host_system_name = host_system_name,
        target_system_name = target_system_name,
        target_libc = target_libc,
        abi_version = abi_version,
        abi_libc_version = abi_libc_version,
        cxx_builtin_include_directories = cxx_builtin_include_directories,
        tool_paths = tool_paths,
        compile_flags = compile_flags,
        dbg_compile_flags = dbg_compile_flags,
        opt_compile_flags = opt_compile_flags,
        fastbuild_compile_flags = fastbuild_compile_flags,
        cxx_flags = cxx_flags,
        c_flags = conlyopts,
        compile_not_cxx_flags = compile_not_cxx_flags,
        link_flags = link_flags,
        link_libs = link_libs,
        opt_link_flags = opt_link_flags,
        unfiltered_compile_flags = unfiltered_compile_flags,
        coverage_compile_flags = coverage_compile_flags,
        coverage_link_flags = coverage_link_flags,
        supports_start_end_lib = supports_start_end_lib,
        builtin_sysroot = sysroot_path,
    )
