load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "action_config",
    "feature",
    "flag_group",
    "flag_set",
    "tool",
    "tool_path",
    "with_feature_set",
)
load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")

def _impl(ctx):
    if ctx.attr.cpu == "rp2040":
        toolchain_identifier = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        toolchain_identifier = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        toolchain_identifier = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        toolchain_identifier = "roborio_linux"
    else:
        fail("Unreachable")

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22"):
        host_system_name = "local"
    elif ctx.attr.cpu == "roborio":
        host_system_name = "roborio"
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "rp2040":
        target_system_name = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        target_system_name = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        target_system_name = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        target_system_name = "roborio"
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "rp2040":
        target_cpu = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        target_cpu = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        target_cpu = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        target_cpu = "roborio"
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "rp2040":
        target_libc = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        target_libc = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        target_libc = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        target_libc = "roborio"
    else:
        fail("Unreachable")

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22" or
        ctx.attr.cpu == "roborio"):
        compiler = "gcc"
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "rp2040":
        abi_version = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        abi_version = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        abi_version = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        abi_version = "roborio"
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "rp2040":
        abi_libc_version = "rp2040"
    elif ctx.attr.cpu == "cortex-m4f":
        abi_libc_version = "cortex-m4f"
    elif ctx.attr.cpu == "cortex-m4f-k22":
        abi_libc_version = "cortex-m4f-k22"
    elif ctx.attr.cpu == "roborio":
        abi_libc_version = "roborio"
    else:
        fail("Unreachable")

    cc_target_os = None

    builtin_sysroot = None

    all_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.clif_match,
        ACTION_NAMES.lto_backend,
    ]

    all_cpp_compile_actions = [
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.clif_match,
    ]

    preprocessor_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_header_parsing,
        ACTION_NAMES.cpp_module_compile,
        ACTION_NAMES.clif_match,
    ]

    codegen_compile_actions = [
        ACTION_NAMES.c_compile,
        ACTION_NAMES.cpp_compile,
        ACTION_NAMES.linkstamp_compile,
        ACTION_NAMES.assemble,
        ACTION_NAMES.preprocess_assemble,
        ACTION_NAMES.cpp_module_codegen,
        ACTION_NAMES.lto_backend,
    ]

    all_link_actions = [
        ACTION_NAMES.cpp_link_executable,
        ACTION_NAMES.cpp_link_dynamic_library,
        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
    ]

    if ctx.attr.cpu == "roborio":
        objcopy_embed_data_action = action_config(
            action_name = "objcopy_embed_data",
            enabled = True,
            tools = [
                tool(
                    path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-objcopy",
                ),
            ],
        )
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        objcopy_embed_data_action = action_config(
            action_name = "objcopy_embed_data",
            enabled = True,
            tools = [tool(path = "gcc_arm_none_eabi/arm-none-eabi-objcopy")],
        )
    else:
        objcopy_embed_data_action = None

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22" or
        ctx.attr.cpu == "roborio"):
        action_configs = [objcopy_embed_data_action]
    else:
        fail("Unreachable")

    opt_post_feature = feature(
        name = "opt_post",
        flag_sets = [
            flag_set(
                actions = all_cpp_compile_actions + [
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                ],
                flag_groups = [flag_group(flags = ["-DAOS_DEBUG=0"])],
            ),
        ],
    )

    supports_pic_feature = feature(name = "supports_pic", enabled = True)

    if ctx.attr.cpu == "cortex-m4f-k22":
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-D__STDC_FORMAT_MACROS",
                                "-D__STDC_CONSTANT_MACROS",
                                "-D__STDC_LIMIT_MACROS",
                                "-D__MK22FX512__",
                                "-DF_CPU=120000000",
                                "-Wl,--gc-sections",
                                "-D__have_long32",
                                "-fstack-protector",
                                "-mcpu=cortex-m4",
                                "-mfpu=fpv4-sp-d16",
                                "-mthumb",
                                "-mfloat-abi=hard",
                                "-fno-strict-aliasing",
                                "-fmessage-length=80",
                                "-fmax-errors=20",
                                "-Wall",
                                "-Wextra",
                                "-Wpointer-arith",
                                "-Wcast-qual",
                                "-Wwrite-strings",
                                "-Wtype-limits",
                                "-Wsign-compare",
                                "-Wformat=2",
                                "-Werror",
                                "-Wstrict-aliasing=2",
                                "-Wno-misleading-indentation",
                                "-Wno-int-in-bool-context",
                                "-Wdouble-promotion",
                                "-pipe",
                                "-g",
                                "-fno-common",
                                "-ffreestanding",
                                "-fbuiltin",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-O2",
                                "-finline-functions",
                                "-ffast-math",
                                "-funroll-loops",
                                "-DNDEBUG",
                                "-ffunction-sections",
                            ],
                        ),
                    ],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
            ],
        )
    elif ctx.attr.cpu == "cortex-m4f":
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-D__STDC_FORMAT_MACROS",
                                "-D__STDC_CONSTANT_MACROS",
                                "-D__STDC_LIMIT_MACROS",
                                "-D__MK64FX512__",
                                "-DF_CPU=120000000",
                                "-Wl,--gc-sections",
                                "-D__have_long32",
                                "-fstack-protector",
                                "-mcpu=cortex-m4",
                                "-mfpu=fpv4-sp-d16",
                                "-mthumb",
                                "-mfloat-abi=hard",
                                "-fno-strict-aliasing",
                                "-fmessage-length=80",
                                "-fmax-errors=20",
                                "-Wall",
                                "-Wextra",
                                "-Wpointer-arith",
                                "-Wcast-qual",
                                "-Wwrite-strings",
                                "-Wtype-limits",
                                "-Wsign-compare",
                                "-Wformat=2",
                                "-Werror",
                                "-Wstrict-aliasing=2",
                                "-Wno-misleading-indentation",
                                "-Wno-int-in-bool-context",
                                "-Wdouble-promotion",
                                "-pipe",
                                "-g",
                                "-fno-common",
                                "-ffreestanding",
                                "-fbuiltin",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-O2",
                                "-finline-functions",
                                "-ffast-math",
                                "-funroll-loops",
                                "-DNDEBUG",
                                "-ffunction-sections",
                            ],
                        ),
                    ],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
            ],
        )
    elif ctx.attr.cpu == "rp2040":
        default_compile_flags_feature = feature(
            name = "default_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-DPICO_BOARD=\"pico\"",
                                "-DPICO_BUILD=1",
                                "-DPICO_NO_HARDWARE=0",
                                "-DPICO_ON_DEVICE=1",
                                "-D__STDC_FORMAT_MACROS",
                                "-D__STDC_CONSTANT_MACROS",
                                "-D__STDC_LIMIT_MACROS",
                                "-Wl,--gc-sections",
                                "-fstack-protector",
                                "-mcpu=cortex-m0plus",
                                "-mthumb",
                                "-fno-strict-aliasing",
                                "-fmessage-length=80",
                                "-fmax-errors=20",
                                "-Wall",
                                "-Wextra",
                                "-Wpointer-arith",
                                "-Wcast-qual",
                                "-Wwrite-strings",
                                "-Wtype-limits",
                                "-Wsign-compare",
                                "-Wformat=2",
                                "-Werror",
                                "-Wstrict-aliasing=2",
                                "-Wno-misleading-indentation",
                                "-Wno-int-in-bool-context",
                                "-Wdouble-promotion",
                                "-pipe",
                                "-g",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-O3",
                                "-finline-functions",
                                "-funroll-loops",
                                "-DNDEBUG",
                                "-ffunction-sections",
                            ],
                        ),
                    ],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
            ],
        )
    else:
        default_compile_flags_feature = None

    if ctx.attr.cpu == "roborio":
        dbg_feature = feature(
            name = "dbg",
            flag_sets = [
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [
                        flag_group(flags = ["-DAOS_DEBUG=1"]),
                        flag_group(flags = ["-fno-omit-frame-pointer"]),
                    ],
                ),
            ],
        )
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        dbg_feature = feature(
            name = "dbg",
            flag_sets = [
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [flag_group(flags = ["-fno-omit-frame-pointer"])],
                ),
            ],
            implies = ["all_modes"],
        )
    else:
        dbg_feature = None

    if ctx.attr.cpu == "roborio":
        fastbuild_feature = feature(
            name = "fastbuild",
            flag_sets = [
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [flag_group(flags = ["-DAOS_DEBUG=0"])],
                ),
            ],
        )
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        fastbuild_feature = feature(name = "fastbuild", implies = ["all_modes"])
    else:
        fastbuild_feature = None

    pie_for_linking_feature = feature(
        name = "pie_for_linking",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.cpp_link_executable],
                flag_groups = [flag_group(flags = ["-pie"])],
            ),
        ],
    )

    if ctx.attr.cpu == "roborio":
        opt_feature = feature(
            name = "opt",
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.objc_compile,
                        ACTION_NAMES.objcpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.linkstamp_compile,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-O2",
                                "-DNDEBUG",
                                "-D_FORTIFY_SOURCE=1",
                                "-ffunction-sections",
                                "-fdata-sections",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = [
                        ACTION_NAMES.cpp_link_executable,
                        ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                        ACTION_NAMES.cpp_link_dynamic_library,
                    ],
                    flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
                ),
            ],
            implies = ["opt_post"],
        )
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        opt_feature = feature(name = "opt", implies = ["all_modes"])
    else:
        opt_feature = None

    pic_feature = feature(
        name = "pic",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.linkstamp_compile,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.cpp_module_compile,
                ],
                flag_groups = [
                    flag_group(flags = ["-fPIC"], expand_if_available = "pic"),
                ],
            ),
        ],
    )

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22"):
        include_paths_feature = feature(
            name = "include_paths",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = ["-iquote", "%{quote_include_paths}"],
                            iterate_over = "quote_include_paths",
                        ),
                        flag_group(
                            flags = ["-I%{include_paths}"],
                            iterate_over = "include_paths",
                        ),
                        flag_group(
                            flags = ["-I", "%{system_include_paths}"],
                            iterate_over = "system_include_paths",
                        ),
                    ],
                ),
            ],
        )
    elif ctx.attr.cpu == "roborio":
        include_paths_feature = feature(
            name = "include_paths",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = ["-iquote", "%{quote_include_paths}"],
                            iterate_over = "quote_include_paths",
                        ),
                        flag_group(
                            flags = ["-I%{include_paths}"],
                            iterate_over = "include_paths",
                        ),
                        flag_group(
                            flags = ["-isystem", "%{system_include_paths}"],
                            iterate_over = "system_include_paths",
                        ),
                    ],
                ),
            ],
        )
    else:
        include_paths_feature = None

    random_seed_feature = feature(
        name = "random_seed",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.cpp_module_compile,
                ],
                flag_groups = [
                    flag_group(
                        flags = ["-frandom-seed=%{output_file}"],
                        expand_if_available = "output_file",
                    ),
                ],
            ),
        ],
    )

    if ctx.attr.cpu == "roborio":
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-lstdc++",
                                "-Ltools/cpp/arm-frc-linux-gnueabi/libs",
                                "-no-canonical-prefixes",
                                "-Wl,-z,relro,-z,now",
                                "-lm",
                                "-pass-exit-codes",
                                "-Wl,--build-id=md5",
                                "-Wl,--hash-style=gnu",
                            ],
                        ),
                    ],
                ),
            ],
        )
    elif ctx.attr.cpu == "cortex-m4f-k22":
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-no-canonical-prefixes",
                                "-mcpu=cortex-m4",
                                "-mfpu=fpv4-sp-d16",
                                "-mthumb",
                                "-mfloat-abi=hard",
                                "-fno-strict-aliasing",
                                "--specs=nano.specs",
                                "-lgcc",
                                "-lstdc++_nano",
                                "-lm",
                                "-lc_nano",
                                "-Tmotors/core/kinetis_512_128.ld",
                                "-Tmotors/core/kinetis_sections.ld",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
            ],
        )
    elif ctx.attr.cpu == "cortex-m4f":
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-no-canonical-prefixes",
                                "-mcpu=cortex-m4",
                                "-mfpu=fpv4-sp-d16",
                                "-mthumb",
                                "-mfloat-abi=hard",
                                "-fno-strict-aliasing",
                                "--specs=nano.specs",
                                "-lgcc",
                                "-lstdc++_nano",
                                "-lm",
                                "-lc_nano",
                                "-Tmotors/core/kinetis_512_256.ld",
                                "-Tmotors/core/kinetis_sections.ld",
                            ],
                        ),
                    ],
                ),
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
                    with_features = [with_feature_set(features = ["opt"])],
                ),
            ],
        )
    elif ctx.attr.cpu == "rp2040":
        default_link_flags_feature = feature(
            name = "default_link_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-no-canonical-prefixes",
                                "-mcpu=cortex-m0plus",
                                "-mthumb",
                                "-fno-strict-aliasing",
                                "-Wl,--build-id=none",
                                "--specs=nosys.specs",
                                "-nostartfiles",
                            ],
                        ),
                    ],
                ),
                # TODO(austin): I'd love to turn --gc-sections on, but that breaks things.
            ],
        )
    else:
        default_link_flags_feature = None

    if ctx.attr.cpu == "roborio":
        all_modes_feature = feature(
            name = "all_modes",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [flag_group(flags = ["-std=gnu99"])],
                ),
                flag_set(
                    actions = all_cpp_compile_actions,
                    flag_groups = [
                        flag_group(
                            flags = ["-std=gnu++20", "-fno-sized-deallocation"],
                        ),
                    ],
                ),
                flag_set(
                    actions = all_cpp_compile_actions + [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.assemble,
                        "c++-link",
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [flag_group(flags = ["-pthread"])],
                ),
            ],
        )
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        all_modes_feature = feature(
            name = "all_modes",
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.c_compile,
                    ],
                    flag_groups = [flag_group(flags = ["--std=gnu11"])],
                ),
                flag_set(
                    actions = all_cpp_compile_actions,
                    flag_groups = [
                        flag_group(
                            flags = ["--std=gnu++1z", "-fno-exceptions", "-fno-rtti"],
                        ),
                    ],
                ),
            ],
        )
    else:
        all_modes_feature = None

    supports_dynamic_linker_feature = feature(name = "supports_dynamic_linker", enabled = True)

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22" or
        ctx.attr.cpu == "roborio"):
        unfiltered_compile_flags_feature = feature(
            name = "unfiltered_compile_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = [
                        ACTION_NAMES.assemble,
                        ACTION_NAMES.preprocess_assemble,
                        ACTION_NAMES.linkstamp_compile,
                        ACTION_NAMES.c_compile,
                        ACTION_NAMES.cpp_compile,
                        ACTION_NAMES.cpp_header_parsing,
                        ACTION_NAMES.cpp_module_compile,
                        ACTION_NAMES.cpp_module_codegen,
                        ACTION_NAMES.lto_backend,
                        ACTION_NAMES.clif_match,
                    ],
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-no-canonical-prefixes",
                                "-Wno-builtin-macro-redefined",
                                "-D__DATE__=\"redacted\"",
                                "-D__TIMESTAMP__=\"redacted\"",
                                "-D__TIME__=\"redacted\"",
                            ],
                        ),
                    ],
                ),
            ],
        )
    else:
        unfiltered_compile_flags_feature = None

    dependency_file_feature = feature(
        name = "dependency_file",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.objc_compile,
                    ACTION_NAMES.objcpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.clif_match,
                ],
                flag_groups = [
                    flag_group(
                        flags = ["-MD", "-MF", "%{dependency_file}"],
                        expand_if_available = "dependency_file",
                    ),
                ],
            ),
        ],
    )

    user_compile_flags_feature = feature(
        name = "user_compile_flags",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.linkstamp_compile,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.lto_backend,
                    ACTION_NAMES.clif_match,
                ],
                flag_groups = [
                    flag_group(
                        flags = ["%{user_compile_flags}"],
                        iterate_over = "user_compile_flags",
                        expand_if_available = "user_compile_flags",
                    ),
                ],
            ),
        ],
    )

    sysroot_feature = feature(
        name = "sysroot",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.linkstamp_compile,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.lto_backend,
                    ACTION_NAMES.clif_match,
                    ACTION_NAMES.cpp_link_executable,
                    ACTION_NAMES.cpp_link_dynamic_library,
                    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                ],
                flag_groups = [
                    flag_group(
                        flags = ["--sysroot=%{sysroot}"],
                        expand_if_available = "sysroot",
                    ),
                ],
            ),
        ],
    )

    compile_flags1_feature = feature(
        name = "compile_flags1",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.lto_backend,
                    ACTION_NAMES.clif_match,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "--sysroot=external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot",
                            "-nostdinc",
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include",
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include-fixed",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = all_cpp_compile_actions + [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                ],
                flag_groups = [flag_group(flags = ["-fno-canonical-system-headers"])],
            ),
            flag_set(
                actions = [
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12",
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/arm-nilrt-linux-gnueabi",
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/backward",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = [
                    ACTION_NAMES.assemble,
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                    ACTION_NAMES.lto_backend,
                    ACTION_NAMES.clif_match,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-isystem",
                            "external/arm_frc_linux_gnueabi_repo/arm-nilrt-linux-gnueabi/sysroot/usr/include",
                            "-mfpu=neon",
                            "-D__STDC_FORMAT_MACROS",
                            "-D__STDC_CONSTANT_MACROS",
                            "-D__STDC_LIMIT_MACROS",
                            "-D_FILE_OFFSET_BITS=64",
                            "-DAOS_ARCHITECTURE_arm_frc",
                            "-U_FORTIFY_SOURCE",
                            "-fstack-protector",
                            "-fPIE",
                            "-fdiagnostics-color=always",
                            "-Wall",
                            "-Wextra",
                            "-Wpointer-arith",
                            "-Wstrict-aliasing",
                            "-Wcast-qual",
                            "-Wwrite-strings",
                            "-Wtype-limits",
                            "-Wsign-compare",
                            "-Wformat=2",
                            "-Werror",
                            "-Wunused-local-typedefs",
                            "-Wno-psabi",
                            "-fno-omit-frame-pointer",
                            "-D__has_feature(x)=0",
                            "-pipe",
                            "-ggdb3",
                        ],
                    ),
                ],
            ),
        ],
    )

    if (ctx.attr.cpu == "rp2040" or
        ctx.attr.cpu == "cortex-m4f" or
        ctx.attr.cpu == "cortex-m4f-k22"):
        features = [
            default_compile_flags_feature,
            default_link_flags_feature,
            dbg_feature,
            opt_feature,
            fastbuild_feature,
            all_modes_feature,
            include_paths_feature,
            user_compile_flags_feature,
            sysroot_feature,
            unfiltered_compile_flags_feature,
        ]
    elif ctx.attr.cpu == "roborio":
        features = [
            default_link_flags_feature,
            compile_flags1_feature,
            opt_feature,
            dependency_file_feature,
            random_seed_feature,
            pic_feature,
            include_paths_feature,
            opt_post_feature,
            dbg_feature,
            fastbuild_feature,
            all_modes_feature,
            pie_for_linking_feature,
            supports_dynamic_linker_feature,
            supports_pic_feature,
            user_compile_flags_feature,
            sysroot_feature,
            unfiltered_compile_flags_feature,
        ]
    else:
        fail("Unreachable")

    if ctx.attr.cpu == "roborio":
        cxx_builtin_include_directories = [
            "%package(@arm_frc_linux_gnueabi_repo//arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include)%",
            "%package(@arm_frc_linux_gnueabi_repo//arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include-fixed)%",
            "%package(@arm_frc_linux_gnueabi_repo//arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/arm-nilrt-linux-gnueabi)%",
            "%package(@arm_frc_linux_gnueabi_repo//arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/backward)%",
        ]
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        cxx_builtin_include_directories = [
            "/usr/lib/gcc/arm-none-eabi/4.8/include",
            "/usr/lib/gcc/arm-none-eabi/4.8/include-fixed",
            "/usr/lib/arm-none-eabi/include",
            "/usr/include/newlib",
        ]
    else:
        fail("Unreachable")

    artifact_name_patterns = []

    make_variables = []

    if ctx.attr.cpu == "roborio":
        tool_paths = [
            tool_path(
                name = "ar",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ar",
            ),
            tool_path(
                name = "as",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-as",
            ),
            tool_path(
                name = "compat-ld",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ld",
            ),
            tool_path(
                name = "cpp",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-cpp",
            ),
            tool_path(name = "dwp", path = "/bin/false"),
            tool_path(
                name = "gcc",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-gcc",
            ),
            tool_path(
                name = "gcov",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-gcov-4.9",
            ),
            tool_path(
                name = "ld",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ld",
            ),
            tool_path(
                name = "nm",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-nm",
            ),
            tool_path(
                name = "objcopy",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-objcopy",
            ),
            tool_path(
                name = "objdump",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-objdump",
            ),
            tool_path(
                name = "strip",
                path = "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-strip",
            ),
        ]
    elif (ctx.attr.cpu == "rp2040" or
          ctx.attr.cpu == "cortex-m4f" or
          ctx.attr.cpu == "cortex-m4f-k22"):
        tool_paths = [
            tool_path(
                name = "ar",
                path = "gcc_arm_none_eabi/arm-none-eabi-ar",
            ),
            tool_path(
                name = "compat-ld",
                path = "gcc_arm_none_eabi/arm-none-eabi-ld",
            ),
            tool_path(
                name = "cpp",
                path = "gcc_arm_none_eabi/arm-none-eabi-cpp",
            ),
            tool_path(
                name = "dwp",
                path = "gcc_arm_none_eabi/arm-none-eabi-dwp",
            ),
            tool_path(
                name = "gcc",
                path = "gcc_arm_none_eabi/arm-none-eabi-gcc",
            ),
            tool_path(
                name = "gcov",
                path = "gcc_arm_none_eabi/arm-none-eabi-gcov",
            ),
            tool_path(
                name = "ld",
                path = "gcc_arm_none_eabi/arm-none-eabi-ld",
            ),
            tool_path(
                name = "nm",
                path = "gcc_arm_none_eabi/arm-none-eabi-nm",
            ),
            tool_path(
                name = "objcopy",
                path = "gcc_arm_none_eabi/arm-none-eabi-objcopy",
            ),
            tool_path(
                name = "objdump",
                path = "gcc_arm_none_eabi/arm-none-eabi-objdump",
            ),
            tool_path(
                name = "strip",
                path = "gcc_arm_none_eabi/arm-none-eabi-strip",
            ),
        ]
    else:
        fail("Unreachable")

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, "Fake executable")
    return [
        cc_common.create_cc_toolchain_config_info(
            ctx = ctx,
            features = features,
            action_configs = action_configs,
            artifact_name_patterns = artifact_name_patterns,
            cxx_builtin_include_directories = cxx_builtin_include_directories,
            toolchain_identifier = toolchain_identifier,
            host_system_name = host_system_name,
            target_system_name = target_system_name,
            target_cpu = target_cpu,
            target_libc = target_libc,
            compiler = compiler,
            abi_version = abi_version,
            abi_libc_version = abi_libc_version,
            tool_paths = tool_paths,
            make_variables = make_variables,
            builtin_sysroot = builtin_sysroot,
            cc_target_os = cc_target_os,
        ),
        DefaultInfo(
            executable = out,
        ),
    ]

cc_toolchain_config = rule(
    implementation = _impl,
    attrs = {
        "cpu": attr.string(mandatory = True, values = ["cortex-m4f", "cortex-m4f-k22", "roborio", "rp2040"]),
    },
    provides = [CcToolchainConfigInfo],
    executable = True,
)
