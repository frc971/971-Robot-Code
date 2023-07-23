load("@rules_rust//rust:defs.bzl", "rust_library")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "ACTION_NAMES")

def _cc_toolchain_flags(ctx, cc_toolchain):
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compiler_path = cc_common.get_tool_for_action(
        feature_configuration = feature_configuration,
        action_name = ACTION_NAMES.cpp_compile,
    )
    preprocessor_defines = []
    for lib in ctx.attr.libs:
        preprocessor_defines.append(lib[CcInfo].compilation_context.defines)
    compile_variables = cc_common.create_compile_variables(
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        user_compile_flags = ctx.fragments.cpp.copts + ctx.fragments.cpp.cxxopts,
        preprocessor_defines = depset(transitive = preprocessor_defines),
    )
    command_line = cc_common.get_memory_inefficient_command_line(
        feature_configuration = feature_configuration,
        action_name = ACTION_NAMES.cpp_compile,
        variables = compile_variables,
    )
    env = cc_common.get_environment_variables(
        feature_configuration = feature_configuration,
        action_name = ACTION_NAMES.cpp_compile,
        variables = compile_variables,
    )
    return command_line, env

# All the stuff about AUTOCXX_RS_FILE and--fix-rs-include-name only applies when
# using --gen-rs-include. We use --gen-rs-archive so none of that matters.
#
# The generated .rs file uses `include!` on the top-level C++ headers imported
# via `#include` in `include_cpp!`. This always operates relative to the source
# file (I don't see any way to change it), nor does autocxx have a way to change
# the path. There are headers involved which use `#pragma once`, so just copying
# them is a bad idea. Instead, we generate forwarding headers.
def _autocxx_library_gen_impl(ctx):
    rust_toolchain = ctx.toolchains[Label("@rules_rust//rust:toolchain")]

    # TODO(Brian): Provide some way to override this globally in WORKSPACE? Need
    # a real strategy for coordinating toolchains and flags, see the TODO below
    # where cc_command_line is used for details.
    if ctx.attr.override_cc_toolchain:
        cc_toolchain = ctx.attr.override_cc_toolchain[cc_common.CcToolchainInfo]
    else:
        cc_toolchain = find_cc_toolchain(ctx)

    # The directory where we generate files. Needs to be unique and in our package.
    gendir = ctx.label.name + "__dir"

    cc_command_line, cc_env = _cc_toolchain_flags(ctx, cc_toolchain)

    includes = []
    in_headers = []
    forwarding_headers = []
    for lib in ctx.attr.libs:
        compilation = lib[CcInfo].compilation_context

        # TODO(Brian): How should we be juggling includes, quote_includes, and system_includes?
        includes.append(compilation.includes)
        includes.append(compilation.quote_includes)
        includes.append(compilation.system_includes)
        in_headers.append(compilation.headers)
        for header in compilation.direct_public_headers:
            # TODO(Brian): This doesn't work if it's being `#include`ed (via
            # `include_cpp!`) using one of the includes paths. Maybe we should
            # add each `includes` prefixed with `gendir` to solve that?
            forwarding = ctx.actions.declare_file("%s/%s" % (gendir, header.short_path))
            forwarding_headers.append(forwarding)
            ctx.actions.write(forwarding, '#include "%s"' % header.short_path)
    includes = depset(transitive = includes)
    action_inputs = depset(
        direct = ctx.files.srcs + ctx.files.cxxbridge_srcs,
        transitive = in_headers + [cc_toolchain.all_files],
    )

    # This is always the name with --gen-rs-archive, regardless of other flags.
    out_rs_json = ctx.actions.declare_file("%s/gen.rs.json" % gendir)
    out_env_file = ctx.actions.declare_file("%s/rustc_env" % gendir)
    ctx.actions.write(
        output = out_env_file,
        # The first path is valid for rust_library/rust_binary/rust_test/etc, the second one
        # is valid for rust_doc_test due to working directory differences.
        content = "AUTOCXX_RS_JSON_ARCHIVE=%s:%s" % (out_rs_json.path, out_rs_json.short_path),
    )

    out_h = ctx.actions.declare_file("%s_cxxgen.h" % ctx.label.name.rstrip("__gen"))
    out_h_guard = out_h.short_path.replace("/", "_").replace(".", "_")
    out_h_contents = [
        "#ifndef %s" % out_h_guard,
        "#define %s" % out_h_guard,
        "// GENERATED FILE, DO NOT EDIT",
        "//",
        "// #includes all of the declarations exported to C++ from %s" % ctx.label,
    ]
    out_cc = []

    # See `gen --help` for details on the naming of these outputs.
    for cc_index in range(ctx.attr.sections_to_generate):
        out_cc.append(ctx.actions.declare_file("%s/gen%d.cc" % (gendir, cc_index)))
        gen_h = ctx.actions.declare_file("%s/gen%d.h" % (gendir, cc_index))
        out_cc.append(gen_h)
        out_h_contents.append("#include \"%s\"" % gen_h.short_path)
        autocxxgen_h = ctx.actions.declare_file("%s/autocxxgen%d.h" % (gendir, cc_index))
        out_cc.append(autocxxgen_h)
        out_h_contents.append("#include \"%s\"" % autocxxgen_h.short_path)

    cxxbridge_cc_srcs = []
    for src in ctx.files.cxxbridge_srcs:
        cxxbridge_cc = ctx.actions.declare_file("%s/cxxbridge.cc" % gendir)
        cxxbridge_cc_srcs.append(cxxbridge_cc)
        cxxbridge_h = ctx.actions.declare_file("%s/cxxbridge.h" % gendir)
        cxxbridge_cc_srcs.append(cxxbridge_h)
        out_h_contents.append("#include \"%s\"" % cxxbridge_h.short_path)
        ctx.actions.run(
            mnemonic = "CxxCodegen",
            executable = ctx.executable._cxx_codegen,
            inputs = [src],
            outputs = [cxxbridge_cc, cxxbridge_h],
            arguments = [src.path, "--output", cxxbridge_h.path, "--output", cxxbridge_cc.path],
        )

    out_h_contents.append("#endif  // %s" % out_h_guard)
    ctx.actions.write(
        output = out_h,
        content = "\n".join(out_h_contents),
    )

    gen_rs = ctx.actions.args()
    gen_rs.add_all(["--outdir", out_rs_json.dirname])
    gen_rs.add("--gen-rs-archive")
    gen_rs.add("--gen-cpp")
    #gen_rs.add("--auto-allowlist")

    gen_rs.add_all(["--generate-exact", ctx.attr.sections_to_generate])

    gen_rs.add_all(ctx.files.srcs)
    gen_rs.add_all(ctx.files.cxxbridge_srcs)

    # TODO: Do these go before or after the --? They're partially redundant with
    # cc_command_line too.
    gen_rs.add_all(includes, before_each = "-I")
    gen_rs.add("--")

    # TODO: These are flags for the cc_toolchain, not the libclang they're being passed to.
    # Figure out how to handle that nicely. Maybe just require they're compatible, and direct
    # people to overriding the toolchain in use instead?
    gen_rs.add_all(cc_command_line)

    gen_rs.add("-Wno-unused-private-field")
    env = dict(cc_env)
    env.update(
        LIBCLANG_PATH = ctx.file._libclang.path,
    )
    if ctx.attr.gen_debug:
        env.update(
            RUST_BACKTRACE = "full",
            RUST_LOG = "autocxx_engine=info",
        )
    ctx.actions.run(
        arguments = [gen_rs],
        outputs = [out_rs_json] + out_cc,
        tools = [ctx.file._libclang],
        inputs = action_inputs,
        env = env,
        executable = ctx.executable._autocxx_gen,
        mnemonic = "AutocxxGen",
    )

    return [
        OutputGroupInfo(
            cc_srcs = out_cc + cxxbridge_cc_srcs,
            hdr_srcs = [out_h],
            compile_data = forwarding_headers + [out_rs_json],
            env_files = [out_env_file],
        ),
    ]

_autocxx_library_gen = rule(
    implementation = _autocxx_library_gen_impl,
    attrs = {
        "libs": attr.label_list(
            mandatory = True,
            providers = [CcInfo],
            doc = "C++ libraries to let Rust use headers from",
        ),
        "srcs": attr.label_list(
            allow_files = [".rs"],
            mandatory = False,
            doc = "Rust sources with `include_cpp!` macros",
            default = [],
        ),
        # TODO(Brian): Do we need to support this? Or just throw them in srcs?
        "cxxbridge_srcs": attr.label_list(
            allow_files = [".rs"],
            mandatory = False,
            doc = "Rust sources with only [cxx::bridge] annotations",
            default = [],
        ),
        "sections_to_generate": attr.int(
            default = 20,
            doc = (
                "The number of `cxx::bridge` sections to support," +
                " including ones created by `autocxx::include_cpp!`." +
                " The default is sufficient for most use cases." +
                " Setting this too large has a small performance impact, setting it" +
                " too low will result in a build failure."
            ),
        ),
        "gen_debug": attr.bool(
            default = False,
            doc = "Print (lots of) debug info about autocxx's codegen at build time.",
        ),
        "_autocxx_gen": attr.label(
            executable = True,
            default = Label("@//third_party/autocxx/gen/cmd:gen"),
            cfg = "exec",
        ),
        "_cxx_codegen": attr.label(
            executable = True,
            default = Label("@cxxbridge-cmd//:cxxbridge-cmd"),
            cfg = "exec",
        ),
        "_libclang": attr.label(
            cfg = "exec",
            default = Label("@llvm_k8//:libclang"),
            allow_single_file = True,
        ),
        "override_cc_toolchain": attr.label(mandatory = False, providers = [cc_common.CcToolchainInfo]),
        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
    },
    toolchains = [
        "@rules_rust//rust:toolchain",
        "@bazel_tools//tools/cpp:toolchain_type",
    ],
    fragments = ["cpp"],
)

def autocxx_library(
        name,
        visibility = None,
        target_compatible_with = None,
        libs = [],
        srcs = [],
        cxxbridge_srcs = [],
        override_cc_toolchain = None,
        deps = [],
        rs_deps = [],
        testonly = None,
        crate_features = None,
        crate_name = None,
        gen_debug = None):
    """A macro to generate Rust <-> C++ interop code with autocxx.

    Creates the following rules:
      * A rust_library with the given name, which includes the given srcs. Note that it will not
        include them directly due to how autocxx works, instead they will be copied into a
        generated file along with changes from autocxx.
      * A cc_library with the given name + `_cc`. This is for C++ code that wants to use APIs
        from the given Rust code. Rust dependencies should _not_ depend on this. The header for C++
        to #include will be named by the given name + `_cxxgen.h`.

      `deps` is for other `autocxx_library` rules. `rs_deps` is for dependencies of the Rust code.
    """
    library_gen_name = "%s__gen" % name
    _autocxx_library_gen(
        name = library_gen_name,
        visibility = ["//visibility:private"],
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        libs = libs,
        srcs = srcs,
        cxxbridge_srcs = cxxbridge_srcs,
        override_cc_toolchain = override_cc_toolchain,
        gen_debug = gen_debug,
    )
    gen_cc_srcs_name = "%s__cc_srcs" % name
    native.filegroup(
        name = gen_cc_srcs_name,
        visibility = ["//visibility:private"],
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        srcs = [library_gen_name],
        output_group = "cc_srcs",
    )
    gen_hdr_srcs_name = "%s__hdr_srcs" % name
    native.filegroup(
        name = gen_hdr_srcs_name,
        visibility = ["//visibility:private"],
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        srcs = [library_gen_name],
        output_group = "hdr_srcs",
    )
    gen_compile_data_name = "%s__compile_data" % name
    native.filegroup(
        name = gen_compile_data_name,
        visibility = ["//visibility:private"],
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        srcs = [library_gen_name],
        output_group = "compile_data",
    )
    gen_env_files_name = "%s__env_files" % name
    native.filegroup(
        name = gen_env_files_name,
        visibility = ["//visibility:private"],
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        srcs = [library_gen_name],
        output_group = "env_files",
    )
    cc_library_name = "%s__cc" % name
    native.cc_library(
        name = cc_library_name,
        visibility = visibility,
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        deps = deps + libs + [
            "@crate_index//:cxx_cc",
        ],
        srcs = [gen_cc_srcs_name],
        hdrs = [gen_hdr_srcs_name],
    )

    rust_library(
        name = name,
        visibility = visibility,
        target_compatible_with = target_compatible_with,
        testonly = testonly,
        srcs = srcs + cxxbridge_srcs,
        proc_macro_deps = [
            "@crate_index//:cxxbridge-macro",
        ],
        crate_features = crate_features,
        crate_name = crate_name,
        deps = deps + rs_deps + [
            cc_library_name,
            "@crate_index//:cxx",
            "//third_party/autocxx",
        ],
        compile_data = [gen_compile_data_name],
        rustc_env_files = [gen_env_files_name],
    )
