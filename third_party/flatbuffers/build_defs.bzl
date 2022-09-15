# Description:
#   BUILD rules for generating flatbuffer files in various languages.

"""
Rules for building C++ flatbuffers with Bazel.
"""

load("@io_bazel_rules_go//go:def.bzl", "go_library")
load("@rules_rust//rust:defs.bzl", "rust_library")
load("@rules_rust//rust:rust_common.bzl", "CrateInfo")
load("@build_bazel_rules_nodejs//:index.bzl", "js_library")
load("@npm//@bazel/typescript:index.bzl", "ts_project")
load("@rules_cc//cc:defs.bzl", "cc_library")

flatc_path = "@com_github_google_flatbuffers//:flatc"

DEFAULT_INCLUDE_PATHS = [
    "./",
]

DEFAULT_FLATC_ARGS = [
    "--gen-object-api",
    "--gen-compare",
    "--keep-prefix",
    "--cpp-std",
    "c++17",
    "--require-explicit-ids",
    "--gen-mutable",
    "--reflect-names",
    "--cpp-ptr-type",
    "flatbuffers::unique_ptr",
    "--force-empty",
    "--scoped-enums",
    "--gen-name-strings",
]

DEFAULT_FLATC_GO_ARGS = [
    "--gen-onefile",
    "--gen-object-api",
    "--require-explicit-ids",
]

DEFAULT_FLATC_RUST_ARGS = [
    "--gen-object-api",
    "--require-explicit-ids",
    "--gen-name-strings",
]

"""Contains information about a set of flatbuffers which have their code for
reading/writing generated in a single library-style rule.

Fields:
    srcs: [File], the .fbs source files
"""
FlatbufferLibraryInfo = provider()

def _flatbuffer_library_compile_impl(ctx):
    outs = []
    commands = []
    for src in ctx.files.srcs:
        if ctx.attr.tables_for_filenames:
            out_dir = None
            for table in ctx.attr.tables_for_filenames:
                out = ctx.actions.declare_file(ctx.attr.out_prefix + table + ctx.attr.output_suffix)
                this_out_dir = "/".join(out.dirname.split("/")[:-(len(ctx.attr.out_prefix.split("/")) - 1)])
                if out_dir:
                    if this_out_dir != out_dir:
                        fail("Trying to write to multiple directories")
                else:
                    out_dir = this_out_dir
                outs.append(out)
        else:
            out = ctx.actions.declare_file(ctx.attr.out_prefix + src.basename.replace(".fbs", "") + ctx.attr.output_suffix)
            outs.append(out)
            out_dir = out.dirname
        arguments = [ctx.executable._flatc.path]
        for path in ctx.attr.include_paths:
            for subpath in ["", ctx.bin_dir.path + "/"]:
                arguments.append("-I")
                arguments.append(subpath + path)
        arguments.append("-I")
        arguments.append("%s.runfiles/com_github_google_flatbuffers" % ctx.executable._flatc.path)
        arguments.extend(ctx.attr.flatc_args)
        arguments.extend(ctx.attr.language_flags)
        arguments.extend([
            "-o",
            out_dir,
        ])
        arguments.append(src.path)
        commands.append(arguments)
    ctx.actions.run_shell(
        outputs = outs,
        inputs = ctx.files.srcs + ctx.files.includes,
        tools = [ctx.executable._flatc],
        command = " && ".join([" ".join(arguments) for arguments in commands]),
        mnemonic = "Flatc",
        progress_message = "Generating flatbuffer files for %{input}:",
    )
    return [DefaultInfo(files = depset(outs), runfiles = ctx.runfiles(files = outs)), FlatbufferLibraryInfo(srcs = ctx.files.srcs)]

_flatbuffer_library_compile = rule(
    implementation = _flatbuffer_library_compile_impl,
    attrs = {
        "srcs": attr.label_list(mandatory = True, allow_files = True),
        "output_suffix": attr.string(mandatory = True),
        "tables_for_filenames": attr.string_list(mandatory = False),
        "language_flags": attr.string_list(mandatory = True),
        "includes": attr.label_list(default = [], allow_files = True),
        "include_paths": attr.string_list(default = []),
        "flatc_args": attr.string_list(default = []),
        "out_prefix": attr.string(default = ""),
        "_flatc": attr.label(executable = True, cfg = "exec", default = Label(flatc_path)),
    },
)

def flatbuffer_library_public(
        name,
        srcs,
        output_suffix,
        language_flag,
        out_prefix = "",
        tables_for_filenames = None,
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_ARGS,
        reflection_name = "",
        reflection_visibility = None,
        compatible_with = None,
        restricted_to = None,
        target_compatible_with = None,
        output_to_bindir = False,
        visibility = None):
    """Generates code files for reading/writing the given flatbuffers in the
    requested language using the public compiler.

    Args:
      name: Rule name.
      srcs: Source .fbs files. Sent in order to the compiler.
      output_suffix: Suffix for output files from flatc.
      language_flag: Target language flag. One of [-c, -j, -js].
      out_prefix: Prepend this path to the front of all generated files except on
          single source targets. Usually is a directory name.
      includes: Optional, list of filegroups of schemas that the srcs depend on.
      include_paths: Optional, list of paths the includes files can be found in.
      flatc_args: Optional, list of additional arguments to pass to flatc.
      reflection_name: Optional, if set this will generate the flatbuffer
        reflection binaries for the schemas.
      reflection_visibility: The visibility of the generated reflection Fileset.
      output_to_bindir: Passed to genrule for output to bin directory.
      compatible_with: Optional, The list of environments this rule can be
        built for, in addition to default-supported environments.
      restricted_to: Optional, The list of environments this rule can be built
        for, instead of default-supported environments.
      target_compatible_with: Optional, The list of target platform constraints
        to use.
      output_to_bindir: Passed to genrule for output to bin directory.


    This rule creates a filegroup(name) with all generated source files, and
    optionally a Fileset([reflection_name]) with all generated reflection
    binaries.
    """
    _flatbuffer_library_compile(
        name = name,
        srcs = srcs,
        output_suffix = output_suffix,
        language_flags = [language_flag],
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        out_prefix = out_prefix,
        tables_for_filenames = tables_for_filenames,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        restricted_to = restricted_to,
        visibility = visibility,
    )

    if reflection_name:
        _flatbuffer_library_compile(
            name = "%s_out" % reflection_name,
            srcs = srcs,
            output_suffix = ".bfbs",
            language_flags = ["-b", "--schema"],
            includes = includes,
            include_paths = include_paths,
            flatc_args = flatc_args,
            out_prefix = out_prefix,
            compatible_with = compatible_with,
            target_compatible_with = target_compatible_with,
            restricted_to = restricted_to,
            visibility = reflection_visibility,
        )

def flatbuffer_cc_library(
        name,
        srcs,
        srcs_filegroup_name = "",
        out_prefix = "",
        deps = [],
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        cc_include_paths = [],
        flatc_args = DEFAULT_FLATC_ARGS,
        visibility = None,
        compatible_with = None,
        restricted_to = None,
        target_compatible_with = None,
        srcs_filegroup_visibility = None,
        gen_reflections = False):
    """A cc_library with the generated reader/writers for the given flatbuffer definitions.

    Args:
      name: Rule name.
      srcs: Source .fbs files. Sent in order to the compiler.
      srcs_filegroup_name: Name of the output filegroup that holds srcs. Pass this
          filegroup into the `includes` parameter of any other
          flatbuffer_cc_library that depends on this one's schemas.
      out_prefix: Prepend this path to the front of all generated files. Usually
          is a directory name.
      deps: Optional, list of other flatbuffer_cc_library's to depend on. Cannot be specified
          alongside includes.
      includes: Optional, list of filegroups of schemas that the srcs depend on.
          Use of this is discouraged, and may be deprecated.
      include_paths: Optional, list of paths the includes files can be found in.
      cc_include_paths: Optional, list of paths to add to the cc_library includes attribute.
      flatc_args: Optional list of additional arguments to pass to flatc
          (e.g. --gen-mutable).
      visibility: The visibility of the generated cc_library. By default, use the
          default visibility of the project.
      target_compatible_with: Optional, the list of constraints the target
        platform must satisfy for this target to be considered compatible.
      srcs_filegroup_visibility: The visibility of the generated srcs filegroup.
          By default, use the value of the visibility parameter above.
      gen_reflections: Optional, if true this will generate the flatbuffer
        reflection binaries for the schemas.
      compatible_with: Optional, The list of environments this rule can be built
        for, in addition to default-supported environments.
      restricted_to: Optional, The list of environments this rule can be built
        for, instead of default-supported environments.
      target_compatible_with: Optional, The list of target platform constraints
        to use.

    This produces:
      filegroup([name]_srcs): all generated .h files.
      filegroup(srcs_filegroup_name if specified, or [name]_includes if not):
          Other flatbuffer_cc_library's can pass this in for their `includes`
          parameter, if they depend on the schemas in this library.
      Fileset([name]_reflection): (Optional) all generated reflection binaries.
      cc_library([name]): library with sources and flatbuffers deps.
    """
    if deps and includes:
        # There is no inherent reason we couldn't support both, but this discourages
        # use of includes without good reason.
        fail("Cannot specify both deps and include in flatbuffer_cc_library.")
    if deps:
        includes = [d + "_includes" for d in deps]
    reflection_name = "%s_reflection" % name if gen_reflections else ""

    srcs_lib = "%s_srcs" % (name)
    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_generated.h",
        language_flag = "-c",
        out_prefix = out_prefix,
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        reflection_name = reflection_name,
        reflection_visibility = visibility,
    )
    native.cc_library(
        name = name,
        hdrs = [
            ":" + srcs_lib,
        ],
        srcs = [
            ":" + srcs_lib,
        ],
        features = [
            "-parse_headers",
        ],
        deps = [
            "@com_github_google_flatbuffers//:runtime_cc",
            "@com_github_google_flatbuffers//:flatbuffers",
        ] + deps,
        includes = cc_include_paths,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        linkstatic = 1,
        visibility = visibility,
    )

    # A filegroup for the `srcs`. That is, all the schema files for this
    # Flatbuffer set.
    native.filegroup(
        name = srcs_filegroup_name if srcs_filegroup_name else "%s_includes" % (name),
        srcs = srcs + includes,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        visibility = srcs_filegroup_visibility if srcs_filegroup_visibility != None else visibility,
    )

def flatbuffer_py_library(
        name,
        srcs,
        namespace,
        tables,
        compatible_with = None,
        target_compatible_with = None,
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_ARGS,
        visibility = None,
        srcs_filegroup_visibility = None):
    """Generates a py_library rule for a given flatbuffer definition.

    Args:
      name: Name of the generated py_library rule.
      srcs: Source .fbs file(s).
      namespace: Namespace of the specified flatbuffer schema. Until
        we make the rules sophisticated enough to figure out what
        python files will be generated from a given schema, the user
        has to manually specify this.
      tables: List of table names--currently, we don't do anything to
        automatically figure out how to handle the fact that a separate
        python file will be generated for every table definition, and that
        we can't know what files will be output until after the file has
        been parsed. As such, we just force the user to manually specify
        things.
    """

    srcs_lib = "%s_srcs" % (name)
    if not tables:
        fail("Must specify the list of tables")
    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = ".py",
        out_prefix = namespace.replace(".", "/") + "/",
        tables_for_filenames = tables,
        language_flag = "--python",
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = ["//visibility:private"],
    )
    native.py_library(
        name = name,
        srcs = [srcs_lib],
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        imports = ["."],
        deps = ["@com_github_google_flatbuffers//:flatpy"],
    )

def flatbuffer_go_library(
        name,
        srcs,
        importpath,
        compatible_with = None,
        target_compatible_with = None,
        includes = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_GO_ARGS,
        visibility = None,
        srcs_filegroup_visibility = None):
    srcs_lib = "%s_srcs" % (name)
    flatc_args = flatc_args + ["--go-namespace", importpath.split("/")[-1]]

    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_generated.go",
        language_flag = "--go",
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = ["//visibility:private"],
    )
    go_library(
        name = name,
        srcs = [srcs_lib],
        deps = ["@com_github_google_flatbuffers//go"],
        importpath = importpath,
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )

def _flatbuffer_rust_lib_gen_impl(ctx):
    # TODO(Brian): I think this needs changes to properly handle multiple .fbs files in a rule.
    uses = []
    for (dep, dep_srcs) in zip(ctx.attr.deps, ctx.attr.dep_srcs):
        for dep_src in dep_srcs[FlatbufferLibraryInfo].srcs:
            uses.append((dep[CrateInfo].name, dep_src.basename.replace(".fbs", "_generated")))
    lib_rs_content = "\n".join(
        [
            "// Automatically generated by the Flatbuffers Bazel rules. Do not modify",
            "#![allow(unused_imports)]",
        ] + ["use %s as %s;" % (crate, use_as) for (crate, use_as) in uses] +
        ["include!(\"%s\");" % src.basename for src in ctx.files.srcs_lib],
    )
    output = ctx.actions.declare_file(ctx.attr.name + "_lib.rs")
    ctx.actions.write(
        output = output,
        content = lib_rs_content,
    )
    return [DefaultInfo(files = depset([output]))]

"""Generates a lib.rs for a flatbuffer_rust_library.

flatc generates individual .rs files for us. It can also generate a top-level mod.rs to be included
in a crate, but that is laid out to include all flatbuffers files in a project. That's not a good
fit for Bazel rules and monorepos, so we generate an alternative that imports all dependencies under
their expected names."""
_flatbuffer_rust_lib_gen = rule(
    implementation = _flatbuffer_rust_lib_gen_impl,
    attrs = {
        "srcs_lib": attr.label(mandatory = True, doc = "The generated srcs for this rule"),
        "dep_srcs": attr.label_list(mandatory = True, providers = [FlatbufferLibraryInfo], doc = "The _srcs rules for all our direct dependencies"),
        "deps": attr.label_list(mandatory = True, providers = [CrateInfo]),
    },
)

def flatbuffer_rust_library(
        name,
        srcs,
        compatible_with = None,
        target_compatible_with = None,
        deps = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_RUST_ARGS,
        include_reflection = True,
        crate_name = None,
        visibility = None,
        srcs_filegroup_visibility = None):
    includes = [d + "_includes" for d in deps]
    srcs_lib = "%s_srcs" % (name)
    lib_gen = "%s_lib_gen" % (name)
    deps = list(deps)
    if include_reflection:
        deps.append("@com_github_google_flatbuffers//reflection:reflection_rust_fbs")

    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        language_flag = "--rust",
        output_suffix = "_generated.rs",
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
        visibility = visibility,
    )
    _flatbuffer_rust_lib_gen(
        name = lib_gen,
        deps = deps,
        dep_srcs = [dep + "_srcs" for dep in deps],
        srcs_lib = srcs_lib,
        visibility = ["//visibility:private"],
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )
    rust_library(
        name = name,
        srcs = [srcs_lib, lib_gen],
        crate_root = lib_gen,
        crate_name = crate_name,
        deps = ["@com_github_google_flatbuffers//rust"] + deps,
        edition = "2018",
        visibility = visibility,
        compatible_with = compatible_with,
        target_compatible_with = target_compatible_with,
    )
