"""
Rules for building typescript flatbuffers with Bazel.
"""

load("@aspect_rules_js//js:defs.bzl", "js_library")
load("@aspect_rules_ts//ts:defs.bzl", "ts_project")
load(":build_defs.bzl", "DEFAULT_INCLUDE_PATHS", "flatbuffer_library_public")

DEFAULT_FLATC_TS_ARGS = [
    "--gen-object-api",
    "--gen-mutable",
    "--reflect-names",
    "--gen-name-strings",
    "--ts-flat-files",
    "--keep-prefix",
]

def flatbuffer_ts_library(
        name,
        srcs,
        compatible_with = None,
        target_compatible_with = None,
        deps = [],
        include_paths = DEFAULT_INCLUDE_PATHS,
        flatc_args = DEFAULT_FLATC_TS_ARGS,
        visibility = None,
        restricted_to = None,
        include_reflection = True,
        package_name = None):
    """Generates a ts_library rule for a given flatbuffer definition.

    Args:
      name: Name of the generated ts_library rule.
      srcs: Source .fbs file(s).
      deps: Other flatbuffer_ts_library's to depend on. Note that currently
            you must specify all your transitive dependencies manually.
      include_paths: Optional, list of paths the includes files can be found in.
      flatc_args: Optional list of additional arguments to pass to flatc
          (e.g. --gen-mutable).
      visibility: The visibility of the generated cc_library. By default, use the
          default visibility of the project.
      compatible_with: Optional, The list of environments this rule can be built
        for, in addition to default-supported environments.
      restricted_to: Optional, The list of environments this rule can be built
        for, instead of default-supported environments.
      target_compatible_with: Optional, The list of target platform constraints
        to use.
      include_reflection: Optional, Whether to depend on the flatbuffer
        reflection library automatically. Only really relevant for the
        target that builds the reflection library itself.
      package_name: Optional, Package name to use for the generated code.
    """
    srcs_lib = "%s_srcs" % (name)

    # frc971-specific modification: Add a genrule that overwrites the imports for any flatbuffer
    # types (mostly just for reflection) because they need to point to external/, not to
    # third_party/.
    # TODO(james): There absolutely are better ways to do this, but this was the quick and dirty
    # one....
    outs = ["%s_generated.ts" % (s.replace(".fbs", "").split("/")[-1]) for s in srcs]
    includes = [d + "_includes" for d in deps]
    flatbuffer_library_public(
        name = srcs_lib,
        srcs = srcs,
        output_suffix = "_pregenerated.ts",
        language_flag = "--ts",
        includes = includes,
        include_paths = include_paths,
        flatc_args = flatc_args + ["--filename-suffix _pregenerated"],
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
    )
    genrule_cmd = " ".join([
        "SRCS=($(SRCS));",
        "OUTS=($(OUTS));",
        "for i in $${!SRCS[@]}; do",
        "sed \"s/'.*reflection\\/reflection_pregenerated/'flatbuffers_reflection\\/reflection_generated/\" $${SRCS[i]} > $${OUTS[i]};",
        "sed -i 's/_pregenerated/_generated/' $${OUTS[i]};",
        "done",
    ])
    native.genrule(
        name = name + "_reimporter.ts",
        srcs = [srcs_lib],
        outs = outs,
        cmd = genrule_cmd,
    )
    ts_project(
        name = name + "_ts",
        srcs = outs,
        declaration = True,
        visibility = visibility,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        supports_workers = False,
        tsconfig = {
            "compilerOptions": {
                "declaration": True,
                "lib": [
                    "ES2015",
                    "ES2020.BigInt",
                    "DOM",
                ],
                "module": "es2015",
                "moduleResolution": "node",
                "strict": True,
                "types": ["node"],
            },
        },
        deps = deps + [
            "@//:node_modules/flatbuffers",
            # TODO(phil): Figure out why @types/node isn't being picked up as a
            # transitivie dependencies.
            "@//:node_modules/@types/node",
        ] + (["@//:node_modules/flatbuffers_reflection"] if include_reflection else []),
    )
    js_library(
        name = name,
        visibility = visibility,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        target_compatible_with = target_compatible_with,
        srcs = [name + "_ts"],
    )
    native.filegroup(
        name = "%s_includes" % (name),
        srcs = srcs + includes,
        compatible_with = compatible_with,
        restricted_to = restricted_to,
        visibility = visibility,
    )
