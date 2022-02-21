genrule(
    name = "make_osqp_configure",
    outs = ["include/osqp_configure.h"],
    cmd = "echo > $@",
)

genrule(
    name = "make_qdldl_version",
    outs = ["lin_sys/direct/qdldl/qdldl_sources/include/qdldl_version.h"],
    cmd = "echo > $@",
)

genrule(
    name = "make_qdldl_types",
    outs = ["lin_sys/direct/qdldl/qdldl_sources/include/qdldl_types.h"],
    cmd = """cat << EOF > $@
#ifndef QDLDL_TYPES_H
# define QDLDL_TYPES_H

# ifdef __cplusplus
extern "C" {
# endif /* ifdef __cplusplus */

#include <limits.h> //for the QDLDL_INT_TYPE_MAX

// QDLDL integer and float types

typedef int    QDLDL_int;   /* for indices */
typedef double  QDLDL_float; /* for numerical values  */
typedef unsigned char   QDLDL_bool;  /* for boolean values  */

//Maximum value of the signed type QDLDL_int.
#define QDLDL_INT_MAX LLONG_MAX

# ifdef __cplusplus
}
# endif /* ifdef __cplusplus */

#endif /* ifndef QDLDL_TYPES_H */
EOF""",
)

cc_library(
    name = "osqp_includes",
    hdrs = [
        "include/auxil.h",
        "include/constants.h",
        "include/cs.h",
        "include/ctrlc.h",
        "include/glob_opts.h",
        "include/kkt.h",
        "include/lin_alg.h",
        "include/lin_sys.h",
        "include/osqp.h",
        "include/osqp_configure.h",
        "include/osqp_error.h",
        "include/polish.h",
        "include/proj.h",
        "include/scaling.h",
        "include/types.h",
        "include/util.h",
        "include/version.h",
        "lin_sys/lib_handler.h",
    ],
    defines = [
        "IS_LINUX",
        #"PRINTING",
        "PROFILING",
        "CTRLC",
        # TODO(austin): Use floats instead
        #"DFLOAT",
        #"DLONG",
        "ENABLE_MKL_PARDISO",
    ],
    includes = [
        "include",
        "lin_sys",
    ],
)

cc_library(
    name = "osqp",
    srcs = [
        "lin_sys/direct/pardiso/pardiso_interface.c",
        "lin_sys/direct/pardiso/pardiso_loader.c",
        "lin_sys/direct/qdldl/amd/src/SuiteSparse_config.c",
        "lin_sys/direct/qdldl/amd/src/amd_1.c",
        "lin_sys/direct/qdldl/amd/src/amd_2.c",
        "lin_sys/direct/qdldl/amd/src/amd_aat.c",
        "lin_sys/direct/qdldl/amd/src/amd_control.c",
        "lin_sys/direct/qdldl/amd/src/amd_defaults.c",
        "lin_sys/direct/qdldl/amd/src/amd_info.c",
        "lin_sys/direct/qdldl/amd/src/amd_order.c",
        "lin_sys/direct/qdldl/amd/src/amd_post_tree.c",
        "lin_sys/direct/qdldl/amd/src/amd_postorder.c",
        "lin_sys/direct/qdldl/amd/src/amd_preprocess.c",
        "lin_sys/direct/qdldl/amd/src/amd_valid.c",
        "lin_sys/direct/qdldl/qdldl_interface.c",
        "lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c",
        "lin_sys/lib_handler.c",
        "src/auxil.c",
        "src/cs.c",
        "src/ctrlc.c",
        "src/error.c",
        "src/kkt.c",
        "src/lin_alg.c",
        "src/lin_sys.c",
        "src/osqp.c",
        "src/polish.c",
        "src/proj.c",
        "src/scaling.c",
        "src/util.c",
    ],
    hdrs = [
        "lin_sys/direct/pardiso/pardiso_interface.h",
        "lin_sys/direct/pardiso/pardiso_loader.h",
        "lin_sys/direct/qdldl/amd/include/SuiteSparse_config.h",
        "lin_sys/direct/qdldl/amd/include/amd.h",
        "lin_sys/direct/qdldl/amd/include/amd_internal.h",
        "lin_sys/direct/qdldl/qdldl_interface.h",
        "lin_sys/direct/qdldl/qdldl_sources/include/qdldl.h",
        "lin_sys/direct/qdldl/qdldl_sources/include/qdldl_types.h",
        "lin_sys/direct/qdldl/qdldl_sources/include/qdldl_version.h",
    ],
    copts = [
        "-Wno-cast-qual",
    ],
    defines = [
        'QDLDL_VERSION_MAJOR="0"',
        'QDLDL_VERSION_MINOR="1"',
        'QDLDL_VERSION_PATCH="6"',
        'QDLDL_VERSION="0.1.6"',
    ],
    includes = [
        "lin_sys/direct/pardiso",
        "lin_sys/direct/qdldl",
        "lin_sys/direct/qdldl/amd/include",
        "lin_sys/direct/qdldl/qdldl_sources/include",
    ],
    linkopts = ["-ldl"],
    target_compatible_with = ["@platforms//os:linux"],
    visibility = ["//visibility:public"],
    deps = [
        ":osqp_includes",
    ],
)
