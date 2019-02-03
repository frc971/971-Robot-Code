licenses(["notice"])

load("@//tools/build_rules:fortran.bzl", "f2c_copts")
load("@//tools/build_rules:select.bzl", "compiler_select")

genrule(
    name = "create_sysdep1",
    srcs = ["F2CLIBS/libf2c/sysdep1.h0"],
    outs = ["extra_includes/sysdep1.h"],
    cmd = "cp $< $@",
)

_xerbla_patch = """
--- xerbla.c   2018-02-19 19:58:03.685420156 -0500
+++ xerbla.c   2018-02-19 19:59:02.993259128 -0500
@@ -55,7 +55,7 @@ static integer c__1 = 1;
 /*     .. */
 /*     .. Executable Statements .. */
 
-    printf("** On entry to %6s, parameter number %2i had an illegal value\\n",
+    printf("** On entry to %6s, parameter number %2li had an illegal value\\n",
\t\tsrname, *info);
 
 
"""

genrule(
    name = "patch_xerbla",
    srcs = ["SRC/xerbla.c"],
    outs = ["patched_xerbla.c"],
    cmd = "\n".join([
        "cp $< $@",
        "$(location @patch) $@ - <<END",
        _xerbla_patch,
        "END",
    ]),
    tools = ["@patch"],
)

_err_patch = """
--- err.c  2018-02-19 20:06:40.532033141 -0500
+++ err.c  2018-02-19 20:10:25.907439219 -0500
@@ -164,8 +164,10 @@ f__fatal(int n, const char *s)
 \tif (f__curunit) {
 \t\tfprintf(stderr,"apparent state: unit %d ",
 \t\t\t(int)(f__curunit-f__units));
-\t\tfprintf(stderr, f__curunit->ufnm ? "named %s\\n" : "(unnamed)\\n",
-\t\t\tf__curunit->ufnm);
+\t\tif (f__curunit->ufnm)
+\t\t\tfprintf(stderr, "named %s\\n", f__curunit->ufnm);
+\t\telse
+\t\t\tfprintf(stderr, "(unnamed)\\n");
 \t\t}
 \telse
 \t\tfprintf(stderr,"apparent state: internal I/O\\n");
"""

genrule(
    name = "patch_err",
    srcs = ["F2CLIBS/libf2c/err.c"],
    outs = ["patched_err.c"],
    cmd = "\n".join([
        "cp $< $@",
        "$(location @patch) $@ - <<END",
        _err_patch,
        "END",
    ]),
    tools = ["@patch"],
)

cc_library(
    name = "clapack",
    srcs = glob(
        include = [
            "SRC/*.c",
            "BLAS/SRC/*.c",
        ],
        exclude = [
            # These are duplicated in SRC (with the ones in SRC a bit more cleaned up).
            "BLAS/SRC/xerbla.c",
            "BLAS/SRC/xerbla_array.c",

            # We need to use a patched version of this.
            "SRC/xerbla.c",

            # Files requiring XBLAS (extended precision), which we don't have.
            "SRC/sgesvxx.c",
            "SRC/sgerfsx.c",
            "SRC/sla_gerfsx_extended.c",
            "SRC/sla_geamv.c",
            "SRC/sla_gercond.c",
            "SRC/sla_gerpvgrw.c",
            "SRC/ssysvxx.c",
            "SRC/ssyrfsx.c",
            "SRC/sla_syrfsx_extended.c",
            "SRC/sla_syamv.c",
            "SRC/sla_syrcond.c",
            "SRC/sla_syrpvgrw.c",
            "SRC/sposvxx.c",
            "SRC/sporfsx.c",
            "SRC/sla_porfsx_extended.c",
            "SRC/sla_porcond.c",
            "SRC/sla_porpvgrw.c",
            "SRC/sgbsvxx.c",
            "SRC/sgbrfsx.c",
            "SRC/sla_gbrfsx_extended.c",
            "SRC/sla_gbamv.c",
            "SRC/sla_gbrcond.c",
            "SRC/sla_gbrpvgrw.c",
            "SRC/sla_lin_berr.c",
            "SRC/slarscl2.c",
            "SRC/slascl2.c",
            "SRC/sla_wwaddw.c",
            "SRC/cgesvxx.c",
            "SRC/cgerfsx.c",
            "SRC/cla_gerfsx_extended.c",
            "SRC/cla_geamv.c",
            "SRC/cla_gercond_c.c",
            "SRC/cla_gercond_x.c",
            "SRC/cla_gerpvgrw.c",
            "SRC/csysvxx.c",
            "SRC/csyrfsx.c",
            "SRC/cla_syrfsx_extended.c",
            "SRC/cla_syamv.c",
            "SRC/cla_syrcond_c.c",
            "SRC/cla_syrcond_x.c",
            "SRC/cla_syrpvgrw.c",
            "SRC/cposvxx.c",
            "SRC/cporfsx.c",
            "SRC/cla_porfsx_extended.c",
            "SRC/cla_porcond_c.c",
            "SRC/cla_porcond_x.c",
            "SRC/cla_porpvgrw.c",
            "SRC/cgbsvxx.c",
            "SRC/cgbrfsx.c",
            "SRC/cla_gbrfsx_extended.c",
            "SRC/cla_gbamv.c",
            "SRC/cla_gbrcond_c.c",
            "SRC/cla_gbrcond_x.c",
            "SRC/cla_gbrpvgrw.c",
            "SRC/chesvxx.c",
            "SRC/cherfsx.c",
            "SRC/cla_herfsx_extended.c",
            "SRC/cla_heamv.c",
            "SRC/cla_hercond_c.c",
            "SRC/cla_hercond_x.c",
            "SRC/cla_herpvgrw.c",
            "SRC/cla_lin_berr.c",
            "SRC/clarscl2.c",
            "SRC/clascl2.c",
            "SRC/cla_wwaddw.c",
            "SRC/dgesvxx.c",
            "SRC/dgerfsx.c",
            "SRC/dla_gerfsx_extended.c",
            "SRC/dla_geamv.c",
            "SRC/dla_gercond.c",
            "SRC/dla_gerpvgrw.c",
            "SRC/dsysvxx.c",
            "SRC/dsyrfsx.c",
            "SRC/dla_syrfsx_extended.c",
            "SRC/dla_syamv.c",
            "SRC/dla_syrcond.c",
            "SRC/dla_syrpvgrw.c",
            "SRC/dposvxx.c",
            "SRC/dporfsx.c",
            "SRC/dla_porfsx_extended.c",
            "SRC/dla_porcond.c",
            "SRC/dla_porpvgrw.c",
            "SRC/dgbsvxx.c",
            "SRC/dgbrfsx.c",
            "SRC/dla_gbrfsx_extended.c",
            "SRC/dla_gbamv.c",
            "SRC/dla_gbrcond.c",
            "SRC/dla_gbrpvgrw.c",
            "SRC/dla_lin_berr.c",
            "SRC/dlarscl2.c",
            "SRC/dlascl2.c",
            "SRC/dla_wwaddw.c",
            "SRC/zgesvxx.c",
            "SRC/zgerfsx.c",
            "SRC/zla_gerfsx_extended.c",
            "SRC/zla_geamv.c",
            "SRC/zla_gercond_c.c",
            "SRC/zla_gercond_x.c",
            "SRC/zla_gerpvgrw.c",
            "SRC/zsysvxx.c",
            "SRC/zsyrfsx.c",
            "SRC/zla_syrfsx_extended.c",
            "SRC/zla_syamv.c",
            "SRC/zla_syrcond_c.c",
            "SRC/zla_syrcond_x.c",
            "SRC/zla_syrpvgrw.c",
            "SRC/zposvxx.c",
            "SRC/zporfsx.c",
            "SRC/zla_porfsx_extended.c",
            "SRC/zla_porcond_c.c",
            "SRC/zla_porcond_x.c",
            "SRC/zla_porpvgrw.c",
            "SRC/zgbsvxx.c",
            "SRC/zgbrfsx.c",
            "SRC/zla_gbrfsx_extended.c",
            "SRC/zla_gbamv.c",
            "SRC/zla_gbrcond_c.c",
            "SRC/zla_gbrcond_x.c",
            "SRC/zla_gbrpvgrw.c",
            "SRC/zhesvxx.c",
            "SRC/zherfsx.c",
            "SRC/zla_herfsx_extended.c",
            "SRC/zla_heamv.c",
            "SRC/zla_hercond_c.c",
            "SRC/zla_hercond_x.c",
            "SRC/zla_herpvgrw.c",
            "SRC/zla_lin_berr.c",
            "SRC/zlarscl2.c",
            "SRC/zlascl2.c",
            "SRC/zla_wwaddw.c",
        ],
    ) + [
        "INSTALL/dlamch.c",
        "INSTALL/slamch.c",
        "patched_xerbla.c",
        "patched_err.c",
        "F2CLIBS/libf2c/s_cat.c",
        "F2CLIBS/libf2c/d_lg10.c",
        "F2CLIBS/libf2c/d_sign.c",
        "F2CLIBS/libf2c/i_dnnt.c",
        "F2CLIBS/libf2c/pow_di.c",
        "F2CLIBS/libf2c/s_copy.c",
        "F2CLIBS/libf2c/s_cmp.c",
        "F2CLIBS/libf2c/i_nint.c",
        "F2CLIBS/libf2c/f77_aloc.c",
        "F2CLIBS/libf2c/exit_.c",
        "F2CLIBS/libf2c/r_cnjg.c",
        "F2CLIBS/libf2c/c_abs.c",
        "F2CLIBS/libf2c/r_imag.c",
        "F2CLIBS/libf2c/c_div.c",
        "F2CLIBS/libf2c/c_exp.c",
        "F2CLIBS/libf2c/d_imag.c",
        "F2CLIBS/libf2c/r_sign.c",
        "F2CLIBS/libf2c/d_cnjg.c",
        "F2CLIBS/libf2c/z_abs.c",
        "F2CLIBS/libf2c/z_div.c",
        "F2CLIBS/libf2c/z_exp.c",
        "F2CLIBS/libf2c/z_sqrt.c",
        "F2CLIBS/libf2c/pow_dd.c",
        "F2CLIBS/libf2c/pow_ri.c",
        "F2CLIBS/libf2c/pow_ci.c",
        "F2CLIBS/libf2c/pow_ii.c",
        "F2CLIBS/libf2c/pow_zi.c",
        "F2CLIBS/libf2c/c_sqrt.c",
        "F2CLIBS/libf2c/r_lg10.c",
        "F2CLIBS/libf2c/i_len.c",
        "F2CLIBS/libf2c/cabs.c",
        "F2CLIBS/libf2c/sig_die.c",
        "F2CLIBS/libf2c/close.c",
        "F2CLIBS/libf2c/open.c",
        "F2CLIBS/libf2c/endfile.c",
        "F2CLIBS/libf2c/util.c",
        "F2CLIBS/libf2c/iio.c",
        "F2CLIBS/libf2c/fmt.c",
        "F2CLIBS/libf2c/rdfmt.c",
        "F2CLIBS/libf2c/wrtfmt.c",
        #'F2CLIBS/libf2c/ctype.c',
        "F2CLIBS/libf2c/wref.c",
        "F2CLIBS/libf2c/fmtlib.c",
        "F2CLIBS/libf2c/lread.c",
        "F2CLIBS/libf2c/rsfe.c",
        "F2CLIBS/libf2c/sfe.c",
        "F2CLIBS/libf2c/dolio.c",
        "F2CLIBS/libf2c/wsfe.c",
        "extra_includes/sysdep1.h",
    ],
    hdrs = glob(
        [
            "INCLUDE/*.h",
            "F2CLIBS/libf2c/*.h",
        ],
        exclude = ["F2CLIBS/libf2c/ctype.h"],
    ),
    copts = f2c_copts + [
        "-Wno-sign-compare",
        "-Wno-cast-qual",
        "-Wno-cast-align",

        # Some files don't #include system headers when they should. sysdep1.h
        # messes with feature test macros, so it always has to come first.
        "-include",
        "sysdep1.h",
        "-include",
        "stdio.h",

        # Don't mangle the names of all the BLAS symbols, because slicot needs to
        # call them directly.
        "-DNO_BLAS_WRAP",
    ] + compiler_select({
        "clang": [
            "-Wno-self-assign",
        ],
        "gcc": [
            "-Wno-discarded-qualifiers",
            "-Wno-maybe-uninitialized",
            "-Wno-unused-but-set-variable",
        ],
    }),
    includes = [
        "F2CLIBS/libf2c",
        "INCLUDE",
        "extra_includes",
    ],
    visibility = ["//visibility:public"],
)
