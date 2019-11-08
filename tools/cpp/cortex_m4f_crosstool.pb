toolchain {
  abi_version: "%NAME%"
  abi_libc_version: "%NAME%"
  builtin_sysroot: ""
  compiler: "gcc"
  host_system_name: "local"
  needsPic: false
  supports_gold_linker: false
  supports_incremental_linker: false
  supports_fission: false
  supports_interface_shared_objects: false
  supports_normalizing_ar: false
  supports_start_end_lib: false
  supports_thin_archives: false
  target_libc: "%NAME%"
  target_cpu: "%NAME%"
  target_system_name: "%NAME%"
  toolchain_identifier: "%NAME%"

  tool_path { name: "ar" path: "gcc_arm_none_eabi/arm-none-eabi-ar" }
  tool_path { name: "compat-ld" path: "gcc_arm_none_eabi/arm-none-eabi-ld" }
  tool_path { name: "cpp" path: "gcc_arm_none_eabi/arm-none-eabi-cpp" }
  tool_path { name: "dwp" path: "gcc_arm_none_eabi/arm-none-eabi-dwp" }
  tool_path { name: "gcc" path: "gcc_arm_none_eabi/arm-none-eabi-gcc" }
  tool_path { name: "gcov" path: "gcc_arm_none_eabi/arm-none-eabi-gcov" }
  # C(++) compiles invoke the compiler (as that is the one knowing where
  # to find libraries), but we provide LD so other rules can invoke the linker.
  tool_path { name: "ld" path: "gcc_arm_none_eabi/arm-none-eabi-ld" }
  tool_path { name: "nm" path: "gcc_arm_none_eabi/arm-none-eabi-nm" }
  tool_path { name: "objcopy" path: "gcc_arm_none_eabi/arm-none-eabi-objcopy" }
  objcopy_embed_flag: "-I"
  objcopy_embed_flag: "binary"
  tool_path { name: "objdump" path: "gcc_arm_none_eabi/arm-none-eabi-objdump" }
  tool_path { name: "strip" path: "gcc_arm_none_eabi/arm-none-eabi-strip" }
  linking_mode_flags { mode: FULLY_STATIC }

  # TODO(bazel-team): In theory, the path here ought to exactly match the path
  # used by gcc. That works because bazel currently doesn't track files at
  # absolute locations and has no remote execution, yet. However, this will need
  # to be fixed, maybe with auto-detection?
  cxx_builtin_include_directory: '/usr/lib/gcc/arm-none-eabi/4.8/include'
  cxx_builtin_include_directory: '/usr/lib/gcc/arm-none-eabi/4.8/include-fixed'
  cxx_builtin_include_directory: '/usr/lib/arm-none-eabi/include'
  cxx_builtin_include_directory: '/usr/include/newlib',

  feature {
    name: "dbg"
    implies: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-fno-omit-frame-pointer"
      }
    }
  }

  feature {
    name: "opt"
    implies: "all_modes"
  }
  feature {
    name: "fastbuild"
    implies: "all_modes"
  }

  feature {
    name: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c-compile"
      flag_group {
        flag: "--std=gnu99"
      }
    }
    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "--std=gnu++1z"
        flag: "-fno-exceptions"
        flag: "-fno-rtti"
      }
    }
  }

  # Anticipated future default.
  # This makes GCC and Clang do what we want when called through symlinks.
  unfiltered_cxx_flag: "-no-canonical-prefixes"
  linker_flag: "-no-canonical-prefixes"

  # Things that the code wants defined.
  compiler_flag: "-D__STDC_FORMAT_MACROS"
  compiler_flag: "-D__STDC_CONSTANT_MACROS"
  compiler_flag: "-D__STDC_LIMIT_MACROS"

  # Some identifiers for what MCU we're using.
  compiler_flag: "-D%CPU%"
  compiler_flag: "-DF_CPU=%F_CPU%"

  compiler_flag: "-Wl,--gc-sections"

  # Newlib's stdint.h does this, but GCC's freestanding stdint.h doesn't use
  # newlib's so we have to do it manually...
  compiler_flag: "-D__have_long32"

  # Make C++ compilation deterministic. Use linkstamping instead of these
  # compiler symbols.
  unfiltered_cxx_flag: "-Wno-builtin-macro-redefined"
  unfiltered_cxx_flag: "-D__DATE__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIMESTAMP__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIME__=\"redacted\""

  # Security hardening on by default.
  # Conservative choice; -D_FORTIFY_SOURCE=2 may be unsafe in some cases.
  # We need to undef it before redefining it as some distributions now have
  # it enabled by default.
  compiler_flag: "-fstack-protector"
  compiler_flag: "-mcpu=cortex-m4"
  compiler_flag: "-mfpu=fpv4-sp-d16"
  compiler_flag: "-mthumb"
  compiler_flag: "-mfloat-abi=hard"
  compiler_flag: "-fno-strict-aliasing"
  linker_flag: "-mcpu=cortex-m4"
  linker_flag: "-mfpu=fpv4-sp-d16"
  linker_flag: "-mthumb"
  linker_flag: "-mfloat-abi=hard"
  linker_flag: "-fno-strict-aliasing"
  linker_flag: "--specs=nano.specs"

  # Pretty much everything needs this, including parts of the glibc STL...
  linker_flag: "-lgcc"
  linker_flag: "-lstdc++_nano"
  linker_flag: "-lm"
  linker_flag: "-lc_nano"
  linker_flag: "-T%LINKER_SCRIPT%"
  linker_flag: "-Tmotors/core/kinetis_sections.ld"

  compiler_flag: "-fmessage-length=80"
  compiler_flag: "-fmax-errors=20"

  compiler_flag: "-Wall"
  compiler_flag: "-Wextra"
  compiler_flag: "-Wpointer-arith"
  compiler_flag: "-Wcast-qual"
  compiler_flag: "-Wwrite-strings"
  compiler_flag: "-Wtype-limits"
  compiler_flag: "-Wsign-compare"
  compiler_flag: "-Wformat=2"
  compiler_flag: "-Werror"
  compiler_flag: "-Wstrict-aliasing=2"

  # TODO(Brian): Drop these once we upgrade Eigen.
  compiler_flag: "-Wno-misleading-indentation"
  compiler_flag: "-Wno-int-in-bool-context"

  # Be annoying about using doubles places we probably didn't mean to, because
  # the FPU only does single-precision.
  compiler_flag: "-Wdouble-promotion"

  # Don't use temp files while compiling.
  compiler_flag: "-pipe"

  # Stamp the binary with a unique identifier.
  # TODO(austin): Put these back in.
  #linker_flag: "-Wl,--build-id=md5"
  #linker_flag: "-Wl,--hash-style=gnu"

  # Enable debug symbols.
  compiler_flag: "-g"

  # Common symbols are weird and not what we want, so just give multiple
  # declaration errors instead.
  compiler_flag: "-fno-common"

  # We're not a hosted environment (no file IO, main is called from our code,
  # etc).
  compiler_flag: "-ffreestanding"
  # However, we still want to optimize things like memcpy.
  compiler_flag: "-fbuiltin"

  compilation_mode_flags {
    mode: OPT

    # Freescale recommends this combination for reducing cycle count.
    # http://www.nxp.com/assets/documents/data/en/application-notes/AN4808.pdf
    compiler_flag: "-O2"
    compiler_flag: "-finline-functions"

    # This is definitely worth it for us. It makes the FPU a lot more useful,
    # especially with complex arithmetic, which matters a lot.
    compiler_flag: "-ffast-math"

    # It seems like this is a good idea, at least for the number crunching code.
    # Might want to look into moving it to copts on specific rules if the code
    # size increase becomes a problem.
    compiler_flag: "-funroll-loops"

    # Disable assertions
    compiler_flag: "-DNDEBUG"

    # Removal of unused code and data at link time (can this increase binary size in some cases?).
    compiler_flag: "-ffunction-sections"
    #compiler_flag: "-fdata-sections"
    linker_flag: "-Wl,--gc-sections"
  }

  feature {
    name: 'include_paths'
    flag_set {
      action: 'preprocess-assemble'
      action: 'c-compile'
      action: 'c++-compile'
      action: 'c++-header-parsing'
      action: 'c++-header-preprocessing'
      action: 'c++-module-compile'
      flag_group {
        iterate_over: 'quote_include_paths'
        flag: '-iquote'
        flag: '%{quote_include_paths}'
      }
      flag_group {
        iterate_over: 'include_paths'
        flag: '-I%{include_paths}'
      }
      flag_group {
        iterate_over: 'system_include_paths'
        flag: '-I'
        flag: '%{system_include_paths}'
      }
    }
  }
}
