major_version: "local"
minor_version: ""
default_target_cpu: "same_as_host"

default_toolchain {
  cpu: "roborio"
  toolchain_identifier: "roborio_linux"
}

default_toolchain {
  cpu: "k8"
  toolchain_identifier: "k8_linux"
}

default_toolchain {
  cpu: "armeabi-v7a"
  toolchain_identifier: "stub_armeabi-v7a"
}

default_toolchain {
  cpu: "web"
  toolchain_identifier: "emscripten_toolchain"
}

default_toolchain {
  cpu: "armhf-debian"
  toolchain_identifier: "clang_linux_armhf"
}

default_toolchain {
  cpu: "cortex-m4f"
  toolchain_identifier: "cortex-m4f"
}

default_toolchain {
  cpu: "cortex-m4f-k22"
  toolchain_identifier: "cortex-m4f-k22"
}

toolchain {
  toolchain_identifier: "stub_armeabi-v7a"
  abi_version: "armeabi-v7a"
  abi_libc_version: "armeabi-v7a"
  builtin_sysroot: ""
  compiler: "compiler"
  host_system_name: "armeabi-v7a"
  needsPic: true
  supports_gold_linker: false
  supports_incremental_linker: false
  supports_fission: false
  supports_interface_shared_objects: false
  supports_normalizing_ar: false
  supports_start_end_lib: false
  supports_thin_archives: false
  target_libc: "armeabi-v7a"
  target_cpu: "armeabi-v7a"
  target_system_name: "armeabi-v7a"

  tool_path { name: "ar" path: "/bin/false" }
  tool_path { name: "compat-ld" path: "/bin/false" }
  tool_path { name: "cpp" path: "/bin/false" }
  tool_path { name: "dwp" path: "/bin/false" }
  tool_path { name: "gcc" path: "/bin/false" }
  tool_path { name: "gcov" path: "/bin/false" }
  tool_path { name: "ld" path: "/bin/false" }

  tool_path { name: "nm" path: "/bin/false" }
  tool_path { name: "objcopy" path: "/bin/false" }
  tool_path { name: "objdump" path: "/bin/false" }
  tool_path { name: "strip" path: "/bin/false" }
}

toolchain {
  abi_version: "local"
  abi_libc_version: "local"
  builtin_sysroot: ""
  compiler: "clang"
  host_system_name: "local"
  needsPic: true
  supports_gold_linker: false
  supports_incremental_linker: false
  supports_fission: false
  supports_interface_shared_objects: false
  supports_normalizing_ar: false
  supports_start_end_lib: false
  supports_thin_archives: false
  target_libc: "local"
  target_cpu: "k8"
  target_system_name: "k8"
  toolchain_identifier: "k8_linux"

  # These paths are relative to //tools/cpp.
  tool_path { name: "ar" path: "clang_6p0/x86_64-linux-gnu-ar" }
  tool_path { name: "compat-ld" path: "clang_6p0/x86_64-linux-gnu-ld" }
  tool_path { name: "cpp" path: "clang_6p0/x86_64-linux-gnu-cpp" }
  tool_path { name: "dwp" path: "clang_6p0/x86_64-linux-gnu-dwp" }
  tool_path { name: "gcc" path: "clang_6p0/x86_64-linux-gnu-clang-6.0" }
  tool_path { name: "gcov" path: "clang_6p0/x86_64-linux-gnu-gcov" }
  # C(++) compiles invoke the compiler (as that is the one knowing where
  # to find libraries), but we provide LD so other rules can invoke the linker.
  tool_path { name: "ld" path: "clang_6p0/x86_64-linux-gnu-ld" }
  tool_path { name: "nm" path: "clang_6p0/x86_64-linux-gnu-nm" }
  tool_path { name: "objcopy" path: "clang_6p0/x86_64-linux-gnu-objcopy" }
  objcopy_embed_flag: "-I"
  objcopy_embed_flag: "binary"
  tool_path { name: "objdump" path: "clang_6p0/x86_64-linux-gnu-objdump" }
  tool_path { name: "strip" path: "clang_6p0/x86_64-linux-gnu-strip" }
  linking_mode_flags { mode: DYNAMIC }

  compiler_flag: "--sysroot=external/clang_6p0_repo/"
  compiler_flag: "-nostdinc"
  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/include/x86_64-linux-gnu",
  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/lib/llvm-6.0/lib/clang/6.0.0/include",

  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/include/c++/7.4.0"
  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/include/x86_64-linux-gnu/c++/7.4.0"
  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/include/c++/7.4.0/backward"
  compiler_flag: "-isystem"
  compiler_flag: "external/clang_6p0_repo/usr/include"

  # TODO(bazel-team): In theory, the path here ought to exactly match the path
  # used by gcc. That works because bazel currently doesn't track files at
  # absolute locations and has no remote execution, yet. However, this will need
  # to be fixed, maybe with auto-detection?
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/include/c++/7.4.0"
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/include/x86_64-linux-gnu/c++/7.4.0"
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/include/c++/7.4.0/backward"
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/local/include"
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/lib/llvm-6.0/lib/clang/6.0.0/include"
  cxx_builtin_include_directory: "%package(@clang_6p0_repo//usr)%/include/x86_64-linux-gnu"
  cxx_flag: "-isystem"
  cxx_flag: "external/clang_6p0_repo/usr/include"

  linker_flag: "-nodefaultlibs"
  linker_flag: "--sysroot=external/clang_6p0_repo/"
  linker_flag: "-lstdc++"
  linker_flag: "-lc"
  linker_flag: "-lgcc"
  linker_flag: "-lgcc_s"
  linker_flag: "-Bexternal/clang_6p0_repo/usr/bin/"
  linker_flag: "-Ltools/cpp/clang_6p0/clang_more_libs"
  linker_flag: "-Lexternal/clang_6p0_repo/lib/x86_64-linux-gnu"
  linker_flag: "-Lexternal/clang_6p0_repo/usr/lib/x86_64-linux-gnu"
  linker_flag: "-Lexternal/clang_6p0_repo/usr/lib/gcc/x86_64-linux-gnu"

  feature {
    name: "opt"
    implies: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

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
        flag: "-DAOS_DEBUG=1"
      }
      flag_group {
        flag: "-fno-omit-frame-pointer"
      }
    }
  }

  feature {
    name: "fastbuild"
    implies: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

  feature {
    name: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c-compile"
      flag_group {
        flag: "-std=gnu99"
      }
    }
    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-std=gnu++1z"
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
  compiler_flag: "-D_FILE_OFFSET_BITS=64"
  # TODO(Brian): Rename this or something.
  compiler_flag: "-DAOS_ARCHITECTURE_arm_frc"

  linker_flag: "-fuse-ld=gold"

  # Make C++ compilation deterministic. Use linkstamping instead of these
  # compiler symbols.
  unfiltered_cxx_flag: "-Wno-builtin-macro-redefined"
  unfiltered_cxx_flag: "-D__DATE__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIMESTAMP__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIME__=\"redacted\""

  # C++17 libraries that tend to cause issues in some libraries that we include.
  unfiltered_cxx_flag: "-Wno-varargs"
  unfiltered_cxx_flag: "-Wno-null-pointer-arithmetic"
  # The mismatched-new-delete seems to be a bit overly strict currently
  # and errors on:
  # char *p = new char;
  # delete p;
  # p = new char[100];
  # delete[] p;
  unfiltered_cxx_flag: "-Wno-mismatched-new-delete"

  # Security hardening on by default.
  # Conservative choice; -D_FORTIFY_SOURCE=2 may be unsafe in some cases.
  # We need to undef it before redefining it as some distributions now have
  # it enabled by default.
  compiler_flag: "-U_FORTIFY_SOURCE"
  compiler_flag: "-D_FORTIFY_SOURCE=1"
  compiler_flag: "-fstack-protector"
  compiler_flag: "-fPIE"
  linker_flag: "-Wl,-z,relro,-z,now"

  # Pretty much everything needs this, including parts of the glibc STL...
  linker_flag: "-lm"

  # Enable coloring even if there's no attached terminal. Bazel removes the
  # escape sequences if --nocolor is specified.
  compiler_flag: "-fcolor-diagnostics"
  compiler_flag: "-fmessage-length=80"
  compiler_flag: "-fmacro-backtrace-limit=0"

  compiler_flag: "-Wall"
  compiler_flag: "-Wextra"
  compiler_flag: "-Wpointer-arith"
  compiler_flag: "-Wstrict-aliasing"
  compiler_flag: "-Wcast-qual"
  compiler_flag: "-Wcast-align"
  compiler_flag: "-Wwrite-strings"
  compiler_flag: "-Wtype-limits"
  compiler_flag: "-Wsign-compare"
  compiler_flag: "-Wformat=2"
  compiler_flag: "-Werror"

  # Keep stack frames for debugging, even in opt mode.
  compiler_flag: "-fno-omit-frame-pointer"

  # Don't use temp files while compiling.
  compiler_flag: "-pipe"

  # Stamp the binary with a unique identifier.
  linker_flag: "-Wl,--build-id=md5"
  linker_flag: "-Wl,--hash-style=gnu"
  linker_flag: "-Wl,--warn-execstack"
  linker_flag: "-Wl,--detect-odr-violations"

  # Enable debug symbols.
  compiler_flag: "-ggdb3"

  compilation_mode_flags {
    mode: OPT

    compiler_flag: "-O2"

    # Disable assertions
    compiler_flag: "-DNDEBUG"

    # Removal of unused code and data at link time (can this increase binary size in some cases?).
    compiler_flag: "-ffunction-sections"
    compiler_flag: "-fdata-sections"
    linker_flag: "-Wl,--gc-sections"
  }
  feature {
    name: "pie_for_linking"
    enabled: true
    flag_set {
      action: "c++-link-executable"
      flag_group {
        flag: "-pie"
      }
    }
  }
}

toolchain {
    # This toolchain was initially sourced from https://github.com/ribrdb/rules_emscripten
    toolchain_identifier: "emscripten_toolchain"
    host_system_name: "web"
    target_system_name: "emscripten-unknown-emscripten"
    target_cpu: "web"
    target_libc: "unknown"
    compiler: "emscripten"
    abi_version: "unknown"
    abi_libc_version: "unknown"
    tool_path {
      name: "gcc"
      path: "emscripten/emcc.sh"
    }
    tool_path {
        name: "ld"
        path: "emscripten/emcc.sh"
    }
    tool_path {
        name: "ar"
        path: "emscripten/emar.sh"
    }
    tool_path {
        name: "cpp"
        path: "/bin/false"
    }
    tool_path {
        name: "gcov"
        path: "/bin/false"
    }
    tool_path {
        name: "nm"
        path: "/bin/false"
    }
    tool_path {
        name: "objdump"
        path: "/bin/false"
    }
    tool_path {
      name: "objcopy"
      path: "/bin/false"
    }
    tool_path {
        name: "strip"
        path: "/bin/false"
    }

    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/include/libcxx"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/lib/libcxxabi/include"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/include/compat"
    compiler_flag: "-isystem"
    compiler_flag: "external/emscripten_toolchain/system/include"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/include"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/include/SSE"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/include/libc"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/lib/libc/musl/arch/emscripten"
    unfiltered_cxx_flag: "-isystem"
    unfiltered_cxx_flag: "external/emscripten_toolchain/system/local/include"

    # Turn off exceptions since emscripten has issues catching them
    compiler_flag: "-fno-exceptions"

    unfiltered_cxx_flag: "-no-canonical-prefixes"
    linker_flag: "-no-canonical-prefixes"

    # Make C++ compilation deterministic. Use linkstamping instead of these
    # compiler symbols.
    unfiltered_cxx_flag: "-Wno-builtin-macro-redefined"
    unfiltered_cxx_flag: "-D__DATE__=\"redacted\""
    unfiltered_cxx_flag: "-D__TIMESTAMP__=\"redacted\""
    unfiltered_cxx_flag: "-D__TIME__=\"redacted\""
    # Enable coloring even if there's no attached terminal. Bazel removes the
    # escape sequences if --nocolor is specified.
    compiler_flag: "-fdiagnostics-color=always"
    compiler_flag: "-Wall"
    compiler_flag: "-Werror"

    compiler_flag: "-ffunction-sections"
    compiler_flag: "-fdata-sections"

  feature {
    name: "opt"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
        flag: "-O2"
        flag: "--closure"
        flag: "1"
      }
    }
    implies: "all_modes"
  }
  feature {
    name: "dbg"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=1"
      }
      flag_group {
        flag: "-fno-omit-frame-pointer"
      }
    }
    implies: "all_modes"
  }
  feature {
    name: "fastbuild"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
    implies: "all_modes"
  }
  feature {
    name: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c-compile"
      flag_group {
        flag: "-std=gnu99"
      }
    }
    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-std=gnu++1y"
      }
    }
  }
}

toolchain {
  toolchain_identifier: "roborio_linux"
  host_system_name: "roborio"
  target_system_name: "roborio"
  target_cpu: "roborio"
  target_libc: "roborio"
  compiler: "gcc"
  abi_version: "roborio"
  abi_libc_version: "roborio"

  builtin_sysroot: ""
  needsPic: true
  supports_gold_linker: false
  supports_incremental_linker: false
  supports_fission: false
  supports_interface_shared_objects: false
  supports_normalizing_ar: false
  supports_start_end_lib: false
  supports_thin_archives: false

  tool_path { name: "ar" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ar" }
  tool_path { name: "as" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-as" }
  tool_path { name: "compat-ld" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ld" }
  tool_path { name: "cpp" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-cpp" }
  tool_path { name: "dwp" path: "/bin/false" }
  tool_path { name: "gcc" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-gcc" }
  tool_path { name: "gcov" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-gcov-4.9" }
  # C(++) compiles invoke the compiler (as that is the one knowing where
  # to find libraries), but we provide LD so other rules can invoke the linker.
  tool_path { name: "ld" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-ld" }
  tool_path { name: "nm" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-nm" }
  tool_path { name: "objcopy" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-objcopy" }
  objcopy_embed_flag: "-I"
  objcopy_embed_flag: "binary"
  tool_path { name: "objdump" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-objdump" }
  tool_path { name: "strip" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-strip" }
  linking_mode_flags { mode: DYNAMIC }

  feature {
    name: "compile_flags1"
    enabled: true
    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-module-compile"
      action: "c++-module-codegen"
      action: "lto-backend"
      action: "clif-match"
      flag_group {
        flag: "--sysroot=external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi"
        flag: "-nostdinc"
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/lib/gcc/arm-frc2020-linux-gnueabi/7.3.0/include"
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/lib/gcc/arm-frc2020-linux-gnueabi/7.3.0/include-fixed"
      }
    }

    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      flag_group {
        flag: "-fno-canonical-system-headers"
      }
    }

    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-module-compile"
      action: "c++-module-codegen"
      flag_group {
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/include/c++/7.3.0"
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/include/c++/7.3.0/arm-frc2020-linux-gnueabi"
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/include/c++/7.3.0/backward"
      }
    }

    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-module-compile"
      action: "c++-module-codegen"
      action: "lto-backend"
      action: "clif-match"
      flag_group {
        flag: "-isystem"
        flag: "external/arm_frc_linux_gnueabi_repo/arm-frc2020-linux-gnueabi/usr/include"

        flag: "-mfpu=neon"

        # Things that the code wants defined.
        flag: "-D__STDC_FORMAT_MACROS"
        flag: "-D__STDC_CONSTANT_MACROS"
        flag: "-D__STDC_LIMIT_MACROS"
        flag: "-D_FILE_OFFSET_BITS=64"

        # TODO(Brian): Rename this or something.
        flag: "-DAOS_ARCHITECTURE_arm_frc"

        # Security hardening on by default.
        # Conservative choice; -D_FORTIFY_SOURCE=2 may be unsafe in some cases.
        # We need to undef it before redefining it as some distributions now have
        # it enabled by default.
        flag: "-U_FORTIFY_SOURCE"
        flag: "-fstack-protector"
        flag: "-fPIE"

        # Enable coloring even if there's no attached terminal. Bazel removes the
        # escape sequences if --nocolor is specified.
        flag: "-fdiagnostics-color=always"

        flag: "-Wall"
        flag: "-Wextra"
        flag: "-Wpointer-arith"
        flag: "-Wstrict-aliasing"
        flag: "-Wcast-qual"
        flag: "-Wwrite-strings"
        flag: "-Wtype-limits"
        flag: "-Wsign-compare"
        flag: "-Wformat=2"
        flag: "-Werror"
        flag: "-Wunused-local-typedefs"
        # We don't use libraries compiled with the broken version.
        flag: "-Wno-psabi"

        # Keep stack frames for debugging, even in opt mode.
        flag: "-fno-omit-frame-pointer"

        flag: "-D__has_feature(x)=0"

        # Don't use temp files while compiling.
        flag: "-pipe"

        # Enable debug symbols.
        flag: "-ggdb3"
      }
    }
  }

  feature {
    name: "opt"
    implies: "opt_post"
    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-module-compile"
      action: "objc-compile"
      action: "objc++-compile"
      action: "c++-header-parsing"
      action: "linkstamp-compile"
      flag_group {
        flag: "-O2"
        flag: "-DNDEBUG"
        flag: "-D_FORTIFY_SOURCE=1"
        flag: "-ffunction-sections"
        flag: "-fdata-sections"
      }
    }
    flag_set {
      action: "c++-link-executable"
      action: "c++-link-nodeps-dynamic-library"
      action: "c++-link-dynamic-library"
      flag_group {
        flag: "-Wl,--gc-sections"
      }
    }
  }

  # TODO(bazel-team): In theory, the path here ought to exactly match the path
  # used by gcc. That works because bazel currently doesn't track files at
  # absolute locations and has no remote execution, yet. However, this will need
  # to be fixed, maybe with auto-detection?

  cxx_builtin_include_directory: "%package(@arm_frc_linux_gnueabi_repo//arm-frc2020-linux-gnueabi/usr/lib/gcc/arm-frc2020-linux-gnueabi/7.3.0/include)%"
  cxx_builtin_include_directory: "%package(@arm_frc_linux_gnueabi_repo//arm-frc2020-linux-gnueabi/usr/lib/gcc/arm-frc2020-linux-gnueabi/7.3.0/include-fixed)%"
  cxx_builtin_include_directory: "%package(@arm_frc_linux_gnueabi_repo//arm-frc2020-linux-gnueabi/usr/include/c++/7.3.0/arm-frc2020-linux-gnueabi)%"
  cxx_builtin_include_directory: "%package(@arm_frc_linux_gnueabi_repo//arm-frc2020-linux-gnueabi/usr/include/c++/7.3.0/backward)%"

  linker_flag: "-lstdc++"
  linker_flag: "-Ltools/cpp/arm-frc-linux-gnueabi/libs"

  feature {
    name: "dependency_file"
    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-module-compile"
      action: "objc-compile"
      action: "objc++-compile"
      action: "c++-header-parsing"
      action: "clif-match"
      expand_if_all_available: "dependency_file"
      flag_group {
        flag: "-MD"
        flag: "-MF"
        flag: "%{dependency_file}"
      }
    }
  }

  feature {
    name: "random_seed"
    flag_set {
      # TODO(austin): Should these also have -frandom-seed set?  Upstream
      # doesn't.
      # action: "linkstamp-compile"
      # action: "c-compile"
      action: "c++-compile"
      action: "c++-module-codegen"
      action: "c++-module-compile"
      flag_group {
        expand_if_all_available: "output_file"
        flag: "-frandom-seed=%{output_file}"
      }
    }
  }

  feature {
    name: "pic"
    flag_set {
      action: "assemble"
      action: "preprocess-assemble"
      action: "linkstamp-compile"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-module-codegen"
      action: "c++-module-compile"
      expand_if_all_available: "pic"
      flag_group {
        flag: "-fPIC"
      }
    }
  }

  feature {
    name: "preprocessor_defines"
    flag_set {
      action: "preprocess-assemble"
      action: "linkstamp-compile"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-module-compile"
      action: "clif-match"
      flag_group {
        iterate_over: "preprocessor_defines"
        flag: "-D%{preprocessor_defines}"
      }
    }
  }

  feature {
    name: "include_paths"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-iquote"
        flag: "%{quote_include_paths}"
        iterate_over: "quote_include_paths"
      }
      flag_group {
        flag: "-I%{include_paths}"
        iterate_over: "include_paths"
      }
      flag_group {
        flag: "-isystem"
        flag: "%{system_include_paths}"
        iterate_over: "system_include_paths"
      }
    }
  }

  feature {
    name: "opt_post"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

  feature {
    name: "dbg"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=1"
      }
      flag_group {
        flag: "-fno-omit-frame-pointer"
      }
    }
  }

  feature {
    name: "fastbuild"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

  feature {
    name: "all_modes"
    enabled: true
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c-compile"
      flag_group {
        flag: "-std=gnu99"
      }
    }
    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-std=gnu++1z"
        flag: "-fno-sized-deallocation"
      }
    }
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c++-link"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      action: "c-compile"
      flag_group {
        # We always want to compile with -pthread semantics.
        flag: "-pthread"
      }
    }
  }

  # Anticipated future default.
  # This makes GCC and Clang do what we want when called through symlinks.
  unfiltered_cxx_flag: "-no-canonical-prefixes"
  linker_flag: "-no-canonical-prefixes"

  # Make C++ compilation deterministic. Use linkstamping instead of these
  # compiler symbols.
  unfiltered_cxx_flag: "-Wno-builtin-macro-redefined"
  unfiltered_cxx_flag: "-D__DATE__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIMESTAMP__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIME__=\"redacted\""

  linker_flag: "-Wl,-z,relro,-z,now"

  # Pretty much everything needs this, including parts of the glibc STL...
  linker_flag: "-lm"

  # Have GCC return the exit code from ld.
  linker_flag: "-pass-exit-codes"

  # Stamp the binary with a unique identifier.
  linker_flag: "-Wl,--build-id=md5"
  linker_flag: "-Wl,--hash-style=gnu"
  #linker_flag: "-Wl,--warn-execstack"
  #linker_flag: "-Wl,--detect-odr-violations"

  feature {
    name: "pie_for_linking"
    enabled: true
    flag_set {
      action: "c++-link-executable"
      flag_group {
        flag: "-pie"
      }
    }
  }
}

toolchain {
  abi_version: "clang_6.0"
  abi_libc_version: "glibc_2.19"
  builtin_sysroot: ""
  compiler: "clang"
  host_system_name: "linux"
  needsPic: true
  supports_gold_linker: false
  supports_incremental_linker: false
  supports_fission: false
  supports_interface_shared_objects: false
  supports_normalizing_ar: true
  supports_start_end_lib: false
  supports_thin_archives: true
  target_libc: "glibc_2.19"
  target_cpu: "armhf-debian"
  target_system_name: "arm_a15"
  toolchain_identifier: "clang_linux_armhf"

  tool_path { name: "ar" path: "linaro_linux_gcc/arm-linux-gnueabihf-ar" }
  tool_path { name: "compat-ld" path: "linaro_linux_gcc/arm-linux-gnueabihf-ld" }
  tool_path { name: "cpp" path: "linaro_linux_gcc/clang_bin/clang" }
  tool_path { name: "dwp" path: "linaro_linux_gcc/arm-linux-gnueabihf-dwp" }
  tool_path { name: "gcc" path: "linaro_linux_gcc/clang_bin/clang" }
  tool_path { name: "gcov" path: "arm-frc-linux-gnueabi/arm-frc-linux-gnueabi-gcov-4.9" }
  # C(++) compiles invoke the compiler (as that is the one knowing where
  # to find libraries), but we provide LD so other rules can invoke the linker.
  tool_path { name: "ld" path: "linaro_linux_gcc/arm-linux-gnueabihf-ld" }
  tool_path { name: "nm" path: "linaro_linux_gcc/arm-linux-gnueabihf-nm" }
  tool_path { name: "objcopy" path: "linaro_linux_gcc/arm-linux-gnueabihf-objcopy" }
  objcopy_embed_flag: "-I"
  objcopy_embed_flag: "binary"
  tool_path { name: "objdump" path: "linaro_linux_gcc/arm-linux-gnueabihf-objdump" }
  tool_path { name: "strip" path: "linaro_linux_gcc/arm-linux-gnueabihf-strip" }
  linking_mode_flags { mode: DYNAMIC }

  compiler_flag: "-target"
  compiler_flag: "armv7a-arm-linux-gnueabif"
  compiler_flag: "--sysroot=external/linaro_linux_gcc_repo/arm-linux-gnueabihf/libc"
  compiler_flag: "-mfloat-abi=hard"
  compiler_flag: "-mfpu=vfpv3-d16"

  compiler_flag: "-nostdinc"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/lib/gcc/arm-linux-gnueabihf/7.4.1/include"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/lib/gcc/arm-linux-gnueabihf/7.4.1/include-fixed"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/arm-linux-gnueabihf/include/c++/7.4.1/arm-linux-gnueabihf"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/arm-linux-gnueabihf/include/c++/7.4.1"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/include/c++/7.4.1/arm-linux-gnueabihf"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/include/c++/7.4.1"
  compiler_flag: "-isystem"
  compiler_flag: "external/linaro_linux_gcc_repo/arm-linux-gnueabihf/libc/usr/include"

  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//include)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/libc/usr/include)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/libc/usr/lib/include)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/libc/lib/gcc/arm-linux-gnueabihf/7.4.1/include-fixed)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//include)%/c++/7.4.1"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/libc/lib/gcc/arm-linux-gnueabihf/7.4.1/include)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/libc/lib/gcc/arm-linux-gnueabihf/7.4.1/include-fixed)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//lib/gcc/arm-linux-gnueabihf/7.4.1/include)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//lib/gcc/arm-linux-gnueabihf/7.4.1/include-fixed)%"
  cxx_builtin_include_directory: "%package(@linaro_linux_gcc_repo//arm-linux-gnueabihf/include)%/c++/7.4.1"

  linker_flag: "-target"
  linker_flag: "armv7a-arm-linux-gnueabif"
  linker_flag: "--sysroot=external/linaro_linux_gcc_repo/arm-linux-gnueabihf/libc"
  linker_flag: "-lstdc++"
  linker_flag: "-Ltools/cpp/linaro_linux_gcc/clang_more_libs"
  linker_flag: "-Lexternal/linaro_linux_gcc_repo/arm-linux-gnueabihf/lib"
  linker_flag: "-Lexternal/linaro_linux_gcc_repo/arm-linux-gnueabihf/libc/lib"
  linker_flag: "-Lexternal/linaro_linux_gcc_repo/arm-linux-gnueabihf/libc/usr/lib"
  linker_flag: "-Lexternal/linaro_linux_gcc_repo/lib/gcc/arm-linux-gnueabihf/7.4.1"
  linker_flag: "-Bexternal/linaro_linux_gcc_repo/lib/gcc/arm-linux-gnueabihf/7.4.1"
  linker_flag: "-Bexternal/linaro_linux_gcc_repo/arm-linux-gnueabihf/bin"
  linker_flag: "-Wl,--dynamic-linker=/lib/ld-linux-armhf.so.3"

  feature {
    name: "opt"
    implies: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

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
        flag: "-DAOS_DEBUG=1"
      }
      flag_group {
        flag: "-fno-omit-frame-pointer"
      }
    }
  }

  feature {
    name: "fastbuild"
    implies: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "c-compile"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-DAOS_DEBUG=0"
      }
    }
  }

  feature {
    name: "all_modes"
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c-compile"
      flag_group {
        flag: "-std=gnu99"
      }
    }
    flag_set {
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      flag_group {
        flag: "-std=gnu++1z"
      }
    }
    flag_set {
      action: "preprocess-assemble"
      action: "assemble"
      action: "c++-link"
      action: "c++-compile"
      action: "c++-header-parsing"
      action: "c++-header-preprocessing"
      action: "c++-module-compile"
      action: "c-compile"
      flag_group {
        # We always want to compile with -pthread semantics.
        flag: "-pthread"
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
  compiler_flag: "-D_FILE_OFFSET_BITS=64"
  # TODO(Brian): Rename this or something.
  compiler_flag: "-DAOS_ARCHITECTURE_armhf"

  # Make C++ compilation deterministic. Use linkstamping instead of these
  # compiler symbols.
  unfiltered_cxx_flag: "-Wno-builtin-macro-redefined"
  unfiltered_cxx_flag: "-Wno-mismatched-new-delete"
  unfiltered_cxx_flag: "-Wno-null-pointer-arithmetic"
  unfiltered_cxx_flag: "-Wno-varargs"
  unfiltered_cxx_flag: "-D__DATE__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIMESTAMP__=\"redacted\""
  unfiltered_cxx_flag: "-D__TIME__=\"redacted\""

  # Security hardening on by default.
  # Conservative choice; -D_FORTIFY_SOURCE=2 may be unsafe in some cases.
  # We need to undef it before redefining it as some distributions now have
  # it enabled by default.
  compiler_flag: "-U_FORTIFY_SOURCE"
  compiler_flag: "-fstack-protector"
  compiler_flag: "-fPIE"
  linker_flag: "-Wl,-z,relro,-z,now"

  # Pretty much everything needs this, including parts of the glibc STL...
  linker_flag: "-lm"

  # Enable coloring even if there's no attached terminal. Bazel removes the
  # escape sequences if --nocolor is specified.
  compiler_flag: "-fdiagnostics-color=always"

  compiler_flag: "-Wall"
  compiler_flag: "-Wextra"
  compiler_flag: "-Wpointer-arith"
  compiler_flag: "-Wstrict-aliasing"
  compiler_flag: "-Wcast-qual"
  compiler_flag: "-Wcast-align"
  compiler_flag: "-Wwrite-strings"
  compiler_flag: "-Wtype-limits"
  compiler_flag: "-Wsign-compare"
  compiler_flag: "-Wformat=2"
  compiler_flag: "-Werror"
  compiler_flag: "-Wunused-local-typedefs"

  # Keep stack frames for debugging, even in opt mode.
  compiler_flag: "-fno-omit-frame-pointer"

  # Don't use temp files while compiling.
  compiler_flag: "-pipe"

  # Stamp the binary with a unique identifier.
  linker_flag: "-Wl,--build-id=md5"
  linker_flag: "-Wl,--hash-style=gnu"
  #linker_flag: "-Wl,--warn-execstack"
  #linker_flag: "-Wl,--detect-odr-violations"

  # Enable debug symbols.
  compiler_flag: "-ggdb3"

  compilation_mode_flags {
    mode: OPT

    compiler_flag: "-O2"

    # Disable assertions
    compiler_flag: "-DNDEBUG"
    compiler_flag: "-D_FORTIFY_SOURCE=1"

    # Removal of unused code and data at link time (can this increase binary size in some cases?).
    compiler_flag: "-ffunction-sections"
    compiler_flag: "-fdata-sections"
    linker_flag: "-Wl,--gc-sections"
  }
  feature {
    name: "pie_for_linking"
    enabled: true
    flag_set {
      action: "c++-link-executable"
      flag_group {
        flag: "-pie"
      }
    }
  }
}
