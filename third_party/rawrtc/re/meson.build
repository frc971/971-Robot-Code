# Project definition
project('rawrre', 'c',
    version: '0.6.0',
    default_options: ['c_std=c99'],
    meson_version: '>=0.46.0')

# Set compiler warning flags
compiler = meson.get_compiler('c')
compiler_args = compiler.get_supported_arguments([
    '-Wall',
    '-Wmissing-declarations',
    '-Wmissing-prototypes',
    '-Wstrict-prototypes',
    '-Wbad-function-cast',
    '-Wsign-compare',
    '-Wnested-externs',
    '-Wshadow',
    '-Waggregate-return',
    '-Wcast-align',
    '-Wextra',
    '-Wold-style-definition',
    '-Wdeclaration-after-statement',
    '-Wuninitialized',  # TODO: Only when optimising? But why?
    '-Wshorten-64-to-32',  # Apple-specific
    '-pedantic',
])
add_project_arguments(compiler_args, language: 'c')

# TODO: Include -g flag for extra debug stuff?
# TODO: Optimise for speed or size?

# Configuration
# TODO: Generate a configuration file instead of defining? Would simplify the
#       configuration.
#       This would allow projects depending on this able to import those
#       defines, too.
compile_args = []
configuration = configuration_data()

# OS
system = host_machine.system()
add_project_arguments('-DOS="@0@"'.format(system), language: 'c')
configuration.set_quoted('OS', system)
if system == 'darwin'
    add_project_arguments('-DDARWIN', language: 'c')
    configuration.set('DARWIN', 1)
elif system == 'dragonfly'
    add_project_arguments('-DDRAGONFLY', language: 'c')
    configuration.set('DRAGONFLY', 1)
elif system == 'freebsd'
    add_project_arguments('-DFREEBSD', language: 'c')
    configuration.set('FREEBSD', 1)
elif system == 'gnu'
    add_project_arguments('-DGNU', language: 'c')
    configuration.set('GNU', 1)
elif system == 'gnu/kfreebsd'
    add_project_arguments(
        '-DKFREEBSD',
        '-D_GNU_SOURCE',
        language: 'c')
    configuration.set('GNU', 1)
    configuration.set('_GNU_SOURCE', 1)
elif system == 'linux'
    add_project_arguments('-DLINUX', language: 'c')
    configuration.set('LINUX', 1)
elif system == 'netbsd'
    add_project_arguments('-DDNETBSD', language: 'c')
    configuration.set('NETBSD', 1)
elif system == 'sunos'
    add_project_arguments('-DSOLARIS', language: 'c')
    configuration.set('SOLARIS', 1)
    compile_args += '-DSOLARIS'
elif system == 'windows'
    add_project_arguments('-DWIN32', language: 'c')
    configuration.set('WIN32', 1)
    compile_args += '-DWIN32'
else
    warning('Unhandled OS: @0@'.format(system))
endif

# Architecture
cpu = host_machine.cpu()
add_project_arguments('-DARCH="@0@"'.format(cpu), language: 'c')
configuration.set_quoted('ARCH', cpu)

# Dependency: Threads
# TODO: Make this overridable
thread_dep = dependency('threads', required: false)

# Dependency: OpenSSL
openssl_dep = dependency('openssl', version: '>=0.9.8', required: false)
# TODO: Make this overridable
if openssl_dep.found()
    add_project_arguments(
        '-DUSE_OPENSSL',
        '-DUSE_TLS',
        language: 'c')
    configuration.set('USE_OPENSSL', 1)
    configuration.set('USE_TLS', 1)
    compile_args += '-DUSE_OPENSSL'

    # Check for DTLS
    if compiler.has_header('openssl/dtls1.h')
        add_project_arguments(
            '-DUSE_OPENSSL_DTLS',
            '-DUSE_DTLS',
            language: 'c')
        configuration.set('USE_OPENSSL_DTLS', 1)
        configuration.set('USE_DTLS', 1)
    endif

    # Check for SRTP
    if compiler.has_header('openssl/srtp.h')
        add_project_arguments(
            '-DUSE_OPENSSL_SRTP',
            '-DUSE_DTLS_SRTP',
            language: 'c')
        configuration.set('USE_OPENSSL_SRTP', 1)
        configuration.set('USE_DTLS_SRTP', 1)
    endif
endif

# Dependency: zlib
# TODO: Make this overridable
# TODO: Arbitrary version, ask maintainers
zlib_dep = dependency('zlib', version: '>=1.2.8', required: false)
if zlib_dep.found()
    add_project_arguments('-DUSE_ZLIB', language: 'c')
    configuration.set('USE_ZLIB', 1)
    compile_args += '-DUSE_ZLIB'
endif

# Dependencies list
dependencies = [
    thread_dep,
    openssl_dep,
    zlib_dep,
]

# Features: Common
add_project_arguments(
    '-DHAVE_INET6',
    '-DHAVE_SELECT',
    '-DHAVE_STDBOOL_H',
    language: 'c')
configuration.set('HAVE_INET6', 1)
configuration.set('HAVE_SELECT', 1)
configuration.set('HAVE_STDBOOL_H', 1)
compile_args += [
    '-DHAVE_INET6',
    '-DHAVE_STDBOOL_H',
]

# Features: Check for fixed size integer types
if compiler.has_header('inttypes.h')
    add_project_arguments('-DHAVE_INTTYPES_H', language: 'c')
    configuration.set('HAVE_INTTYPES_H', 1)
    compile_args += '-DHAVE_INTTYPES_H'
endif

# Features: Check for route
have_net_route = compiler.has_header('net/route.h')
if have_net_route
    add_project_arguments('-DHAVE_NET_ROUTE_H', language: 'c')
    configuration.set('HAVE_NET_ROUTE_H', 1)
endif

# Features: Check for sysctl
have_sysctl = compiler.has_header('sys/sysctl.h')
if have_sysctl
    add_project_arguments('-DHAVE_SYS_SYSCTL_H', language: 'c')
    configuration.set('HAVE_SYS_SYSCTL_H', 1)
endif

# Features: OS-specific
if system == 'windows'
    # Windows
    add_project_arguments(
        '-DHAVE_IO_H',
        '-D_WIN32_WINNT=0x0501',
        '-D__ssize_t_defined',
        language: 'c')
    configuration.set('HAVE_IO_H', 1)
    configuration.set('_WIN32_WINNT', 0x0501)
    configuration.set('__ssize_t_defined', 1)
    compile_args += '-D__ssize_t_defined'

    # Additional linking
    dependencies += compiler.find_library('wsock32', required: true)
    dependencies += compiler.find_library('ws2_32', required: true)
    dependencies += compiler.find_library('iphlpapi', required: true)

    # TODO: APP_LFLAGS	+= -Wl,--export-all-symbols
else
    # UNIX
    add_project_arguments(
        '-DHAVE_FORK',
        '-DHAVE_INET_NTOP',
        '-DHAVE_PWD_H',
        '-DHAVE_SELECT_H',
        '-DHAVE_SETRLIMIT',
        '-DHAVE_SIGNAL',
        '-DHAVE_STRERROR_R',
        '-DHAVE_STRINGS_H',
        '-DHAVE_SYS_TIME_H',
        '-DHAVE_UNAME',
        '-DHAVE_UNISTD_H',
        language: 'c')
    configuration.set('HAVE_FORK', 1)
    configuration.set('HAVE_INET_NTOP', 1)
    configuration.set('HAVE_PWD_H', 1)
    configuration.set('HAVE_SELECT_H', 1)
    configuration.set('HAVE_SETRLIMIT', 1)
    configuration.set('HAVE_SIGNAL', 1)
    configuration.set('HAVE_STRERROR_R', 1)
    configuration.set('HAVE_STRINGS_H', 1)
    configuration.set('HAVE_SYS_TIME_H', 1)
    configuration.set('HAVE_UNAME', 1)
    configuration.set('HAVE_UNISTD_H', 1)

    # Solaris requires some additional linking
    if system == 'sunos'
        dependencies += compiler.find_library('socket', required: true)
        dependencies += compiler.find_library('nsl', required: true)
    endif

    # Check for pthread
    if compiler.has_header('pthread.h')
        add_project_arguments('-DHAVE_PTHREAD', language: 'c')
        configuration.set('HAVE_PTHREAD', 1)
    endif

    # Check for ifaddrs
    ifaddrs_prefix = '''
    #include <sys/socket.h>
    #define __USE_MISC 1   /**< Use MISC code */
    #include <net/if.h>
    #include <ifaddrs.h>
    '''
    if compiler.has_function('getifaddrs', prefix: ifaddrs_prefix)
        add_project_arguments('-DHAVE_GETIFADDRS', language: 'c')
        configuration.set('HAVE_GETIFADDRS', 1)
    endif

    # Check for dlfcn
    if compiler.has_header('dlfcn.h')
        # Linux, GNU and Solaris require linking
        if system == 'linux' or system == 'gnu' or system == 'sunos'
            dependencies += compiler.find_library('dl', required: true)
        endif
        add_project_link_arguments('-rdynamic', language: 'c')
        add_project_arguments('-DHAVE_DLFCN', language: 'c')
        configuration.set('HAVE_DLFCN', 1)
    endif

    # Check for epoll
    if compiler.has_header('sys/epoll.h')
        add_project_arguments('-DHAVE_EPOLL', language: 'c')
        configuration.set('HAVE_EPOLL', 1)
    endif

    # Check for resolv
    resolv_prefix = '''
    #define _BSD_SOURCE 1
    #define _DEFAULT_SOURCE 1
    #include <sys/types.h>
    #include <netinet/in.h>
    #include <arpa/nameser.h>
    #include <resolv.h>
    '''
    if compiler.has_type('struct __res_state', prefix: resolv_prefix)
        # OSX/iOS and Solaris require linking
        if system == 'darwin' or system == 'sunos'
            dependencies += compiler.find_library('resolv', required: true)
        endif
        add_project_arguments('-DHAVE_RESOLV', language: 'c')
        configuration.set('HAVE_RESOLV', 1)
    endif

    # Check for kqueue
    kqueue_prefix = '''
    #include <sys/types.h>
    #include <sys/event.h>
    #include <sys/time.h>
    '''
    if compiler.has_function('kqueue', prefix: kqueue_prefix)
        add_project_arguments('-DHAVE_KQUEUE', language: 'c')
        configuration.set('HAVE_KQUEUE', 1)
    endif

    # Check for arc4random
    if compiler.has_function('arc4random', prefix: '#include <stdlib.h>')
        add_project_arguments('-DHAVE_ARC4RANDOM', language: 'c')
        configuration.set('HAVE_ARC4RANDOM', 1)
    endif

    # Features OSX/iOS is lacking
    if not (system == 'darwin')
        # OSX/iOS's poll() does not support devices
        add_project_arguments(
            '-DHAVE_POLL',
            '-DHAVE_INET_PTON',
            language: 'c')
        configuration.set('HAVE_POLL', 1)
        configuration.set('HAVE_INET_PTON', 1)
    endif
endif

# Features: Routing for Linux and *BSD
if system == 'linux' or (have_sysctl and have_net_route)
    add_project_arguments('-DHAVE_ROUTE_LIST', language: 'c')
    configuration.set('HAVE_ROUTE_LIST', 1)
endif

# Version
version = meson.project_version()
version_array = version.split('.')
add_project_arguments(
    '-DVERSION="@0@"'.format(version),
    '-DVER_MAJOR=@0@'.format(version_array[0]),
    '-DVER_MINOR=@0@'.format(version_array[1]),
    '-DVER_PATCH=@0@'.format(version_array[2]),
    language: 'c')
configuration.set_quoted('VERSION', version)
configuration.set('VER_MAJOR', version_array[0])
configuration.set('VER_MINOR', version_array[1])
configuration.set('VER_PATCH', version_array[2])

# TODO: Define 'RELEASE' when using build type 'release'
#       Also, compile_args += '-DMBUF_DEBUG=1' in that case
# TODO: Check if we need to do anything for gcov
# TODO: Check if we need to do anything for GNU profiling
# TODO: Port packaging section
# TODO: Port clang section
# TODO: Port documentation section

# Includes
include_dir = include_directories('include')
subdir('include')

# Sources & Modules
# TODO: Make which to build configurable
subdir('src')

# Build library
re = library(meson.project_name(), sources,
    dependencies: dependencies,
    include_directories: include_dir,
    install: true,
    version: version)

# Install headers
install_headers(includes, subdir: 're')

# Declare dependency
re_dep = declare_dependency(
    compile_args: compile_args,
    include_directories: include_dir,
    link_with: re)

# Generate pkg-config file
pkg = import('pkgconfig')
description = '''Generic library for real-time communications with
                 async IO support'''
description = ' '.join(description.split())
pkg.generate(re,
    name: 'libre',
    description: description,
    url: 'http://www.creytiv.com/re.html',
    extra_cflags: compile_args,  # https://github.com/creytiv/re/issues/167
    subdirs: 're')

# TODO: Ensure 'install' has the same result as when invoking 'make'
