# Project definition
project('rawrtcdc', 'c',
    version: '0.1.3',
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
    '-Wuninitialized',
    '-Wshorten-64-to-32',
    '-pedantic',
])
add_project_arguments(compiler_args, language: 'c')

# Configuration
configuration = configuration_data()

# Dependency: re
# Note: We need to force using our own fork until re has accepted all our patches
re_dep = dependency('librawrre',
    version: '>=0.6.0',
    fallback: ['re', 're_dep'],
    required: true)

# Dependency: usrsctp
sctp_debug = get_option('debug_level') >= 7
usrsctp_dep = dependency('usrsctp',
    version: ['>=1.0.0', '<2'],
    fallback: ['usrsctp', 'usrsctp_dep'],
    default_options: [
        'sctp_build_programs=false',
        'sctp_debug=@0@'.format(sctp_debug),
        'sctp_inet=false',
        'sctp_inet6=false',
    ],
    required: true)

# Dependency: rawrtcc
rawrtcc_dep = dependency('rawrtcc',
    version: '>=0.1.3',
    fallback: ['rawrtcc', 'rawrtcc_dep'],
    required: true)

# Dependencies list
dependencies = [
    re_dep,
    usrsctp_dep,
    rawrtcc_dep,
]

# Feature: Hardware CRC32-C (requires SSE 4.2)
# Note: If it compiles, it only enables the check for SSE 4.2 at runtime.
code = '''
#include <stdint.h>
int main(int argc, char* argv[]) {
    uint32_t eax, ecx;
    eax = 1;
    __asm__("cpuid" : "=c"(ecx) : "a"(eax) : "%ebx", "%edx");
    return 0;
}
'''
have_cpuid = compiler.compiles(code) and host_machine.cpu() == 'x86_64'
configuration.set10('RAWRTCDC_ENABLE_SSE42_CRC32C', have_cpuid)

# Options
configuration.set10('RAWRTCDC_HAVE_SCTP_REDIRECT_TRANSPORT', get_option('sctp_redirect_transport'))

# Version
version = meson.project_version()
version_array = version.split('.')
configuration.set_quoted('RAWRTCDC_VERSION', version)
configuration.set('RAWRTCDC_VERSION_MAJOR', version_array[0])
configuration.set('RAWRTCDC_VERSION_MINOR', version_array[1])
configuration.set('RAWRTCDC_VERSION_PATCH', version_array[2])

# Set debug level
configuration.set('RAWRTC_DEBUG_LEVEL', get_option('debug_level'))

# Includes
include_dir = include_directories('include')
subdir('include')

# Sources
subdir('src')

# Build library
rawrtcdc = library(meson.project_name(), sources,
    dependencies: dependencies,
    include_directories: include_dir,
    install: true,
    version: version)

# Declare dependency
rawrtcdc_dep = declare_dependency(
    include_directories: include_dir,
    link_with: rawrtcdc)

# Generate pkg-config file
pkg = import('pkgconfig')
pkg.generate(rawrtcdc,
    name: meson.project_name(),
    description: 'A standalone WebRTC and ORTC data channel implementation.',
    url: 'https://github.com/rawrtc/rawrtc-data-channel')
