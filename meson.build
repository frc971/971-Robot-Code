# Project definition
project('rawrtcc', 'c',
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

# Dependencies list
dependencies = [
    re_dep,
]

# Version
version = meson.project_version()
version_array = version.split('.')
configuration.set_quoted('RAWRTCC_VERSION', version)
configuration.set('RAWRTCC_VERSION_MAJOR', version_array[0])
configuration.set('RAWRTCC_VERSION_MINOR', version_array[1])
configuration.set('RAWRTCC_VERSION_PATCH', version_array[2])

# Set debug level
configuration.set('RAWRTC_DEBUG_LEVEL', get_option('debug_level'))

# Includes
include_dir = include_directories('include')
subdir('include')

# Sources
subdir('src')

# Build library
rawrtcc = library(meson.project_name(), sources,
    dependencies: dependencies,
    include_directories: include_dir,
    install: true,
    version: version)

# Declare dependency
rawrtcc_dep = declare_dependency(
    include_directories: include_dir,
    link_with: rawrtcc)

# Generate pkg-config file
pkg = import('pkgconfig')
pkg.generate(rawrtcc,
    name: meson.project_name(),
    description: 'Common code used in RAWRTC projects.',
    url: 'https://github.com/rawrtc/rawrtc-common')
