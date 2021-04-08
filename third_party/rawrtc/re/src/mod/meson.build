sources += files('mod.c')

if configuration.has('HAVE_DLFCN')
    sources += files('dl.c')
endif

if system == 'windows'
    sources += files('win32/dll.c')
endif
