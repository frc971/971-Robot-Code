sources += files([
    'client.c',
    'cstr.c',
    'dname.c',
    'hdr.c',
    'ns.c',
    'rr.c',
    'rrlist.c',
])

if configuration.has('HAVE_RESOLV')
    sources += files('res.c')
endif

if system == 'windows'
    sources += files('win32/srv.c')
elif system == 'darwin'
    sources += files('darwin/srv.c')
    dependencies += dependency(
        'appleframeworks',
        modules: ['SystemConfiguration', 'CoreFoundation'],
        required: true)
endif
