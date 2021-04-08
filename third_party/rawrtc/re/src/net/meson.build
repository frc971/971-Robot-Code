sources += files([
    'if.c',
    'net.c',
    'netstr.c',
    'rt.c',
    'sock.c',
    'sockopt.c',
])

if system == 'windows'
    sources += files('win32/wif.c')
else
    sources += files('posix/pif.c')
endif

if configuration.has('HAVE_ROUTE_LIST')
    if system == 'linux'
        sources += files('linux/rt.c')
    else
        sources += files('bsd/brt.c')
    endif
endif

if configuration.has('HAVE_GETIFADDRS')
    sources += files('ifaddrs.c')
endif
