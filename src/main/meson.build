sources += files([
    'init.c',
    'main.c',
    'method.c',
])

if configuration.has('HAVE_EPOLL')
    sources += files('epoll.c')
endif

if openssl_dep.found()
    sources += files('openssl.c')
endif
