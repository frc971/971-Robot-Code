# TODO: What about lock.c?
if configuration.has('HAVE_PTHREAD')
    sources += files('rwlock.c')
endif

if system == 'windows'
    sources += files('win32/lock.c')
endif
