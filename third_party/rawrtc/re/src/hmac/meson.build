sources += files('hmac_sha1.c')

# TODO: What about apple/hmac.c?
if openssl_dep.found()
    sources += files('openssl/hmac.c')
else
    sources += files('hmac.c')
endif
