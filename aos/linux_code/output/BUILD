package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'http_server',
  srcs = [
    'HTTPServer.cpp',
    'evhttp_ctemplate_emitter.cc',
    'ctemplate_cache.cc',
  ],
  hdrs = [
    'HTTPServer.h',
    'evhttp_ctemplate_emitter.h',
    'ctemplate_cache.h',
  ],
  deps = [
    '//third_party/libevent',
    '//third_party/ctemplate',
    '//aos/common:once',
    '//aos/common:scoped_fd',
    '//aos/common/logging',
  ],
)
