licenses(['notice'])

load('/tools/build_rules/select', 'compiler_select')

cc_library(
  name = 'libevent',
  visibility = ['//visibility:public'],
  srcs = [
    'select.c',
    'poll.c',
    'epoll.c',
    'signal.c',

    'event.c',
    'evthread.c',
    'buffer.c',
    'bufferevent.c',
    'bufferevent_sock.c',
    'bufferevent_filter.c',
    'bufferevent_pair.c',
    'listener.c',
    'bufferevent_ratelim.c',
    'evmap.c',
    'log.c',
    'evutil.c',
    'evutil_rand.c',
    'strlcpy.c',

    'event_tagging.c',
    'http.c',
    'evdns.c',
    'evrpc.c',

    'evthread_pthread.c',

    'include/event2/event-config.h',

    #'compat/sys/queue.h',
  ] + glob([
    '*-internal.h',
  ]),
  hdrs = [
    'event.h',
    'evhttp.h',
    'evdns.h',
    'evrpc.h',
    'evutil.h',
  ],
  includes = ['.', 'include'],
  copts = [
    '-Ithird_party/libevent/compat',
    '-Ithird_party/libevent/include',

    # TODO(Brian): Fix the places in the code it uses char* as an intermediate
    # type while doing offsetof stuff.
    '-Wno-cast-align',

    '-Wno-unused-parameter',
    '-Wno-format-nonliteral',
    '-Wno-cast-qual',
    '-Wno-unused-function',
  ] + compiler_select({
      'gcc': [],
      'clang': ['-Wno-incompatible-pointer-types-discards-qualifiers']
  }),
)
