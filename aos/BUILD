filegroup(
  name = 'prime_binaries',
  visibility = ['//visibility:public'],
  srcs = [
    '//aos/linux_code:core',
    '//aos/common/logging:log_streamer',
    '//aos/common/logging:log_displayer',
    '//aos/linux_code/starter',
  ],
)
filegroup(
  name = 'prime_start_binaries',
  visibility = ['//visibility:public'],
  srcs = [
    '//aos/common/logging:binary_log_writer',
  ],
)

filegroup(
  name = 'prime_binaries_stripped',
  visibility = ['//visibility:public'],
  srcs = [
    # starter is hard coded to look for a non-stripped core...
    '//aos/linux_code:core',
    '//aos/common/logging:log_streamer.stripped',
    '//aos/common/logging:log_displayer.stripped',
    '//aos/linux_code/starter',
  ],
)
filegroup(
  name = 'prime_start_binaries_stripped',
  visibility = ['//visibility:public'],
  srcs = [
    '//aos/common/logging:binary_log_writer.stripped',
  ],
)
