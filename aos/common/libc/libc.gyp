# Converted to bazel
{
  'targets': [
    {
      'target_name': 'aos_strsignal',
      'type': 'static_library',
      'sources': [
        'aos_strsignal.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
    },
    {
      'target_name': 'aos_strsignal_test',
      'type': 'executable',
      'sources': [
        'aos_strsignal_test.cc',
      ],
      'dependencies': [
        'aos_strsignal',
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'dirname',
      'type': 'static_library',
      'sources': [
        'dirname.cc',
      ],
    },
    {
      'target_name': 'dirname_test',
      'type': 'executable',
      'sources': [
        'dirname_test.cc',
      ],
      'dependencies': [
        'dirname',
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'aos_strerror',
      'type': 'static_library',
      'sources': [
        'aos_strerror.cc',
      ],
    },
    {
      'target_name': 'aos_strerror_test',
      'type': 'executable',
      'sources': [
        'aos_strerror_test.cc',
      ],
      'dependencies': [
        'aos_strerror',
        '<(EXTERNALS):gtest',
      ],
    },
  ],
}
