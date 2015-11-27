py_library(
  name = 'gflags',
  visibility = ['//visibility:public'],
  srcs = [
    'gflags_validators.py',
    'gflags2man.py',
    'gflags.py',
  ],
)

py_library(
  name = 'gflags_googletest',
  srcs = [
    'tests/gflags_googletest.py',
  ],
)

py_test(
  name = 'gflags_validators_test',
  srcs = [
    'tests/gflags_validators_test.py',
  ],
  deps = [
    ':gflags',
    ':gflags_googletest',
  ],
  size = 'small',
)

py_library(
  name = 'flags_modules_for_testing',
  srcs = [
    'tests/flags_modules_for_testing/__init__.py',
    'tests/flags_modules_for_testing/module_bar.py',
    'tests/flags_modules_for_testing/module_baz.py',
    'tests/flags_modules_for_testing/module_foo.py',
  ],
  deps = [
    ':gflags',
  ],
)

py_test(
  name = 'gflags_unittest',
  srcs = [
    'tests/gflags_unittest.py',
  ],
  deps = [
    ':flags_modules_for_testing',
    ':gflags',
    ':gflags_googletest',
  ],
  size = 'small',
)

py_test(
  name = 'gflags_helpxml_test',
  srcs = [
    'tests/gflags_helpxml_test.py',
  ],
  deps = [
    ':flags_modules_for_testing',
    ':gflags',
    ':gflags_googletest',
  ],
  size = 'small',
)
