py_binary(
  name = 'generate_compile_command',
  srcs = [
    'generate_compile_command.py',
  ],
  deps = [
    '//third_party/bazel:extra_actions_proto_py',
  ],
)

action_listener(
  name = 'generate_compile_commands_listener',
  visibility = ['//visibility:public'],
  mnemonics = [
    'CppCompile',
  ],
  extra_actions = [':generate_compile_commands_action'],
)

extra_action(
  name = 'generate_compile_commands_action',
  tools = [
    ':generate_compile_command',
  ],
  out_templates = [
    '$(ACTION_ID)_compile_command',
  ],
  cmd = '$(location :generate_compile_command) $(EXTRA_ACTION_FILE)' +
        ' $(output $(ACTION_ID)_compile_command)',
)
