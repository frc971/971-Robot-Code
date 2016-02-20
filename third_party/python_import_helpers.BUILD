# This repository is used to create helper __init__.py files which import things
# under third_party at the root of the Python import namespace instead of
# requiring third_party.xyz. in front of them.

genrule(
  name = 'google_protobuf_importer_init',
  outs = ['google/__init__.py'],
  cmd = 'echo "%s" > $@' % '\n'.join([
    'import sys',
    'import third_party.protobuf.google.protobuf',
    'sys.modules[\'google.protobuf\'] = third_party.protobuf.google.protobuf',
  ]),
)

py_library(
  name = 'google_protobuf_importer',
  visibility = ['//visibility:public'],
  srcs = [':google_protobuf_importer_init'],
)
