#!/usr/bin/python3

# This script converts simple Gyp files to their Bazel equivalents.
# It is not intended to be particularly extensible; it is only going to be used
# once over all of our code and then forgotten about.
# This "only has to work on exactly the code we have now" property also means
# it's pretty picky about many things and doesn't try to handle anything
# beyond what we actually have.
#
# It takes a list of folders to deal with.
#
# Running this script requires PyYAML <pyyaml.org> to be installed.

import sys
import os
import yaml
import collections
import re

'''Converts a Gyp filename to its Bazel equivalent.

Args:
  gyp_file_name: The name of the gyp file this is contained in.
                 This is important for resolving relative paths etc.
  file: The name of the file to deal with.
'''
def gyp_file_to_bazel(gyp_file_name, file):
  if file.startswith('<(AOS)/'):
    return '//aos' + file[6:]
  elif file.startswith('<(DEPTH)/'):
    return '//' + file[9:]
  else:
    if '(' in file:
      raise RuntimeError('Bad variable in file "%s"' % file)
    split_gyp = os.path.dirname(gyp_file_name).split('/')
    rest = []
    dotdots = 0
    doing_dotdots = True
    for component in file.split('/'):
      if component == '..':
        dotdots += 1
        if not doing_dotdots:
          raise RuntimeError('Bad use of .. in file "%s"' % file)
      else:
        doing_dotdots = False
        rest.append(component)
    return '/'.join(['/'] + split_gyp[:-dotdots] + rest)

'''Converts a Gyp target to its Bazel equivalent.

Args:
  gyp_file_name: The name of the gyp file this is contained in.
                 This is important for resolving relative paths etc.
  target: The name of the target to deal with.
'''
def gyp_target_to_bazel(gyp_file_name, target):
  if not ':' in target:
    if '.' in target:
      raise RuntimeError('Have a filename instead of a target: "%s"' % target)
    return ':' + target
  if target[0] == ':':
    return target

  # These thin wrappers won't be copied.
  if target == '<(AOS)/build/aos.gyp:logging':
    return '//aos/common/logging'
  if target == '<(AOS)/build/aos.gyp:logging_interface':
    return '//aos/common/logging:logging_interface'

  # These are getting moved to the right place manually.
  if target == '<(AOS)/common/common.gyp:condition':
    return '//aos/linux_code/ipc_lib:condition'
  if target == '<(AOS)/common/common.gyp:mutex':
    return '//aos/linux_code/ipc_lib:mutex'
  if target == '<(AOS)/common/common.gyp:event':
    return '//aos/linux_code/ipc_lib:event'

  # By building ..., we can mostly ignore these.
  if (target == '<(AOS)/build/aos_all.gyp:Prime' or
      target == '../../frc971/frc971.gyp:All'):
    return '//aos:prime_binaries'

  split = target.split(':')
  if len(split) != 2:
    raise RuntimeError('Not sure how to parse target "%s"' % target)

  if split[0] == '<(EXTERNALS)':
    return '//third_party/' + split[1]

  split_path = split[0].rsplit('/', 1)
  if len(split_path) != 2:
    raise RuntimeError('TODO(Brian): Handle referring to this .gyp file as %s!' % split[0])
  if not split_path[1].endswith('.gyp'):
    raise RuntimeError('Not sure how to deal with gyp filename "%s"' % split[0])

  folder = gyp_file_to_bazel(gyp_file_name, split_path[0])

  if not folder.endswith(split_path[1][:-4]):
    raise RuntimeError('Not sure how to deal with non-matching gyp file "%s"' % target)

  return '%s:%s' % (folder, split[1])

'''Represents a Bazel build target.

Subclasses represent actual concrete things which are emitted into a BUILD
file.'''
class BuildTarget(object):
  def __init__(self, type, name):
    self.__type = type
    self.__name = name

  def add_dep(self, bazel_dep):
    self.__deps.append(bazel_dep)

  def _type(self):
    return self.__type

  '''Returns a collections.OrderedDict with all of the attributes on the
  Bazel rule this represents.

  Subclasses are expected to override this and add their own attributes
  in the appropriate order.'''
  def attrs(self):
    r = collections.OrderedDict()
    r['name'] = self.__name
    return r

  '''Returns a set of load statements.

  Subclasses are expected to override this and add their own loads.

  Each element of the result is the arguments to a single load call.'''
  def loads(self):
    return set()

  '''Returns the Bazel representation of a given object as an attribute
  value.'''
  def __to_bazel_string(o):
    if isinstance(o, str):
      return repr(o)
    if hasattr(o, '__iter__'):
      r = ['[']
      for c in o:
        r.append('    %s,' % BuildTarget.__to_bazel_string(c))
      r.append('  ]')
      return '\n'.join(r)
    else:
      return str(o)

  def __str__(self):
    r = [self.__type + '(']
    for name, value in self.attrs().items():
      if value:
        r.append('  %s = %s,' % (name, BuildTarget.__to_bazel_string(value)))
    r.append(')')
    return '\n'.join(r)

'''Represents a cc_* target.'''
class CcBuildTarget(BuildTarget):
  def __init__(self, type, name):
    if not type.startswith('cc_'):
      raise

    super(CcBuildTarget, self).__init__(type, name)

    self.__srcs = []
    self.__hdrs = []
    self.__deps = []
    self.__tags = []

  def add_src(self, src):
    self.__srcs.append(src)

  def add_hdr(self, hdr):
    self.__hdrs.append(hdr)

  def add_dep(self, dep):
    self.__deps.append(dep)

  def add_tag(self, tag):
    if self._type() != 'cc_test':
      raise RuntimeError(
        'Trying to add tag %s to non-test type %s' % (tag, self._type()))
    self.__tags.append(tag)

  def attrs(self):
    r = super(CcBuildTarget, self).attrs();
    r['srcs'] = self.__srcs
    r['hdrs'] = self.__hdrs
    r['tags'] = self.__tags
    r['deps'] = self.__deps
    return r

'''Represents a filegroup target.'''
class FilegroupTarget(BuildTarget):
  def __init__(self, name):
    super(FilegroupTarget, self).__init__('filegroup', name)

    self.__srcs = []

  def add_src(self, src):
    self.__srcs.append(src)

  def attrs(self):
    r = super(FilegroupTarget, self).attrs();
    r['srcs'] = self.__srcs
    return r

'''Represents a queue_library target.'''
class QueueTarget(BuildTarget):
  def __init__(self, name):
    super(QueueTarget, self).__init__('queue_library', name)

    self.__srcs = []
    self.__deps = []

  def add_src(self, src):
    self.__srcs.append(src)

  def add_dep(self, dep):
    self.__deps.append(dep)

  def loads(self):
    return set((('/aos/build/queues', 'queue_library'),))

  def attrs(self):
    r = super(QueueTarget, self).attrs();
    r['srcs'] = self.__srcs
    r['deps'] = self.__deps
    return r

def _warn_attr(keys_to_handle, name, gyp_file_name, attr):
  if attr in keys_to_handle:
    print('Target %s in %s has %s' % (name, gyp_file_name, attr),
          file=sys.stderr)
    keys_to_handle.remove(attr)

def main(argv):
  for d in argv:
    build_targets = []

    d = d.rstrip('/')
    gyp_file_name = os.path.join(d, os.path.split(d)[-1] + '.gyp')
    with open(gyp_file_name, 'r') as gyp_file:
      gyp = yaml.load(gyp_file)
      if 'targets' not in gyp:
        print('No targets entry found in %s!' % gyp_file_name, file=sys.stderr)
        return 1
      if list(gyp.keys()) != ['targets']:
        print('Unknown keys of %s from %s' % (gyp.keys(), gyp_file_name),
              file=sys.stderr)
      targets = gyp['targets']
      for gyp_target in targets:
        target = None
        keys_to_handle = set(gyp_target.keys())
        if 'export_dependent_settings' in gyp_target:
          keys_to_handle.remove('export_dependent_settings')
        name = gyp_target['target_name']
        keys_to_handle.remove('target_name')
        _warn_attr(keys_to_handle, name, gyp_file_name, 'actions')
        _warn_attr(keys_to_handle, name, gyp_file_name, 'conditions')
        _warn_attr(keys_to_handle, name, gyp_file_name, 'copies')
        _warn_attr(keys_to_handle, name, gyp_file_name, 'hard_dependency')
        _warn_attr(keys_to_handle, name, gyp_file_name,
                   'direct_dependent_settings')

        # These are getting moved to the right place manually.
        if gyp_file_name == 'aos/common/common.gyp':
          if name == 'condition' or name == 'mutex' or name == 'event':
            continue
        # By building ..., this becomes irrelevant.
        if gyp_file_name == 'frc971/frc971.gyp':
          if name == 'All':
            continue

        if 'variables' in gyp_target:
          if 'no_rsync' in gyp_target['variables']:
            del gyp_target['variables']['no_rsync']

        type = gyp_target['type']
        keys_to_handle.remove('type')
        if (type in ['static_library', 'executable'] and
            not 'includes' in gyp_target):
          cc_type = {
              'static_library': 'cc_library',
              'executable': 'cc_binary',
            }[type]
          if re.match('.*_test$', name) and cc_type == 'cc_binary':
            cc_type = 'cc_test'
          target = CcBuildTarget(cc_type, name)

          if 'dependencies' in gyp_target:
            for dep in gyp_target['dependencies']:
              target.add_dep(gyp_target_to_bazel(gyp_file_name, dep))
            keys_to_handle.remove('dependencies')
          if 'sources' in gyp_target:
            for src in gyp_target['sources']:
              # In //aos/common:queue_types, this will get dealt with manually
              # along with the actions.
              if src == '<(print_field_cc)':
                continue

              if '/' in src:
                raise RuntimeError(
                  'Bad folder for %s in target %s in %s' % (src, name,
                                                            gyp_file_name))

              target.add_src(src)

              # This is sort of a heuristic: if there's a header file matching
              # the source file, add it as an hdr. This is going to require some
              # manual cleanup, but it'll be close.
              src_filename = os.path.join(os.path.dirname(gyp_file_name), src)
              if not os.path.exists(src_filename):
                raise RuntimeError(
                  'Can not find source %s in target %s' % (src_filename,
                                                           name))
              header = src_filename.rsplit('.', 2)[0] + '.h'
              if os.path.exists(header):
                target.add_hdr(src.rsplit('.', 2)[0] + '.h')
            keys_to_handle.remove('sources')
          if 'variables' in gyp_target:
            vars = gyp_target['variables']
            if 'is_special_test' in vars:
              if vars['is_special_test'] != 1:
                raise RuntimeError(
                  'Unexpected is_special_test value in target %s' % name)
              target.add_tag('manual')
              del vars['is_special_test']
        elif type == 'none':
          target = FilegroupTarget(name)
          for dep in gyp_target['dependencies']:
            target.add_src(gyp_target_to_bazel(gyp_file_name, dep))
          keys_to_handle.remove('dependencies')
        elif 'includes' in gyp_target:
          includes = gyp_target['includes']
          keys_to_handle.remove('includes')
          if len(includes) != 1:
            raise RuntimeError(
              'Not sure how to handle multiple includes in %s' % gyp_target)
          include = gyp_file_to_bazel(gyp_file_name, includes[0])
          if include == '//aos/build/queues.gypi':
            vars = gyp_target['variables']
            keys_to_handle.remove('variables')
            if 'header_path' not in vars:
              raise RuntimeError(
                'No header_path for target %s in %s' % (name, gyp_file_name))
            if list(vars.keys()) != ['header_path']:
              raise RuntimeError(
                'Extra variables for target %s in %s' % (name, gyp_file_name))
            if vars['header_path'] != os.path.dirname(gyp_file_name):
              raise RuntimeError(
                'Incorrect header_path for target %s in %s' % (name,
                                                               gyp_file_name))

            target = QueueTarget(name)
            for src in gyp_target['sources']:
              if '/' in src:
                raise RuntimeError(
                  '.q src %s in bad dir for target %s in %s' % (src,
                                                                name,
                                                                gyp_file_name))
              target.add_src(src)
            keys_to_handle.remove('sources')
            if 'dependencies' in gyp_target:
              for dep in gyp_target['dependencies']:
                target.add_dep(gyp_target_to_bazel(gyp_file_name, dep))
              keys_to_handle.remove('dependencies')
          else:
            raise RuntimeError(
              'Unknown include %s for target %s in %s' % (include, name,
                                                          gyp_file_name))
        else:
          raise RuntimeError(
            'Unknown type %s for target %s in %s' % (type, name, gyp_file_name))

        if not target:
          raise

        if (gyp_file_name == 'y2015/http_status/http_status.gyp' and
            name == 'http_status'):
          # We'll handle these manually.
          keys_to_handle.remove('include_dirs')
        if (gyp_file_name == 'aos/common/common.gyp' and
            name == 'queue_types'):
          # These will get handled manually as part of dealing with the
          # actions.
          keys_to_handle.remove('variables')

        # If there were variables but they all got deleted, then we don't
        # actually have any more to handle.
        if 'variables' in keys_to_handle and not gyp_target['variables']:
          keys_to_handle.remove('variables')
        if keys_to_handle:
          raise RuntimeError(
            'Unhandled keys for target %s in %s: %s' % (name, gyp_file_name,
                                                        keys_to_handle))
        build_targets.append(target)

    if not build_targets:
      print('No output targets for %s' % d, file=sys.stderr)
      continue

    with open(os.path.join(d, 'BUILD'), 'w') as build_file:
      build_file.write(
          'package(default_visibility = [\'//visibility:public\'])\n')
      loads = set()
      for t in build_targets:
        loads |= t.loads()
      if loads:
        build_file.write('\n')
        for load in sorted(loads):
          build_file.write('load(%s)\n' % (', '.join([repr(part) for part
                                                      in load])))
      for t in build_targets:
        build_file.write('\n')
        build_file.write(str(t))
        build_file.write('\n')

if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
