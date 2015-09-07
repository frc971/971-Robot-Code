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

  if target == '<(AOS)/build/aos.gyp:logging':
    return '//aos/common/logging'

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
    self.__deps = []

  def add_src(self, src):
    self.__srcs.append(src)

  def add_dep(self, dep):
    self.__deps.append(dep)

  def attrs(self):
    r = super(CcBuildTarget, self).attrs();
    r['srcs'] = self.__srcs
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

  def add_src(self, src):
    self.__srcs.append(src)

  def loads(self):
    return set((('aos/build/queues', 'queue_library'),))

  def attrs(self):
    r = super(QueueTarget, self).attrs();
    r['srcs'] = self.__srcs
    return r

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
        name = gyp_target['target_name']
        type = gyp_target['type']
        if (type in ['static_library', 'executable'] and
            not 'includes' in gyp_target):
          cc_type = {
              'static_library': 'cc_library',
              'executable': 'cc_binary',
            }[type]
          target = CcBuildTarget(cc_type, name)

          for dep in gyp_target['dependencies']:
            target.add_dep(gyp_target_to_bazel(gyp_file_name, dep))
          for src in gyp_target['sources']:
            target.add_src(src)
        elif type == 'none':
          target = FilegroupTarget(name)
          for dep in gyp_target['dependencies']:
            target.add_src(gyp_target_to_bazel(gyp_file_name, dep))
        elif 'includes' in gyp_target:
          includes = gyp_target['includes']
          if len(includes) != 1:
            raise RuntimeError(
              'Not sure how to handle multiple includes in %s' % gyp_target)
          include = gyp_file_to_bazel(gyp_file_name, includes[0])
          if include == '//aos/build/queues.gypi':
            vars = gyp_target['variables']
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
          else:
            raise RuntimeError(
              'Unknown include %s for target %s in %s' % (include, name,
                                                          gyp_file_name))
        else:
          raise RuntimeError(
            'Unknown type %s for target %s in %s' % (type, name, gyp_file_name))

        if not target:
          raise
        build_targets.append(target)

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
