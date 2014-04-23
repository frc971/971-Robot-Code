#!/usr/bin/python3

import argparse
import sys
import subprocess
import re
import os
import os.path
import string
import shutil
import errno

class Processor(object):
  class UnknownPlatform(Exception):
    def __init__(self, message):
      self.message = message

  class Platform(object):
    def outdir(self):
      return os.path.join(
          Processor.aos_path(), '..', 'output', self.outname())
    def build_ninja(self):
      return os.path.join(self.outdir(), 'build.ninja')

  def aos_path():
    return os.path.join(os.path.dirname(__file__), '..')

class CRIOProcessor(Processor):
  class Platform(Processor.Platform):
    def __init__(self, debug):
      super(CRIOProcessor.Platform, self).__init__()

      self.debug = debug

    def __repr__(self):
      return 'CRIOProcessor.Platform(debug=%s)' % self.debug
    def __str__(self):
      return 'crio%s' % ('-debug' if self.debug else '')

    def outname(self):
      return 'crio-debug' if self.debug else 'crio'
    def os(self):
      return 'vxworks'
    def gyp_platform(self):
      return 'crio'
    def architecture(self):
      return 'ppc'
    def compiler(self):
      return 'gcc'

  def __init__(self):
    super(CRIOProcessor, self).__init__()

    if 'WIND_BASE' in os.environ:
      self.wind_base = os.environ['WIND_BASE']
    else:
      self.wind_base = '/usr/local/powerpc-wrs-vxworks/wind_base'

  def parse_platforms(self, string):
    if string is None or string == 'crio':
      return (CRIOProcessor.Platform(False),)
    elif string == 'crio-debug':
      return (CRIOProcessor.Platform(True),)
    else:
      raise Processor.UnknownPlatform('Unknown cRIO platform "%s".' % string)

  def build_env(self):
    return {'WIND_BASE': self.wind_base}
  def extra_gyp_flags(self):
    return ('-DWIND_BASE=%s' % self.wind_base,)

  def is_crio(self): return True

class PrimeProcessor(Processor):
  class Platform(Processor.Platform):
    def __init__(self, architecture, compiler, debug):
      super(PrimeProcessor.Platform, self).__init__()

      self.architecture = architecture
      self.compiler = compiler
      self.debug = debug

    def __repr__(self):
      return 'PrimeProcessor.Platform(architecture=%s, compiler=%s, debug=%s)' \
          % (self.architecture, self.compiler, self.debug)
    def __str__(self):
      return '%s-%s%s' % (self.architecture, self.compiler,
                          '-debug' if self.debug else '')

    def os(self):
      return 'linux'
    def gyp_platform(self):
      return '%s-%s-%s' % (self.os(), self.architecture, self.compiler)

    def outname(self):
      return str(self)

  ARCHITECTURES = ['arm', 'amd64']
  COMPILERS = ['clang', 'gcc']

  def __init__(self):
    super(Processor, self).__init__()

    platforms = []
    for architecture in PrimeProcessor.ARCHITECTURES:
      for compiler in PrimeProcessor.COMPILERS:
        for debug in [True, False]:
          platforms.append(
              PrimeProcessor.Platform(architecture, compiler, debug))
    self.platforms = frozenset(platforms)
    self.default_platforms = self.select_platforms(debug=False)

  def build_env(self):
    return {}
  def extra_gyp_flags(self):
    return ()
  def is_crio(self): return False

  def parse_platforms(self, string):
    if string is None:
      return self.default_platforms
    r = self.default_platforms
    for part in string.split(','):
      if part[0] == '+':
        r = r | self.select_platforms_string(part[1:])
      elif part[0] == '-':
        r = r - self.select_platforms_string(part[1:])
      elif part[0] == '=':
        r = self.select_platforms_string(part[1:])
      else:
        r = r - (self.platforms - self.select_platforms_string(part))
    return r

  def select_platforms(self, architecture=None, compiler=None, debug=None):
    r = []
    for platform in self.platforms:
      if architecture is None or platform.architecture == architecture:
        if compiler is None or platform.compiler == compiler:
          if debug is None or platform.debug == debug:
            r.append(platform)
    return set(r)

  def select_platforms_string(self, string):
    r = []
    architecture, compiler, debug = None, None, None
    for part in string.split('-'):
      if part in PrimeProcessor.ARCHITECTURES:
        architecture = part
      elif part in PrimeProcessor.COMPILERS:
        compiler = part
      elif part in ['debug', 'dbg']:
        debug = True
      elif part in ['release', 'nodebug', 'ndb']:
        debug = False
      else:
        raise Processor.UnknownPlatform('Unknown platform string component "%s".' % part)
    return self.select_platforms(
        architecture=architecture,
        compiler=compiler,
        debug=debug)

def main():
  class TryParsingAgain(Exception):
    pass

  class TryAgainArgumentParser(argparse.ArgumentParser):
    def __init__(self, **kwargs):
      super(TryAgainArgumentParser, self).__init__(**kwargs)

    def error(self, message):
      raise TryParsingAgain

  def SetUpParser(parser, args):
    def AddBuildArgs(parser):
      parser.add_argument(
          'target',
          help='target to build',
          nargs='*')
    def AddCommonArgs(parser):
      parser.add_argument(
          'platforms',
          help='platform(s) to act on',
          nargs='?')

    parser.add_argument('--processor', required=True, help='prime or crio')
    parser.add_argument('--main_gyp', required=True, help='main .gyp file')
    subparsers = parser.add_subparsers(dest='action_name')

    build_parser = subparsers.add_parser(
        'build',
        help='build the code (default)')
    AddCommonArgs(build_parser)
    AddBuildArgs(build_parser)

    clean_parser = subparsers.add_parser(
        'clean',
        help='remove all output directories')
    AddCommonArgs(clean_parser)

    deploy_parser = subparsers.add_parser(
        'deploy',
        help='build and download the code')
    AddCommonArgs(deploy_parser)
    AddBuildArgs(deploy_parser)
    deploy_parser.add_argument(
      '-n', '--dry-run',
      help="don't actually download anything",
      action='store_true')

    return parser.parse_args(args)

  try:
    parser = TryAgainArgumentParser()
    args = SetUpParser(parser, sys.argv[1:])
  except TryParsingAgain:
    parser = argparse.ArgumentParser()
    REQUIRED_ARGS_END = 5
    args = SetUpParser(parser, sys.argv[1:REQUIRED_ARGS_END] + ['build'] +
                               sys.argv[(REQUIRED_ARGS_END):])

  if args.processor == 'crio':
    processor = CRIOProcessor()
  elif args.processor == 'prime':
    processor = PrimeProcessor()
  else:
    parser.exit(status=1, message='Unknown processor "%s".' % args.processor)

  if 'target' in args:
    targets = args.target[:]
  else:
    targets = []
  unknown_platform_error = None
  try:
    platforms = processor.parse_platforms(args.platforms)
  except Processor.UnknownPlatform as e:
    unknown_platform_error = e.message
    targets.append(args.platforms)
    platforms = processor.parse_platforms(None)
  if not platforms:
    print("No platforms selected!", file=sys.stderr)
    exit(1)

  def download_externals(argument):
    subprocess.check_call(
        (os.path.join(Processor.aos_path(), 'build', 'download_externals.sh'),
         argument),
        stdin=open('/dev/null', 'r'))

  if processor.is_crio():
    download_externals('crio')
  else:
    for architecture in PrimeProcessor.ARCHITECTURES:
      if platforms & processor.select_platforms(architecture=architecture):
        download_externals(architecture)

  class ToolsConfig(object):
    def __init__(self):
      self.variables = {'AOS': Processor.aos_path()}
      with open(os.path.join(Processor.aos_path(), 'build', 'tools_config'), 'r') as f:
        for line in f:
          if line[0] == '#':
            pass
          elif line.isspace():
            pass
          else:
            new_name, new_value = line.rstrip().split('=')
            for name, value in self.variables.items():
              new_value = new_value.replace('${%s}' % name, value)
            self.variables[new_name] = new_value
    def __getitem__(self, key):
      return self.variables[key]

  tools_config = ToolsConfig()

  def handle_clean_error(function, path, excinfo):
    if issubclass(OSError, excinfo[0]):
      if excinfo[1].errno == errno.ENOENT:
        # Who cares if the file we're deleting isn't there?
        return
    raise excinfo[1]

  def need_to_run_gyp(platform):
    try:
      build_mtime = os.stat(platform.build_ninja()).st_mtime
    except OSError as e:
      if e.errno == errno.ENOENT:
        return True
      else:
        raise e
    pattern = re.compile('.*\.gyp[i]$')
    for dirname, _, files in os.walk(os.path.join(Processor.aos_path(), '..')):
      for f in [f for f in files if pattern.match(f)]:
        if (os.stat(os.path.join(dirname, f)).st_mtime > build_mtime):
          return True
    return False

  for platform in platforms:
    print('Building %s...' % platform, file=sys.stderr)
    if args.action_name == 'clean':
      shutil.rmtree(platform.outdir(), onerror=handle_clean_error)
    else:
      if need_to_run_gyp(platform):
        print('Running gyp...', file=sys.stderr)
        gyp = subprocess.Popen(
            (tools_config['GYP'],
             '--check',
             '--depth=%s' % os.path.join(Processor.aos_path(), '..'),
             '--no-circular-check',
             '-f', 'ninja',
             '-I%s' % os.path.join(Processor.aos_path(), 'build', 'aos.gypi'),
             '-I/dev/stdin', '-Goutput_dir=output',
             '-DOS=%s' % platform.os(),
             '-DPLATFORM=%s' % platform.gyp_platform(),
             '-DARCHITECTURE=%s' % platform.architecture,
             '-DCOMPILER=%s' % platform.compiler,
             '-DDEBUG=%s' % ('yes' if platform.debug else 'no')) +
            processor.extra_gyp_flags() + (args.main_gyp,),
            stdin=subprocess.PIPE)
        gyp.communicate(("""
{
  'target_defaults': {
    'configurations': {
      '%s': {}
    }
  }
}""" % platform.outname()).encode())
        if gyp.returncode:
          print("Running gyp failed!", file=sys.stderr)
          exit(1)
        if processor.is_crio():
          subprocess.check_call(
              ('sed', '-i',
               's/nm -gD/nm/g', platform.build_ninja()),
              stdin=open('/dev/null', 'r'))
        print('Done running gyp.', file=sys.stderr)
      else:
        print("Not running gyp.", file=sys.stderr)

      try:
        build_env = dict(processor.build_env())
        build_env['TERM'] = os.environ['TERM']
        build_env['PATH'] = os.environ['PATH']
        subprocess.check_call(
            (tools_config['NINJA'],
             '-C', platform.outdir()) + tuple(targets),
            stdin=open('/dev/null', 'r'),
            env=build_env)
      except subprocess.CalledProcessError as e:
        if unknown_platform_error is not None:
          print(unknown_platform_error, file=sys.stderr)
        raise e

    # TODO(brians): deploy and tests
    print('Done building %s...' % platform, file=sys.stderr)

if __name__ == '__main__':
  main()
