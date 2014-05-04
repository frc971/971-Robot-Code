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

def aos_path():
  return os.path.join(os.path.dirname(__file__), '..')

def get_ip(device):
  FILENAME = os.path.normpath(os.path.join(aos_path(), '..', 'output', 'ip_base.txt'))
  if not os.access(FILENAME, os.R_OK):
    os.makedirs(os.path.dirname(FILENAME), exist_ok=True)
    with open(FILENAME, 'w') as f:
      f.write('10.9.71')
  with open(FILENAME, 'r') as f:
    base = f.readline()
  if device == 'prime':
    return base + '.179'
  elif device == 'robot':
    return base + '.2'
  else:
    raise Exception('Unknown device %s to get an IP address for.' % device)

def user_output(message):
  print('build.py: ' + message, file=sys.stderr)

class Processor(object):
  class UnknownPlatform(Exception):
    def __init__(self, message):
      self.message = message

  class Platform(object):
    def outdir(self):
      return os.path.join(
          aos_path(), '..', 'output', self.outname())
    def build_ninja(self):
      return os.path.join(self.outdir(), 'build.ninja')

    def do_deploy(self, dry_run, command):
      real_command = (('echo',) + command) if dry_run else command
      subprocess.check_call(real_command, stdin=open(os.devnull, 'r'))

  # TODO(brians): Verify that this (and its callers) catch everything from a
  # fresh install.
  def do_check_installed(self, other_packages):
    all_packages = () + other_packages
    try:
      result = subprocess.check_output(
          ('dpkg-query', '--show') + all_packages,
          stdin=open(os.devnull, 'r'),
          stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
      user_output('Some packages not installed:\n'
                  + e.output.decode('utf-8').rstrip())
      exit(1)

class CRIOProcessor(Processor):
  class Platform(Processor.Platform):
    def __init__(self, debug, wind_base):
      super(CRIOProcessor.Platform, self).__init__()

      self.debug = debug
      self.wind_base = wind_base

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

    # TODO(brians): test this
    def deploy(self, dry_run):
      self.do_deploy(dry_run,
                     ('ncftpput', get_ip('robot'), '/',
                      os.path.join(self.outdir(), 'lib', 'FRC_UserProgram.out')))

    def build_env(self):
      return {'WIND_BASE': self.wind_base}

  def __init__(self):
    super(CRIOProcessor, self).__init__()

    if 'WIND_BASE' in os.environ:
      self.wind_base = os.environ['WIND_BASE']
    else:
      self.wind_base = '/usr/local/powerpc-wrs-vxworks/wind_base'

  def parse_platforms(self, string):
    if string is None or string == 'crio':
      return (CRIOProcessor.Platform(False, self.wind_base),)
    elif string == 'crio-debug' or string == 'debug':
      return (CRIOProcessor.Platform(True, self.wind_base),)
    else:
      raise Processor.UnknownPlatform('Unknown cRIO platform "%s".' % string)

  def extra_gyp_flags(self):
    return ('-DWIND_BASE=%s' % self.wind_base,)

  def is_crio(self): return True

  def check_installed(self):
    # TODO(brians): Add powerpc-wrs-vxworks (a new enough version too).
    self.do_check_installed(
        ('ncftp',))

class PrimeProcessor(Processor):
  class Platform(Processor.Platform):
    def __init__(self, architecture, compiler, debug, sanitizer):
      super(PrimeProcessor.Platform, self).__init__()

      self.architecture = architecture
      self.compiler = compiler
      self.debug = debug
      self.sanitizer = sanitizer

    def __repr__(self):
      return 'PrimeProcessor.Platform(architecture=%s, compiler=%s, debug=%s' \
          ', sanitizer=%s)' \
          % (self.architecture, self.compiler, self.debug, self.sanitizer)
    def __str__(self):
      return '%s-%s%s-%s' % (self.architecture, self.compiler,
                          '-debug' if self.debug else '', self.sanitizer)

    def os(self):
      return 'linux'
    def gyp_platform(self):
      return '%s-%s-%s' % (self.os(), self.architecture, self.compiler)

    def outname(self):
      return str(self)

    # TODO(brians): test this
    def deploy(self, dry_run):
      """Downloads code to the prime in a way that avoids clashing too badly with starter
      """
      SUM = 'md5sum'
      TARGET_DIR = '/home/driver/robot_code/bin'
      TEMP_DIR = '/tmp/aos_downloader'
      TARGET = 'driver@' + get_ip('prime')

      from_dir = os.path.join(self.outdir(), 'outputs')
      sums = subprocess.check_output((SUM,) + tuple(os.listdir(from_dir)),
                                     stdin=open(os.devnull, 'r'),
                                     cwd=from_dir)
      to_download = subprocess.check_output(
          ('ssh', TARGET,
           """rm -rf {TMPDIR} && mkdir {TMPDIR} && cd {TO_DIR}
             && echo '{SUMS}' | {SUM} --check --quiet
             |& grep -F FAILED | sed 's/^\\(.*\\): FAILED.*"'$'"/\\1/g'""".format(
               TMPDIR=TEMP_DIR, TO_DIR=TARGET_DIR, SUMS=sums, SUM=SUM)))
      if not to_download:
        user_output("Nothing to download")
        return
      self.do_deploy(
          dry_run,
          ('scp', '-o', 'Compression yes') + to_download
          + (('%s:%s' % (TARGET, TEMP_DIR)),))
      if not dry_run:
        subprocess.check_call(
            ('ssh', TARGET,
             """mv {TMPDIR}/* {TO_DIR}
             && echo 'Done moving new executables into place'
             && ionice -c 3 bash -c 'sync && sync && sync'""".format(
                 TMPDIR=TEMP_DIR, TO_DIR=TARGET_DIR)))

    def build_env(self):
      r = {}
      if self.compiler == 'clang' or self.compiler == 'gcc_4.8':
        r['LD_LIBRARY_PATH'] = '/opt/clang-3.5/lib64'
      if self.sanitizer == 'address':
        r['ASAN_SYMBOLIZER_PATH'] = '/opt/clang-3.5/bin/llvm-symbolizer'
        r['ASAN_OPTIONS'] = 'detect_leaks=1:check_initialization_order=1:strict_init_order=1'
      elif self.sanitizer == 'memory':
        r['MSAN_SYMBOLIZER_PATH'] = '/opt/clang-3.5/bin/llvm-symbolizer'
      elif self.sanitizer == 'thread':
        r['TSAN_OPTIONS'] = 'external_symbolizer_path=/opt/clang-3.5/bin/llvm-symbolizer'

      r['CCACHE_COMPRESS'] = 'yes'
      r['CCACHE_DIR'] = \
          os.path.abspath(os.path.join(aos_path(), '..', 'output', 'ccache_dir'))
      r['CCACHE_HASHDIR'] = 'yes'
      if self.compiler == 'clang':
        # clang doesn't like being run directly on the preprocessed files.
        r['CCACHE_CPP2'] = 'yes'
      # Without this, ccache slows down because of the generated header files.
      # The race condition that this opens up isn't a problem because the build
      # system finishes modifying header files before compiling anything that
      # uses them.
      r['CCACHE_SLOPPINESS'] = 'include_file_mtime'

      if self.architecture == 'amd64':
        r['PATH'] = os.path.join(aos_path(), 'build', 'bin-ld.gold') + \
            ':' + os.environ['PATH']

      return r

  ARCHITECTURES = ('arm', 'amd64')
  COMPILERS = ('clang', 'gcc', 'gcc_4.8')
  SANITIZERS = ('address', 'undefined', 'integer', 'memory', 'thread', 'none')
  SANITIZER_TEST_WARNINGS = {
      'memory': (True,
"""We don't have all of the libraries instrumented which leads to lots of false
errors with msan (especially stdlibc++).
TODO(brians): Figure out a way to deal with it."""),
      'undefined': (False,
"""There are several warnings in other people's code that ubsan catches.
The following have been verified non-interesting:
    include/c++/4.8.2/array:*: runtime error: reference binding to null pointer of type 'int'
      This happens with ::std::array<T, 0> and it doesn't seem to cause any issues.
    output/downloaded/eigen-3.2.1/Eigen/src/Core/util/Memory.h:782:*: runtime error: load of misaligned address 0x* for type 'const int', which requires 4 byte alignment
      That's in the CPUID detection code which only runs on x86."""),
  }
  PIE_SANITIZERS = ('memory', 'thread')

  def __init__(self, is_test, is_deploy):
    super(Processor, self).__init__()

    platforms = []
    for architecture in PrimeProcessor.ARCHITECTURES:
      for compiler in PrimeProcessor.COMPILERS:
        for debug in [True, False]:
          if architecture == 'arm' and compiler == 'gcc_4.8':
            # We don't have a compiler to use here.
            continue
          platforms.append(
              PrimeProcessor.Platform(architecture, compiler, debug, 'none'))
    for sanitizer in PrimeProcessor.SANITIZERS:
      for compiler in ('gcc_4.8', 'clang'):
        if compiler == 'gcc_4.8' and (sanitizer == 'undefined' or
                                      sanitizer == 'integer' or
                                      sanitizer == 'memory'):
          # GCC 4.8 doesn't support these sanitizers.
          continue
        # We already added sanitizer == 'none' above.
        if sanitizer != 'none':
          platforms.append(
              PrimeProcessor.Platform('amd64', compiler, True, sanitizer))
    self.platforms = frozenset(platforms)
    if is_test:
      self.default_platforms = self.select_platforms(architecture='amd64',
                                                     debug=True)
      for sanitizer in PrimeProcessor.SANITIZER_TEST_WARNINGS:
        self.default_platforms -= self.select_platforms(sanitizer=sanitizer)
    elif is_deploy:
      # TODO(brians): Switch to deploying the code built with clang.
      self.default_platforms = self.select_platforms(architecture='arm',
                                                     compiler='gcc',
                                                     debug=False)
    else:
      self.default_platforms = self.select_platforms(debug=False)

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
        selected = self.select_platforms_string(part)
        r = r - (self.platforms - selected)
        if not r:
          r = selected
    return r

  def select_platforms(self, architecture=None, compiler=None, debug=None, sanitizer=None):
    r = []
    for platform in self.platforms:
      if architecture is None or platform.architecture == architecture:
        if compiler is None or platform.compiler == compiler:
          if debug is None or platform.debug == debug:
            if sanitizer is None or platform.sanitizer == sanitizer:
              r.append(platform)
    return set(r)

  def select_platforms_string(self, string):
    r = []
    architecture, compiler, debug, sanitizer = None, None, None, None
    for part in string.split('-'):
      if part in PrimeProcessor.ARCHITECTURES:
        architecture = part
      elif part in PrimeProcessor.COMPILERS:
        compiler = part
      elif part in ['debug', 'dbg']:
        debug = True
      elif part in ['release', 'nodebug', 'ndb']:
        debug = False
      elif part in PrimeProcessor.SANITIZERS:
        sanitizer = part
      else:
        raise Processor.UnknownPlatform('Unknown platform string component "%s".' % part)
    return self.select_platforms(
        architecture=architecture,
        compiler=compiler,
        debug=debug,
        sanitizer=sanitizer)

  def check_installed(self):
    self.do_check_installed(
        ('clang-3.5', 'gcc-4.7-arm-linux-gnueabihf',
         'g++-4.7-arm-linux-gnueabihf', 'openssh-client'))

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

    tests_parser = subparsers.add_parser(
        'tests',
        help='run tests')
    AddCommonArgs(tests_parser)
    AddBuildArgs(tests_parser)

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
    processor = PrimeProcessor(args.action_name == 'tests',
                               args.action_name == 'deploy')
  else:
    parser.exit(status=1, message='Unknown processor "%s".' % args.processor)
  processor.check_installed()

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
    user_output("No platforms selected!")
    exit(1)

  def download_externals(argument):
    subprocess.check_call(
        (os.path.join(aos_path(), 'build', 'download_externals.sh'),
         argument),
        stdin=open(os.devnull, 'r'))

  if processor.is_crio():
    download_externals('crio')
  else:
    to_download = set()
    for architecture in PrimeProcessor.ARCHITECTURES:
      for sanitizer in PrimeProcessor.PIE_SANITIZERS:
        if platforms & processor.select_platforms(architecture=architecture,
                                                  sanitizer=sanitizer):
          to_download.add(architecture + '-fPIE')
        if platforms & processor.select_platforms(architecture=architecture,
                                                  sanitizer='none'):
          to_download.add(architecture)
    for download_target in to_download:
      download_externals(download_target)

  class ToolsConfig(object):
    def __init__(self):
      self.variables = {'AOS': aos_path()}
      with open(os.path.join(aos_path(), 'build', 'tools_config'), 'r') as f:
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
    if not os.path.exists(platform.build_ninja()):
      return True
    dirs = os.listdir(os.path.join(aos_path(), '..'))
    # Looking through these folders takes a long time and isn't useful.
    dirs.remove('output')
    dirs.remove('.git')
    return not not subprocess.check_output(
        ('find',) + tuple(os.path.join(aos_path(), '..', d) for d in dirs)
         + ('-newer', platform.build_ninja(),
         '(', '-name', '*.gyp', '-or', '-name', '*.gypi', ')'),
        stdin=open(os.devnull, 'r'))

  def env(platform):
    build_env = dict(platform.build_env())
    if not 'TERM' in build_env:
      build_env['TERM'] = os.environ['TERM']
    if not 'PATH' in build_env:
      build_env['PATH'] = os.environ['PATH']
    return build_env

  to_build = []
  for platform in platforms:
    to_build.append(str(platform))
  if len(to_build) > 1:
    to_build[-1] = 'and ' + to_build[-1]
  user_output('Building %s...' % ', '.join(to_build))

  if args.action_name == 'tests':
    for sanitizer, warning in PrimeProcessor.SANITIZER_TEST_WARNINGS.items():
      warned_about = platforms & processor.select_platforms(sanitizer=sanitizer)
      if warned_about:
        user_output(warning[1])
        if warning[0]:
          # TODO(brians): Add a --force flag or something?
          user_output('Refusing to run tests for sanitizer %s.' % sanitizer)
          exit(1)

  num = 1
  for platform in platforms:
    user_output('Building %s (%d/%d)...' % (platform, num, len(platforms)))
    if args.action_name == 'clean':
      shutil.rmtree(platform.outdir(), onerror=handle_clean_error)
    else:
      if need_to_run_gyp(platform):
        user_output('Running gyp...')
        gyp = subprocess.Popen(
            (tools_config['GYP'],
             '--check',
             '--depth=%s' % os.path.join(aos_path(), '..'),
             '--no-circular-check',
             '-f', 'ninja',
             '-I%s' % os.path.join(aos_path(), 'build', 'aos.gypi'),
             '-I/dev/stdin', '-Goutput_dir=output',
             '-DOS=%s' % platform.os(),
             '-DPLATFORM=%s' % platform.gyp_platform(),
             '-DARCHITECTURE=%s' % platform.architecture,
             '-DCOMPILER=%s' % platform.compiler.split('_')[0],
             '-DFULL_COMPILER=%s' % platform.compiler,
             '-DDEBUG=%s' % ('yes' if platform.debug else 'no'),
             '-DSANITIZER=%s' % platform.sanitizer,
             '-DSANITIZER_FPIE=%s' % ('-fPIE'
                 if platform.sanitizer in PrimeProcessor.PIE_SANITIZERS
                 else '')) +
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
          user_output("Running gyp failed!")
          exit(1)
        if processor.is_crio():
          subprocess.check_call(
              ('sed', '-i',
               's/nm -gD/nm/g', platform.build_ninja()),
              stdin=open(os.devnull, 'r'))
        user_output('Done running gyp')
      else:
        user_output("Not running gyp")

      try:
        subprocess.check_call(
            (tools_config['NINJA'],
             '-C', platform.outdir()) + tuple(targets),
            stdin=open(os.devnull, 'r'),
            env=env(platform))
      except subprocess.CalledProcessError as e:
        if unknown_platform_error is not None:
          user_output(unknown_platform_error)
        raise e

    if args.action_name == 'deploy':
      platform.deploy(args.dry_run)
    elif args.action_name == 'tests':
      dirname = os.path.join(platform.outdir(), 'tests')
      for f in targets or os.listdir(dirname):
        user_output('Running test %s...' % f)
        subprocess.check_call(
            os.path.join(dirname, f),
            env=env(platform))
        user_output('Test %s succeeded' % f)

    user_output('Done building %s (%d/%d)' % (platform, num, len(platforms)))
    num += 1

if __name__ == '__main__':
  main()
