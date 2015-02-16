#!/usr/bin/python3

import sys
import subprocess
import re
import os
import os.path
import string
import shutil
import errno
import queue
import threading
import pty
import signal

class TestThread(threading.Thread):
  """Runs 1 test and keeps track of its current state.

  A TestThread is either waiting to start the test, actually running it, done,
  running it, or stopped. The first 3 always happen in that order and can
  change to stopped at any time.

  It will finish (ie join() will return) once the process has exited, at which
  point accessing process to see the status is OK.

  Attributes:
    executable: The file path of the executable to run.
    args: A tuple of arguments to give the executable.
    env: The environment variables to set.
    done_queue: A queue.Queue to place self on once done running the test.
    start_semaphore: A threading.Semaphore to wait on before starting.
    process_lock: A lock around process.
    process: The currently executing test process or None. Synchronized by
        process_lock.
    stopped: True if we're stopped.
    output: A queue of lines of output from the test.
  """

  class OutputCopier(threading.Thread):
    """Copies the output of a test from its output pty into a queue.

    This is necessary because otherwise everything locks up if the test writes
    too much output and fills up the pty's buffer.
    """

    def __init__(self, name, fd, queue):
      super(TestThread.OutputCopier, self).__init__(
          name=(name + '.OutputCopier'))

      self.fd = fd
      self.queue = queue

    def run(self):
      with os.fdopen(self.fd) as to_read:
        try:
          for line in to_read:
            self.queue.put(line)
        except IOError as e:
# An EIO from the master side of the pty means we hit the end.
          if e.errno == errno.EIO:
            return
          else:
            raise e

  def __init__(self, executable, args, env, done_queue, start_semaphore):
    super(TestThread, self).__init__(
        name=os.path.split(executable)[-1])

    self.executable = executable
    self.args = args
    self.env = env
    self.done_queue = done_queue
    self.start_semaphore = start_semaphore

    self.output = queue.Queue()

    self.process_lock = threading.Lock()
    self.process = None
    self.stopped = False
    self.returncode = None
    self.output_copier = None

  def run(self):
    def setup_test_process():
# Shove it into its own process group so we can kill any subprocesses easily.
      os.setpgid(0, 0)

    with self.start_semaphore:
      with self.process_lock:
        if self.stopped:
          return
      test_output('Starting test %s...' % self.name)
      output_to_read, subprocess_output = pty.openpty()
      self.output_copier = TestThread.OutputCopier(self.name, output_to_read,
                                                   self.output)
      self.output_copier.start()
      try:
        with self.process_lock:
          self.process = subprocess.Popen((self.name,) + self.args,
                                          executable=self.executable,
                                          env=self.env,
                                          stderr=subprocess.STDOUT,
                                          stdout=subprocess_output,
                                          stdin=open(os.devnull, 'r'),
                                          preexec_fn=setup_test_process)
      finally:
        os.close(subprocess_output)
      self.process.wait()
      with self.process_lock:
        self.returncode = self.process.returncode
        self.process = None
        if not self.stopped:
          self.output_copier.join()
          self.done_queue.put(self)

  def kill_process(self):
    """Forcibly terminates any running process."""
    with self.process_lock:
      if not self.process:
        return
      try:
        os.killpg(self.process.pid, signal.SIGKILL)
      except OSError as e:
        if e.errno == errno.ESRCH:
          # We don't really care if it's already gone.
          pass
        else:
          raise e
  def stop(self):
    """Changes self to the stopped state."""
    with self.process_lock:
      self.stopped = True

def aos_path():
  """Returns:
    A relative path to the aos directory.
  """
  return os.path.join(os.path.dirname(__file__), '..')

def get_ip_base():
  """Retrieves the IP address base."""
  FILENAME = os.path.normpath(os.path.join(aos_path(), '..',
                                           'output', 'ip_base.txt'))
  if not os.access(FILENAME, os.R_OK):
    os.makedirs(os.path.dirname(FILENAME), exist_ok=True)
    with open(FILENAME, 'w') as f:
      f.write('roboRIO-971.local')
  with open(FILENAME, 'r') as f:
    base = f.readline().strip()
  return base

def get_ip(device):
  """Retrieves the IP address for a given device."""
  base = get_ip_base()
  if device == 'prime':
    return base + '.179'
  elif device == 'robot':
    return base + '.2'
  elif device == 'roboRIO':
    return base
  else:
    raise Exception('Unknown device %s to get an IP address for.' % device)

def get_user(device):
  """Retrieves the user for a given device."""
  if device == 'prime':
    return 'driver'
  elif device == 'roboRIO':
    return 'admin'
  else:
    raise Exception('Unknown device %s to get a user for.' % device)

def get_temp_dir(device):
  """Retrieves the temporary download directory for a given device."""
  if device == 'prime':
    return '/tmp/aos_downloader'
  elif device == 'roboRIO':
    return '/home/admin/tmp/aos_downloader'
  else:
    raise Exception('Unknown device %s to get a temp_dir for.' % device)

def get_target_dir(device):
  """Retrieves the tempory deploy directory for a given device."""
  if device == 'prime':
    return '/home/driver/robot_code/bin'
  elif device == 'roboRIO':
    return '/home/admin/robot_code'
  else:
    raise Exception('Unknown device %s to get a temp_dir for.' % device)

def user_output(message):
  """Prints message to the user."""
  print('build.py: ' + message, file=sys.stderr)

# A lock to avoid making a mess intermingling test-related messages.
test_output_lock = threading.RLock()
def test_output(message):
  """Prints message to the user. Intended for messages related to tests."""
  with test_output_lock:
    print('tests: ' + message, file=sys.stdout)

def call_download_externals(argument):
  """Calls download_externals.sh for a given set of externals.

  Args:
    argument: The argument to pass to the shell script to tell it what to
        download.
  """
  subprocess.check_call(
      (os.path.join(aos_path(), 'build', 'download_externals.sh'),
       argument),
      stdin=open(os.devnull, 'r'))

class Processor(object):
  """Represents a processor architecture we can build for."""

  class UnknownPlatform(Exception):
    def __init__(self, message):
      super(Processor.UnknownPlatform, self).__init__()
      self.message = message

  class Platform(object):
    """Represents a single way to build the code."""

    def outdir(self):
      """Returns:
        The path of the directory build outputs get put in to.
      """
      return os.path.join(
          aos_path(), '..', 'output', self.outname())
    def build_ninja(self):
      """Returns:
        The path of the build.ninja file.
      """
      return os.path.join(self.outdir(), 'build.ninja')

    def do_deploy(self, dry_run, command):
      """Helper for subclasses to implement deploy.

      Args:
        dry_run: If True, prints the command instead of actually running it.
        command: A tuple of command-line arguments.
      """
      real_command = (('echo',) + command) if dry_run else command
      subprocess.check_call(real_command, stdin=open(os.devnull, 'r'))

    def deploy(self, dry_run):
      """Downloads the compiled code to the target computer."""
      raise NotImplementedError('deploy should be overriden')
    def outname(self):
      """Returns:
        The name of the directory the code will be compiled to.
      """
      raise NotImplementedError('outname should be overriden')
    def os(self):
      """Returns:
        The name of the operating system this platform is for.

        This will be used as the value of the OS gyp variable.
      """
      raise NotImplementedError('os should be overriden')
    def gyp_platform(self):
      """Returns:
        The platform name the .gyp files know.

        This will be used as the value of the PLATFORM gyp variable.
      """
      raise NotImplementedError('gyp_platform should be overriden')
    def architecture(self):
      """Returns:
        The processor architecture for this platform.

        This will be used as the value of the ARCHITECTURE gyp variable.
      """
      raise NotImplementedError('architecture should be overriden')
    def compiler(self):
      """Returns:
        The compiler used for this platform.

        Everything before the first _ will be used as the value of the
        COMPILER gyp variable and the whole thing will be used as the value
        of the FULL_COMPILER gyp variable.
      """
      raise NotImplementedError('compiler should be overriden')
    def sanitizer(self):
      """Returns:
        The sanitizer used on this platform.

        This will be used as the value of the SANITIZER gyp variable.

        "none" if there isn't one.
      """
      raise NotImplementedError('sanitizer should be overriden')
    def debug(self):
      """Returns:
        Whether or not this platform compiles with debugging information.

        The DEBUG gyp variable will be set to "yes" or "no" based on this.
      """
      raise NotImplementedError('debug should be overriden')
    def build_env(self):
      """Returns:
        A map of environment variables to set while building this platform.
      """
      raise NotImplementedError('build_env should be overriden')
    def priority(self):
      """Returns:
        A relative priority for this platform relative to other ones.

      Higher priority platforms will get built, tested, etc first. Generally,
      platforms which give higher-quality compiler errors etc should come first.
      """
      return 0

  def check_installed(self, platforms, is_deploy):
    """Makes sure that everything necessary to build platforms are installed."""
    raise NotImplementedError('check_installed should be overriden')
  def parse_platforms(self, platforms_string):
    """Args:
      string: A user-supplied string saying which platforms to select.

    Returns:
      A tuple of Platform objects.

    Raises:
      Processor.UnknownPlatform: Parsing string didn't work out.
    """
    raise NotImplementedError('parse_platforms should be overriden')
  def extra_gyp_flags(self):
    """Returns:
      A tuple of extra flags to pass to gyp (if any).
    """
    return ()
  def modify_ninja_file(self, ninja_file):
    """Modifies a freshly generated ninja file as necessary.

    Args:
      ninja_file: Path to the file to modify.
    """
    pass
  def download_externals(self, platforms):
    """Calls download_externals as appropriate to build platforms.

    Args:
      platforms: A list of platforms to download external libraries for.
    """
    raise NotImplementedError('download_externals should be overriden')

  def do_check_installed(self, other_packages):
    """Helper for subclasses to implement check_installed.

    Args:
      other_packages: A tuple of platform-specific packages to check for."""
    all_packages = other_packages
    # Necessary to build stuff.
    all_packages += ('ccache', 'make')
    # Necessary to download stuff to build.
    all_packages += ('wget', 'git', 'subversion', 'patch', 'unzip', 'bzip2')
    # Necessary to build externals stuff.
    all_packages += ('python', 'gcc', 'g++')
    not_found = []
    try:
      # TODO(brians): Check versions too.
      result = subprocess.check_output(
          ('dpkg-query',
           r"--showformat='${binary:Package}\t${db:Status-Abbrev}\n'",
           '--show') + all_packages,
          stdin=open(os.devnull, 'r'),
          stderr=subprocess.STDOUT)
      for line in result.decode('utf-8').rstrip().splitlines(True):
        match = re.match('^([^\t]+)\t[^i][^i]$', line)
        if match:
          not_found.append(match.group(1))
    except subprocess.CalledProcessError as e:
      output = e.output.decode('utf-8').rstrip()
      for line in output.splitlines(True):
        match = re.match(r'dpkg-query: no packages found matching (.*)',
                         line)
        if match:
          not_found.append(match.group(1))
    if not_found:
      user_output('Some packages not installed: %s.' % ', '.join(not_found))
      user_output('Try something like `sudo apt-get install %s`.' %
                  ' '.join(not_found))
      exit(1)

class PrimeProcessor(Processor):
  """A Processor subclass for building prime code."""

  class Platform(Processor.Platform):
    def __init__(self, architecture, compiler, debug, sanitizer):
      super(PrimeProcessor.Platform, self).__init__()

      self.__architecture = architecture
      self.__compiler = compiler
      self.__debug = debug
      self.__sanitizer = sanitizer

    def __repr__(self):
      return 'PrimeProcessor.Platform(architecture=%s, compiler=%s, debug=%s' \
          ', sanitizer=%s)' \
          % (self.architecture(), self.compiler(), self.debug(),
             self.sanitizer())
    def __str__(self):
      return '%s-%s%s-%s' % (self.architecture(), self.compiler(),
                             '-debug' if self.debug() else '', self.sanitizer())

    def os(self):
      return 'linux'
    def gyp_platform(self):
      return '%s-%s-%s' % (self.os(), self.architecture(), self.compiler())
    def architecture(self):
      return self.__architecture
    def compiler(self):
      return self.__compiler
    def sanitizer(self):
      return self.__sanitizer
    def debug(self):
      return self.__debug

    def outname(self):
      return str(self)

    def priority(self):
      r = 0
      if self.compiler() == 'gcc':
        r -= 100
      elif self.compiler() == 'clang':
        r += 100
      if self.sanitizer() != 'none':
        r -= 50
      elif self.debug():
        r -= 10
      if self.architecture() == 'amd64':
        r += 5
      return r

    def deploy(self, dry_run):
      # Downloads code to the prime in a way that avoids clashing too badly with
      # starter (like the naive download everything one at a time).
      if self.compiler().endswith('_frc'):
        device = 'roboRIO'
      else:
        device = 'prime'
      SUM = 'md5sum'
      TARGET_DIR = get_target_dir(device)
      TEMP_DIR = get_temp_dir(device)
      TARGET = get_user(device) + '@' + get_ip(device)

      from_dir = os.path.join(self.outdir(), 'outputs')
      sums = subprocess.check_output((SUM,) + tuple(os.listdir(from_dir)),
                                     stdin=open(os.devnull, 'r'),
                                     cwd=from_dir)
      to_download = subprocess.check_output(
          ('ssh', TARGET,
           """rm -rf {TMPDIR} && mkdir -p {TMPDIR} && \\
             mkdir -p {TO_DIR} && cd {TO_DIR} \\
             && echo '{SUMS}' | {SUM} -c \\
             |& grep -F FAILED | sed 's/^\\(.*\\): FAILED.*$/\\1/g'""".
           format(TMPDIR=TEMP_DIR, TO_DIR=TARGET_DIR, SUMS=sums.decode('utf-8'),
                  SUM=SUM)))
      if not to_download:
        user_output("Nothing to download")
        return
      self.do_deploy(
          dry_run,
          ('scp', '-o', 'Compression yes')
          + tuple([os.path.join(from_dir, f) for f in to_download.decode('utf-8').split('\n')[:-1]])
          + (('%s:%s' % (TARGET, TEMP_DIR)),))
      if not dry_run:
        mv_cmd = ['mv {TMPDIR}/* {TO_DIR} ']
        if device == 'roboRIO':
          mv_cmd.append('&& chmod u+s {TO_DIR}/starter_exe ')
        mv_cmd.append('&& echo \'Done moving new executables into place\' ')
        mv_cmd.append('&& bash -c \'sync && sync && sync\'')
        subprocess.check_call(
            ('ssh', TARGET,
             ''.join(mv_cmd).format(TMPDIR=TEMP_DIR, TO_DIR=TARGET_DIR)))

    def build_env(self):
      OTHER_SYSROOT = '/usr/lib/llvm-3.5'
      SYMBOLIZER_PATH = OTHER_SYSROOT + 'bin/llvm-symbolizer'
      r = {}
      if self.compiler() == 'clang' or self.compiler() == 'gcc_4.8':
        r['LD_LIBRARY_PATH'] = OTHER_SYSROOT + 'lib'
      if self.sanitizer() == 'address':
        r['ASAN_SYMBOLIZER_PATH'] = SYMBOLIZER_PATH
        r['ASAN_OPTIONS'] = \
            'detect_leaks=1:check_initialization_order=1:strict_init_order=1' \
            ':detect_stack_use_after_return=1:detect_odr_violation=2' \
            ':allow_user_segv_handler=1'
      elif self.sanitizer() == 'memory':
        r['MSAN_SYMBOLIZER_PATH'] = SYMBOLIZER_PATH
      elif self.sanitizer() == 'thread':
        r['TSAN_OPTIONS'] = 'external_symbolizer_path=' + SYMBOLIZER_PATH

      r['CCACHE_COMPRESS'] = 'yes'
      r['CCACHE_DIR'] = os.path.abspath(os.path.join(aos_path(), '..', 'output',
                                                     'ccache_dir'))
      r['CCACHE_HASHDIR'] = 'yes'
      if self.compiler().startswith('clang'):
        # clang doesn't like being run directly on the preprocessed files.
        r['CCACHE_CPP2'] = 'yes'
      # Without this, ccache slows down because of the generated header files.
      # The race condition that this opens up isn't a problem because the build
      # system finishes modifying header files before compiling anything that
      # uses them.
      r['CCACHE_SLOPPINESS'] = 'include_file_mtime'

      if self.architecture() == 'amd64':
        r['PATH'] = os.path.join(aos_path(), 'build', 'bin-ld.gold') + \
            ':' + os.environ['PATH']

      return r

  ARCHITECTURES = ('arm', 'amd64')
  COMPILERS = ('clang', 'gcc', 'gcc_frc')
  SANITIZERS = ('address', 'undefined', 'integer', 'memory', 'thread', 'none')
  SANITIZER_TEST_WARNINGS = {
      'memory': (True,
"""We don't have all of the libraries instrumented which leads to lots of false
  errors with msan (especially stdlibc++).
  TODO(brians): Figure out a way to deal with it."""),
  }
  PIE_SANITIZERS = ('memory', 'thread')

  def __init__(self, is_test, is_deploy):
    super(PrimeProcessor, self).__init__()

    platforms = []
    for architecture in PrimeProcessor.ARCHITECTURES:
      for compiler in PrimeProcessor.COMPILERS:
        for debug in [True, False]:
          if ((architecture == 'arm' and not compiler.endswith('_frc')) or
              (architecture == 'amd64' and compiler.endswith('_frc'))):
            # We don't have a compiler to use here.
            continue
          platforms.append(
              self.Platform(architecture, compiler, debug, 'none'))
    for sanitizer in PrimeProcessor.SANITIZERS:
      for compiler in ('clang',):
        if compiler == 'gcc_4.8' and (sanitizer == 'undefined' or
                                      sanitizer == 'integer' or
                                      sanitizer == 'memory'):
          # GCC 4.8 doesn't support these sanitizers.
          continue
        if sanitizer == 'none':
          # We already added sanitizer == 'none' above.
          continue
        platforms.append(
            self.Platform('amd64', compiler, True, sanitizer))
    self.__platforms = frozenset(platforms)

    if is_test:
      default_platforms = self.select_platforms(architecture='amd64',
                                                debug=True)
      for sanitizer, warning in PrimeProcessor.SANITIZER_TEST_WARNINGS.items():
        if warning[0]:
          default_platforms -= self.select_platforms(sanitizer=sanitizer)
    elif is_deploy:
      default_platforms = self.select_platforms(architecture='arm',
                                                compiler='gcc_frc',
                                                debug=False)
    else:
      default_platforms = self.select_platforms(debug=False)
    self.__default_platforms = frozenset(default_platforms)

  def platforms(self):
    return self.__platforms
  def default_platforms(self):
    return self.__default_platforms

  def download_externals(self, platforms):
    to_download = set()
    for architecture in PrimeProcessor.ARCHITECTURES:
      pie_sanitizers = set()
      for sanitizer in PrimeProcessor.PIE_SANITIZERS:
        pie_sanitizers.update(self.select_platforms(architecture=architecture,
                                                    sanitizer=sanitizer))
      if platforms & pie_sanitizers:
        to_download.add(architecture + '-fPIE')

      frc_platforms = self.select_platforms(architecture=architecture,
                                            compiler='gcc_frc')
      if platforms & frc_platforms:
        to_download.add(architecture + '_frc')

      if platforms & (self.select_platforms(architecture=architecture) -
                      pie_sanitizers - frc_platforms):
        to_download.add(architecture)

    for download_target in to_download:
      call_download_externals(download_target)

  def parse_platforms(self, platform_string):
    if platform_string is None:
      return self.default_platforms()
    r = self.default_platforms()
    for part in platform_string.split(','):
      if part == 'all':
        r = self.platforms()
      elif part[0] == '+':
        r = r | self.select_platforms_string(part[1:])
      elif part[0] == '-':
        r = r - self.select_platforms_string(part[1:])
      elif part[0] == '=':
        r = self.select_platforms_string(part[1:])
      else:
        selected = self.select_platforms_string(part)
        r = r - (self.platforms() - selected)
        if not r:
          r = selected
    return r

  def select_platforms(self, architecture=None, compiler=None, debug=None,
                       sanitizer=None):
    r = []
    for platform in self.platforms():
      if architecture is None or platform.architecture() == architecture:
        if compiler is None or platform.compiler() == compiler:
          if debug is None or platform.debug() == debug:
            if sanitizer is None or platform.sanitizer() == sanitizer:
              r.append(platform)
    return set(r)

  def select_platforms_string(self, platforms_string):
    architecture, compiler, debug, sanitizer = None, None, None, None
    for part in platforms_string.split('-'):
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
      elif part == 'all':
        architecture = compiler = debug = sanitizer = None
      else:
        raise Processor.UnknownPlatform(
            '"%s" not recognized as a platform string component.' % part)
    return self.select_platforms(
        architecture=architecture,
        compiler=compiler,
        debug=debug,
        sanitizer=sanitizer)

  def check_installed(self, platforms, is_deploy):
    packages = set(('lzip', 'm4', 'realpath'))
    packages.add('ruby')
    packages.add('clang-3.5')
    packages.add('clang-format-3.5')
    for platform in platforms:
      if (platform.compiler() == 'clang' or platform.compiler() == 'gcc_4.8' or
          platform.compiler() == 'clang_frc'):
        packages.add('clang-3.5')
      if platform.compiler() == 'gcc_4.8':
        packages.add('libcloog-isl3:amd64')
      if is_deploy:
        packages.add('openssh-client')
      if platform.compiler() == 'gcc' and platform.architecture() == 'amd64':
        packages.add('gcc-4.7')
        packages.add('g++-4.7')
      elif platform.compiler() == 'gcc_frc' or platform.compiler() == 'clang_frc':
        packages.add('gcc-4.9-arm-frc-linux-gnueabi')
        packages.add('g++-4.9-arm-frc-linux-gnueabi')

    self.do_check_installed(tuple(packages))

class Bot3PrimeProcessor(PrimeProcessor):
  """A very simple subclass of PrimeProcessor whose main function is to allow
  the building of third robot targets in separate directories from those of
  the main robot."""
  class Platform(PrimeProcessor.Platform):
    def __str__(self):
      return 'bot3-%s' % (super(Bot3PrimeProcessor.Platform, self).__str__())


def strsignal(num):
  # It ends up with SIGIOT instead otherwise, which is weird.
  if num == signal.SIGABRT:
    return 'SIGABRT'
  # SIGCLD is a weird way to spell it.
  if num == signal.SIGCHLD:
    return 'SIGCHLD'

  SIGNALS_TO_NAMES = dict((getattr(signal, n), n)
                          for n in dir(signal) if n.startswith('SIG')
                          and '_' not in n)
  return SIGNALS_TO_NAMES.get(num, 'Unknown signal %d' % num)

def main():
  sys.argv.pop(0)
  exec_name = sys.argv.pop(0)
  def print_help(exit_status=None, message=None):
    if message:
      print(message)
    sys.stdout.write(
"""Usage: {name} [-j n] [action] [-n] [platform] [target|extra_flag]...
Arguments:
  -j, --jobs               Explicitly specify how many jobs to run at a time.
                           Defaults to the number of processors + 2.
  -n, --dry-run            Don't actually do whatever.
                           Currently only meaningful for deploy.
  action                   What to do. Defaults to build.
                           build: Build the code.
                           clean: Remove all the built output.
                           tests: Build and then run tests.
                           deploy: Build and then download.
  platform                 What variants of the code to build.
                           Defaults to something reasonable.
                           See below for details.
  target...                Which targets to build/test/etc.
                           Defaults to everything.
  extra_flag...            Extra flags associated with the targets.
                           --gtest_*: Arguments to pass on to tests.

Specifying targets:
 Targets are combinations of architecture, compiler, and debug flags. Which
  ones actually get run is built up as a set. It defaults to something
  reasonable for the action (specified below).
 The platform specification (the argument given to this script) is a comma-
  separated sequence of hyphen-separated platforms, each with an optional
  prefix.
 Each selector (the things separated by commas) selects all of the platforms
  which match all of its components. Its effect on the set of current platforms
  depends on the prefix character.
 Here are the prefix characters:
    +          Adds the selected platforms.
    -          Removes the selected platforms.
    =          Sets the current set to the selected platforms.
    [none]     Removes all non-selected platforms.
               If this makes the current set empty, acts like =.
  There is also the special psuedo-platform "all" which selects all platforms.
 All of the available platforms:
  {all_platforms}
 Default platforms for deploying:
  {deploy_platforms}
 Default platforms for testing:
  {test_platforms}
 Default platforms for everything else:
  {default_platforms}

Examples of specifying targets:
 build everything: "all"
 only build things with clang: "clang"
 build everything that uses GCC 4.8 (not just the defaults): "=gcc_4.8"
 build all of the arm targets that use clang: "clang-arm" or "arm-clang"
""".format(
    name=exec_name,
    all_platforms=str_platforms(PrimeProcessor(False, False).platforms()),
    deploy_platforms=str_platforms(PrimeProcessor(False, True).default_platforms()),
    test_platforms=str_platforms(PrimeProcessor(True, False).default_platforms()),
    default_platforms=str_platforms(PrimeProcessor(False, False).default_platforms()),
    ))
    if exit_status is not None:
      sys.exit(exit_status)

  def sort_platforms(platforms):
    return sorted(
        platforms, key=lambda platform: (-platform.priority(), str(platform)))

  def str_platforms(platforms):
    r = []
    for platform in sort_platforms(platforms):
      r.append(str(platform))
    if len(r) > 1:
      r[-1] = 'and ' + r[-1]
    return ', '.join(r)

  class Arguments(object):
    def __init__(self):
      self.jobs = os.sysconf('SC_NPROCESSORS_ONLN') + 2
      self.action_name = 'build'
      self.dry_run = False
      self.targets = []
      self.platform = None
      self.extra_flags = []

  args = Arguments()

  if len(sys.argv) < 2:
    print_help(1, 'Not enough arguments')
  args.processor = sys.argv.pop(0)
  args.main_gyp = sys.argv.pop(0)
  VALID_ACTIONS = ['build', 'clean', 'deploy', 'tests']
  while sys.argv:
    arg = sys.argv.pop(0)
    if arg == '-j' or arg == '--jobs':
      args.jobs = int(sys.argv.pop(0))
      continue
    if arg in VALID_ACTIONS:
      args.action_name = arg
      continue
    if arg == '-n' or arg == '--dry-run':
      if args.action_name != 'deploy':
        print_help(1, '--dry-run is only valid for deploy')
      args.dry_run = True
      continue
    if arg == '-h' or arg == '--help':
      print_help(0)
    if re.match('^--gtest_.*$', arg):
      if args.action_name == 'tests':
        args.extra_flags.append(arg)
        continue
      else:
        print_help(1, '--gtest_* is only valid for tests')
    if args.platform:
      args.targets.append(arg)
    else:
      args.platform = arg

  if args.processor == 'prime':
    processor = PrimeProcessor(args.action_name == 'tests',
                               args.action_name == 'deploy')
  elif args.processor == 'bot3_prime':
    processor = Bot3PrimeProcessor(args.action_name == 'tests',
                                   args.action_name == 'deploy')
  else:
    print_help(1, message='Unknown processor "%s".' % args.processor)

  unknown_platform_error = None
  try:
    platforms = processor.parse_platforms(args.platform)
  except Processor.UnknownPlatform as e:
    unknown_platform_error = e.message
    args.targets.insert(0, args.platform)
    platforms = processor.parse_platforms(None)
  if not platforms:
    print_help(1, 'No platforms selected')

  processor.check_installed(platforms, args.action_name == 'deploy')
  processor.download_externals(platforms)

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
    _, _ = function, path
    if issubclass(OSError, excinfo[0]):
      if excinfo[1].errno == errno.ENOENT:
        # Who cares if the file we're deleting isn't there?
        return
    raise excinfo[1]

  def need_to_run_gyp(platform):
    """Determines if we need to run gyp again or not.

    The generated build files are supposed to re-run gyp again themselves, but
    that doesn't work (or at least it used to not) and we sometimes want to
    modify the results anyways.

    Args:
      platform: The platform to check for.
    """
    if not os.path.exists(platform.build_ninja()):
      return True
    if os.path.getmtime(__file__) > os.path.getmtime(platform.build_ninja()):
      return True
    dirs = os.listdir(os.path.join(aos_path(), '..'))
    # Looking through these folders takes a long time and isn't useful.
    if dirs.count('output'):
      dirs.remove('output')
    if dirs.count('.git'):
      dirs.remove('.git')
    return not not subprocess.check_output(
        ('find',) + tuple(os.path.join(aos_path(), '..', d) for d in dirs)
        + ('-newer', platform.build_ninja(),
           '(', '-name', '*.gyp', '-or', '-name', '*.gypi', ')'),
        stdin=open(os.devnull, 'r'))

  def env(platform):
    """Makes sure we pass through important environmental variables.

    Returns:
      An environment suitable for passing to subprocess.Popen and friends.
    """
    build_env = dict(platform.build_env())
    if not 'TERM' in build_env:
      build_env['TERM'] = os.environ['TERM']
    if not 'PATH' in build_env:
      build_env['PATH'] = os.environ['PATH']
    return build_env

  sorted_platforms = sort_platforms(platforms)
  user_output('Building %s...' % str_platforms(sorted_platforms))

  if args.action_name == 'tests':
    for sanitizer, warning in PrimeProcessor.SANITIZER_TEST_WARNINGS.items():
      warned_about = platforms & processor.select_platforms(sanitizer=sanitizer)
      if warned_about:
        user_output(warning[1])
        if warning[0]:
          # TODO(brians): Add a --force flag or something to override this?
          user_output('Refusing to run tests for sanitizer %s.' % sanitizer)
          exit(1)

  num = 1
  for platform in sorted_platforms:
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
             '-DARCHITECTURE=%s' % platform.architecture(),
             '-DCOMPILER=%s' % platform.compiler().split('_')[0],
             '-DFULL_COMPILER=%s' % platform.compiler(),
             '-DDEBUG=%s' % ('yes' if platform.debug() else 'no'),
             '-DSANITIZER=%s' % platform.sanitizer(),
             '-DEXTERNALS_EXTRA=%s' %
             ('-fPIE' if platform.sanitizer() in PrimeProcessor.PIE_SANITIZERS
              else ('_frc' if platform.compiler().endswith('_frc') else ''))) +
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
        processor.modify_ninja_file(platform.build_ninja())
        user_output('Done running gyp')
      else:
        user_output("Not running gyp")

      try:
        call = (tools_config['NINJA'],
                '-C', platform.outdir()) + tuple(args.targets)
        if args.jobs:
          call += ('-j', str(args.jobs))
        subprocess.check_call(call,
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
      done_queue = queue.Queue()
      running = []
      test_start_semaphore = threading.Semaphore(args.jobs)
      if args.targets:
        to_run = []
        for target in args.targets:
          if target.endswith('_test'):
            to_run.append(target)
      else:
        to_run = os.listdir(dirname)
      for f in to_run:
        thread = TestThread(os.path.join(dirname, f), tuple(args.extra_flags),
                            env(platform), done_queue,
                            test_start_semaphore)
        running.append(thread)
        thread.start()
      try:
        while running:
          done = done_queue.get()
          running.remove(done)
          with test_output_lock:
            test_output('Output from test %s:' % done.name)
            try:
              while True:
                line = done.output.get(False)
                if not sys.stdout.isatty():
                  # Remove color escape codes.
                  line = re.sub(r'\x1B\[[0-9;]*[a-zA-Z]', '', line)
                sys.stdout.write(line)
            except queue.Empty:
              pass
            if not done.returncode:
              test_output('Test %s succeeded' % done.name)
            else:
              if done.returncode < 0:
                sig = -done.returncode
                test_output('Test %s was killed by signal %d (%s)' % \
                            (done.name, sig, strsignal(sig)))
              elif done.returncode != 1:
                test_output('Test %s exited with %d' % \
                            (done.name, done.returncode))
              else:
                test_output('Test %s failed' % done.name)
              user_output('Aborting because of test failure for %s.' % \
                          platform)
              exit(1)
      finally:
        if running:
          test_output('Killing other tests...')
# Stop all of them before killing processes because otherwise stopping some of
# them tends to let other ones that are waiting to start go.
          for thread in running:
            thread.stop()
          for thread in running:
            test_output('\tKilling %s' % thread.name)
            thread.kill_process()
            thread.kill_process()
          test_output('Waiting for other tests to die')
          for thread in running:
            thread.kill_process()
            thread.join()
          test_output('Done killing other tests')

    user_output('Done building %s (%d/%d)' % (platform, num, len(platforms)))
    num += 1

if __name__ == '__main__':
  main()
