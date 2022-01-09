#!/usr/bin/env python3

import sys
import os
import os.path
import re
import subprocess
import tempfile
import urllib.request
import argparse
import hashlib

def initialize_apt(apt_dir, apt_args, args):
  os.mkdir(os.path.join(apt_dir, 'etc'))
  os.mkdir(os.path.join(apt_dir, 'etc', 'apt'))
  os.mkdir(os.path.join(apt_dir, 'etc', 'apt', 'trusted.gpg.d'))
  os.mkdir(os.path.join(apt_dir, 'etc', 'apt', 'preferences.d'))
  os.mkdir(os.path.join(apt_dir, 'var'))
  os.mkdir(os.path.join(apt_dir, 'var', 'lib'))
  os.mkdir(os.path.join(apt_dir, 'var', 'lib', 'dpkg'))
  with open(os.path.join(apt_dir, 'var', 'lib', 'dpkg', 'status'), 'w'):
    pass
  with open(os.path.join(apt_dir, 'etc', 'apt', 'sources.list'), 'w') as f:
    f.write("""
deb http://deb.debian.org/debian/ {release} main contrib non-free
deb-src http://deb.debian.org/debian/ {release} main contrib non-free

deb https://security.debian.org/debian-security {release}-security main contrib non-free
deb-src https://security.debian.org/debian-security {release}-security main contrib non-free

deb http://deb.debian.org/debian/ {release}-updates main contrib non-free
deb-src http://deb.debian.org/debian/ {release}-updates main contrib non-free

deb http://deb.debian.org/debian {release}-backports main contrib non-free
deb-src http://deb.debian.org/debian {release}-backports main contrib non-free
""".format(release=args.release))
  for key in args.apt_key:
    basename = os.path.basename(key)
    urllib.request.urlretrieve(key, os.path.join(apt_dir, 'etc', 'apt', 'trusted.gpg.d', basename))
  subprocess.check_call(["apt-get"] + apt_args + ["update"])

def get_deps(apt_args, package):
  env = dict(os.environ)
  del env['LD_LIBRARY_PATH']
  out = subprocess.check_output(["apt-rdepends"] + apt_args + [package], env=env)
  deps = out.splitlines()
  return set([dep for dep in deps if not dep.startswith(b" ")])

def get_all_deps(apt_args, packages):
  deps = set()
  for package in packages or ():
    deps.update(get_deps(apt_args, package))
  return deps

def map_virtual_packages(packages):
  '''Maps known virtual packages to the preferred concrete packages which
  provide them.'''
  for package in packages:
    if package == b'python-numpy-abi9':
      yield b'python-numpy'
      continue
    if package == b'python3-numpy-abi9':
      yield b'python3-numpy'
      continue
    if package == b'libjack-0.125':
      yield b'libjack-jackd2-0'
      continue
    if package == b'fonts-freefont':
      yield b'fonts-freefont-ttf'
      continue
    if package == b'gsettings-backend':
      yield b'dconf-gsettings-backend'
      continue
    if package == b'gdal-abi-2-4-0':
      yield b'libgdal20'
      continue
    if package == b'libglu1':
      yield b'libglu1-mesa'
      continue
    if package == b'liblapack.so.3':
      yield b'liblapack3'
      continue
    if package == b'libopencl1':
      yield b'ocl-icd-libopencl1'
      continue
    if package == b'libgcc1':
      yield b'libgcc-s1'
      continue
    if package == b'libopencl-1.2-1':
      yield b'ocl-icd-libopencl1'
      continue
    if package == b'libblas.so.3':
      yield b'libblas3'
      continue
    yield package

def download_deps(apt_args, packages, excludes, force_includes):
  deps = get_all_deps(apt_args, packages)
  exclude_deps = get_all_deps(apt_args, excludes)
  deps -= exclude_deps
  force_include_deps = get_all_deps(apt_args, force_includes)
  deps |= force_include_deps
  env = dict(os.environ)
  del env['LD_LIBRARY_PATH']
  subprocess.check_call([b"apt-get"] + [a.encode('utf-8') for a in apt_args] + [b"download"] + list(map_virtual_packages(deps)), env=env)

def fixup_files():
  # Gotta remove those pesky epoch numbers in the file names. Bazel doesn't
  # like them.
  regex = re.compile(".%3a")
  contents = os.listdir(os.getcwd())
  for deb in contents:
    new_name = regex.sub("", deb)
    if new_name != deb:
      os.rename(deb, new_name)

def sha256_checksum(filename, block_size=65536):
  sha256 = hashlib.sha256()
  with open(filename, 'rb') as f:
    for block in iter(lambda: f.read(block_size), b''):
      sha256.update(block)
  return sha256.hexdigest()

def print_file_list():
  contents = os.listdir(os.getcwd())
  contents.sort()
  print("_files = {")
  for deb in contents:
    print('  "%s": "%s",' % (deb, sha256_checksum(deb)))
  print("}")

_ALWAYS_EXCLUDE = [
    "dbus-session-bus",
    "debconf",
    "debconf-2.0",
    "default-dbus-session-bus",
    "dpkg",
    "install-info",
    "libc-dev",
    "libc6",
    "libc6-dev",
]

def main(argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("--exclude", "-e", type=str, action="append", help="A package to exclude from the list")
  parser.add_argument("--force-include", type=str, action="append", help="Force include this and its dependencies. Even if listed in excludes.")
  parser.add_argument("--arch", type=str, default="amd64", help="Architecture to download files for.")
  parser.add_argument("--apt-dir", type=str, help=" ".join([
    "File to generate and store apt files in.",
    "Helpful for saving time when downloading multiple groups of packages.",
    "Some flags will be ignored in favor of the values used to create this folder, so be careful.",
    ]))
  parser.add_argument("--release", type=str, default="bullseye", help="Debian release to use.")
  parser.add_argument("--apt-key", type=str, action="append", default=[
    "https://ftp-master.debian.org/keys/archive-key-11.asc",
    "https://ftp-master.debian.org/keys/archive-key-11-security.asc",
  ], help="URL of an additional apt archive key to trust.")
  parser.add_argument("package", nargs="+", help="The packages to download.")
  args = parser.parse_args(argv[1:])
  if args.apt_dir:
    apt_dir = args.apt_dir
  else:
    apt_dir = tempfile.mkdtemp()
  apt_args = ["-o", "Dir=" + apt_dir, "-o", "APT::Architecture=" + args.arch]
  if not args.apt_dir:
    print("Creating apt files in %s" % apt_dir)
    initialize_apt(apt_dir, apt_args, args)
  folder = tempfile.mkdtemp()
  os.chdir(folder)
  excludes = args.exclude or []
  # Exclude common packages that don't make sense to include in everything all
  # the time.
  excludes += _ALWAYS_EXCLUDE
  download_deps(apt_args, args.package, excludes, args.force_include)
  fixup_files()
  print_file_list()
  print("Your packages are all in %s" % folder)

if __name__ == "__main__":
  sys.exit(main(sys.argv))
