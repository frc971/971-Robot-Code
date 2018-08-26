#!/usr/bin/env python3

import sys
import os
import re
import subprocess
import tempfile
import argparse
import hashlib

def get_deps(package):
  out = subprocess.check_output(["apt-rdepends", package])
  deps = out.splitlines()
  return set([dep for dep in deps if not dep.startswith(b" ")])

def get_all_deps(packages):
  deps = set()
  for package in packages or ():
    deps.update(get_deps(package))
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
    yield package

def download_deps(packages, excludes, force_includes):
  deps = get_all_deps(packages)
  exclude_deps = get_all_deps(excludes)
  deps -= exclude_deps
  force_include_deps = get_all_deps(force_includes)
  deps |= force_include_deps
  subprocess.check_call([b"apt-get", b"download"] + list(map_virtual_packages(deps)))

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
    "debconf",
    "debconf-2.0",
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
  parser.add_argument("package", nargs="+", help="The packages to download.")
  args = parser.parse_args(argv[1:])
  folder = tempfile.mkdtemp()
  os.chdir(folder)
  excludes = args.exclude or []
  # Exclude common packages that don't make sense to include in everything all
  # the time.
  excludes += _ALWAYS_EXCLUDE
  download_deps(args.package, excludes, args.force_include)
  fixup_files()
  print_file_list()
  print("Your packages are all in %s" % folder)

if __name__ == "__main__":
  sys.exit(main(sys.argv))
