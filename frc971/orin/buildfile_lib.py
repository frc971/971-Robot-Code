#!/usr/bin/env python3

from __future__ import annotations
import apt_pkg
import sys
import collections
import contextlib
import datetime
import functools
import jinja2
import os
import pathlib
import platform
import re
import shlex
import shutil
import subprocess
import tempfile

apt_pkg.init_system()


class NameVersion:
    """ Class representing a package name and optionally a version constraint. """

    def __init__(self, nameversion: str):
        # We are processing package names here like:
        #   python3:any
        #   python3-markdown (= 3.4.1-2)
        if '(' in nameversion:
            s = nameversion.split(' (')
            self.name = s[0].strip()

            v = s[1][:-1].split(' ')

            self.operator = v[0]
            self.version = v[1]
        else:
            self.name = nameversion.strip()
            self.operator = None
            self.version = None

        # Rip off :amd64 or :aarch64 from the name if it is here.
        if ':' in self.name:
            self.name = self.name.split(':')[0]

    def matches(self, other: 'NameVersion') -> bool:
        """If self meets the requirements defined by other."""
        if other.name != self.name:
            return False

        if other.operator is None:
            return True

        # libz1 is special and doesn't have a version...  Don't stress it for now until we learn why.
        if self.operator is None and self.name == 'libz1':
            return True

        vc = apt_pkg.version_compare(self.version, other.version)
        if vc < 0:
            return other.operator in ('<=', '<<')
        elif vc == 0:
            return other.operator in ('=', '>=', '<=')
        elif vc > 0:
            return other.operator in ('>=', '>>')

    def __repr__(self) -> str:
        if self.operator is not None:
            return f"NameVersion({self.name} ({self.operator} {self.version}))"
        else:
            return f"NameVersion({self.name})"


class Package:

    def __init__(self, name: str, provides: str, version: str, depends: str,
                 files: list[str]):
        self.name = NameVersion(f"{name} (= {version})")

        self.provides = [self.name]

        if provides:
            for package_and_version in provides.split(","):
                self.provides.append(NameVersion(package_and_version))

        self.depends = []
        if depends:
            for package_and_version in depends.split(", "):
                if ' | ' in package_and_version:
                    oneof = []
                    for oneof_package_and_version in package_and_version.split(
                            ' | '):
                        oneof.append(NameVersion(oneof_package_and_version))
                    self.depends.append(oneof)
                else:
                    self.depends.append([NameVersion(package_and_version)])

        self.files = files

    def update_filetypes(self, directories: set[str], symlinks: dict[str,
                                                                     str]):
        if hasattr(self, 'directories') or hasattr(self, 'symlinks'):
            return

        self.directories = []
        self.symlinks = dict()
        files = []
        for f in self.files:
            if f in directories:
                self.directories.append(f)
            elif f in symlinks:
                self.symlinks[f] = symlinks[f]
            else:
                files.append(f)

        self.files = files

    def matches(self, other: NameVersion) -> bool:
        """If self meets the requirements defined by other."""
        return any(p.matches(other) for p in self.provides)

    def resolved_depends(self, packages: dict['Package']) -> list['Package']:
        result = set()

        # The dependencies are lists of lists of dependencies.  At least one
        # element from each inner list needs to match for it to be valid.  Most
        # of the dependencies are going to be a single element list.
        for p_or_list in self.depends:
            resolved_set = set()
            for oneof_package in p_or_list:
                if oneof_package.name not in packages:
                    continue

                resolved_oneof_package = packages[oneof_package.name]
                if resolved_oneof_package.matches(oneof_package):
                    resolved_set.add(resolved_oneof_package)

            if len(resolved_set) == 0:
                raise RuntimeError(
                    f"Failed to find dependencies for {p_or_list}: {repr(self)}"
                )

            result.update(resolved_set)

        return sorted(list(result), key=lambda x: x.name.name)

    def headers(self) -> list[str]:
        return [h for h in self.files if h.startswith('/usr/include')]

    def objects(self) -> list[str]:
        result = []
        for file in self.files:
            if not file.startswith('/usr'):
                continue

            # Gotta love GDB extensions ...libc.so....py.  Ignore them.
            if file.endswith('.py'):
                continue

            # We want to find things like libfoo.so.1.2.3.4.5.  The .so needs to be last.
            opath = file
            found_so = False
            while True:
                opath, ext = os.path.splitext(opath)
                if ext == '':
                    break
                elif ext == '.so':
                    found_so = True

            if found_so:
                result.append(file)

        return sorted(result)

    def __repr__(self) -> str:
        return f"{{ {repr(self.provides[0])}, \"provides\": {repr(self.provides[1:])}, \"depends\": {repr(self.depends)} }}"


class PkgConfig:

    def __init__(self, contents, package):
        # The pkgconfig file format lets you specify variables and the expand
        # them into the various fields.  These are in the form
        #   asdf=15234
        self.variables = dict()

        self.package = package
        self.libs = []
        self.cflags = []
        self.requires = []
        for line in contents.split('\n'):
            line = line.strip()
            # Parse everything so we learn if a new field shows up we don't
            # know how to parse.
            if line == '':
                pass
            elif line[0] == '#':
                pass
            elif line.startswith('Name:'):
                self.name = self.expand(line.removeprefix('Name:').strip())
            elif line.startswith('Description:'):
                self.description = self.expand(
                    line.removeprefix('Description:').strip())
            elif line.startswith('Version:'):
                self.version = self.expand(
                    line.removeprefix('Version:').strip())
            elif line.startswith('Libs:'):
                self.libs = self.expand(
                    line.removeprefix('Libs:').strip()).split()
            elif line.startswith('Cflags:'):
                self.cflags = self.expand(
                    line.removeprefix('Cflags:').strip()).split()
            elif line.startswith('URL:') or line.startswith('Url:'):
                pass
            elif line.startswith('Cflags.private:'):
                pass
            elif line.startswith('Requires:'):
                # Parse a Requires line of the form:
                # Requires: glib-2.0 >= 2.56.0, gobject-2.0
                self.requires += [
                    f.split()[0] for f in self.expand(
                        line.removeprefix('Requires:').strip()).split(',') if f
                ]
            elif line.startswith('Requires.private:'):
                # Parse a Requires.private line of the form:
                # Requires.private: gmodule-2.0
                self.requires += [
                    f.split()[0] for f in self.expand(
                        line.removeprefix('Requires.private:').strip()).split(
                            ',') if f
                ]
            elif line.startswith('Libs.private:'):
                pass
            elif line.startswith('Conflicts:'):
                pass
            elif re.match('^[-a-zA-Z_0-9]* *=.*$', line):
                split_line = re.split(' *= *', line)
                self.variables[split_line[0]] = self.expand(split_line[1])
            else:
                raise ValueError('Unknown line in pkgconfig file')

        if self.name is None:
            raise RuntimeError("Failed to find Name.")

    def expand(self, line: str) -> str:
        """ Expands a string with variable expansions in it like bash (${foo}). """
        for var in self.variables:
            line = line.replace('${' + var + '}', self.variables[var])
        return line


class Filesystem:

    def __init__(self, partition):
        self.partition = partition
        # TODO(austin): I really want to be able to run this on an amd64
        # filesystem too, which won't work with qemu-aarch64-static.  Pull it
        # into a library.
        result = subprocess.run([
            "sudo", "chroot", "--userspec=0:0", f"{self.partition}",
            "/usr/bin/qemu-aarch64-static", "/bin/bash", "-c",
            "qemu-aarch64-static /usr/bin/dpkg-query -W -f='Version: ${Version}\nPackage: ${Package}\nProvides: ${Provides}\nDepends: ${Depends}\n${db-fsys:Files}--\n'"
        ],
                                check=True,
                                stdout=subprocess.PIPE)

        # Mapping from all package names (str) to their corresponding Package
        # objects for that package.
        self.packages = dict()

        package_in_progress = {'files': []}
        files = set()
        for line in result.stdout.decode('utf-8').strip().split('\n'):
            if line == '--':
                # We found the end of line deliminator, save the package and
                # clear everything out.
                new_package = Package(package_in_progress['Package'],
                                      package_in_progress['Provides'],
                                      package_in_progress['Version'],
                                      package_in_progress['Depends'],
                                      package_in_progress['files'])

                for provides in new_package.provides:
                    self.packages[provides.name] = new_package

                # Wipe everything so we detect if any fields are missing.
                package_in_progress = {'files': []}
            elif line.startswith("Version: "):
                package_in_progress['Version'] = line.removeprefix("Version: ")
            elif line.startswith("Package: "):
                package_in_progress['Package'] = line.removeprefix("Package: ")
            elif line.startswith("Provides: "):
                package_in_progress['Provides'] = line.removeprefix(
                    "Provides: ")
            elif line.startswith("Depends: "):
                package_in_progress['Depends'] = line.removeprefix("Depends: ")
            else:
                assert (line.startswith(' '))
                f = line.removeprefix(' ')
                package_in_progress['files'].append(f)
                files.add(f)

        self.directories = set()
        self.symlinks = dict()

        for root, walked_dirs, walked_files in os.walk(self.partition):
            for entry in walked_files + walked_dirs:
                full_target = os.path.join(root, entry)
                if pathlib.Path(full_target).is_symlink():
                    target = full_target.removeprefix(self.partition)
                    self.symlinks[target] = os.readlink(full_target)

        for file in files:
            full_target = f"{self.partition}/{file}"
            try:
                if pathlib.Path(full_target).is_symlink():
                    self.symlinks[file] = os.readlink(full_target)

                if pathlib.Path(full_target).is_dir():
                    self.directories.add(file)
            except PermissionError:
                # Assume it is a file...
                print("Failed to read", file, file=sys.stderr)
                pass

            # Directories are all the things before the last /
            for parent in pathlib.Path(file).parents:
                self.directories.add(parent)

        # Now, populate self.files with a mapping from each file to the owning
        # package so we can do file ownership lookups.
        visited = set()
        self.files = dict()
        for package in self.packages.values():
            if package in visited:
                continue
            visited.add(package)

            for f in package.files:
                if f in self.directories:
                    continue

                if f in self.files:
                    print("Duplicate file",
                          repr(f),
                          ' current',
                          package,
                          ' already',
                          self.files[f],
                          file=sys.stderr)
                    if not f.startswith('/usr/share'):
                        assert (f not in self.files)
                self.files[f] = package

        # For each package, update the file list to track dependencies and symlinks correctly.
        for p in self.packages.values():
            p.update_filetypes(self.directories, self.symlinks)

        # Print out all the libraries and where they live as known to ldconfig
        result = subprocess.run(
            [
                '/usr/sbin/ldconfig', '-C',
                f'{self.partition}/etc/ld.so.cache', '-p'
            ],
            check=True,
            stdout=subprocess.PIPE,
        )

        self.ldconfig_cache = dict()
        for line in result.stdout.decode('utf-8').split('\n'):
            if line.startswith('\t'):
                split_line = re.split(' \\(libc6,(AArch64|x86-64)\\) => ',
                                      line.strip())
                self.ldconfig_cache[split_line[0]] = split_line[2]

        self.pkgcfg = dict()
        for pkgconfig in [
                '/usr/local/lib/aarch64-linux-gnu/pkgconfig',
                '/usr/local/lib/pkgconfig',
                '/usr/local/share/pkgconfig',
                '/usr/lib/aarch64-linux-gnu/pkgconfig',
                '/usr/lib/pkgconfig',
                '/usr/share/pkgconfig',
        ]:
            candidate_folder = f"{self.partition}/{pkgconfig}"
            if not os.path.exists(candidate_folder):
                continue

            for f in os.listdir(candidate_folder):
                full_filename = f"{candidate_folder}/{f}"
                if pathlib.Path(full_filename).is_dir():
                    continue
                if not f.endswith('.pc'):
                    continue

                package_name = f.removesuffix('.pc')

                with open(f"{candidate_folder}/{f}", "r") as file:
                    self.pkgcfg[package_name] = PkgConfig(
                        file.read(), self.files[f'{pkgconfig}/{f}'])

    def resolve_symlink(self, path: str) -> str:
        """ Implements symlink resolution using self.symlinks. """
        # Only need to support absolute links since we don't have a concept of cwd.

        # Implements the symlink algorithm in
        # https://android.googlesource.com/platform/bionic.git/+/android-4.0.1_r1/libc/bionic/realpath.c
        assert (path[0] == '/')

        left = path.split('/')[1:]

        if len(path) == 0:
            return path

        resolved = ['']

        while len(left) > 0:
            if left[0] == '.':
                left = left[1:]
            elif left[0] == '..':
                assert (len(resolved) >= 1)
                resolved = resolved[:-1]
                left = left[1:]
            else:
                resolved.append(left[0])
                merged = '/'.join(resolved)
                if merged in self.symlinks:
                    symlink = self.symlinks[merged]
                    # Absolute symlink, blow away the previously accumulated path
                    if symlink[0] == '/':
                        resolved = ['']
                        left = symlink[1:].split('/') + left[1:]
                    else:
                        # Relative symlink, replace the symlink name in the path with the newly found target.
                        resolved = resolved[:-1]
                        left = symlink.split('/') + left[1:]
                else:
                    left = left[1:]

        return '/'.join(resolved)

    def exists(self, path: str) -> bool:
        if path in self.files or path in self.symlinks or path in self.directories:
            return True
        return False

    def resolve_object(self,
                       obj: str,
                       requesting_obj: str | None = None,
                       rpath: str | None = None) -> str:
        if obj in self.ldconfig_cache:
            return self.resolve_symlink(self.ldconfig_cache[obj])
        elif requesting_obj is not None:
            to_search = ""
            if rpath is not None:
                to_search = os.path.join(rpath, obj)
            else:
                to_search = os.path.join(os.path.split(requesting_obj)[0], obj)

            if self.exists(to_search):
                return self.resolve_symlink(to_search)

        raise FileNotFoundError(obj)

    @functools.cache
    def object_dependencies(self, obj: str) -> (str, str | None):
        result = subprocess.run(
            ['objdump', '-p', f'{self.partition}/{obj}'],
            check=True,
            stdout=subprocess.PIPE,
        )

        # Part of the example output.  We only want NEEDED from the dynamic section.
        #
        #    RELRO off    0x0000000000128af0 vaddr 0x0000000000128af0 paddr 0x0000000000128af0 align 2**0
        #          filesz 0x0000000000003510 memsz 0x0000000000003510 flags r--
        #
        # Dynamic Section:
        #   NEEDED               libtinfo.so.6
        #   NEEDED               libc.so.6
        #   INIT                 0x000000000002f000
        #   FINI                 0x00000000000efb94

        deps = []
        rpath = None
        for line in result.stdout.decode('utf-8').split('\n'):
            if 'NEEDED' in line:
                deps.append(line.strip().split()[1])
            if 'RPATH' in line:
                rpath = line.strip().split()[1]

        return deps, rpath


def generate_build_file(partition):
    subprocess.run([
        "cp", "/usr/bin/qemu-aarch64-static",
        f"{partition}/usr/bin/qemu-aarch64-static"
    ])

    filesystem = Filesystem(partition)

    packages_to_eval = [
        filesystem.packages['glib-2.0-dev'],
        filesystem.packages['opencv-dev'],
        filesystem.packages['libc6-dev'],
        filesystem.packages['libstdc++'],
        filesystem.packages['libnpp-11-8-dev'],
        filesystem.packages['gstreamer1.0-dev'],
        filesystem.packages['orc-dev'],
        filesystem.packages['libgstrtp-1.0-0'],
        filesystem.packages['gstreamer1.0-plugins-bad-dev'],
    ]

    # Now, we want to figure out what the dependencies of each of the packages are.
    # Generate the dependency tree starting from an initial list of packages.
    # Then, figure out how to link the .so's in.

    # Recursively walk the tree using dijkstra's algorithm to generate targets
    # for each set of headers.
    print('Walking tree for', [p.name.name for p in packages_to_eval],
          file=sys.stderr)

    rules = []
    objs_to_eval = []

    # Set of packages already generated in case our graph hits a package
    # multiple times.
    packages_visited_set = set()
    while packages_to_eval:
        next_package = packages_to_eval.pop()
        if next_package in packages_visited_set:
            continue
        packages_visited_set.add(next_package)

        hdrs = next_package.headers()
        objects = next_package.objects()

        deps = []
        for p in next_package.resolved_depends(filesystem.packages):
            if p not in packages_visited_set:
                packages_to_eval.append(p)

            # These two form a circular dependency...
            # Don't add them since libc6 has no headers in it.
            if next_package.name.name == 'libgcc-s1' and p.name.name == 'libc6':
                continue

            if next_package not in p.resolved_depends(filesystem.packages):
                deps.append(p.name.name)
            else:
                print("Removing !!!!", p.name.name)

        if objects:
            objs_to_eval += objects

        hdrs.sort()
        deps.sort()
        hdrs = [f'        "{h[1:]}",\n' for h in hdrs]
        hdrs_files = ''.join(hdrs)
        deps_joined = ''.join([f'        ":{d}-headers",\n' for d in deps])

        filegroup_srcs = ''.join(
            [f'        "{f[1:]}",\n' for f in next_package.files] +
            [f'        ":{d}-filegroup",\n' for d in deps])

        rules.append(
            f'filegroup(\n    name = "{next_package.name.name}-filegroup",\n    srcs = [\n{filegroup_srcs}    ],\n)'
        )
        rules.append(
            f'cc_library(\n    name = "{next_package.name.name}-headers",\n    hdrs = [\n{hdrs_files}    ],\n    visibility = ["//visibility:public"],\n    deps = [\n{deps_joined}    ],\n)'
        )

    skip_set = set()
    # These two are linker scripts.  Since they are soooo deep in the
    # hierarchy, let's not stress parsing them correctly.
    skip_set.add('/usr/lib/libc.so')
    skip_set.add('/usr/lib/gcc/aarch64-linux-gnu/12/libgcc_s.so')

    obj_set = set()
    obj_set.update(skip_set)

    standard_includes = set()
    standard_includes.add('/usr/lib/')
    standard_includes.add('/usr/include')
    standard_includes.add('/usr/include/aarch64-linux-gnu')
    standard_includes.add('/usr/include/x86-64-linux-gnu')
    for pkg in filesystem.pkgcfg:
        try:
            contents = filesystem.pkgcfg[pkg]
            resolved_libraries = [
                filesystem.resolve_object('lib' + f.removeprefix('-l') + '.so')
                for f in contents.libs if f.startswith('-l')
            ]
            for lib in resolved_libraries:
                objs_to_eval.append(lib)

            includes = []
            for flag in contents.cflags:
                if flag.startswith('-I/') and flag.removeprefix(
                        '-I') not in standard_includes:
                    includes.append(flag.removeprefix('-I/'))
            pname = contents.package.name.name
            if contents.package not in packages_visited_set:
                if pname[-3:] != "dev":
                    break

                for provides in contents.package.provides:
                    provides_without_dev = provides.name[:-4]

                    worked = False
                    for package in packages_visited_set:
                        if provides_without_dev in [
                                provid.name for provid in package.provides
                        ]:
                            pname = package.name.name
                            worked = True
                            break
                    if worked:
                        break

                if not worked:
                    print("Got so far with", provides_without_dev, "on", pname,
                          "but thats not in the visited set.")
            rule_deps = ''.join(
                sorted([
                    '        ":' + l[1:].replace('/', '_') + '-lib",\n'
                    for l in resolved_libraries
                ] + [f'        ":{pname}-headers",\n'] +
                       [f'        ":{dep}",\n' for dep in contents.requires]))
            includes.sort()
            if len(includes) > 0:
                includes_string = '    includes = ["' + '", "'.join(
                    includes) + '"],\n'
            else:
                includes_string = ''
            rules.append(
                f'# pkgconf -> {pkg}\ncc_library(\n    name = "{pkg}",\n{includes_string}    visibility = ["//visibility:public"],\n    deps = [\n{rule_deps}    ],\n)'
            )
            # Look up which package this is from to include the headers
            # Depend on all the libraries
            # Parse -I -> includes

        except FileNotFoundError:
            print('Failed to instantiate package', repr(pkg), file=sys.stderr)
            pass
    while objs_to_eval:
        obj = objs_to_eval.pop()
        if obj in obj_set:
            continue
        obj_set.add(obj)

        deps, rpath = filesystem.object_dependencies(obj)
        resolved_deps = []
        for d in deps:
            resolved_obj = filesystem.resolve_object(d,
                                                     requesting_obj=obj,
                                                     rpath=rpath)
            resolved_deps.append(resolved_obj)
            if resolved_obj not in obj_set:
                objs_to_eval.append(resolved_obj)

        resolved_deps.sort()
        rule_name = obj[1:].replace('/', '_')
        rule_deps = ''.join([
            '        ":{}-lib",\n'.format(d[1:].replace('/', '_'))
            for d in resolved_deps if d not in skip_set
        ])
        rules.append(
            f'cc_library(\n    name = "{rule_name}-lib",\n    srcs = ["{obj[1:]}"],\n    deps = [\n{rule_deps}    ],\n)'
        )

    with open("orin_debian_rootfs.BUILD.template", "r") as file:
        template = jinja2.Template(file.read())

    substitutions = {
        "SYSROOT_SRCS": """glob(
        include = [
            "include/**",
            "lib/**",
            "lib64/**",
            "usr/include/**",
            "usr/local/**",
            "usr/lib/**",
            "usr/lib64/**",
        ],
        exclude = [
            "usr/share/**",
        ],
    )""",
        "RULES": '\n\n'.join(rules),
    }

    with open("../../compilers/orin_debian_rootfs.BUILD", "w") as file:
        file.write(template.render(substitutions))

    subprocess.run(['buildifier', "../../compilers/orin_debian_rootfs.BUILD"])
