#!/usr/bin/python3

from __future__ import annotations
import apt_pkg
import collections
import contextlib
import datetime
import functools
import jinja2
import os
import pathlib
import re
import shlex
import shutil
import subprocess

# Name of debian image to be created/modified
IMAGE = "arm64_bookworm_debian_yocto.img"

# Path to yocto build for the orin (using meta-frc971)
YOCTO = "/home/austin/local/jetpack/robot-yocto/build"

REQUIRED_DEPS = ["debootstrap", "u-boot-tools", "xfsprogs"]

apt_pkg.init_system()


@contextlib.contextmanager
def scoped_loopback(image):
    """Mounts an image as a loop back device."""
    result = subprocess.run(["sudo", "losetup", "--show", "-f", image],
                            check=True,
                            stdout=subprocess.PIPE)
    device = result.stdout.decode('utf-8').strip()
    print("Mounted", image, "to", repr(device))
    try:
        yield device
    finally:
        subprocess.run(["sudo", "losetup", "-d", device], check=True)


@contextlib.contextmanager
def scoped_mount(image):
    """Mounts an image as a partition."""
    partition = f"{image}.partition"
    try:
        os.mkdir(partition)
    except FileExistsError:
        pass

    result = subprocess.run(["sudo", "mount", "-o", "loop", image, partition],
                            check=True)

    try:
        yield partition
    finally:
        subprocess.run(
            ["sudo", "rm", f"{partition}/usr/bin/qemu-aarch64-static"])
        subprocess.run(["sudo", "umount", partition], check=True)


def check_required_deps(deps):
    """Checks if the provided list of dependencies is installed."""
    missing_deps = []
    for dep in deps:
        result = subprocess.run(["dpkg-query", "-W", "-f='${Status}'", dep],
                                check=True,
                                stdout=subprocess.PIPE)

        if "install ok installed" not in result.stdout.decode('utf-8'):
            missing_deps.append(dep)

    if len(missing_deps) > 0:
        print("Missing dependencies, please install:")
        print("sudo apt-get install", " ".join(missing_deps))
        exit()


def make_image(image):
    """Makes an image and creates an xfs filesystem on it."""
    print("Creating image ", f"{image}")
    result = subprocess.run([
        "dd", "if=/dev/zero", f"of={image}", "bs=1", "count=0",
        "seek=8589934592"
    ],
                            check=True)

    with scoped_loopback(image) as loopback:
        subprocess.run([
            "sudo", "mkfs.xfs", "-d", "su=128k", "-d", "sw=1", "-L", "rootfs",
            loopback
        ],
                       check=True)


def target_unescaped(cmd):
    """Runs a command as root with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=0:0", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c", cmd
    ],
                   check=True)


def target(cmd):
    """Runs a command as root with escaping."""
    target_unescaped(shlex.join([shlex.quote(c) for c in cmd]))


def pi_target_unescaped(cmd):
    """Runs a command as pi with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=pi:pi", "--groups=pi", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c", cmd
    ],
                   check=True)


def pi_target(cmd):
    """Runs a command as pi with escaping."""
    pi_target_unescaped(shlex.join([shlex.quote(c) for c in cmd]))


def copyfile(owner, permissions, file):
    """Copies a file from contents/{file} with the provided owner and permissions."""
    print("copyfile", owner, permissions, file)
    subprocess.run(["sudo", "cp", f"contents/{file}", f"{PARTITION}/{file}"],
                   check=True)
    subprocess.run(["sudo", "chmod", permissions, f"{PARTITION}/{file}"],
                   check=True)
    target(["chown", owner, f"/{file}"])


def target_mkdir(owner_group, permissions, folder):
    """Creates a directory recursively with the provided permissions and ownership."""
    print("target_mkdir", owner_group, permissions, folder)
    owner, group = owner_group.split(':')
    target(
        ["install", "-d", "-m", permissions, "-o", owner, "-g", group, folder])


def list_packages():
    """Lists all installed packages.

    Returns:
      A dictionary with keys as packages, and values as versions.
    """
    result = subprocess.run([
        "sudo", "chroot", "--userspec=0:0", f"{PARTITION}",
        "qemu-aarch64-static", "/bin/bash", "-c",
        "dpkg-query -W -f='${Package} ${Version}\n'"
    ],
                            check=True,
                            stdout=subprocess.PIPE)

    device = result.stdout.decode('utf-8').strip()

    r = {}
    for line in result.stdout.decode('utf-8').strip().split('\n'):
        package, version = line.split(' ')
        r[package] = version

    return r


def list_yocto_packages():
    """Lists all packages in the Yocto folder.

    Returns:
      list of Package classes.
    """
    Package = collections.namedtuple(
        'Package', ['path', 'name', 'version', 'architecture'])
    result = []
    pathlist = pathlib.Path(f"{YOCTO}/tmp/deploy/deb").glob('**/*.deb')
    for path in pathlist:
        # Strip off the path, .deb, and split on _ to parse the package info.
        s = os.path.basename(str(path))[:-4].split('_')
        result.append(Package(str(path), s[0], s[1], s[2]))

    return result


def install_packages(new_packages, existing_packages):
    """Installs the provided yocto packages, if they are new."""
    # To install the yocto packages, first copy them into a folder in /tmp, then install them, then clean the folder up.
    target(["mkdir", "-p", "/tmp/yocto_packages"])
    try:
        to_install = []
        for package in new_packages:
            if package.name in existing_packages and existing_packages[
                    package.name] == package.version:
                print('Skipping', package)
                continue

            subprocess.run([
                "sudo", "cp", package.path,
                f"{PARTITION}/tmp/yocto_packages/{os.path.basename(package.path)}"
            ],
                           check=True)
            to_install.append(package)

        if len(to_install) > 0:
            target(["dpkg", "-i"] + [
                f"/tmp/yocto_packages/{os.path.basename(package.path)}"
                for package in to_install
            ])

    finally:
        target(["rm", "-rf", "/tmp/yocto_packages"])


def install_virtual_packages(virtual_packages):
    """Builds and installs the provided virtual packages."""
    try:
        target(["mkdir", "-p", "/tmp/yocto_packages"])
        for virtual_package in virtual_packages:
            subprocess.run(
                ["dpkg-deb", "--build", f"virtual_packages/{virtual_package}"],
                check=True)
            subprocess.run([
                "sudo", "cp", f"virtual_packages/{virtual_package}.deb",
                f"{PARTITION}/tmp/yocto_packages/{virtual_package}.deb"
            ],
                           check=True)

        target(["dpkg", "-i"] + [
            f"/tmp/yocto_packages/{package}.deb"
            for package in virtual_packages
        ])

        for virtual_package in virtual_packages:
            subprocess.run(["rm", f"virtual_packages/{virtual_package}.deb"],
                           check=True)

    finally:
        target(["rm", "-rf", "/tmp/yocto_packages"])


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

    def matches(self, other: NameVersion) -> bool:
        """If self meets the requirements defined by other."""
        if other.name != self.name:
            return False

        if other.operator is None:
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

    def resolved_depends(self, packages: dict[Package]) -> list[Package]:
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
                else:
                    found_so = False

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
            elif line.startswith('URL:'):
                pass
            elif line.startswith('Cflags.private:'):
                pass
            elif line.startswith('Requires:'):
                pass
            elif line.startswith('Requires.private:'):
                pass
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
            "qemu-aarch64-static", "/bin/bash", "-c",
            "dpkg-query -W -f='Version: ${Version}\nPackage: ${Package}\nProvides: ${Provides}\nDepends: ${Depends}\n${db-fsys:Files}--\n'"
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
                print("Failed to read", file)
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
                    print("Duplicate file", repr(f), ' current', package,
                          ' already', self.files[f])
                    if not f.startswith('/usr/share'):
                        assert (f not in self.files)
                self.files[f] = package

        # For each package, update the file list to track dependencies and symlinks correctly.
        for p in self.packages.values():
            p.update_filetypes(self.directories, self.symlinks)

        # Print out all the libraries and where they live as known to ldconfig
        result = subprocess.run(
            ['ldconfig', '-C', f'{self.partition}/etc/ld.so.cache', '-p'],
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
                       requesting_obj: str | None = None) -> str:
        if obj in self.ldconfig_cache:
            return self.resolve_symlink(self.ldconfig_cache[obj])
        elif requesting_obj is not None:
            to_search = os.path.join(os.path.split(requesting_obj)[0], obj)
            if self.exists(to_search):
                return self.resolve_symlink(to_search)

        raise FileNotFoundError(obj)

    @functools.cache
    def object_dependencies(self, obj: str) -> str:
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
        for line in result.stdout.decode('utf-8').split('\n'):
            if 'NEEDED' in line:
                deps.append(line.strip().split()[1])

        return deps


def generate_build_file(partition):
    filesystem = Filesystem(partition)

    packages_to_eval = [
        filesystem.packages['libopencv-dev'],
        filesystem.packages['libc6-dev'],
        filesystem.packages['libstdc++-12-dev'],
        filesystem.packages['libnpp-11-8-dev'],
    ]

    # Recursively walk the tree using dijkstra's algorithm to generate targets
    # for each set of headers.
    print('Walking tree for', [p.name.name for p in packages_to_eval])

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

            deps.append(p.name.name)

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
    skip_set.add('/usr/lib/aarch64-linux-gnu/libc.so')
    skip_set.add('/usr/lib/gcc/aarch64-linux-gnu/12/libgcc_s.so')

    obj_set = set()
    obj_set.update(skip_set)

    while objs_to_eval:
        obj = objs_to_eval.pop()
        if obj in obj_set:
            continue
        obj_set.add(obj)

        deps = filesystem.object_dependencies(obj)
        resolved_deps = []
        for d in deps:
            resolved_obj = filesystem.resolve_object(d, requesting_obj=obj)
            resolved_deps.append(resolved_obj)
            if resolved_obj not in obj_set:
                objs_to_eval.append(resolved_obj)

        resolved_deps.sort()
        rule_name = obj[1:].replace('/', '_')
        rule_deps = ''.join([
            '        ":%s",\n'.format(d[1:].replace('/', '_'))
            for d in resolved_deps if d not in skip_set
        ])
        rules.append(
            f'cc_library(\n    name = "{rule_name}",\n    srcs = ["{obj[1:]}"],\n    deps = [\n{rule_deps}    ],\n)'
        )

    standard_includes = set()
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

            if contents.package not in packages_visited_set:
                continue

            includes = []
            for flag in contents.cflags:
                if flag.startswith('-I/') and flag.removeprefix(
                        '-I') not in standard_includes:
                    includes.append(flag.removeprefix('-I/'))

            rule_deps = ''.join(
                sorted([
                    '        ":' + l[1:].replace('/', '_') + '",\n'
                    for l in resolved_libraries
                ] + [f'        ":{contents.package.name.name}-headers",\n']))
            includes.sort()
            if len(includes) > 0:
                includes_string = '    includes = ["' + '", "'.join(
                    includes) + '"],\n'
            else:
                includes_string = ''
            rules.append(
                f'cc_library(\n    name = "{pkg}",\n{includes_string}    visibility = ["//visibility:public"],\n    deps = [\n{rule_deps}    ],\n)'
            )
            # Look up which package this is from to include the headers
            # Depend on all the libraries
            # Parse -I -> includes
        except FileNotFoundError:
            print('Failed to instantiate package', repr(pkg))
            pass

    # Now, we want to figure out what the dependencies of opencv-dev are.
    # Generate the dependency tree starting from an initial list of packages.

    # Then, figure out how to link the .so's in.  Sometimes, multiple libraries exist per .deb, one target for all?

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
            "usr/local/cuda-11.8/include/thrust/**",
            "usr/local/cuda-11.8/include/nv/**",
            "usr/local/cuda-11.8/include/cuda/**",
            "usr/local/cuda-11.8/include/cub/**",
        ],
    )""",
        "RULES": '\n\n'.join(rules),
    }

    with open("../../compilers/orin_debian_rootfs.BUILD", "w") as file:
        file.write(template.render(substitutions))


def do_package(partition):
    tarball = datetime.date.today().strftime(
        f"{os.getcwd()}/%Y-%m-%d-bookworm-arm64-nvidia-rootfs.tar")
    print(tarball)

    subprocess.run([
        "sudo",
        "tar",
        "--sort=name",
        "--mtime=0",
        "--owner=0",
        "--group=0",
        "--numeric-owner",
        "--exclude=./usr/share/ca-certificates",
        "--exclude=./home",
        "--exclude=./root",
        "--exclude=./usr/src",
        "--exclude=./usr/lib/mesa-diverted",
        "--exclude=./usr/bin/X11",
        "--exclude=./usr/lib/systemd/system/system-systemd*cryptsetup.slice",
        "--exclude=./dev",
        "--exclude=./usr/local/cuda-11.8/bin/fatbinary",
        "--exclude=./usr/local/cuda-11.8/bin/ptxas",
        "--exclude=./usr/local/cuda-11.8/include/thrust",
        "--exclude=./usr/local/cuda-11.8/include/nv",
        "--exclude=./usr/local/cuda-11.8/include/cuda",
        "--exclude=./usr/local/cuda-11.8/include/cub",
        "--exclude=./usr/share",
        "-cf",
        tarball,
        ".",
    ],
                   cwd=partition,
                   check=True)

    # Pack ptxas and fatbinary into the spots that clang expect them to make compiling easy.
    nvidia_cuda_toolkit_path = 'nvidia-cuda-toolkit'
    if not os.path.exists(nvidia_cuda_toolkit_path):
        os.mkdir(nvidia_cuda_toolkit_path)

        subprocess.run(['apt-get', 'download', 'nvidia-cuda-toolkit'],
                       cwd=nvidia_cuda_toolkit_path,
                       check=True)

        subprocess.run(
            ['dpkg', '-x',
             os.listdir(nvidia_cuda_toolkit_path)[0], '.'],
            cwd=nvidia_cuda_toolkit_path,
            check=True)

    subprocess.run([
        "sudo", "tar", "--sort=name", "--mtime=0", "--owner=0", "--group=0",
        "--numeric-owner",
        '--transform=s|usr/bin/ptxas|usr/local/cuda-11.8/bin/ptxas|',
        '--transform=s|usr/bin/fatbinary|usr/local/cuda-11.8/bin/aarch64-unknown-linux-gnu-fatbinary|',
        "--append", "-f", tarball, "usr/bin/fatbinary", "usr/bin/ptxas"
    ],
                   cwd=nvidia_cuda_toolkit_path,
                   check=True)

    subprocess.run(["sha256sum", tarball], check=True)


def main():
    check_required_deps(REQUIRED_DEPS)

    new_image = not os.path.exists(IMAGE)
    if new_image:
        make_image(IMAGE)

    with scoped_mount(IMAGE) as partition:
        if new_image:
            subprocess.run([
                "sudo", "debootstrap", "--arch=arm64", "--no-check-gpg",
                "--foreign", "bookworm", partition,
                "http://deb.debian.org/debian/"
            ],
                           check=True)

        subprocess.run([
            "sudo", "cp", "/usr/bin/qemu-aarch64-static",
            f"{partition}/usr/bin/"
        ],
                       check=True)

        global PARTITION
        PARTITION = partition

        if new_image:
            target(["/debootstrap/debootstrap", "--second-stage"])

            target([
                "useradd", "-m", "-p",
                '$y$j9T$85lzhdky63CTj.two7Zj20$pVY53UR0VebErMlm8peyrEjmxeiRw/rfXfx..9.xet1',
                '-s', '/bin/bash', 'pi'
            ])
            target(["addgroup", "debug"])
            target(["addgroup", "crypto"])
            target(["addgroup", "trusty"])

        if not os.path.exists(
                f"{partition}/etc/apt/sources.list.d/bullseye-backports.list"):
            copyfile("root:root", "644",
                     "etc/apt/sources.list.d/bullseye-backports.list")
            target(["apt-get", "update"])

        target([
            "apt-get", "-y", "install", "gnupg", "wget", "systemd",
            "systemd-resolved", "locales"
        ])

        target(["localedef", "-i", "en_US", "-f", "UTF-8", "en_US.UTF-8"])

        target_mkdir("root:root", "755", "run/systemd")
        target_mkdir("systemd-resolve:systemd-resolve", "755",
                     "run/systemd/resolve")
        copyfile("systemd-resolve:systemd-resolve", "644",
                 "run/systemd/resolve/stub-resolv.conf")
        target(["systemctl", "enable", "systemd-resolved"])

        target([
            "apt-get", "-y", "install", "bpfcc-tools", "sudo",
            "openssh-server", "python3", "bash-completion", "git", "v4l-utils",
            "cpufrequtils", "pmount", "rsync", "vim-nox", "chrony",
            "libopencv-calib3d406", "libopencv-contrib406",
            "libopencv-core406", "libopencv-features2d406",
            "libopencv-flann406", "libopencv-highgui406",
            "libopencv-imgcodecs406", "libopencv-imgproc406",
            "libopencv-ml406", "libopencv-objdetect406", "libopencv-photo406",
            "libopencv-shape406", "libopencv-stitching406",
            "libopencv-superres406", "libopencv-video406",
            "libopencv-videoio406", "libopencv-videostab406",
            "libopencv-viz406", "libopencv-dev", "libnice10", "pmount",
            "libnice-dev", "feh", "libgstreamer1.0-0",
            "libgstreamer-plugins-base1.0-0", "libgstreamer-plugins-bad1.0-0",
            "gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good",
            "gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-ugly",
            "gstreamer1.0-nice", "usbutils", "locales", "trace-cmd", "clinfo",
            "jq", "strace", "sysstat", "lm-sensors", "can-utils", "xfsprogs",
            "gstreamer1.0-tools", "bridge-utils", "net-tools", "apt-file",
            "parted", "xxd", "libv4l-dev", "file", "pkexec", "libxkbfile1"
        ])
        target(["apt-get", "clean"])

        target(["usermod", "-a", "-G", "sudo", "pi"])
        target(["usermod", "-a", "-G", "video", "pi"])
        target(["usermod", "-a", "-G", "systemd-journal", "pi"])
        target(["usermod", "-a", "-G", "dialout", "pi"])

        virtual_packages = [
            'libglib-2.0-0', 'libglvnd', 'libgtk-3-0', 'libxcb-glx', 'wayland'
        ]

        install_virtual_packages(virtual_packages)

        yocto_package_names = [
            'tegra-argus-daemon',
            'tegra-firmware',
            'tegra-firmware-tegra234',
            'tegra-firmware-vic',
            'tegra-firmware-xusb',
            'tegra-libraries-argus-daemon-base',
            'tegra-libraries-camera',
            'tegra-libraries-core',
            'tegra-libraries-cuda',
            'tegra-libraries-eglcore',
            'tegra-libraries-glescore',
            'tegra-libraries-glxcore',
            'tegra-libraries-multimedia',
            'tegra-libraries-multimedia-utils',
            'tegra-libraries-multimedia-v4l',
            'tegra-libraries-nvsci',
            'tegra-libraries-vulkan',
            'tegra-nvphs',
            'tegra-nvphs-base',
            'libnvidia-egl-wayland1',
            'tegra-mmapi',
            'tegra-mmapi-dev',
            'cuda-cudart-11-8',
            'cuda-cudart-11-8-dev',
            'cuda-cudart-11-8-stubs',
            'libcurand-11-8',
            'libcurand-11-8-dev',
            'libcurand-11-8-stubs',
            'cuda-nvcc-11-8',
            'tegra-cmake-overrides',
            'cuda-target-environment',
            'libnpp-11-8',
            'libnpp-11-8-stubs',
            'libnpp-11-8-dev',
            'cuda-cccl-11-8',
            'cuda-nvcc-11-8',
            'cuda-nvcc-headers-11-8',
            'nsight-systems-cli',
            'nsight-systems-cli-qdstrmimporter',
        ]
        yocto_packages = list_yocto_packages()
        packages = list_packages()

        install_packages([
            package for package in yocto_packages
            if package.name in yocto_package_names
        ], packages)

        # Now, install the kernel and modules after all the normal packages are in.
        yocto_packages_to_install = [
            package for package in yocto_packages
            if (package.name.startswith('kernel-module-') or package.name.
                startswith('kernel-5.10') or package.name == 'kernel-modules')
        ]

        packages_to_remove = []

        # Remove kernel-module-* packages + kernel- package.
        for key in packages:
            if key.startswith('kernel-module') or key.startswith(
                    'kernel-5.10'):
                already_installed = False
                for index, yocto_package in enumerate(
                        yocto_packages_to_install):
                    if key == yocto_package.name and packages[
                            key] == yocto_package.version:
                        already_installed = True
                        del yocto_packages_to_install[index]
                        break
                if not already_installed:
                    packages_to_remove.append(key)

        print("Removing", packages_to_remove)
        if len(packages_to_remove) > 0:
            target(['dpkg', '--purge'] + packages_to_remove)
        print("Installing",
              [package.name for package in yocto_packages_to_install])

        install_packages(yocto_packages_to_install, packages)

        target(["systemctl", "enable", "nvargus-daemon.service"])

        copyfile("root:root", "644", "etc/sysctl.d/sctp.conf")
        copyfile("root:root", "644", "etc/systemd/logind.conf")
        copyfile("root:root", "555",
                 "etc/bash_completion.d/aos_dump_autocomplete")
        copyfile("root:root", "644", "etc/security/limits.d/rt.conf")
        copyfile("root:root", "644", "etc/systemd/system/usb-mount@.service")
        copyfile("root:root", "644", "etc/chrony/chrony.conf")
        target_mkdir("root:root", "700", "root/bin")
        target_mkdir("pi:pi", "755", "home/pi/.ssh")
        copyfile("pi:pi", "600", "home/pi/.ssh/authorized_keys")
        target_mkdir("root:root", "700", "root/bin")
        copyfile("root:root", "644", "etc/systemd/system/grow-rootfs.service")
        copyfile("root:root", "500", "root/bin/change_hostname.sh")
        copyfile("root:root", "700", "root/trace.sh")
        copyfile("root:root", "440", "etc/sudoers")
        copyfile("root:root", "644", "etc/fstab")
        copyfile("root:root", "644",
                 "var/nvidia/nvcam/settings/camera_overrides.isp")
        copyfile("root.root", "644", "/etc/ld.so.conf.d/yocto.conf")

        target_mkdir("root:root", "755", "etc/systemd/network")
        copyfile("root:root", "644", "etc/systemd/network/eth0.network")
        copyfile("root:root", "644", "etc/systemd/network/80-can.network")
        copyfile("root:root", "644", "etc/udev/rules.d/nvidia.rules")
        target(["/root/bin/change_hostname.sh", "pi-971-1"])

        target(["systemctl", "enable", "systemd-networkd"])
        target(["systemctl", "enable", "grow-rootfs"])

        target(["apt-file", "update"])

        target(["ldconfig"])

        if not os.path.exists(f"{partition}/home/pi/.dotfiles"):
            pi_target_unescaped(
                "cd /home/pi/ && git clone --separate-git-dir=/home/pi/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && rsync --recursive --verbose --exclude .git tmpdotfiles/ /home/pi/ && rm -r tmpdotfiles && git --git-dir=/home/pi/.dotfiles/ --work-tree=/home/pi/ config --local status.showUntrackedFiles no"
            )
            pi_target(["vim", "-c", "\":qa!\""])

            target_unescaped(
                "cd /root/ && git clone --separate-git-dir=/root/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles && rsync --recursive --verbose --exclude .git tmpdotfiles/ /root/ && rm -r tmpdotfiles && git --git-dir=/root/.dotfiles/ --work-tree=/root/ config --local status.showUntrackedFiles no"
            )
            target(["vim", "-c", "\":qa!\""])

        generate_build_file(partition)

        do_package(partition)


if __name__ == '__main__':
    main()
