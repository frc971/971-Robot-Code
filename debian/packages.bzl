load("@rules_pkg//:pkg.bzl", "pkg_tar")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

# In order to use deb packages in the build you have to follow these steps:
#
# 1. Create a "download_packages" build step in //debian/BUILD. List the
#    packages you care about and exclude the ones you don't care about.
#    Invoke "bazel run" on the "download_packages" target you just created.
#    Save the "_files" dictionary it prints into a .bzl file in the //debian
#    folder. You will need to have the apt-rdepends package installed.
#    If you want to get packages for a different architecture or distribution,
#    you can pass flags here to control those. See the --help for details.
# 2. The "download_packages" steps prints the location of the deb packages
#    after it prints the "_files" dictionary. Take the deb packages from there
#    and upload them to https://software.frc971.org/Build-Dependencies/.
# 3. Add the newly uploaded deb packages as WORKSPACE entries using the
#    "generate_repositories_for_debs" helper. Load the "_files" dictionary
#    created earlier and the "generate_repositories_for_debs" helper and call
#    them together in the WORKSPACE file.
# 4. Add a "generate_deb_tarball" target to //debian/BUILD. Pass in the
#    "_files" dictionary created earlier by loading it from the .bzl file.
# 5. Invoke "bazel build" on the "generate_deb_tarball" target you just created
#    and upload the resulting tarball to https://software.frc971.org/Build-Dependencies.
# 6. Add a new "new_http_archive" entry to the WORKSPACE file for the tarball
#    you just uploaded.

def download_packages(name, packages, excludes = [], force_includes = [], force_excludes = [], target_compatible_with = None):
    """Downloads a set of packages as well as their dependencies.

    You can also specify excludes in case some of the dependencies are meta
    packages.

    Use "bazel run" on these targets to download the packages and generate the
    list to use in a .bzl file. Once you have the packages on
    https://software.frc971.org/Build-Dependencies/ you can add them to a to
    combine_packages rule.

    force_includes lets you include packages that are excluded by default. The
    dependencies of these force-included packages are also force-included. To
    counter-act that, you can use "force_excludes". The force-excluded packages
    are excluded even if they're pulled in as a dependency from a
    "force_includes" package.
    """
    package_list = " ".join(packages)
    excludes_list = " ".join(["--exclude=%s" % e for e in excludes])
    force_includes = " ".join(["--force-include=%s" % i for i in force_includes])
    force_excludes = " ".join(["--force-exclude=%s" % e for e in force_excludes])
    native.genrule(
        name = name + "_gen",
        outs = ["%s.sh" % name],
        executable = True,
        cmd = """
cat > $@ <<'END'
#!/bin/bash

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "$${RUNFILES_DIR:-/dev/null}/$$f" 2>/dev/null || \\
  source "$$(grep -sm1 "^$$f " "$${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \\
  source "$$0.runfiles/$$f" 2>/dev/null || \\
  source "$$(grep -sm1 "^$$f " "$$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \\
  source "$$(grep -sm1 "^$$f " "$$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \\
  { echo>&2 "ERROR: cannot find $$f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---


exec "$$(rlocation org_frc971/debian/download_packages)" %s %s %s %s "$$@"
END""" % (force_includes, force_excludes, excludes_list, package_list),
        target_compatible_with = target_compatible_with,
    )
    native.sh_binary(
        name = name,
        srcs = ["%s.sh" % name],
        deps = [
            "@bazel_tools//tools/bash/runfiles",
        ],
        data = [
            "//debian:download_packages",
        ],
        target_compatible_with = target_compatible_with,
    )

def _convert_deb_to_target(deb):
    """Converts a debian package filename to a valid bazel target name."""
    target = deb
    target = target.replace("-", "_")
    target = target.replace(".", "_")
    target = target.replace(":", "_")
    target = target.replace("+", "x")
    target = target.replace("~", "_")
    return "deb_%s_repo" % target

def generate_repositories_for_debs(files, base_url = "https://software.frc971.org/Build-Dependencies"):
    """A WORKSPACE helper to add all the deb packages in the dictionary as a repo.

    The files dictionary must be one generated with the "download_packages"
    helper above.
    """
    for f in files.keys():
        name = _convert_deb_to_target(f)
        if name not in native.existing_rules():
            http_file(
                name = name,
                urls = [base_url + "/" + f],
                sha256 = files[f],
                downloaded_file_path = f,
            )

def generate_deb_tarball(name, files, target_compatible_with = None):
    """Takes all debs in the dictionary and generates one tarball from them.

    This can then be uploaded and used as another WORKSPACE entry.
    """
    deps = []
    for f in files.keys():
        dep = _convert_deb_to_target(f)
        deps.append(dep)
        if ("generate_%s_tarball" % dep) not in native.existing_rules():
            native.genrule(
                name = "generate_%s_tarball" % dep,
                srcs = ["@%s//file" % dep],
                outs = ["extracted_%s.tar" % dep],
                cmd = "dpkg-deb --fsys-tarfile $(SRCS) > $@",
                target_compatible_with = target_compatible_with,
            )

    pkg_tar(
        name = name,
        extension = "tar.gz",
        deps = ["extracted_%s.tar" % dep for dep in deps],
        target_compatible_with = target_compatible_with,
    )
