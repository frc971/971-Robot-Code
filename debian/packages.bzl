load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

# In order to use deb packages in the build you have to follow these steps:
#
# 1. Create a "download_packages" build step in //debian/BUILD. List the
#    packages you care about and exclude the ones you don't care about.
#    Invoke "bazel build" on the "download_packages" target you just created.
#    Save the "_files" dictionary it prints into a .bzl file in the //debian
#    folder. You will need to have the apt-rdepends package installed.
# 2. The "download_packages" steps prints the location of the deb packages
#    after it prints the "_files" dictionary. Take the deb packages from there
#    and upload them to http://www.frc971.org/Build-Dependencies/.
# 3. Add the newly uploaded deb packages as WORKSPACE entries using the
#    "generate_repositories_for_debs" helper. Load the "_files" dictionary
#    created earlier and the "generate_repositories_for_debs" helper and call
#    them together in the WORKSPACE file.
# 4. Add a "generate_deb_tarball" target to //debian/BUILD. Pass in the
#    "_files" dictionary created earlier by loading it from the .bzl file.
# 5. Invoke "bazel build" on the "generate_deb_tarball" target you just created
#    and upload the resulting tarball to http://www.frc971.org/Build-Dependencies.
# 6. Add a new "new_http_archive" entry to the WORKSPACE file for the tarball
#    you just uploaded.

# TODO(phil): Deal with armhf packages. Right now only works for amd64.

def download_packages(name, packages, excludes = [], force_includes = []):
    """Downloads a set of packages as well as their dependencies.

    You can also specify excludes in case some of the dependencies are meta
    packages.

    Use "bazel run" on these targets to download the packages and generate the
    list to use in a .bzl file. Once you have the packages on
    http://www.frc971.org/Build-Dependencies/ you can add them to a to
    combine_packages rule.
    """
    package_list = " ".join(packages)
    excludes_list = " ".join(["--exclude=%s" % e for e in excludes])
    force_includes = " ".join(["--force-include=%s" % i for i in force_includes])
    native.genrule(
        name = name,
        outs = ["%s_output.txt" % name],
        tags = [
            "local",
            "manual",
        ],
        tools = [
            "//debian:download_packages",
        ],
        # TODO(phil): Deal with stderr a bit better. It spews more stuff out than I
        # would like it to.
        cmd = "$(location //debian:download_packages) %s %s %s | tee $@ >&2" %
              (force_includes, excludes_list, package_list),
    )

def _convert_deb_to_target(deb):
    """Converts a debian package filename to a valid bazel target name."""
    target = deb.split("_")[0]
    target = target.replace("-", "_")
    target = target.replace(".", "_")
    target = target.replace(":", "_")
    target = target.replace("+", "x")
    return "deb_%s_repo" % target

def generate_repositories_for_debs(files, base_url = "http://www.frc971.org/Build-Dependencies"):
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

def generate_deb_tarball(name, files):
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
            )

    pkg_tar(
        name = name,
        extension = "tar.gz",
        deps = ["extracted_%s.tar" % dep for dep in deps],
    )
