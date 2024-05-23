import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml
from python.runfiles import runfiles

RUNFILES = runfiles.Create()

FAKE_NPM_BIN = RUNFILES.Rlocation(
    "org_frc971/tools/foxglove/creation_wrapper_npm")
BUILDOZER_BIN = RUNFILES.Rlocation(
    "com_github_bazelbuild_buildtools/buildozer/buildozer_/buildozer")

WORKSPACE_DIR = Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
WORKING_DIR = Path(os.environ["BUILD_WORKING_DIRECTORY"])


def create_npm_link(temp_dir: Path, env: dict[str, str]):
    """Set up the creation_wrapper_npm.py script as the "npm" binary."""
    bin_dir = temp_dir / "bin"
    bin_dir.mkdir()
    npm = bin_dir / "npm"
    npm.symlink_to(FAKE_NPM_BIN)
    env["PATH"] = f"{temp_dir / 'bin'}:{env['PATH']}"


def run_create_foxglove_extension(argv: list[str], name: str):
    """Runs the create-foxglove-extension binary.

    Args:
        argv: The list of command line arguments passed to this wrapper.
        name: The (directory) name of the new extension to be created.
    """
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = Path(temp_dir)
        env = os.environ.copy()
        create_npm_link(temp_dir, env)

        env["BAZEL_BINDIR"] = WORKING_DIR
        env.pop("RUNFILES_DIR", None)
        env.pop("RUNFILES_MANIFEST_FILE", None)

        subprocess.run(argv[1:], check=True, env=env, cwd=WORKING_DIR)
        # For some reason, the `foxglove-extension` binary doesn't set up the
        # ts-loader dependency. Do it manually here.
        subprocess.run(["npm", "install", "ts-loader@^9"],
                       check=True,
                       env=env,
                       cwd=WORKING_DIR / name)


def add_new_js_project(name: str):
    """Tell Bazel about the new project."""
    # The name of the Bazel package for the new extension.
    package_name = WORKING_DIR.relative_to(WORKSPACE_DIR) / name

    # Add the new "node_modules" directory to the ignore list.
    bazelignore_file = WORKSPACE_DIR / ".bazelignore"
    bazelignore = bazelignore_file.read_text()
    bazelignore_entry = str(package_name / "node_modules")
    if bazelignore_entry not in bazelignore.splitlines():
        bazelignore = bazelignore.rstrip("\n") + "\n"
        bazelignore_file.write_text(bazelignore + bazelignore_entry + "\n")

    # Add the new project to the workspace list. This ensures the lock file
    # gets updated properly.
    pnpm_workspace_file = WORKSPACE_DIR / "pnpm-workspace.yaml"
    pnpm_workspace = yaml.load(pnpm_workspace_file.read_text(),
                               Loader=yaml.CLoader)
    if str(package_name) not in pnpm_workspace["packages"]:
        pnpm_workspace["packages"].append(str(package_name))
        pnpm_workspace_file.write_text(yaml.dump(pnpm_workspace))

    # Add the new project to the workspace. This ensures that all of its
    # dependencies get downloaded by Bazel.
    subprocess.check_call([
        BUILDOZER_BIN,
        f"add data @//{package_name}:package.json",
        "WORKSPACE:npm",
    ],
                          cwd=WORKSPACE_DIR)

    # Regenerate the lock file with the new project's dependencies included.
    subprocess.check_call([
        "bazel",
        "run",
        "--",
        "@pnpm//:pnpm",
        "--dir",
        WORKSPACE_DIR,
        "install",
        "--lockfile-only",
    ],
                          cwd=WORKSPACE_DIR)


def main(argv):
    """Runs the main logic."""

    # Assume that the only argument the user passed in is the name of the
    # extension. We can probably do better here, but oh well.
    create_foxglove_extension_args = argv[2:]
    name = create_foxglove_extension_args[0]

    run_create_foxglove_extension(argv, name)
    add_new_js_project(name)

    # Generate a BUILD file.
    build_file_template = WORKSPACE_DIR / "tools/foxglove/BUILD.bazel.tmpl"
    build_file = WORKING_DIR / name / "BUILD.bazel"
    build_file.write_text(build_file_template.read_text())

    # Fix up the tsconfig.json. For some reason the inheritance for the `lib`
    # field doesn't work out of the box. We're using string manipulation since
    # we don't have a readily-available "JSON with comments" parser.
    tsconfig_file = WORKING_DIR / name / "tsconfig.json"
    tsconfig = tsconfig_file.read_text()
    tsconfig = tsconfig.replace('"lib": ["dom"]', '"lib": ["dom", "es2022"]')
    tsconfig_file.write_text(tsconfig)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
