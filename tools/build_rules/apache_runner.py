"""Starts up Apache to provide HTTPS + LDAP for another web server.

This script is used by the apache_wrapper() rule as the main entrypoint for its
"executable". This script sets up a minimal Apache environment in a directory
in /tmp.

Both Apache and the wrapped server binary are started by this script. The
wrapped server should bind to the port specified by the APACHE_WRAPPED_PORT
environment variable.

See the documentation for apache_wrapper() for more information.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

import jinja2

DUMMY_CERT_ANSWERS = """\
US
California
Mountain View
FRC971
Software
frc971.org
dummy@frc971.org
"""

def main(argv):
  parser = argparse.ArgumentParser()
  parser.add_argument("--binary", type=str, required=True)
  parser.add_argument("--https_port", type=int, default=7000)
  parser.add_argument("--wrapped_port", type=int, default=7500)
  parser.add_argument(
    "--ldap_info",
    type=str,
    help="JSON file containing 'ldap_bind_dn', 'ldap_url', and 'ldap_password' entries.",
    default="",
  )
  args = parser.parse_args(argv[1:])

  if not args.ldap_info:
    args.ldap_info = os.path.join(os.environ["BUILD_WORKSPACE_DIRECTORY"], "ldap.json")

  with open("tools/build_rules/apache_template.conf", "r") as file:
    template = jinja2.Template(file.read())

  with open(args.ldap_info, "r") as file:
    substitutions = json.load(file)

  for key in ("ldap_bind_dn", "ldap_url", "ldap_password"):
    if key not in substitutions:
      raise KeyError(f"The ldap_info JSON file must contain key '{key}'.")

  substitutions.update({
    "https_port": args.https_port,
    "wrapped_port": args.wrapped_port,
  })

  config_text = template.render(substitutions)

  with tempfile.TemporaryDirectory() as temp_dir:
    temp_dir = Path(temp_dir)
    with open(temp_dir / "apache2.conf", "w") as file:
      file.write(config_text)

    # Create a directory for error logs and such.
    logs_dir = temp_dir / "logs"
    os.mkdir(logs_dir)

    print("-" * 60)
    print(f"Logs are in {logs_dir}/")
    print("-" * 60)

    # Make modules available.
    modules_path = Path("external/apache2/usr/lib/apache2/modules")
    os.symlink(modules_path.resolve(), temp_dir / "modules")

    # Generate a testing cert.
    subprocess.run([
        "openssl",
        "req",
        "-x509",
        "-nodes",
        "-days=365",
        "-newkey=rsa:2048",
        "-keyout=" + str(temp_dir / "apache-selfsigned.key"),
        "-out="  + str(temp_dir / "apache-selfsigned.crt"),
      ],
      check=True,
      input=DUMMY_CERT_ANSWERS,
      text=True,
    )

    # Start the wrapped binary in the background.
    # Tell it via the environment what port to listen on.
    env = os.environ.copy()
    env["APACHE_WRAPPED_PORT"] = str(args.wrapped_port)
    wrapped_binary = subprocess.Popen([args.binary], env=env)

    # Start the apache server.
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "external/apache2/usr/lib/x86_64-linux-gnu"
    try:
      subprocess.run(
        ["external/apache2/usr/sbin/apache2", "-X", "-d", str(temp_dir)],
        check=True,
        env=env,
      )
    finally:
      wrapped_binary.send_signal(signal.SIGINT)
      wrapped_binary.wait()

if __name__ == "__main__":
  sys.exit(main(sys.argv))
