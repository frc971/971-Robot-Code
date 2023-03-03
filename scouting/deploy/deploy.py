import argparse
from pathlib import Path
import subprocess
import sys


def main(argv):
    """Installs the scouting application on the scouting server."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--deb",
        type=str,
        required=True,
        help="The .deb file to deploy.",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="scouting.frc971.org",
        help="The SSH host to install the scouting web server to.",
    )
    parser.add_argument(
        "--clear-db",
        action="store_true",
        help=("If set, will stop the existing scouting server and clear the "
              "database before deploying the new one."),
    )
    args = parser.parse_args(argv[1:])
    deb = Path(args.deb)

    if args.clear_db:
        print("Stopping the scouting app.")
        subprocess.run(
            f"ssh -tt {args.host} sudo systemctl stop scouting.service",
            shell=True,
            # In case the scouting app isn't installed, ignore the error here.
            check=False,
            stdin=sys.stdin)
        print("Clearing the database.")
        subprocess.run(
            " ".join([
                f"ssh -tt {args.host}",
                "\"sudo -u postgres psql",
                # Drop all tables in the same schema.
                "-c 'drop schema public cascade;'",
                # Create an empty schema for the scouting app to use.
                "-c 'create schema public;'",
                # List all tables as a sanity check.
                "-c '\dt'",
                "postgres\"",
            ]),
            shell=True,
            check=True,
            stdin=sys.stdin)

    # Copy the .deb to the scouting server, install it, and delete it again.
    subprocess.run(["rsync", "-L", args.deb, f"{args.host}:/tmp/{deb.name}"],
                   check=True,
                   stdin=sys.stdin)
    subprocess.run(f"ssh -tt {args.host} sudo dpkg -i /tmp/{deb.name}",
                   shell=True,
                   check=True,
                   stdin=sys.stdin)
    subprocess.run(f"ssh {args.host} rm -f /tmp/{deb.name}",
                   shell=True,
                   check=True,
                   stdin=sys.stdin)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
