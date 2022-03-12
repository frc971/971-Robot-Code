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
    args = parser.parse_args(argv[1:])
    deb = Path(args.deb)

    # Copy the .deb to the scouting server, install it, and delete it again.
    subprocess.run(["rsync", "-L", args.deb, f"{args.host}:/tmp/{deb.name}"],
                   check=True, stdin=sys.stdin)
    subprocess.run(f"ssh -tt {args.host} sudo dpkg -i /tmp/{deb.name}",
                   shell=True, check=True, stdin=sys.stdin)
    subprocess.run(f"ssh {args.host} rm -f /tmp/{deb.name}",
                   shell=True, check=True, stdin=sys.stdin)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
