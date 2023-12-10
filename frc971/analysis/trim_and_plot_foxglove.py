import http.server as SimpleHTTPServer
from RangeHTTPServer import RangeRequestHandler
import argparse
import os
import shutil
import subprocess
from tempfile import mkdtemp

parser = argparse.ArgumentParser(
    description="""Trims & generates MCAP file from log.

Serves foxglove locally, and prints out a URL by which the log can be accessed.

By default, will trim a log to the time period during which the robot was
enabled. Skips this stip if --skip_trim is passed.""")
parser.add_argument('--port',
                    action='store',
                    default=8000,
                    type=int,
                    help='Specify port on which to serve foxglove.')
parser.add_argument('--skip_trim',
                    action='store_true',
                    default=False,
                    help='If set, do not trim the logfile..')
parser.add_argument('log',
                    action='store',
                    default=None,
                    type=str,
                    nargs='+',
                    help='Log(s) to plot.')
args = parser.parse_args()

tmpdir = mkdtemp(prefix="foxglove_")
shutil.copytree("external/foxglove_studio", tmpdir, dirs_exist_ok=True)

trimmed_aos_log = args.log if args.skip_trim else [tmpdir + "/trimmed/"]
output_mcap = tmpdir + "/log.mcap"

if not args.skip_trim:
    subprocess.run(["frc971/analysis/trim_log_to_enabled", "--output_folder"] +
                   trimmed_aos_log + args.log).check_returncode()
subprocess.run(["aos/util/log_to_mcap", "--output_path", output_mcap] +
               trimmed_aos_log).check_returncode()

mcap_url = f"http://localhost:{args.port}/log.mcap"
url_parameters = f"?ds=remote-file&ds.url={mcap_url}"
print(f"Serving files from {tmpdir}")
print(f"Local URL: http://localhost:{args.port}/{url_parameters}")
print(f"Live Website URL: https://studio.foxglove.dev/{url_parameters}")
os.chdir(tmpdir)
SimpleHTTPServer.test(HandlerClass=RangeRequestHandler, port=args.port)
