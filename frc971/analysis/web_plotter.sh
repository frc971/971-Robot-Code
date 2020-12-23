# This script provides basic plotting of a logfile.
# Basic usage:
# $ bazel run -c opt //frc971/analysis:web_plotter -- --node node_name /path/to/logfile
./aos/network/log_web_proxy_main --data_dir=frc971/analysis $@
