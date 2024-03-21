We have some support for using [Foxglove Studio](https://studio.foxglove.dev)
for visualizing robot data.

# Accessing Foxglove

You have three main options for using foxglove studio:
1. Go to https://studio.foxglove.dev and use the most up-to-date studio. This
   is convenient; it won't work when you do not have Internet access, and
   has some limitations when it comes to accessing unsecured websockets.
2. Download the foxglove desktop application.
3. Run our local copy by running `bazel run //aos/analysis:local_foxglove`
   This will work offline, and serves foxglove at http://localhost:8000 by
   default.

# Log Visualization

If looking at data from a log, you will first need to convert one of our AOS
logs to MCAP so that it can be viewed in foxglove. In order to do so,
run `bazel run -c opt //aos/util:log_to_mcap -- /path/to/log --output_path /tmp/log.mcap`.
This will create an MCAP file at the specified path, which you can then open
in any of the various foxglove options.

Troubleshooting:
* If you get the error `Check failed: output_`: Check whether `/tmp/log.mcap` already exists under another owner. If so, use a different filename, e.g. `/tmp/<your_name>_log.mcap`

# Live Visualization

On the pis, we run a `foxglove_websocket` application by default. This exposes
a websocket on the 8765 port. How you connect to this varies depending on
what method you are using to create a foxglove instance.

If using https://studio.foxglove.dev, you cannot directly access
ws://10.9.71.10X:8765 due to security constraints. Instead, you will have to
port forward by doing something like `ssh -L 8765:localhost:8765 pi@10.9.71.101`
to expose the port locally, and then use the `ws://localhost:8765` websocket.

If using the local foxglove, you can just use the pi IP address directly.

I have not tried using the desktop Foxglove application for this.
