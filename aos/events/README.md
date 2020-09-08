## How to run ping & pong

Running ping<->pong is a nice way to test that you can run some basic code and shows how messaging can work between two nodes

### Setup real-time niceties:
  1. Add the following lines to `/etc/security/limits.d/rt.conf`, replacing "USERNAME" with the username you're running under.  You'll probably need to do this as root, e.g., `sudo nano /etc/security/limits.d/rt.conf`
```
USERNAME - nice -20
USERNAME - rtprio 95
USERNAME - memlock unlimited
```

  2. Reboot your machine to pick up the changes

### Compile and run the code
  1. Compile the code for ping and pong, as well as aos_dump for looking at the messages.  We'll assume throughout that you're running from the top level directory of the 971 code.
  ```
  bazel build -c opt //aos/events:ping //aos/events:pong //aos:aos_dump
  ```

  2. In 2 separate windows, run the ping and pong commands using the `pingpong_config.json` config file:
    1. `bazel-bin/aos/events/ping --config bazel-bin/aos/events/pingpong_config.json`
    2. `bazel-bin/aos/events/pong --config bazel-bin/aos/events/pingpong_config.json`

  3. In a third window, explore the message stream using `aos_dump`.  Some things you can do:
    1. List the channels:
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json`
    2. Listen to a specific topic on a channel-- copy one of the channels listed in the first step and put it at the end of the aos_dump command (e.g., "/test aos.examples.Ping")
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json /test aos.examples.Ping`
    3. Listen to multiple topics on a channel (e.g., all the topics published on "/test")
       `bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json /test`


NOTE: To make life easier, you can alias `aos_dump` to include the path and the config file (you may want to specify the full path to the aos_dump executable so it can be run from anywhere)
```
alias aos_dump='bazel-bin/aos/aos_dump --config bazel-bin/aos/events/pingpong_config.json'
```

## Logging

In addition to running ping and pong, this is a good example to explore event logging.

  1. Start by compiling the code:
  ```
  bazel build -c opt //aos/events/logging:logger_main
  ```

  2. Create a folder for the log files, e.g., 
  ```
  mkdir /tmp/log_folder
  ```

  3. Run the command to log the data:
  ```
  bazel-bin/aos/events/logging/logger_main --config bazel-bin/aos/events/pingpong_config.json --logging_folder /tmp/log_folder/
  ```

A log file should be created in /tmp/log_folder, with a name something like `fbs_log-001.bfbs`.

If you're running ping and pong at the same time, you should be able to watch the log file grow in size as events are being logged.  For example, running `ls -lh /tmp/log_folder/*` will list the files and their sizes.  Doing this periodically (e.g., every 15-30 seconds) should show the new log file growing in size.

   4. View the contents of the log file (using `log_cat` functionality found in the `logging` sub-directory):
      1. Compile `log_cat`:
      ```
      bazel build -c opt //aos/events/logging:log_cat
      ```

      2. Run the binary pointed at the recorded event log (this assumes you're running from the base directory of the repository and that the file is named `fbs_log-001.bfbs`):
      ```
      bazel-bin/aos/events/logging/log_cat /tmp/log_folder/fbs_log-001.bfbs
      ```

## EXERCISE:
   1. Modify code in ping and pong to see a difference in their behavior, e.g., increment the counter by 2's instead of 1
