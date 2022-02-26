Scouting web server
================================================================================

The `//scouting` target runs the webserver and hosts all the web pages. Run it
like so:

    $ bazel run //scouting

You can customize the port like so:

    $ bazel run //scouting -- --port 1234

See all options like this:

    $ bazel run //scouting -- --help
