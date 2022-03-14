Scouting App
================================================================================
There are 2 big parts to the scouting app:
- The database
- The webserver

If you want to run the scouting app, you will need to run both of the above
mentioned parts.


Running the database
--------------------------------------------------------------------------------
Run the database like so:

    $ bazel run //scouting/db/testdb_server -- --port 2345

Choose a port that no one else is using. Anything above 1024 should be fine.

Wait until the database has fully started up before starting the webserver.


Running the webserver
--------------------------------------------------------------------------------
The `//scouting` target runs the webserver and hosts all the web pages. Run it
like so:

    $ bazel run //scouting -- --testdb_port 2345 --port 1234

The `--testdb_port` value must match the port you selected when running the
database.

The `--port` value must be one that no one else is using. Anything above 1024
should be fine.


Viewing the scouting app on your computer
--------------------------------------------------------------------------------
When you run the webserver on the build server, you cannot access the
scouting app without a port forward. You can use a separate terminal for this.

    $ ssh -L 1234:localhost:1234 <build_server>

where `1234` is the port that your instance of the webserver is using.
`<build_server>` is the SSH Host entry in your `~/.ssh/config` file for the
build server.

You can then visit <http://localhost:1234/> to look at the webserver.


Running the webserver with HTTPS
--------------------------------------------------------------------------------
You can test HTTPS and LDAP interation by running the webserver in a slightly
different way.

    $ bazel run //scouting:https -- --testdb_port 2345 --https_port 3456

The `--testdb_port` value must match the port you selected when running the
database.

The `--https_port` value is the port at which the webserver is available via
HTTPS. See the documentation in
[`tools/build_rules/apache.bzl`](tools/build_rules/apache.bzl) for more
information. The documentation tells you how to set up an `ldap.json`
configuration.

You can then visit <https://localhost:3456/> to look at the webserver.
