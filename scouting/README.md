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

### Connecting directly to the database
You can use the `psql` program to look at the contents of the database
directly.
```console
$ bazel run @postgresql_amd64//:psql -- --port=2345 --host=localhost --username=test --dbname=postgres

postgres=# select * from team_match_stats;
```


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

Getting data from thebluealliance.com
--------------------------------------------------------------------------------
We get a few pieces of information from The Blue Alliance:
* the match schedule, and
* the team rankings.

For this to work, you need to set up a config file that contains your API key
for The Blue Alliance. Follow these steps if you want to get data from The Blue
Alliance.

1. Make an account on <www.thebluealliance.com>. If helpful, you can use your
   Google account or Apple account to log in.
2. On the Account page, scroll down to the "Read API Keys" section.
3. Enter something like "scouting app" in the "Description" field and click the
   "Add New Key" button.
4. Scroll back down to the "Read API Keys" section.
5. Copy the X-TBA-Auth-Key in the table. It looks like a long string of random
   characters and numbers.

Now that you have your API key, create a config file called
`scouting_config.json` in the root of the 971-Robot-Code repo. It should have
the following contents:
```json
{
    "api_key": "<api key>"
}
```
where `<api key>` needs to be replaced by the key you copied from your Account
page on <www.thebluealliance.com>.

When running the webserver, add the `-tba_config` option, specifying the
absolute path of the config file.

```console
$ bazel run //scouting -- ... --tba_config=$PWD/scouting_config.json
```

### Scraping ranking data
In order to scrape ranking data, add the competition-specific information to
`scouting_config.json`. It should look like this:
```json
{
    "api_key": "<api key>",
    "year": <year>,
    "event_code": "<event code>",
}
```
where `<year>` is the year of the event and `<event_code>` is the short code
for the event. A list of event codes is available
[here](http://frclinks.com/#eventcodes).


Debugging Cypress tests
--------------------------------------------------------------------------------
See the [dedicated section](../tools/js#debugging-cypress-tests) for this.
