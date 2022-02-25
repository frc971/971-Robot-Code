The scouting web server
================================================================================

This directory contains the code that combines to make the scouting web server.

`main.go`
--------------------------------------------------------------------------------
This is the main application that brings all the pieces together. Run it like
so:
```bash
bazel run //scouting/webserver:webserver
```

`server/`
--------------------------------------------------------------------------------
This directory contains the code that manages the web server itself. It's
responsible for starting and stopping the server. It also exposes a `Handle()`
method that lets other libraries enhance the server's functionality.

`static/`
--------------------------------------------------------------------------------
This directory contains the code that serves static files at the root of the
server. Make sure that none of the files you're serving clash with any of the
paths used by the other libraries that enhance the server's functionality. E.g.
if POST requests for match data are serviced at `/requests/xyz`, then don't
serve any files in a `requests` directory.

`requests/`
--------------------------------------------------------------------------------
This directory contains the code that services requests from the web page. The
web page sends serialized flatbuffers (see the
`scouting/webserver/requests/messages` directory) and receives serialized
flatbuffers in response.

### `requests/debug/cli`
This directory contains a debug application that lets you interact with the
webserver. It allows you to make call calls that the web page would normally
make.
