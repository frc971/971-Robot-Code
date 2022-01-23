"""This is a dummy server to demonstrate the apache_wrapper() rule.

Don't run this server on its own. Instead run the
`//build_tests:apache_https_demo` binary.

When you authenticate against Apache via LDAP, this server prints the username
you authenticated with and the path from the GET request.
"""

import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import os

def parse_auth(authorization):
    auth_type, auth_info = authorization.split(" ", maxsplit=1)
    if auth_type != "Basic":
        raise ValueError(f"Unknown auth type: {auth_type}")
    username, _ = base64.b64decode(auth_info).decode().split(":", 1)
    return username

class Server(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()

    def _write(self, text):
        self.wfile.write(text.encode("utf-8"))

    def do_GET(self):
        self._set_response()
        self._write(f"GET request for {self.path}")
        self._write("<br/>")
        username = parse_auth(self.headers["Authorization"])
        self._write(f"Hello, {username}")

def main():
    port = int(os.environ["APACHE_WRAPPED_PORT"])
    server_address = ("", port)
    httpd = HTTPServer(server_address, Server)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()

if __name__ == "__main__":
    main()
