# TODO(phil): Delete this and replace it with a selenium test. Preferably
# written in either Javascript or Go.

import socket
import subprocess
import time
import unittest
import urllib.request

class TestDebugCli(unittest.TestCase):

    def setUp(self):
        self.webserver = subprocess.Popen(["scouting/scouting"])

        # Wait for the server to respond to requests.
        while True:
            try:
                connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connection.connect(("localhost", 8080))
                connection.close()
                break
            except ConnectionRefusedError:
                connection.close()
                time.sleep(0.01)

    def tearDown(self):
        self.webserver.terminate()
        self.webserver.wait()

    def test_index_html(self):
        """Makes sure that we the scouting server is serving our main index.html file."""
        with urllib.request.urlopen("http://localhost:8080/") as file:
            html = file.read().decode("utf-8")
        self.assertIn("<my-app></my-app>", html)


if __name__ == "__main__":
    unittest.main()
