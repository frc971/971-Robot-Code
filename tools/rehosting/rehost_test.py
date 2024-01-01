import unittest
import unittest.mock
from tools.rehosting import rehost
from pathlib import Path
import os


class TestRehost(unittest.TestCase):

    def test_url_validation(self):
        self.assertEqual("http://google.com",
                         rehost.validate_url("http://google.com"))
        self.assertRaisesRegex(Exception, "Invalid URL", rehost.validate_url,
                               "file:///some/secret")
        self.assertRaisesRegex(Exception, "Invalid URL", rehost.validate_url,
                               "http://10.0.0.0/secret")

    def test_url_to_path(self):
        test_dir = os.getenv("TEST_TMPDIR", "/tmp/")
        with unittest.mock.patch.object(rehost, "BUILD_DEPENDENCIES_PATH",
                                        test_dir):
            existing_file = test_dir + "/exists.com"
            with open(existing_file, 'w') as f:
                f.write('string')
            self.assertEqual(
                Path(test_dir) / "example.com/foo/bar",
                rehost.url_to_path("https://example.com/foo/bar"))
            self.assertRaisesRegex(ValueError,
                                   f"not in the subpath of '{test_dir}'",
                                   rehost.url_to_path,
                                   "https://example.com/../../bar")
            self.assertRaisesRegex(FileExistsError, "There is already a file",
                                   rehost.url_to_path, "https://exists.com")


if __name__ == "__main__":
    unittest.main()
