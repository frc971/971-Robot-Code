from urllib.request import urlopen
from urllib.parse import urlparse
import validators
import shutil
import stat
from pathlib import Path
import os
import sys

BUILD_DEPENDENCIES_PATH = "/data/files/frc971/Build-Dependencies/"
WWW_GROUP = "www-data"


def get_url() -> str:
    return sys.argv[1]


def validate_url(url: str) -> str:
    # We have no reason to allow people do download things from IP addresses directly.
    if not validators.url(
            url, simple_host=True, skip_ipv4_addr=True, skip_ipv6_addr=True):
        raise ValueError(f"Invalid URL {url}")
    return url


def url_to_path(url: str) -> Path:
    parsed = urlparse(url)
    # Strip out the http:// and any other extraneous junk:
    path = (Path(BUILD_DEPENDENCIES_PATH) /
            (parsed.netloc + parsed.path)).resolve()
    # Confirm that someone didn't sneak in a URL that looks like http://foo.bar/../../../.. or something.
    path.relative_to(BUILD_DEPENDENCIES_PATH)
    if path.exists():
        raise FileExistsError(f"There is already a file uploaded for {url}.")
    return path


def download():
    url = validate_url(get_url())
    path = url_to_path(url)
    path.parent.mkdir(mode=0o775, parents=True, exist_ok=True)

    with urlopen(url) as downloaded:
        with open(path, 'wb') as output:
            output.write(downloaded.read())

    relative_path = path.relative_to(BUILD_DEPENDENCIES_PATH)
    path.chmod(stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
    try:
        shutil.chown(path, group=WWW_GROUP)
        for parent in relative_path.parents:
            shutil.chown(Path(BUILD_DEPENDENCIES_PATH) / parent,
                         group=WWW_GROUP)
    except Exception:
        # The chown's sometimes fail if they get to a manually-created/touched
        # directory; don't worry about that if it happens..
        pass


if __name__ == "__main__":
    download()
