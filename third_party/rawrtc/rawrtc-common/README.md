# RAWRTCC

[![CircleCI build status][circleci-badge]][circleci-url]
[![Travis CI build status][travis-ci-badge]][travis-ci-url]
[![Join our chat on Gitter][gitter-icon]][gitter]

Contains common functionality required by both [RAWRTC][rawrtc] and
[RAWRTCDC][rawrtcdc].

## Prerequisites

The following tools are required:

* [git][git]
* [meson][meson] >= 0.46.0
* [ninja][ninja] >= 1.5
* Optional: pkg-config (`pkgconf` for newer FreeBSD versions)

## Build

```bash
cd <path-to-rawrtcc>
mkdir build
meson build
cd build
ninja
```

## Contributing

When creating a pull request, it is recommended to run `format-all.sh` to
apply a consistent code style.



[circleci-badge]: https://circleci.com/gh/rawrtc/rawrtc-common.svg?style=shield
[circleci-url]: https://circleci.com/gh/rawrtc/rawrtc-common
[travis-ci-badge]: https://travis-ci.org/rawrtc/rawrtc-common.svg?branch=master
[travis-ci-url]: https://travis-ci.org/rawrtc/rawrtc-common
[gitter]: https://gitter.im/rawrtc/Lobby
[gitter-icon]: https://badges.gitter.im/rawrtc/Lobby.svg

[rawrtc]: https://github.com/rawrtc/rawrtc
[rawrtcdc]: https://github.com/rawrtc/rawrtc-data-channel

[git]: https://git-scm.com
[meson]: https://mesonbuild.com
[ninja]: https://ninja-build.org
