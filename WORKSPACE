new_local_repository(
  name = 'usr_repo',
  path = '/usr',
  build_file = 'debian/usr.BUILD',
)

new_git_repository(
  name = 'slycot_repo',
  remote = 'https://github.com/avventi/Slycot.git',
  build_file = 'debian/slycot.BUILD',
  commit = '5af5f283cb23cbe23c4dfea4d5e56071bdbd6e70',
)

bind(
  name = 'slycot',
  actual = '@slycot_repo//:slycot',
)

# TODO(brian): Make these point to something which isn't hard-coded to come off
# the host system...
bind(
  name = 'librt',
  actual = '@usr_repo//:librt',
)

bind(
  name = 'libdl',
  actual = '@usr_repo//:libdl',
)

bind(
  name = 'libm',
  actual = '@usr_repo//:libm',
)

bind(
  name = 'libpthread',
  actual = '@usr_repo//:libpthread',
)

new_http_archive(
  name = 'arm-frc-linux-gnueabi-repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '9d92b513b627c4aaa93d4d8049b4c6b96a532b64df11b27fde4dead58347a9f6',
  url = 'http://frc971.org/Build-Dependencies/arm-frc-linux-gnueabi_4.9.3.tar.gz',
)
