new_local_repository(
  name = 'usr_repo',
  path = '/usr',
  build_file = 'debian/BUILD.usr',
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
