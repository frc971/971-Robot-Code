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
