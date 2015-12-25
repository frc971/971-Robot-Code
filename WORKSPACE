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

new_http_archive(
  name = 'arm-frc-linux-gnueabi-repo',
  build_file = 'tools/cpp/arm-frc-linux-gnueabi/arm-frc-linux-gnueabi.BUILD',
  sha256 = '9d92b513b627c4aaa93d4d8049b4c6b96a532b64df11b27fde4dead58347a9f6',
  url = 'http://frc971.org/Build-Dependencies/arm-frc-linux-gnueabi_4.9.3.tar.gz',
)

new_git_repository(
  name = 'python-gflags-repo',
  remote = 'https://github.com/gflags/python-gflags.git',
  build_file = 'debian/gflags.BUILD',
  commit = '41c4571864f0db5823e07715317e7388e94faabc',
)

bind(
  name = 'python-gflags',
  actual = '@python-gflags-repo//:gflags',
)

new_http_archive(
  name = 'python-glog-repo',
  build_file = 'debian/glog.BUILD',
  sha256 = '953fd80122c48023d1148e6d1bda2763fcab59c8a81682bb298238a5935547b0',
  url = 'https://pypi.python.org/packages/source/g/glog/glog-0.1.tar.gz',
  strip_prefix = 'glog-0.1',
)

bind(
  name = 'python-glog',
  actual = '@python-glog-repo//:glog',
)

new_http_archive(
  name = 'allwpilib_ni-libraries_repo',
  build_file = 'debian/ni-libraries.BUILD',
  sha256 = '821687afbee2d7531fb3e47d8d58ac10005695e59685be3ac3aa00b3179faf52',
  url = 'http://frc971.org/Build-Dependencies/allwpilib_ni-libraries_20749ed.tar.gz',
  strip_prefix = 'ni-libraries',
)

bind(
  name = 'ni-libraries',
  actual = '@allwpilib_ni-libraries_repo//:ni-libraries',
)
