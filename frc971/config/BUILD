filegroup(
  name = 'rio_robotCommand',
  srcs = [ 'robotCommand' ],
)

sh_binary(
  name = 'setup_roborio',
  srcs = [ 'setup_roborio.sh' ],
  visibility = [ '//visibility:public' ],
  data = [
    ':rio_robotCommand',
    '@arm_frc_linux_gnueabi_repo//:compiler_pieces',
  ],
)
