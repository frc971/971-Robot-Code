licenses(['notice'])

cc_library(
  name = 'eigen',
  visibility = ['//visibility:public'],
  srcs = glob(['Eigen/src/**/*.h']),
  includes = ['.'],
  hdrs = glob(['Eigen/*'], exclude=[
    # Stuff that we don't have the dependencies for.
    'Eigen/CholmodSupport',
    'Eigen/MetisSupport',
    'Eigen/PaStiXSupport',
    'Eigen/PardisoSupport',
    'Eigen/SPQRSupport',
    'Eigen/SuperLUSupport',
    'Eigen/UmfPackSupport',
  ]) + ['unsupported/Eigen/MatrixFunctions'] +
  glob(['unsupported/Eigen/src/MatrixFunctions/*.h']),
)
