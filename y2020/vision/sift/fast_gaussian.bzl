def fast_gaussian(sigmas, sizes):
  files = []
  for _, sigma_name, _ in sigmas:
    for cols, rows in sizes:
      files.append("fast_gaussian_%dx%d_%s" % (cols, rows, sigma_name))
  for _, sigma_name, _ in sigmas:
    for cols, rows in sizes:
      files.append("fast_gaussian_subtract_%dx%d_%s" % (cols, rows, sigma_name))
  for cols, rows in sizes:
    files.append('fast_subtract_%dx%d' % (cols, rows))

  params = struct(
    sigmas = sigmas,
    sizes = sizes,
  )

  headers = [f + '.h' for f in files] + [
    'fast_gaussian_all.h',
  ]
  objects = [f + '.o' for f in files] + [
    'fast_gaussian_runtime.o',
  ]
  htmls = [f + '.html' for f in files]

  native.genrule(
    name = "generate_fast_gaussian",
    tools = [
        ":fast_gaussian_runner",
    ],
    cmd = ' '.join([
      '$(location fast_gaussian_runner)',
      "'" + params.to_json() + "'",
      # TODO(Brian): This should be RULEDIR once we have support for that.
      '$(@D)',
      '$(TARGET_CPU)',
    ]),
    outs = headers + objects + htmls,
    restricted_to = [
      "//tools:k8",
      "//tools:armhf-debian",
    ],
  )

  native.cc_library(
    name = 'fast_gaussian_all',
    hdrs = ['fast_gaussian_all.h'],
    srcs = headers + objects,
    deps = [
      '//third_party:halide_runtime',
    ],
    restricted_to = [
      "//tools:k8",
      "//tools:armhf-debian",
    ],
  )
