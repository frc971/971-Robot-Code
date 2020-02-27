from __future__ import print_function

import json
import sys
import subprocess
import os
import threading

from bazel_tools.tools.python.runfiles import runfiles

def main(params):
  r = runfiles.Create()
  generator = r.Rlocation('org_frc971/y2020/vision/sift/fast_gaussian_generator')

  ruledir = sys.argv[2]
  target_cpu = sys.argv[3]

  target = {
      'armhf-debian': 'arm-32-linux-no_asserts',
      'k8': 'x86-64-linux-no_asserts',
  }[target_cpu]

  commands = []

  amd64_debian_sysroot = r.Rlocation('amd64_debian_sysroot/usr/lib/x86_64-linux-gnu/libc.so.6').rsplit('/', 4)[0]
  env = os.environ.copy()
  env['LD_LIBRARY_PATH'] = ':'.join([
      # TODO(brian): Figure this out again.  It is a bit aggressive.
      #amd64_debian_sysroot + '/lib/x86_64-linux-gnu',
      #amd64_debian_sysroot + '/lib',
      #amd64_debian_sysroot + '/usr/lib/x86_64-linux-gnu',
      #amd64_debian_sysroot + '/usr/lib',
  ])

  all_header = [
      '#ifndef Y2020_VISION_SIFT_FAST_GAUSSIAN_ALL_H_',
      '#define Y2020_VISION_SIFT_FAST_GAUSSIAN_ALL_H_',
      '#include "HalideBuffer.h"',
  ]

  for cols, rows in params['sizes']:
    for sigma, sigma_name, filter_width in params['sigmas']:
      name = "fast_gaussian_%dx%d_%s" % (cols, rows, sigma_name)

      commands.append([
          generator,
          '-g', 'gaussian_generator',
          '-o', ruledir,
          '-f', name,
          '-e', 'o,h,html',
          'target=%s-no_runtime' % target,
          'cols=%s' % cols,
          'rows=%s' % rows,
          'sigma=%s' % sigma,
          'filter_width=%s' % filter_width,
      ])
      all_header += [
          '#include "y2020/vision/sift/%s.h"' % name,
      ]

      name = "fast_gaussian_subtract_%dx%d_%s" % (cols, rows, sigma_name)

      commands.append([
          generator,
          '-g', 'gaussian_and_subtract_generator',
          '-o', ruledir,
          '-f', name,
          '-e', 'o,h,html',
          'target=%s-no_runtime' % target,
          'cols=%s' % cols,
          'rows=%s' % rows,
          'sigma=%s' % sigma,
          'filter_width=%s' % filter_width,
      ])
      all_header += [
          '#include "y2020/vision/sift/%s.h"' % name,
      ]

    name = 'fast_subtract_%dx%d' % (cols, rows)
    commands.append([
        generator,
        '-g', 'subtract_generator',
        '-o', ruledir,
        '-f', name,
        '-e', 'o,h,html',
        'target=%s-no_runtime' % target,
        'cols=%s' % cols,
        'rows=%s' % rows,
    ])
    all_header += [
        '#include "y2020/vision/sift/%s.h"' % name,
    ]
  commands.append([
      generator,
      '-r', 'fast_gaussian_runtime',
      '-o', ruledir,
      '-e', 'o',
      'target=%s' % target,
  ])

  all_header += [
      'namespace frc971 {',
      'namespace vision {',
      '// 0 is success. 1 is non-implemented size. Negative is a Halide error.',
      'inline int DoGeneratedFastGaussian(',
      '    Halide::Runtime::Buffer<const int16_t, 2> input,',
      '    Halide::Runtime::Buffer<int16_t, 2> output,',
      '    double sigma) {',
  ]

  for sigma, sigma_name, filter_width in params['sigmas']:
    for cols, rows in params['sizes']:
      name = "fast_gaussian_%dx%d_%s" % (cols, rows, sigma_name)
      all_header += [
          '  if (input.dim(0).extent() == %s' % cols,
          '      && input.dim(1).extent() == %s' % rows,
          '      && sigma == %s) {' % sigma,
          '    return %s(input, output);' % name,
          '  }',
      ]

  all_header += [
      '  return 1;',
      '}',
      'inline int DoGeneratedFastGaussianAndSubtract(',
      '    Halide::Runtime::Buffer<const int16_t, 2> input,',
      '    Halide::Runtime::Buffer<int16_t, 2> blurred,',
      '    Halide::Runtime::Buffer<int16_t, 2> difference,',
      '    double sigma) {',
  ]

  for sigma, sigma_name, filter_width in params['sigmas']:
    for cols, rows in params['sizes']:
      name = "fast_gaussian_subtract_%dx%d_%s" % (cols, rows, sigma_name)
      all_header += [
          '  if (input.dim(0).extent() == %s' % cols,
          '      && input.dim(1).extent() == %s' % rows,
          '      && sigma == %s) {' % sigma,
          '    return %s(input, blurred, difference);' % name,
          '  }',
      ]

  all_header += [
      '  return 1;',
      '}',
      'inline int DoGeneratedFastSubtract('
      '    Halide::Runtime::Buffer<const int16_t, 2> input_a,',
      '    Halide::Runtime::Buffer<const int16_t, 2> input_b,',
      '    Halide::Runtime::Buffer<int16_t, 2> output) {',
  ]
  for cols, rows in params['sizes']:
    name = 'fast_subtract_%dx%d' % (cols, rows)
    all_header += [
        '  if (input_a.dim(0).extent() == %s' % cols,
        '      && input_a.dim(1).extent() == %s) {' % rows,
        '    return %s(input_a, input_b, output);' % name,
        '  }',
    ]
  all_header += [
      '  return 1;',
      '}',
      '}  // namespace vision',
      '}  // namespace frc971',
      '#endif  // Y2020_VISION_SIFT_FAST_GAUSSIAN_ALL_H_',
  ]

  with open(os.path.join(ruledir, 'fast_gaussian_all.h'), 'w') as f:
    f.writelines([line + '\n' for line in all_header])

  commands_lock = threading.Lock()
  success = [True]

  def run_commands():
    while True:
      with commands_lock:
        if not commands:
          return
        if not success[0]:
          return
        command = commands.pop()
      try:
        subprocess.check_call(command, env=env)
      except:
        with commands_lock:
          success[0] = False
        raise
  threads = [threading.Thread(target=run_commands) for _ in range(4)]
  for thread in threads:
    thread.start()
  for thread in threads:
    thread.join()
  if not success[0]:
    sys.exit(1)

if __name__ == '__main__':
  main(json.loads(sys.argv[1]))
