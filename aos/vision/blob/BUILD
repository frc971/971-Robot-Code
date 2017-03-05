load('/tools/build_rules/gtk_dependent', 'gtk_dependent_cc_binary', 'gtk_dependent_cc_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'range_image',
  hdrs = ['range_image.h'],
  srcs = ['range_image.cc'],
  deps = [
    '//aos/vision/math:vector',
    '//aos/vision/debug:overlay',
    '//aos/vision/math:segment',
    '//aos/vision/image:image_types',
    '//third_party/eigen',
  ],
)

cc_library(
  name = 'region_alloc',
  hdrs = ['region_alloc.h'],
  deps = [
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'contour',
  hdrs = ['contour.h'],
  srcs = ['contour.cc'],
  deps = [
    '//aos/vision/debug:overlay',
    '//aos/vision/math:segment',
    ':range_image',
    ':region_alloc',
  ],
)

cc_library(
  name = 'threshold',
  hdrs = ['threshold.h'],
  deps = [
    ':range_image',
    '//aos/vision/image:image_types',
  ]
)

cc_library(
  name = 'hierarchical_contour_merge',
  hdrs = ['hierarchical_contour_merge.h'],
  srcs = ['hierarchical_contour_merge.cc'],
  deps = [
    ':contour',
    ':disjoint_set',
    ':range_image',
    '//third_party/eigen',
  ]
)

cc_library(
  name = 'disjoint_set',
  hdrs = ['disjoint_set.h'],
)

cc_library(
  name = 'find_blob',
  hdrs = ['find_blob.h'],
  srcs = ['find_blob.cc'],
  deps = [
    '//aos/vision/debug:overlay',
    '//aos/vision/math:segment',
    ':disjoint_set',
    ':range_image',
    '//third_party/eigen',
  ]
)

cc_library(
  name = 'codec',
  hdrs = ['codec.h'],
  srcs = ['codec.cc'],
  deps = [
    '//aos/vision/debug:overlay',
    '//aos/vision/math:segment',
    ':range_image',
    '//third_party/eigen',
  ],
)

cc_test(
  name = 'codec_test',
  srcs = ['codec_test.cc'],
  deps = [
    ':codec',
    '//aos/testing:googletest',
  ],
)

cc_library(
  name = 'move_scale',
  hdrs = ['move_scale.h'],
  srcs = ['move_scale.cc'],
  deps = [
    ':range_image',
    '//aos/vision/image:image_types',
  ],
)

cc_library(
  name = 'test_utils',
  hdrs = ['test_utils.h'],
  srcs = ['test_utils.cc'],
  deps = [
    ':range_image',
  ],
)

cc_library(
  name = 'transpose',
  hdrs = ['transpose.h'],
  srcs = ['transpose.cc'],
  deps = [
    ':range_image',
  ],
)

cc_test(
  name = 'transpose_test',
  srcs = ['transpose_test.cc'],
  deps = [
    ':transpose',
    ':test_utils',
    '//aos/testing:googletest',
  ],
)

gtk_dependent_cc_library(
  name = 'stream_view',
  hdrs = ['stream_view.h'],
  deps = [
    ':range_image',
    '//aos/vision/debug:debug_window',
    '//aos/vision/image:image_types',
  ],
)
