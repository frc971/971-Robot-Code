#!/usr/bin/python3

import cv2
import sys
import flatbuffers

import frc971.vision.sift.TrainingImage as TrainingImage
import frc971.vision.sift.TrainingData as TrainingData
import frc971.vision.sift.Feature as Feature

def main():
  image4_cleaned_path = sys.argv[1]
  output_path = sys.argv[2]

  image4_cleaned = cv2.imread(image4_cleaned_path)

  image = cv2.cvtColor(image4_cleaned, cv2.COLOR_BGR2GRAY)
  image = cv2.resize(image, (640, 480))
  sift = cv2.SIFT_create()
  keypoints, descriptors = sift.detectAndCompute(image, None)

  fbb = flatbuffers.Builder(0)

  features_vector = []

  for keypoint, descriptor in zip(keypoints, descriptors):
    Feature.FeatureStartDescriptorVector(fbb, len(descriptor))
    for n in reversed(descriptor):
      assert n == round(n)
      fbb.PrependUint8(int(round(n)))
    descriptor_vector = fbb.EndVector(len(descriptor))

    Feature.FeatureStart(fbb)

    Feature.FeatureAddDescriptor(fbb, descriptor_vector)
    Feature.FeatureAddX(fbb, keypoint.pt[0])
    Feature.FeatureAddY(fbb, keypoint.pt[1])
    Feature.FeatureAddSize(fbb, keypoint.size)
    Feature.FeatureAddAngle(fbb, keypoint.angle)
    Feature.FeatureAddResponse(fbb, keypoint.response)
    Feature.FeatureAddOctave(fbb, keypoint.octave)

    features_vector.append(Feature.FeatureEnd(fbb))

  TrainingImage.TrainingImageStartFeaturesVector(fbb, len(features_vector))
  for feature in reversed(features_vector):
    fbb.PrependUOffsetTRelative(feature)
  features_vector_table = fbb.EndVector(len(features_vector))

  TrainingImage.TrainingImageStart(fbb)
  TrainingImage.TrainingImageAddFeatures(fbb, features_vector_table)
  # TODO(Brian): Fill out the transformation matrices.
  training_image_offset = TrainingImage.TrainingImageEnd(fbb)

  TrainingData.TrainingDataStartImagesVector(fbb, 1)
  fbb.PrependUOffsetTRelative(training_image_offset)
  images_offset = fbb.EndVector(1)

  TrainingData.TrainingDataStart(fbb)
  TrainingData.TrainingDataAddImages(fbb, images_offset)
  fbb.Finish(TrainingData.TrainingDataEnd(fbb))

  bfbs = fbb.Output()

  output_prefix = [
      b'#ifndef Y2020_VISION_SIFT_DEMO_SIFT_H_',
      b'#define Y2020_VISION_SIFT_DEMO_SIFT_H_',
      b'#include <string_view>',
      b'namespace frc971 {',
      b'namespace vision {',
      b'inline std::string_view DemoSiftData() {',
  ]
  output_suffix = [
      b'  return std::string_view(kData, sizeof(kData));',
      b'}',
      b'}  // namespace vision',
      b'}  // namespace frc971',
      b'#endif  // Y2020_VISION_SIFT_DEMO_SIFT_H_',
  ]

  with open(output_path, 'wb') as output:
    for line in output_prefix:
      output.write(line)
      output.write(b'\n')
    output.write(b'alignas(64) static constexpr char kData[] = "')
    for byte in fbb.Output():
      output.write(b'\\x' + (b'%x' % byte).zfill(2))
    output.write(b'";\n')
    for line in output_suffix:
      output.write(line)
      output.write(b'\n')

if __name__ == '__main__':
  main()
