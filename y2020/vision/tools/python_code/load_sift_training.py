#!/usr/bin/python3

import cv2
import sys
import flatbuffers
import target_definition

import frc971.vision.sift.TrainingImage as TrainingImage
import frc971.vision.sift.TrainingData as TrainingData
import frc971.vision.sift.Feature as Feature

def main():

  output_path = sys.argv[1]
  print("Writing file to ", output_path)

  target_data_list = target_definition.compute_target_definition()

  fbb = flatbuffers.Builder(0)

  images_vector = []

  for target_data in target_data_list:

    features_vector = []

    for keypoint, keypoint_3d, descriptor in zip(target_data.keypoint_list,
                                                 target_data.keypoint_list_3d,
                                                 target_data.descriptor_list):

      Feature.FeatureStartDescriptorVector(fbb, len(descriptor))
      for n in reversed(descriptor):
        fbb.PrependFloat32(n)
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

      ## TODO: Write 3d vector here

    TrainingImage.TrainingImageStartFeaturesVector(fbb, len(features_vector))
    for feature in reversed(features_vector):
      fbb.PrependUOffsetTRelative(feature)
    features_vector_table = fbb.EndVector(len(features_vector))

    TrainingImage.TrainingImageStart(fbb)
    TrainingImage.TrainingImageAddFeatures(fbb, features_vector_table)
    # TODO(Brian): Fill out the transformation matrices.
    images_vector.append(TrainingImage.TrainingImageEnd(fbb))

  TrainingData.TrainingDataStartImagesVector(fbb, len(images_vector))
  for training_image in reversed(images_vector):
    fbb.PrependUOffsetTRelative(training_image)
  images_vector_table = fbb.EndVector(len(images_vector))

  TrainingData.TrainingDataStart(fbb)
  TrainingData.TrainingDataAddImages(fbb, images_vector_table)
  fbb.Finish(TrainingData.TrainingDataEnd(fbb))

  bfbs = fbb.Output()

  output_prefix = [
      b'#ifndef Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
      b'#define Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
      b'#include <string_view>',
      b'namespace frc971 {',
      b'namespace vision {',
      b'inline std::string_view SiftTrainingData() {',
  ]
  output_suffix = [
      b'  return std::string_view(kData, sizeof(kData));',
      b'}',
      b'}  // namespace vision',
      b'}  // namespace frc971',
      b'#endif  // Y2020_VISION_TOOLS_PYTHON_CODE_TRAINING_DATA_H_',
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
