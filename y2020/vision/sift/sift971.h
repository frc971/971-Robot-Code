#ifndef Y2020_VISION_SIFT_SIFT971_H_
#define Y2020_VISION_SIFT_SIFT971_H_

#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

namespace frc971 {
namespace vision {

/*!
 SIFT implementation.

 The class implements SIFT algorithm by D. Lowe.
 */
class SIFT971_Impl : public cv::Feature2D {
 public:
  explicit SIFT971_Impl(int nfeatures = 0, int nOctaveLayers = 3,
                        double contrastThreshold = 0.04,
                        double edgeThreshold = 10, double sigma = 1.6);

  //! returns the descriptor size in floats (128)
  int descriptorSize() const override;

  //! returns the descriptor type
  int descriptorType() const override;

  //! returns the default norm type
  int defaultNorm() const override;

  //! finds the keypoints and computes descriptors for them using SIFT
  //! algorithm. Optionally it can compute descriptors for the user-provided
  //! keypoints
  void detectAndCompute(cv::InputArray img, cv::InputArray mask,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::OutputArray descriptors,
                        bool useProvidedKeypoints = false) override;

  void buildGaussianPyramid(const cv::Mat &base, std::vector<cv::Mat> &pyr,
                            int nOctaves) const;
  void buildDoGPyramid(const std::vector<cv::Mat> &pyr,
                       std::vector<cv::Mat> &dogpyr) const;
  void buildGaussianAndDifferencePyramid(const cv::Mat &base,
                                         std::vector<cv::Mat> &pyr,
                                         std::vector<cv::Mat> &dogpyr,
                                         int nOctaves) const;
  void findScaleSpaceExtrema(const std::vector<cv::Mat> &gauss_pyr,
                             const std::vector<cv::Mat> &dog_pyr,
                             std::vector<cv::KeyPoint> &keypoints) const;

 protected:
  CV_PROP_RW int nfeatures;
  CV_PROP_RW int nOctaveLayers;
  CV_PROP_RW double contrastThreshold;
  CV_PROP_RW double edgeThreshold;
  CV_PROP_RW double sigma;

 private:
  cv::Mat createInitialImage(const cv::Mat &img, bool doubleImageSize) const;

  bool use_fast_gaussian_pyramid_ = true;
  bool use_fast_subtract_dogpyr_ = true;
  bool use_fast_guassian_initial_ = true;
  bool use_fused_pyramid_difference_ = true;
  bool use_fast_pyramid_difference_ = true;
};

}  // namespace vision
}  // namespace frc971

#endif  // Y2020_VISION_SIFT_SIFT971_H_
