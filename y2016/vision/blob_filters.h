#ifndef Y2016_VISION_BLOB_FILTERS_H_
#define Y2016_VISION_BLOB_FILTERS_H_

#include "aos/vision/blob/codec.h"
#include "aos/vision/blob/find_blob.h"
#include "aos/vision/blob/hierarchical_contour_merge.h"
#include "aos/vision/blob/range_image.h"
#include "aos/vision/blob/threshold.h"
#include "aos/vision/debug/overlay.h"
#include "aos/vision/math/segment.h"
#include "aos/vision/math/vector.h"

namespace aos {
namespace vision {

struct SelectedBlob {
  SelectedBlob(const RangeImage &blob_inp) : blob(blob_inp) {}
  RangeImage blob;
  Vector<2> upper_left;
  Vector<2> upper_right;
  Vector<2> lower_right;
  Vector<2> lower_left;

  void Flip(ImageFormat fmt) {
    auto image_width = fmt.w;
    auto image_height = fmt.h;
    blob.Flip(fmt);

    Vector<2> tmp = lower_right;
    lower_right = upper_left;
    upper_left = tmp;
    tmp = lower_left;
    lower_left = upper_right;
    upper_right = tmp;

    // now flip the box
    lower_right = Vector<2>(image_width - lower_right.x(),
                            image_height - lower_right.y());
    upper_right = Vector<2>(image_width - upper_right.x(),
                            image_height - upper_right.y());
    lower_left =
        Vector<2>(image_width - lower_left.x(), image_height - lower_left.y());
    upper_left =
        Vector<2>(image_width - upper_left.x(), image_height - upper_left.y());
  }
};

class CornerFinder {
 public:
  CornerFinder(float merge_rate, int min_line_length)
      : merge_rate_(merge_rate), min_len_(min_line_length) {}

  CornerFinder() : CornerFinder(1.0, 25) {}

  // score how well the line matches the points. Lower score is better.
  double LineScore(Vector<2> A, Vector<2> B, FittedLine line);

  // We want to save the "best" two lines. Here we will difine that as the
  // longest
  // two we see.
  void EnqueueLine(std::vector<FittedLine> *list, FittedLine line);
  // Lines come in as two sides of the tape, we will flip them around so they
  // have the same orientation then avrage the two end points to get a line in
  // the center.
  Segment<2> MakeLine(const std::vector<FittedLine> &list);

  // take the given blob and find lines to represent it, then return the
  // target
  // corners from those lines. Left first, then bottom.
  std::vector<std::pair<Vector<2>, Vector<2>>> Find(
      const std::vector<SelectedBlob> &blobl);

  // Enable overlay debugging.
  void EnableOverlay(PixelLinesOverlay *overlay) {
    do_overlay_ = true;
    overlay_ = overlay;
  }

 private:
  // Parker did some sort of optimization with the memory.
  AnalysisAllocator alloc_;

  // check if we shuld draw the overlay
  bool do_overlay_ = false;
  PixelLinesOverlay *overlay_ = NULL;

  // how smooth do we want the lines
  float merge_rate_;
  // how short do we want the lines
  int min_len_;
};

class BlobFilterBase {
 public:
  BlobFilterBase(int min_blob_area, int max_blob_area)
      : min_area_(min_blob_area), max_area_(max_blob_area) {}

  std::vector<SelectedBlob> PreFilter(const BlobList &blobl);

  std::vector<SelectedBlob> FilterBlobs(const BlobList &blobl) {
    return PostFilter(PreFilter(blobl));
  }

  // Enable overlay debugging.
  void EnableOverlay(PixelLinesOverlay *overlay) {
    do_overlay_ = true;
    overlay_ = overlay;
  }

 private:
  virtual std::vector<SelectedBlob> PostFilter(
      std::vector<SelectedBlob> blobl) = 0;

 protected:
  // absolute minimum for even looking at a blob.
  int min_area_;
  // absolute maximum for even looking at a blob.
  int max_area_;

  // check if we shuld draw the overlay
  bool do_overlay_ = false;
  PixelLinesOverlay *overlay_ = NULL;
};

class HistogramBlobFilter : public BlobFilterBase {
 public:
  HistogramBlobFilter(ImageFormat fmt, int hist_size, int min_blob_area,
                      int max_blob_area)
      : BlobFilterBase(min_blob_area, max_blob_area),
        fmt_(fmt),
        hist_size_(hist_size),
        hist_step_(1.0 / (double)hist_size) {
    MakeGoalHist(false);
  }

  // Enable image debugging.
  void EnableImageHist(ImagePtr *img) {
    do_imgdbg_ = true;
    image_ = img;
  }

 private:
  // Returns the point closest to the goal point.
  bool PickClosest(const Vector<2> &goal, const Vector<2> &A,
                   const Vector<2> &B);
  // Main filter function.
  std::vector<SelectedBlob> PostFilter(std::vector<SelectedBlob> blobl);

  // calc and compare the histograms to the desired
  double CheckHistogram(SelectedBlob *blob, const Vector<2> &ul,
                        const Vector<2> &ur, const Vector<2> &lr,
                        const Vector<2> &ll);

  // sum over values between these two points and normalize
  // see Bresenham's Line Algorithm for the logic of moving
  // over all the pixels between these two points.
  double calcHistComponent(const RangeImage *blob, const Vector<2> &start,
                           const Vector<2> &end);

  void MakeGoalHist(bool is_90);

  // just a distance function
  double chiSquared(int length, double *histA, double *histB) {
    double sum = 0;
    for (int i = 0; i < length; i++) {
      double diff = *(histB + i) - *(histA + i);
      sum += (diff * diff) / *(histA + i);
    }
    return sum;
  }

  // squared euclidiean dist function
  double L22_dist(int length, std::vector<double> &histA,
                  std::vector<double> histB) {
    double sum = 0;
    for (int i = 0; i < length; i++) {
      double diff = histB[i] - histA[i];
      sum += (diff * diff);
    }
    return sum;
  }

  // size of image
  ImageFormat fmt_;
  // Number of elements in eaach histogram.
  int hist_size_;
  // Percent step of the above size.
  double hist_step_;
  // histogram summing in y.
  std::vector<double> horiz_hist_;
  // histogram summing in x direction.
  std::vector<double> vert_hist_;
  // histogram summing in x from greatest to least.
  std::vector<double> vert_hist_fliped_;
  bool do_imgdbg_ = false;
  ImagePtr *image_ = NULL;
};

}  // namespace vision
}  // namespace aos

#endif  // Y2016_VISION_BLOB_FILTERS_H_
