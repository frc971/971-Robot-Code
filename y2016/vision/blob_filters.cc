#include "y2016/vision/blob_filters.h"
#include <unistd.h>

namespace aos {
namespace vision {

double CornerFinder::LineScore(Vector<2> A, Vector<2> B, FittedLine line) {
  Vector<2> st(line.st.x, line.st.y);
  Vector<2> ed(line.ed.x, line.ed.y);

  double dist_st_a = A.SquaredDistanceTo(st);
  double dist_st_b = B.SquaredDistanceTo(st);

  if (dist_st_b < dist_st_a) {
    Vector<2> tmp = B;
    B = A;
    A = tmp;
  }

  return A.SquaredDistanceTo(st) + B.SquaredDistanceTo(ed);
}

void CornerFinder::EnqueueLine(std::vector<FittedLine> *list, FittedLine line) {
  if (list->size() < 2) {
    list->emplace_back(line);
    return;
  }

  Vector<2> st(line.st.x, line.st.y);
  Vector<2> ed(line.ed.x, line.ed.y);

  double llen = st.SquaredDistanceTo(ed);
  FittedLine ins = line;
  for (int i = 0; i < (int)list->size(); i++) {
    Vector<2> ist((*list)[i].st.x, (*list)[i].st.y);
    Vector<2> ied((*list)[i].ed.x, (*list)[i].ed.y);
    double ilen = ist.SquaredDistanceTo(ied);
    if (ilen < llen) {
      // our new line is longer, so lets use that.
      // but we still need to check the one we are replacing against the
      // others.
      FittedLine tmp = (*list)[i];
      (*list)[i] = ins;
      ins = tmp;
      llen = ilen;
    }
  }
}

Segment<2> CornerFinder::MakeLine(const std::vector<FittedLine> &list) {
  assert(list.size() == 2);
  Vector<2> st0(list[0].st.x, list[0].st.y);
  Vector<2> ed0(list[0].ed.x, list[0].ed.y);
  Vector<2> st1(list[1].st.x, list[1].st.y);
  Vector<2> ed1(list[1].ed.x, list[1].ed.y);

  if (ed1.SquaredDistanceTo(st0) < st1.SquaredDistanceTo(st0)) {
    Vector<2> sttmp = st1;
    st1 = ed1;
    ed1 = sttmp;
  }

  return Segment<2>((st0 + st1), (ed0 + ed1)).Scale(0.5);
}

std::vector<std::pair<Vector<2>, Vector<2>>> CornerFinder::Find(
    const std::vector<SelectedBlob> &blobl) {
  std::vector<std::pair<Vector<2>, Vector<2>>> res;
  alloc_.reset();
  for (const SelectedBlob &blob : blobl) {
    ContourNode *n = RangeImgToContour(blob.blob, &alloc_);
    std::vector<FittedLine> lines;
    HierarchicalMerge(n, &lines, merge_rate_, min_len_);

    if (do_overlay_) {
      for (FittedLine &line : lines) {
        overlay_->AddLine(Vector<2>(line.st.x, line.st.y),
                           Vector<2>(line.ed.x, line.ed.y), {255, 0, 0});
      }
    }

    std::vector<FittedLine> leftLine;
    std::vector<FittedLine> rightLine;
    std::vector<FittedLine> bottomLine;

    for (auto &line : lines) {
      double left_score = LineScore(blob.upper_left, blob.lower_left, line);
      double right_score = LineScore(blob.upper_right, blob.lower_right, line);
      double bottom_score = LineScore(blob.lower_left, blob.lower_right, line);
      if (left_score < right_score && left_score < bottom_score) {
        if (line.st.x > 0 && line.st.y > 0 && line.ed.x > 0 && line.ed.y > 0) {
          EnqueueLine(&leftLine, line);
        }
      } else if (right_score < left_score && right_score < bottom_score) {
        EnqueueLine(&rightLine, line);
      } else {
        EnqueueLine(&bottomLine, line);
      }
    }

    if (leftLine.size() == 2 && rightLine.size() == 2 &&
        bottomLine.size() == 2) {
      Segment<2> left(MakeLine(leftLine));
      Segment<2> right(MakeLine(rightLine));
      Segment<2> bottom(MakeLine(bottomLine));

      if (do_overlay_) {
        overlay_->AddLine(left.A(), left.B(), {155, 0, 255});
        overlay_->AddLine(right.A(), right.B(), {255, 155, 0});
        overlay_->AddLine(bottom.A(), bottom.B(), {255, 0, 155});
      }

      res.emplace_back(left.Intersect(bottom), right.Intersect(bottom));
    }
  }
  return res;
}

//*****************************************************************************

std::vector<SelectedBlob> BlobFilterBase::PreFilter(const BlobList &blobl) {
  std::vector<SelectedBlob> filt;
  for (const RangeImage &blob : blobl) {
    int area = blob.calc_area();
    if (area > min_area_ && area < max_area_) {
      filt.emplace_back(SelectedBlob(blob));
    }
  }
  return filt;
}

//*****************************************************************************

bool HistogramBlobFilter::PickClosest(const Vector<2> &goal, const Vector<2> &A,
                                      const Vector<2> &B) {
  double sq_dist_a = goal.SquaredDistanceTo(A);
  double sq_dist_b = goal.SquaredDistanceTo(B);
  if (sq_dist_a < sq_dist_b) {
    // A is closest
    return true;
  } else {
    // B is closest
    return false;
  }
}

namespace {
void CalcBoundingBox(const RangeImage &rimage, Vector<2> &ul, Vector<2> &ur,
                     Vector<2> &lr, Vector<2> &ll) {
  const auto &ranges = rimage.ranges();
  int mini = rimage.mini();
  double x_min = ranges[0][0].st;
  double x_max = ranges[0][0].ed;
  double y_min = mini;
  double y_max = mini + ranges.size();
  for (auto &range : ranges) {
    for (auto &interval : range) {
      if (interval.st < x_min) {
        x_min = interval.st;
      }
      if (interval.ed > x_max) {
        x_max = interval.ed;
      }
    }
  }
  ul = Vector<2>(x_min, y_min);
  ur = Vector<2>(x_max, y_min);
  lr = Vector<2>(x_max, y_max);
  ll = Vector<2>(x_min, y_max);
}
}  // namespace

std::vector<SelectedBlob> HistogramBlobFilter::PostFilter(
    std::vector<SelectedBlob> blobl) {
  std::vector<SelectedBlob> ret;
  for (int ti = 0; ti < (int)blobl.size(); ti++) {
    Vector<2> upper_left(1280, 960);
    Vector<2> upper_right(0.0, 960);
    Vector<2> lower_right(0.0, 0.0);
    Vector<2> lower_left(1280, 0.0);

    Vector<2> tul;
    Vector<2> tur;
    Vector<2> tlr;
    Vector<2> tll;
    CalcBoundingBox(blobl[ti].blob, tul, tur, tlr, tll);
    for (int j = 0; j < (int)blobl[ti].blob.ranges().size(); j++) {
      auto &range = blobl[ti].blob.ranges()[j];
      Vector<2> first(range[0].st, j + blobl[ti].blob.mini());
      Vector<2> last(range.back().ed, j + blobl[ti].blob.mini());
      if (!PickClosest(tul, upper_left, first)) {
        upper_left = first;
      }
      if (!PickClosest(tll, lower_left, first)) {
        lower_left = first;
      }
      if (!PickClosest(tur, upper_right, last)) {
        upper_right = last;
      }
      if (!PickClosest(tlr, lower_right, last)) {
        lower_right = last;
      }
    }
    blobl[ti].upper_left = upper_left;
    blobl[ti].upper_right = upper_right;
    blobl[ti].lower_right = lower_right;
    blobl[ti].lower_left = lower_left;

    double error = CheckHistogram(&blobl[ti], upper_left, upper_right,
                                  lower_right, lower_left);
    const double k_hist_threshold = 0.05;
    if (error < k_hist_threshold) {
      ret.emplace_back(blobl[ti]);
    }
  }

  return ret;
}

double HistogramBlobFilter::CheckHistogram(SelectedBlob *blob,
                                           const Vector<2> &ul,
                                           const Vector<2> &ur,
                                           const Vector<2> &lr,
                                           const Vector<2> &ll) {
  // found horiz histogram
  std::vector<double> hist_lr(hist_size_);
  // step size along left edge
  Vector<2> delta_left = (ul - ll) * (hist_step_);
  // step size along right edge
  Vector<2> delta_right = (ur - lr) * (hist_step_);
  // sum each left to right line for the histogram
  Vector<2> s;
  Vector<2> e;
  for (int i = 0; i < hist_size_; i++) {
    s = ll + (i * delta_left);
    e = lr + (i * delta_right);
    hist_lr[i] = calcHistComponent(&blob->blob, s, e);
    if (do_overlay_) {
      double a = hist_lr[i];
      Vector<2> mid = a * s + (1.0 - a) * e;
      overlay_->AddLine(s, mid, {0, 0, 255});
      overlay_->AddLine(mid, e, {255, 255, 0});
    }
  }
  double check_vert_up = L22_dist(hist_size_, vert_hist_, hist_lr);
  double check_vert_fliped = L22_dist(hist_size_, vert_hist_fliped_, hist_lr);

  // found vert histogram
  std::vector<double> hist_ub(hist_size_);
  // step size along bottom edge
  Vector<2> delta_bottom = (ll - lr) * (hist_step_);
  // step size along top edge
  Vector<2> delta_top = (ul - ur) * (hist_step_);
  // sum each top to bottom line for the histogram
  for (int i = 0; i < hist_size_; i++) {
    s = ur + (i * delta_top);
    e = lr + (i * delta_bottom);
    hist_ub[i] = calcHistComponent(&blob->blob, s, e);
    if (do_overlay_) {
      double a = hist_ub[i];
      Vector<2> mid = a * s + (1.0 - a) * e;
      overlay_->AddLine(s, mid, {0, 0, 255});
      overlay_->AddLine(mid, e, {255, 255, 0});
    }
  }
  double check_horiz = L22_dist(hist_size_, horiz_hist_, hist_ub);

  if (do_overlay_) {
    Vector<2> A = blob->upper_left + Vector<2>(-10, 10);
    Vector<2> B = blob->upper_left - Vector<2>(-10, 10);
    overlay_->AddLine(A, B, {255, 255, 255});
    A = blob->upper_right + Vector<2>(-10, 10);
    B = blob->upper_right - Vector<2>(-10, 10);
    overlay_->AddLine(A, B, {255, 255, 255});
    A = blob->lower_right + Vector<2>(-10, 10);
    B = blob->lower_right - Vector<2>(-10, 10);
    overlay_->AddLine(A, B, {255, 255, 255});
    A = blob->lower_left + Vector<2>(-10, 10);
    B = blob->lower_left - Vector<2>(-10, 10);
    overlay_->AddLine(A, B, {255, 255, 255});
  }
  // If this target is better upside down, if it is flip the blob.
  // The horizontal will not be effected, so we will not change that.
  double check_vert;
  if (check_vert_up < check_vert_fliped) {
    // normal one is better, leave it alone
    check_vert = check_vert_up;
  } else {
    check_vert = check_vert_fliped;
    blob->Flip(fmt_);
  }
  if (do_overlay_) {
    Vector<2> A = blob->upper_left + Vector<2>(10, 10);
    Vector<2> B = blob->upper_left - Vector<2>(10, 10);
    overlay_->AddLine(A, B, {255, 0, 255});
    A = blob->upper_right + Vector<2>(10, 10);
    B = blob->upper_right - Vector<2>(10, 10);
    overlay_->AddLine(A, B, {255, 0, 255});
    A = blob->lower_right + Vector<2>(10, 10);
    B = blob->lower_right - Vector<2>(10, 10);
    overlay_->AddLine(A, B, {255, 0, 255});
    A = blob->lower_left + Vector<2>(10, 10);
    B = blob->lower_left - Vector<2>(10, 10);
    overlay_->AddLine(A, B, {255, 0, 255});
  }

  // average the two distances
  double check = (check_vert + check_horiz) / (2.0 * hist_size_);
  return check;
}

double HistogramBlobFilter::calcHistComponent(const RangeImage *blob,
                                              const Vector<2> &start,
                                              const Vector<2> &end) {
  int startx = (int)std::floor(start.x());
  int endx = (int)std::floor(end.x());
  int starty = (int)std::floor(start.y()) - blob->mini();
  int endy = (int)std::floor(end.y()) - blob->mini();
  int dx = std::abs(endx - startx);
  int dy = std::abs(endy - starty);
  int sx = (startx < endx) ? 1 : -1;
  int sy = (starty < endy) ? 1 : -1;
  int error = dx - dy;

  int total = 0;
  int value = 0;
  int total_error;
  while (1) {
    total++;
    if (starty < 0 || starty >= (int)blob->ranges().size()) {
      printf("starty (%d) size(%d)\n", starty, (int)blob->ranges().size());
      fflush(stdout);
      return 0;
    }
    const std::vector<ImageRange> &rangel = blob->ranges()[starty];
    for (const ImageRange &range : rangel) {
      if (startx >= range.st && startx <= range.ed) {
        value++;
        if (do_imgdbg_) {
          image_->get_px(startx, starty + blob->mini()) = {255, 255, 255};
        }
      }
    }

    // bresenham logic to move along a line
    if (startx == endx && starty == endy) break;
    total_error = 2 * error;
    if (total_error > -dy) {
      error -= dy;
      startx += sx;
    }
    if (total_error < dx) {
      error += dx;
      starty += sy;
    }
  }
  return (double)value / (double)total;
}

void HistogramBlobFilter::MakeGoalHist(bool is_90) {
  // calculate a desired histogram before we start
  double targ_height = 14.0;
  double targ_width = 20.0;
  double tape_width = 2.0;
  horiz_hist_.resize(hist_size_);
  vert_hist_fliped_.resize(hist_size_);
  vert_hist_.resize(hist_size_);
  int j = 0;
  for (double i = 0; j < hist_size_; i += hist_step_) {
    if (is_90) {
      assert(false);
    } else {
      if (i < (tape_width / targ_height)) {
        vert_hist_[j] = 1.0;
      } else {
        vert_hist_[j] = 2 * tape_width / targ_width;
      }

      if (i < tape_width / targ_width || i > 1.0 - (tape_width / targ_width)) {
        horiz_hist_[j] = 1.0;
      } else {
        horiz_hist_[j] = tape_width / targ_height;
      }
    }
    j++;
  }
  for (int i = 0; i < hist_size_; i++) {
    vert_hist_fliped_[hist_size_ - i - 1] = vert_hist_[i];
  }
}

}  // namespace vision
}  // namespace aos
