#include "y2017/vision/target_finder.h"

#include <math.h>

namespace y2017 {
namespace vision {

// Blobs now come in three types:
//  0) normal blob.
//  1) first part of a split blob.
//  2) second part of a split blob.
void ComputeXShiftPolynomial(int type, const RangeImage &img,
                             TargetComponent *target) {
  target->img = &img;
  RangeImage t_img = Transpose(img);
  int spacing = 10;
  int n = t_img.size() - spacing * 2;
  target->n = n;
  if (n <= 0) {
    printf("Empty blob aborting (%d).\n", n);
    return;
  }
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n * 2, 4);
  Eigen::VectorXf b = Eigen::VectorXf::Zero(n * 2);
  int i = 0;
  for (const auto &row : t_img) {
    // We decided this was a split target, but this is not a split row.
    if (i >= spacing && i - spacing < n) {
      int j = (i - spacing) * 2;
      // normal blob or the first part of a split.
      if (type == 0 || type == 1) {
        b(j) = row[0].st;
      } else {
        b(j) = row[1].st;
      }
      A(j, 0) = (i) * (i);
      A(j, 1) = (i);
      A(j, 2) = 1;
      ++j;
      // normal target or the second part of a split.
      if (type == 0 || type == 2) {
        b(j) = row[row.size() - 1].ed;
      } else {
        b(j) = row[0].ed;
      }
      A(j, 0) = i * i;
      A(j, 1) = i;
      A(j, 3) = 1;
    }
    ++i;
  }
  Eigen::VectorXf sol =
      A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  target->a = sol(0);
  target->b = sol(1);
  target->c_0 = sol(2);
  target->c_1 = sol(3);

  target->mini = t_img.min_y();

  Eigen::VectorXf base = A * sol;
  Eigen::VectorXf error_v = b - base;
  target->fit_error = error_v.dot(error_v);
}

double TargetFinder::DetectConnectedTarget(const RangeImage &img) {
  using namespace aos::vision;
  RangeImage t_img = Transpose(img);
  int total = 0;
  int split = 0;
  int count = t_img.mini();
  for (const auto &row : t_img) {
    if (row.size() == 1) {
      total++;
    } else if (row.size() == 2) {
      split++;
    }
    count++;
  }
  return (double)split / total;
}

std::vector<TargetComponent> TargetFinder::FillTargetComponentList(
    const BlobList &blobs) {
  std::vector<TargetComponent> list;
  TargetComponent newTarg;
  for (std::size_t i = 0; i < blobs.size(); ++i) {
    double split_ratio;
    if ((split_ratio = DetectConnectedTarget(blobs[i])) > 0.50) {
      // Split type blob, do it two parts.
      ComputeXShiftPolynomial(1, blobs[i], &newTarg);
      list.emplace_back(newTarg);
      ComputeXShiftPolynomial(2, blobs[i], &newTarg);
      list.emplace_back(newTarg);
    } else {
      // normal type blob.
      ComputeXShiftPolynomial(0, blobs[i], &newTarg);
      list.emplace_back(newTarg);
    }
  }

  return list;
}

aos::vision::RangeImage TargetFinder::Threshold(aos::vision::ImagePtr image) {
  return aos::vision::DoThreshold(image, [&](aos::vision::PixelRef &px) {
    if (px.g > 88) {
      uint8_t min = std::min(px.b, px.r);
      uint8_t max = std::max(px.b, px.r);
      if (min >= px.g || max >= px.g) return false;
      uint8_t a = px.g - min;
      uint8_t b = px.g - max;
      return (a > 10 && b > 10);
    }
    return false;
  });
}

void TargetFinder::PreFilter(BlobList &imgs) {
  imgs.erase(std::remove_if(imgs.begin(), imgs.end(),
                            [](RangeImage &img) {
                              // We can drop images with a small number of
                              // pixels, but images
                              // must be over 20px or the math will have issues.
                              return (img.npixels() < 100 || img.height() < 25);
                            }),
             imgs.end());
}

bool TargetFinder::FindTargetFromComponents(
    std::vector<TargetComponent> component_list, Target *final_target) {
  using namespace aos::vision;
  if (component_list.size() < 2 || final_target == NULL) {
    // We don't enough parts for a traget.
    return false;
  }

  // A0 * c + A1*s = b
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(4, 2);
  // A0: Offset component will be constant across all equations.
  A(0, 0) = 1;
  A(1, 0) = 1;
  A(2, 0) = 1;
  A(3, 0) = 1;

  // A1: s defines the scaling and defines an expexted target.
  // So these are the center heights of the top and bottom of the two targets.
  A(0, 1) = -1;
  A(1, 1) = 0;
  A(2, 1) = 2;
  A(3, 1) = 4;

  // Track which pair is the best fit.
  double best_error = -1;
  double best_offset = -1;
  Eigen::VectorXf best_v;
  // Write down the two indicies.
  std::pair<int, int> selected;
  // We are regressing the combined estimated center,  might write that down.
  double regressed_y_center = 0;

  Eigen::VectorXf b = Eigen::VectorXf::Zero(4);
  for (size_t i = 0; i < component_list.size(); i++) {
    for (size_t j = 0; j < component_list.size(); j++) {
      if (i == j) {
        continue;
      } else {
        if (component_list[i].a < 0.0 || component_list[j].a < 0.0) {
          // one of the targets is upside down (ie curved up), this can't
          // happen.
          continue;
        }
        // b is the target offests.
        b(0) = component_list[j].EvalMinTop();
        b(1) = component_list[j].EvalMinBot();
        b(2) = component_list[i].EvalMinTop();
        b(3) = component_list[i].EvalMinBot();
      }

      Eigen::VectorXf sol =
          A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

      Eigen::VectorXf base = A * sol;
      Eigen::VectorXf error_v = b - base;
      double error = error_v.dot(error_v);
      // offset in scrren x of the two targets.
      double offset = std::abs(component_list[i].CenterPolyOne() -
                               component_list[j].CenterPolyOne());
      // How much do we care about offset. As far as I've seen, error is like
      // 5-20, offset are around 10. Value selected for worst garbage can image.
      const double offsetWeight = 2.1;
      error += offsetWeight * offset;
      if ((best_error < 0 || error < best_error) && !isnan(error)) {
        best_error = error;
        best_offset = offset;
        best_v = error_v;
        selected.first = i;
        selected.second = j;
        regressed_y_center = sol(0);
      }
    }
  }

  // If we missed or the error is ridiculous just give up here.
  if (best_error < 0 || best_error > 300.0 || isnan(best_error)) {
    fprintf(stderr, "Bogus target dude (%f).\n", best_error);
    return false;
  }

  fprintf(stderr,
          "Selected (%d, %d):\n\t"
          "err(%.2f, %.2f, %.2f, %.2f)(%.2f)(%.2f).\n\t"
          "c00(%.2f, %.2f)(%.2f)\n",
          selected.first, selected.second, best_v(0), best_v(1), best_v(2),
          best_v(3), best_error, best_offset,
          component_list[selected.first].CenterPolyOne(),
          component_list[selected.second].CenterPolyOne(),
          component_list[selected.first].CenterPolyOne() -
              component_list[selected.second].CenterPolyOne());

  double avgOff = (component_list[selected.first].mini +
                   component_list[selected.second].mini) /
                  2.0;
  double avgOne = (component_list[selected.first].CenterPolyOne() +
                   component_list[selected.second].CenterPolyOne()) /
                  2.0;

  final_target->screen_coord.x(avgOne + avgOff);
  final_target->screen_coord.y(regressed_y_center);
  final_target->comp1 = component_list[selected.first];
  final_target->comp2 = component_list[selected.second];

  return true;
}

namespace {

constexpr double kInchesToMeters = 0.0254;

}  // namespace

void RotateAngle(aos::vision::Vector<2> vec, double angle, double *rx,
                 double *ry) {
  double cos_ang = std::cos(angle);
  double sin_ang = std::sin(angle);
  *rx = vec.x() * cos_ang - vec.y() * sin_ang;
  *ry = vec.x() * sin_ang + vec.y() * cos_ang;
}

void TargetFinder::GetAngleDist(const aos::vision::Vector<2>& target,
                                double down_angle, double *dist,
                                double *angle) {
  // TODO(ben): Will put all these numbers in a config file before
  // the first competition. I hope.
  double focal_length = 1418.6;
  double mounted_angle_deg = 33.5;
  double camera_angle = mounted_angle_deg * M_PI / 180.0 - down_angle;
  double window_height = 960.0;
  double window_width = 1280.0;

  double target_height = 78.0;
  double camera_height = 21.5;
  double tape_width = 2;
  double world_height = tape_width + target_height - camera_height;

  double target_to_boiler = 9.5;
  double robot_to_camera = 9.5;
  double added_dist = target_to_boiler + robot_to_camera;

  double px = target.x() - window_width / 2.0;
  double py = target.y() - window_height / 2.0;
  double pz = focal_length;
  RotateAngle(aos::vision::Vector<2>(pz, -py), camera_angle, &pz, &py);
  double pl = std::sqrt(pz * pz + px * px);

  *dist = kInchesToMeters * (world_height * pl / py - added_dist);
  *angle = -std::atan2(px, pz);
}

}  // namespace vision
}  // namespace y2017
