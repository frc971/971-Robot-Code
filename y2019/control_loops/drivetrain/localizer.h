#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATER_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATER_H_

#include <cmath>
#include <memory>

#include "frc971/control_loops/pose.h"
#include "y2019/control_loops/drivetrain/camera.h"
#include "frc971/control_loops/drivetrain/hybrid_ekf.h"

namespace y2019 {
namespace control_loops {

template <int num_cameras, int num_targets, int num_obstacles,
          int max_targets_per_frame, typename Scalar = double>
class TypedLocalizer
    : public ::frc971::control_loops::drivetrain::HybridEkf<Scalar> {
 public:
  typedef TypedCamera<num_targets, num_obstacles, Scalar> Camera;
  typedef typename Camera::TargetView TargetView;
  typedef typename Camera::Pose Pose;
  typedef ::frc971::control_loops::drivetrain::HybridEkf<Scalar> HybridEkf;
  typedef typename HybridEkf::State State;
  typedef typename HybridEkf::StateSquare StateSquare;
  typedef typename HybridEkf::Input Input;
  typedef typename HybridEkf::Output Output;
  using HybridEkf::kNInputs;
  using HybridEkf::kNOutputs;
  using HybridEkf::kNStates;

  // robot_pose should be the object that is used by the cameras, such that when
  // we update robot_pose, the cameras will change what they report the relative
  // position of the targets as.
  // Note that the parameters for the cameras should be set to allow slightly
  // larger fields of view and slightly longer range than the true cameras so
  // that we can identify potential matches for targets even when we have slight
  // modelling errors.
  TypedLocalizer(
      const ::frc971::control_loops::drivetrain::DrivetrainConfig<Scalar>
          &dt_config,
      Pose *robot_pose)
      : ::frc971::control_loops::drivetrain::HybridEkf<Scalar>(dt_config),
        robot_pose_(robot_pose) {}

  // Performs a kalman filter correction with a single camera frame, consisting
  // of up to max_targets_per_frame targets and taken at time t.
  // camera is the Camera used to take the images.
  void UpdateTargets(
      const Camera &camera,
      const ::aos::SizedArray<TargetView, max_targets_per_frame> &targets,
      ::aos::monotonic_clock::time_point t) {
    if (targets.empty()) {
      return;
    }

    if (!SanitizeTargets(targets)) {
      AOS_LOG(ERROR, "Throwing out targets due to in insane values.\n");
      return;
    }

    if (t > HybridEkf::latest_t()) {
      AOS_LOG(ERROR,
              "target observations must be older than most recent encoder/gyro "
              "update.\n");
      return;
    }

    Output z;
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;
    TargetViewToMatrices(targets[0], &z, &R);

    // In order to perform the correction steps for the targets, we will
    // separately perform a Correct step for each following target.
    // This way, we can have the first correction figure out the mappings
    // between targets in the image and targets on the field, and then re-use
    // those mappings for all the remaining corrections.
    // As such, we need to store the EKF functions that the remaining targets
    // will need in arrays:
    ::aos::SizedArray<::std::function<Output(const State &, const Input &)>,
                      max_targets_per_frame> h_functions;
    ::aos::SizedArray<::std::function<Eigen::Matrix<Scalar, kNOutputs,
                                                    kNStates>(const State &)>,
                      max_targets_per_frame> dhdx_functions;
    HybridEkf::Correct(
        z, nullptr,
        ::std::bind(&TypedLocalizer::MakeH, this, camera, targets, &h_functions,
                    &dhdx_functions, ::std::placeholders::_1,
                    ::std::placeholders::_2, ::std::placeholders::_3,
                    ::std::placeholders::_4),
        {}, {}, R, t);
    // Fetch cache:
    for (size_t ii = 1; ii < targets.size(); ++ii) {
      TargetViewToMatrices(targets[ii], &z, &R);
      HybridEkf::Correct(z, nullptr, {}, h_functions[ii], dhdx_functions[ii], R,
                         t);
    }
  }

 private:
  // The threshold to use for completely rejecting potentially bad target
  // matches.
  // TODO(james): Tune
  static constexpr Scalar kRejectionScore = 1.0;

  // Checks that the targets coming in make some sense--mostly to prevent NaNs
  // or the such from propagating.
  bool SanitizeTargets(
      const ::aos::SizedArray<TargetView, max_targets_per_frame> &targets) {
    for (const TargetView &view : targets) {
      const typename TargetView::Reading reading = view.reading;
      if (!(::std::isfinite(reading.heading) &&
            ::std::isfinite(reading.distance) &&
            ::std::isfinite(reading.skew) && ::std::isfinite(reading.height))) {
        AOS_LOG(ERROR, "Got non-finite values in target.\n");
        return false;
      }
      if (reading.distance < 0) {
        AOS_LOG(ERROR, "Got negative distance.\n");
        return false;
      }
      if (::std::abs(::aos::math::NormalizeAngle(reading.skew)) > M_PI_2) {
        AOS_LOG(ERROR, "Got skew > pi / 2.\n");
        return false;
      }
    }
    return true;
  }

  // Computes the measurement (z) and noise covariance (R) matrices for a given
  // TargetView.
  void TargetViewToMatrices(const TargetView &view, Output *z,
                            Eigen::Matrix<Scalar, kNOutputs, kNOutputs> *R) {
    *z << view.reading.heading, view.reading.distance,
        ::aos::math::NormalizeAngle(view.reading.skew);
    // TODO(james): R should account as well for our confidence in the target
    // matching. However, handling that properly requires thing a lot more about
    // the probabilities.
    R->setZero();
    R->diagonal() << ::std::pow(view.noise.heading, 2),
        ::std::pow(view.noise.distance, 2), ::std::pow(view.noise.skew, 2);
  }

  // This is the function that will be called once the Ekf has inserted the
  // measurement into the right spot in the measurement queue and needs the
  // output functions to actually perform the corrections.
  // Specifically, this will take the estimate of the state at that time and
  // figure out how the targets seen by the camera best map onto the actual
  // targets on the field.
  // It then fills in the h and dhdx functions that are called by the Ekf.
  void MakeH(
      const Camera &camera,
      const ::aos::SizedArray<TargetView, max_targets_per_frame> &target_views,
      ::aos::SizedArray<::std::function<Output(const State &, const Input &)>,
                        max_targets_per_frame> *h_functions,
      ::aos::SizedArray<::std::function<Eigen::Matrix<Scalar, kNOutputs,
                                                      kNStates>(const State &)>,
                        max_targets_per_frame> *dhdx_functions,
      const State &X_hat, const StateSquare &P,
      ::std::function<Output(const State &, const Input &)> *h,
      ::std::function<
          Eigen::Matrix<Scalar, kNOutputs, kNStates>(const State &)> *dhdx) {
    // Because we need to match camera targets ("views") to actual field
    // targets, and because we want to take advantage of the correlations
    // between the targets (i.e., if we see two targets in the image, they
    // probably correspond to different on-field targets), the matching problem
    // here is somewhat non-trivial. Some of the methods we use only work
    // because we are dealing with very small N (e.g., handling the correlations
    // between multiple views has combinatoric complexity, but since N = 3,
    // it's not an issue).
    //
    // High-level steps:
    // 1) Set the base robot pose for the cameras to the Pose implied by X_hat.
    // 2) Fetch all the expected target views from the camera.
    // 3) Determine the "magnitude" of the Kalman correction from each potential
    //    view/target pair.
    // 4) Match based on the combination of targets with the smallest
    //    corrections.
    // 5) Calculate h and dhdx for each pair of targets.
    //
    // For the "magnitude" of the correction, we do not directly use the
    // standard Kalman correction formula. Instead, we calculate the correction
    // we would get from each component of the measurement and take the L2 norm
    // of those. This prevents situations where a target matches very poorly but
    // produces an overall correction of near-zero.
    // TODO(james): I do not know if this is strictly the correct method to
    // minimize likely error, but should be reasonable.
    //
    // For the matching, we do the following (see MatchFrames):
    // 1. Compute the best max_targets_per_frame matches for each view.
    // 2. Exhaust every possible combination of view/target pairs and
    //    choose the best one.
    // When we don't think the camera should be able to see as many targets as
    // we actually got in the frame, then we do permit doubling/tripling/etc.
    // up on potential targets once we've exhausted all the targets we think
    // we can see.

    // Set the current robot pose so that the cameras know where they are
    // (all the cameras have robot_pose_ as their base):
    *robot_pose_->mutable_pos() << X_hat(0, 0), X_hat(1, 0), 0.0;
    robot_pose_->set_theta(X_hat(2, 0));

    // Compute the things we *think* the camera should be seeing.
    // Note: Because we will not try to match to any targets that are not
    // returned by this function, we generally want the modelled camera to have
    // a slightly larger field of view than the real camera, and be able to see
    // slightly smaller targets.
    const ::aos::SizedArray<TargetView, num_targets> camera_views =
        camera.target_views();

    // Each row contains the scores for each pair of target view and camera
    // target view. Values in each row will not be populated past
    // camera.target_views().size(); of the rows, only the first
    // target_views.size() shall be populated.
    // Higher scores imply a worse match. Zero implies a perfect match.
    Eigen::Matrix<Scalar, max_targets_per_frame, num_targets> scores;
    scores.setConstant(::std::numeric_limits<Scalar>::infinity());
    // Each row contains the indices of the best matches per view, where
    // index 0 is the best, 1 the second best, and 2 the third, etc.
    // -1 indicates an unfilled field.
    Eigen::Matrix<int, max_targets_per_frame, max_targets_per_frame>
        best_matches;
    best_matches.setConstant(-1);
    // The H matrices for each potential matching. This has the same structure
    // as the scores matrix.
    ::std::array<::std::array<Eigen::Matrix<Scalar, kNOutputs, kNStates>,
                              max_targets_per_frame>,
                 num_targets> all_H_matrices;

    // Iterate through and fill out the scores for each potential pairing:
    for (size_t ii = 0; ii < target_views.size(); ++ii) {
      const TargetView &target_view = target_views[ii];
      Output z;
      Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;
      TargetViewToMatrices(target_view, &z, &R);

      for (size_t jj = 0; jj < camera_views.size(); ++jj) {
        // Compute the ckalman update for this step:
        const TargetView &view = camera_views[jj];
        const Eigen::Matrix<Scalar, kNOutputs, kNStates> H =
            HMatrix(*view.target, camera.pose());
        const Eigen::Matrix<Scalar, kNStates, kNOutputs> PH = P * H.transpose();
        const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> S = H * PH + R;
        // Note: The inverse here should be very cheap so long as kNOutputs = 3.
        const Eigen::Matrix<Scalar, kNStates, kNOutputs> K = PH * S.inverse();
        const Output err = z - Output(view.reading.heading,
                                      view.reading.distance, view.reading.skew);
        // In order to compute the actual score, we want to consider each
        // component of the error separately, as well as considering the impacts
        // on the each of the states separately. As such, we calculate what
        // the separate updates from each error component would be, and sum
        // the impacts on the states.
        Output scorer;
        for (size_t kk = 0; kk < kNOutputs; ++kk) {
          // TODO(james): squaredNorm or norm or L1-norm? Do we care about the
          // square root? Do we prefer a quadratic or linear response?
          scorer(kk, 0) = (K.col(kk) * err(kk, 0)).squaredNorm();
        }
        // Compute the overall score--note that we add in a term for the height,
        // scaled by a manual fudge-factor. The height is not accounted for
        // in the Kalman update because we are not trying to estimate the height
        // of the robot directly.
        Scalar score =
            scorer.squaredNorm() +
            ::std::pow((view.reading.height - target_view.reading.height) /
                           target_view.noise.height / 20.0,
                       2);
        scores(ii, jj) = score;
        all_H_matrices[ii][jj] = H;

        // Update the best_matches matrix:
        int insert_target = jj;
        for (size_t kk = 0; kk < max_targets_per_frame; ++kk) {
          int idx = best_matches(ii, kk);
          // Note that -1 indicates an unfilled value.
          if (idx == -1 || scores(ii, idx) > scores(ii, insert_target)) {
            best_matches(ii, kk) = insert_target;
            insert_target = idx;
            if (idx == -1) {
              break;
            }
          }
        }
      }
    }

    if (camera_views.size() == 0) {
      AOS_LOG(DEBUG, "Unable to identify potential target matches.\n");
      // If we can't get a match, provide H = zero, which will make this
      // correction step a nop.
      *h = [](const State &, const Input &) { return Output::Zero(); };
      *dhdx = [](const State &) {
        return Eigen::Matrix<Scalar, kNOutputs, kNStates>::Zero();
      };
      for (size_t ii = 0; ii < target_views.size(); ++ii) {
        h_functions->push_back(*h);
        dhdx_functions->push_back(*dhdx);
      }
    } else {
      // Go through and brute force the issue of what the best combination of
      // target matches are. The worst case for this algorithm will be
      // max_targets_per_frame!, which is awful for any N > ~4, but since
      // max_targets_per_frame = 3, I'm not really worried.
      ::std::array<int, max_targets_per_frame> best_frames =
          MatchFrames(scores, best_matches, target_views.size());
      for (size_t ii = 0; ii < target_views.size(); ++ii) {
        size_t view_idx = best_frames[ii];
        if (view_idx < 0 || view_idx >= camera_views.size()) {
          AOS_LOG(ERROR, "Somehow, the view scorer failed.\n");
          h_functions->push_back(
              [](const State &, const Input &) { return Output::Zero(); });
          dhdx_functions->push_back([](const State &) {
            return Eigen::Matrix<Scalar, kNOutputs, kNStates>::Zero();
          });
          continue;
        }
        const Eigen::Matrix<Scalar, kNOutputs, kNStates> best_H =
            all_H_matrices[ii][view_idx];
        const TargetView best_view = camera_views[view_idx];
        const TargetView target_view = target_views[ii];
        const Scalar match_score = scores(ii, view_idx);
        if (match_score > kRejectionScore) {
          AOS_LOG(DEBUG,
                  "Rejecting target at (%f, %f, %f, %f) due to high score.\n",
                  target_view.reading.heading, target_view.reading.distance,
                  target_view.reading.skew, target_view.reading.height);
          h_functions->push_back(
              [](const State &, const Input &) { return Output::Zero(); });
          dhdx_functions->push_back([](const State &) {
            return Eigen::Matrix<Scalar, kNOutputs, kNStates>::Zero();
          });
        } else {
          h_functions->push_back([this, &camera, best_view, target_view](
                                     const State &X, const Input &) {
            // This function actually handles determining what the Output should
            // be at a given state, now that we have chosen the target that
            // we want to match to.
            *robot_pose_->mutable_pos() << X(0, 0), X(1, 0), 0.0;
            robot_pose_->set_theta(X(2, 0));
            const Pose relative_pose =
                best_view.target->pose().Rebase(&camera.pose());
            const Scalar heading = relative_pose.heading();
            const Scalar distance = relative_pose.xy_norm();
            const Scalar skew = ::aos::math::NormalizeAngle(
                relative_pose.rel_theta() - heading);
            return Output(heading, distance, skew);
          });

          // TODO(james): Experiment to better understand whether we want to
          // recalculate H or not.
          dhdx_functions->push_back(
              [best_H](const Eigen::Matrix<Scalar, kNStates, 1> &) {
                return best_H;
              });
        }
      }
      *h = h_functions->at(0);
      *dhdx = dhdx_functions->at(0);
    }
  }

  Eigen::Matrix<Scalar, kNOutputs, kNStates> HMatrix(
      const Target &target, const Pose &camera_pose) {
    // To calculate dheading/d{x,y,theta}:
    // heading = arctan2(target_pos - camera_pos) - camera_theta
    Eigen::Matrix<Scalar, 3, 1> target_pos = target.pose().abs_pos();
    Eigen::Matrix<Scalar, 3, 1> camera_pos = camera_pose.abs_pos();
    Scalar diffx = target_pos.x() - camera_pos.x();
    Scalar diffy = target_pos.y() - camera_pos.y();
    Scalar norm2 = diffx * diffx + diffy * diffy;
    Scalar dheadingdx = diffy / norm2;
    Scalar dheadingdy = -diffx / norm2;
    Scalar dheadingdtheta = -1.0;

    // To calculate ddistance/d{x,y}:
    // distance = sqrt(diffx^2 + diffy^2)
    Scalar distance = ::std::sqrt(norm2);
    Scalar ddistdx = -diffx / distance;
    Scalar ddistdy = -diffy / distance;

    // Skew = target.theta - camera.theta - heading
    //      = target.theta - arctan2(target_pos - camera_pos)
    Scalar dskewdx = -dheadingdx;
    Scalar dskewdy = -dheadingdy;
    Eigen::Matrix<Scalar, kNOutputs, kNStates> H;
    H.setZero();
    H(0, 0) = dheadingdx;
    H(0, 1) = dheadingdy;
    H(0, 2) = dheadingdtheta;
    H(1, 0) = ddistdx;
    H(1, 1) = ddistdy;
    H(2, 0) = dskewdx;
    H(2, 1) = dskewdy;
    return H;
  }

  // A helper function for the fuller version of MatchFrames; this just
  // removes some of the arguments that are only needed during the recursion.
  // n_views is the number of targets actually seen in the camera image (i.e.,
  // the number of rows in scores/best_matches that are actually populated).
  ::std::array<int, max_targets_per_frame> MatchFrames(
      const Eigen::Matrix<Scalar, max_targets_per_frame, num_targets> &scores,
      const Eigen::Matrix<int, max_targets_per_frame, max_targets_per_frame> &
          best_matches,
      int n_views) {
    ::std::array<int, max_targets_per_frame> best_set;
    best_set.fill(-1);
    Scalar best_score;
    // We start out without having "used" any views/targets:
    ::aos::SizedArray<bool, max_targets_per_frame> used_views;
    for (int ii = 0; ii < n_views; ++ii) {
      used_views.push_back(false);
    }
    MatchFrames(scores, best_matches, used_views, {{false}}, &best_set,
                &best_score);
    return best_set;
  }

  // Recursively iterates over every plausible combination of targets/views
  // that there is and determines the lowest-scoring combination.
  // used_views and used_targets indicate which rows/columns of the
  // scores/best_matches matrices should be ignored. When used_views is all
  // true, that means that we are done recursing.
  void MatchFrames(
      const Eigen::Matrix<Scalar, max_targets_per_frame, num_targets> &scores,
      const Eigen::Matrix<int, max_targets_per_frame, max_targets_per_frame> &
          best_matches,
      const ::aos::SizedArray<bool, max_targets_per_frame> &used_views,
      const ::std::array<bool, num_targets> &used_targets,
      ::std::array<int, max_targets_per_frame> *best_set, Scalar *best_score) {
    *best_score = ::std::numeric_limits<Scalar>::infinity();
    // Iterate by letting each target in the camera frame (that isn't in
    // used_views) choose it's best match that isn't already taken. We then set
    // the appropriate flags in used_views and used_targets and call MatchFrames
    // to let all the other views sort themselves out.
    for (size_t ii = 0; ii < used_views.size(); ++ii) {
      if (used_views[ii]) {
        continue;
      }
      int best_match = -1;
      for (size_t jj = 0; jj < max_targets_per_frame; ++jj) {
        if (best_matches(ii, jj) == -1) {
          // If we run out of potential targets from the camera, then there
          // are more targets in the frame than we think there should be.
          // In this case, we are going to be doubling/tripling/etc. up
          // anyhow. So we just give everyone their top choice:
          // TODO(james): If we ever are dealing with larger numbers of
          // targets per frame, do something to minimize doubling-up.
          best_match = best_matches(ii, 0);
          break;
        }
        best_match = best_matches(ii, jj);
        if (!used_targets[best_match]) {
          break;
        }
      }
      // If we reach here and best_match = -1, that means that no potential
      // targets were generated by the camera, and we should never have gotten
      // here.
      AOS_CHECK(best_match != -1);
      ::aos::SizedArray<bool, max_targets_per_frame> sub_views = used_views;
      sub_views[ii] = true;
      ::std::array<bool, num_targets> sub_targets = used_targets;
      sub_targets[best_match] = true;
      ::std::array<int, max_targets_per_frame> sub_best_set;
      Scalar score;
      MatchFrames(scores, best_matches, sub_views, sub_targets, &sub_best_set,
                  &score);
      score += scores(ii, best_match);
      sub_best_set[ii] = best_match;
      if (score < *best_score) {
        *best_score = score;
        *best_set = sub_best_set;
      }
    }
    // best_score will be infinite if we did not find a result due to there
    // being no targets that weren't set in used_vies; this is the
    // base case of the recursion and so we set best_score to zero:
    if (!::std::isfinite(*best_score)) {
      *best_score = 0.0;
    }
  }

  // The pose that is used by the cameras to determine the location of the robot
  // and thus the expected view of the targets.
  Pose *robot_pose_;
};  // class TypedLocalizer

}  // namespace control_loops
}  // namespace y2019

#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_LOCALIZATER_H_
