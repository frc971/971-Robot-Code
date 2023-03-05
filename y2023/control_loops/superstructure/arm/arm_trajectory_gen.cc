#include <iostream>
#include <memory>

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/double_jointed_arm/graph.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "y2023/constants.h"
#include "y2023/control_loops/superstructure/arm/arm_constants.h"
#include "y2023/control_loops/superstructure/arm/arm_trajectories_generated.h"
#include "y2023/control_loops/superstructure/arm/generated_graph.h"
#include "y2023/control_loops/superstructure/arm/trajectory.h"
#include "y2023/control_loops/superstructure/roll/integral_hybrid_roll_plant.h"

using frc971::control_loops::arm::SearchGraph;
using y2023::control_loops::superstructure::arm::AlphaThetaFbs;
using y2023::control_loops::superstructure::arm::CosSpline;
using y2023::control_loops::superstructure::arm::kArmConstants;
using y2023::control_loops::superstructure::arm::NSpline;
using y2023::control_loops::superstructure::arm::Path;
using y2023::control_loops::superstructure::arm::Trajectory;

DEFINE_string(output, "", "Defines the location of the output file");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  const frc971::control_loops::arm::Dynamics dynamics(kArmConstants);
  std::vector<y2023::control_loops::superstructure::arm::TrajectoryAndParams>
      trajectories;
  Eigen::DiagonalMatrix<double, 3> alpha_unitizer(
      (::Eigen::DiagonalMatrix<double, 3>().diagonal()
           << (1.0 / y2023::constants::Values::kArmAlpha0Max()),
       (1.0 / y2023::constants::Values::kArmAlpha1Max()),
       (1.0 / y2023::constants::Values::kArmAlpha2Max()))
          .finished());
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                    HybridKalman<3, 1, 1>>
      hybrid_roll_joint_loop = y2023::control_loops::superstructure::roll::
          MakeIntegralHybridRollLoop();

  // Optimizes paths
  auto search_graph =
      y2023::control_loops::superstructure::arm::MakeSearchGraph(
          &dynamics, &trajectories, alpha_unitizer,
          y2023::constants::Values::kArmVMax(), &hybrid_roll_joint_loop);

  auto edges = search_graph.edges();

  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  std::vector<flatbuffers::Offset<
      y2023::control_loops::superstructure::arm::TrajectoryAndParamsFbs>>
      trajectory_and_params_offsets;

  std::vector<y2023::control_loops::superstructure::arm::EdgeFbs> fbs_edges;

  // Generating flatbuffer

  for (const auto &trajectory : trajectories) {
    auto nspline = trajectory.trajectory.path().spline().spline();
    auto cos_spline = trajectory.trajectory.path().spline();
    auto path = trajectory.trajectory.path();

    auto control_points_offset = fbb.CreateVector(
        nspline.control_points().data(), nspline.control_points().size());

    std::vector<AlphaThetaFbs> roll;

    for (const auto &alpha_theta : cos_spline.roll()) {
      roll.emplace_back(alpha_theta.alpha, alpha_theta.theta);
    }
    auto cos_spline_roll = fbb.CreateVectorOfStructs(roll);

    auto path_distances =
        fbb.CreateVector(path.distances().data(), path.distances().size());

    auto trajectory_max_dvelocity_unfiltered = fbb.CreateVector(
        trajectory.trajectory.max_dvelocity_unfiltered().data(),
        trajectory.trajectory.max_dvelocity_unfiltered().size());

    auto trajectory_max_dvelocity_backward_accel = fbb.CreateVector(
        trajectory.trajectory.max_dvelocity_backward_accel().data(),
        trajectory.trajectory.max_dvelocity_backward_accel().size());

    auto trajectory_max_dvelocity_forwards_accel = fbb.CreateVector(
        trajectory.trajectory.max_dvelocity_forwards_accel().data(),
        trajectory.trajectory.max_dvelocity_forwards_accel().size());

    auto trajectory_max_dvelocity_backward_voltage = fbb.CreateVector(
        trajectory.trajectory.max_dvelocity_backward_voltage().data(),
        trajectory.trajectory.max_dvelocity_backward_voltage().size());

    auto trajectory_max_dvelocity_forwards_voltage = fbb.CreateVector(
        trajectory.trajectory.max_dvelocity_forwards_voltage().data(),
        trajectory.trajectory.max_dvelocity_forwards_voltage().size());

    auto trajectory_alpha_unitizer =
        fbb.CreateVector(trajectory.trajectory.alpha_unitizer().data(),
                         trajectory.trajectory.alpha_unitizer().size());

    auto trajectory_and_params_name = fbb.CreateString(trajectory.name);

    auto trajectory_and_params_alpha_unitizer = fbb.CreateVector(
        trajectory.alpha_unitizer.data(), trajectory.alpha_unitizer.size());

    y2023::control_loops::superstructure::arm::SplineFbs::Builder
        spline_builder(fbb);

    spline_builder.add_control_points(control_points_offset);

    auto spline_offset = spline_builder.Finish();

    y2023::control_loops::superstructure::arm::CosSplineFbs::Builder
        cos_spline_builder(fbb);

    cos_spline_builder.add_spline(spline_offset);

    cos_spline_builder.add_roll(cos_spline_roll);

    auto cos_spline_offset = cos_spline_builder.Finish();

    y2023::control_loops::superstructure::arm::PathFbs::Builder path_builder(
        fbb);

    path_builder.add_spline(cos_spline_offset);
    path_builder.add_distances(path_distances);

    auto path_offset = path_builder.Finish();

    y2023::control_loops::superstructure::arm::TrajectoryFbs::Builder
        trajectory_builder(fbb);

    trajectory_builder.add_path(path_offset);
    trajectory_builder.add_num_plan_points(
        trajectory.trajectory.num_plan_points());
    trajectory_builder.add_step_size(trajectory.trajectory.step_size());
    trajectory_builder.add_max_dvelocity_unfiltered(
        trajectory_max_dvelocity_unfiltered);
    trajectory_builder.add_max_dvelocity_backward_accel(
        trajectory_max_dvelocity_backward_accel);
    trajectory_builder.add_max_dvelocity_forwards_accel(
        trajectory_max_dvelocity_forwards_accel);
    trajectory_builder.add_max_dvelocity_backward_voltage(
        trajectory_max_dvelocity_backward_voltage);
    trajectory_builder.add_max_dvelocity_forwards_voltage(
        trajectory_max_dvelocity_forwards_voltage);
    trajectory_builder.add_alpha_unitizer(trajectory_alpha_unitizer);

    auto trajectory_offset = trajectory_builder.Finish();

    y2023::control_loops::superstructure::arm::TrajectoryAndParamsFbs::Builder
        trajectory_and_params_builder(fbb);

    trajectory_and_params_builder.add_vmax(trajectory.vmax);
    trajectory_and_params_builder.add_alpha_unitizer(
        trajectory_and_params_alpha_unitizer);
    trajectory_and_params_builder.add_trajectory(trajectory_offset);
    trajectory_and_params_builder.add_name(trajectory_and_params_name);

    trajectory_and_params_offsets.emplace_back(
        trajectory_and_params_builder.Finish());
  }

  for (const auto &edge : edges) {
    fbs_edges.emplace_back(edge.start, edge.end, edge.cost);
  }

  auto arm_trajectories_trajctory_and_params_offsets =
      fbb.CreateVector(trajectory_and_params_offsets.data(),
                       trajectory_and_params_offsets.size());

  auto arm_trajectories_edges = fbb.CreateVectorOfStructs(fbs_edges);

  y2023::control_loops::superstructure::arm::ArmTrajectories::Builder
      arm_trajectories_builder(fbb);

  arm_trajectories_builder.add_trajectories(
      arm_trajectories_trajctory_and_params_offsets);
  arm_trajectories_builder.add_edges(arm_trajectories_edges);

  auto arm_trajectories_offset = arm_trajectories_builder.Finish();

  fbb.Finish(arm_trajectories_offset);

  auto detatched_buffer = aos::FlatbufferDetachedBuffer<
      y2023::control_loops::superstructure::arm::ArmTrajectories>(
      fbb.Release());

  // This writes to a binary file so we can cache the optimization results
  aos::WriteFlatbufferToFile(FLAGS_output, detatched_buffer);
}
