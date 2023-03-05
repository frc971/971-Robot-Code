from __future__ import print_function
import sys
import numpy as np
import y2023.control_loops.python.graph_paths as graph_paths


def index_function_name(name):
    return "%sIndex" % name


def path_function_name(name):
    return "Make%sPath" % name


def add_edge(cc_file, name, segment, index, reverse):
    segment.VerifyPoints()

    cc_file.append("  // Adding edge %d" % index)
    vmax = "vmax"
    if segment.vmax:
        vmax = "::std::min(vmax, %f)" % segment.vmax

    alpha_unitizer = "alpha_unitizer"
    if segment.alpha_unitizer is not None:
        alpha_unitizer = "(::Eigen::Matrix<double, 3, 3>() << %f, %f, %f, %f, %f, %f, %f, %f, %f).finished()" % (
            segment.alpha_unitizer[0, 0],
            segment.alpha_unitizer[0, 1],
            segment.alpha_unitizer[0, 2],
            segment.alpha_unitizer[1, 0],
            segment.alpha_unitizer[1, 1],
            segment.alpha_unitizer[1, 2],
            segment.alpha_unitizer[2, 0],
            segment.alpha_unitizer[2, 1],
            segment.alpha_unitizer[2, 2],
        )
    cc_file.append("  trajectories->emplace_back(%s," % (vmax))
    cc_file.append("                             %s," % (alpha_unitizer))
    if reverse:
        cc_file.append(
            "                             Trajectory(dynamics, &hybrid_roll_joint_loop->plant(), Path::Reversed(%s()), 0.005), "
            % (path_function_name(str(name))))
    else:
        cc_file.append(
            "                             Trajectory(dynamics, &hybrid_roll_joint_loop->plant(), %s(), 0.005), "
            % (path_function_name(str(name))))
    cc_file.append(f"\"{path_function_name(str(name))}\");")

    start_index = None
    end_index = None
    for key in sorted(graph_paths.points.keys()):
        point = graph_paths.points[key]
        if (point[:2] == segment.start
            ).all() and point[2] == segment.alpha_rolls[0][1]:
            start_index = key
        if (point[:2] == segment.end
            ).all() and point[2] == segment.alpha_rolls[-1][1]:
            end_index = key

    if reverse:
        start_index, end_index = end_index, start_index

    cc_file.append(
        "  edges.push_back(SearchGraph::Edge{%s(), %s()," %
        (index_function_name(start_index), index_function_name(end_index)))
    cc_file.append(
        "                     (trajectories->back().trajectory.path().length() + 0.2)});"
    )

    # TODO(austin): Allow different vmaxes for different paths.
    cc_file.append("  trajectories->back().trajectory.OptimizeTrajectory(")
    cc_file.append("      trajectories->back().alpha_unitizer,")
    cc_file.append("      trajectories->back().vmax);")
    cc_file.append("")


def main(argv):
    cc_file = []
    cc_file.append("#include <memory>")
    cc_file.append("")
    cc_file.append(
        "#include \"frc971/control_loops/double_jointed_arm/graph.h\"")
    cc_file.append(
        "#include \"y2023/control_loops/superstructure/arm/generated_graph.h\""
    )
    cc_file.append(
        "#include \"y2023/control_loops/superstructure/arm/trajectory.h\"")
    cc_file.append(
        "#include \"y2023/control_loops/superstructure/roll/integral_hybrid_roll_plant.h\""
    )
    cc_file.append("")

    cc_file.append("using frc971::control_loops::arm::SearchGraph;")
    cc_file.append(
        "using y2023::control_loops::superstructure::arm::Trajectory;")
    cc_file.append("using y2023::control_loops::superstructure::arm::Path;")
    cc_file.append("using y2023::control_loops::superstructure::arm::NSpline;")
    cc_file.append(
        "using y2023::control_loops::superstructure::arm::CosSpline;")

    cc_file.append("")
    cc_file.append("namespace y2023 {")
    cc_file.append("namespace control_loops {")
    cc_file.append("namespace superstructure {")
    cc_file.append("namespace arm {")

    h_file = []
    h_file.append(
        "#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")
    h_file.append(
        "#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")
    h_file.append("")
    h_file.append("#include <memory>")
    h_file.append("")
    h_file.append(
        "#include \"frc971/control_loops/double_jointed_arm/graph.h\"")
    h_file.append(
        "#include \"y2023/control_loops/superstructure/arm/arm_constants.h\"")
    h_file.append(
        "#include \"y2023/control_loops/superstructure/arm/trajectory.h\"")
    h_file.append(
        "#include \"y2023/control_loops/superstructure/arm/arm_trajectories_generated.h\""
    )

    h_file.append("")
    h_file.append("namespace y2023 {")
    h_file.append("namespace control_loops {")
    h_file.append("namespace superstructure {")
    h_file.append("namespace arm {")

    h_file.append("using frc971::control_loops::arm::SearchGraph;")
    h_file.append(
        "using y2023::control_loops::superstructure::arm::Trajectory;")
    h_file.append("using y2023::control_loops::superstructure::arm::Path;")
    h_file.append(
        "using y2023::control_loops::superstructure::arm::kArmConstants;")

    h_file.append(
        "using y2023::control_loops::superstructure::arm::TrajectoryAndParamsFbs;"
    )

    h_file.append("")
    h_file.append("struct TrajectoryAndParams {")
    h_file.append("  TrajectoryAndParams(double new_vmax,")
    h_file.append(
        "                      const ::Eigen::Matrix<double, 3, 3> &new_alpha_unitizer,"
    )
    h_file.append(
        "                      Trajectory &&new_trajectory, std::string_view new_name)"
    )
    h_file.append("      : vmax(new_vmax),")
    h_file.append("        alpha_unitizer(new_alpha_unitizer),")
    h_file.append("        trajectory(::std::move(new_trajectory)),")
    h_file.append("         name(new_name) {}")
    h_file.append(
        "TrajectoryAndParams(const frc971::control_loops::arm::Dynamics *dynamics, const StateFeedbackHybridPlant<3, 1, 1> *roll, const TrajectoryAndParamsFbs &trajectory_and_params_fbs)"
    )
    h_file.append(": vmax(trajectory_and_params_fbs.vmax()),")
    h_file.append(
        "alpha_unitizer((trajectory_and_params_fbs.alpha_unitizer()->data(),")
    h_file.append("     trajectory_and_params_fbs.alpha_unitizer()->data() +")
    h_file.append(
        "         trajectory_and_params_fbs.alpha_unitizer()->size())),")
    h_file.append(
        "trajectory(dynamics, roll, *trajectory_and_params_fbs.trajectory()),")
    h_file.append("name(trajectory_and_params_fbs.name()->string_view()) {}")
    h_file.append("  double vmax;")
    h_file.append("  ::Eigen::Matrix<double, 3, 3> alpha_unitizer;")
    h_file.append("  Trajectory trajectory;")
    h_file.append(" std::string_view name;")
    h_file.append("};")
    h_file.append("")

    # Now dump out the vertices and associated constexpr vertex name functions.
    for index, key in enumerate(sorted(graph_paths.points.keys())):
        point = graph_paths.points[key]
        h_file.append("")
        h_file.append("constexpr uint32_t %s() { return %d; }" %
                      (index_function_name(key), index))
        h_file.append("::Eigen::Matrix<double, 3, 1> %sPoint();" % key)
        cc_file.append("::Eigen::Matrix<double, 3, 1> %sPoint() {" % key)
        cc_file.append(
            "  return (::Eigen::Matrix<double, 3, 1>() << %f, %f, %f).finished();"
            % (point[0], point[1], point[2]))
        cc_file.append("}")

    front_points = [
        index_function_name(point[1]) + "()"
        for point in graph_paths.front_points
    ]
    h_file.append("")
    h_file.append("constexpr ::std::array<uint32_t, %d> FrontPoints() {" %
                  len(front_points))
    h_file.append("  return ::std::array<uint32_t, %d>{{%s}};" %
                  (len(front_points), ", ".join(front_points)))
    h_file.append("}")

    back_points = [
        index_function_name(point[1]) + "()"
        for point in graph_paths.back_points
    ]
    h_file.append("")
    h_file.append("constexpr ::std::array<uint32_t, %d> BackPoints() {" %
                  len(back_points))
    h_file.append("  return ::std::array<uint32_t, %d>{{%s}};" %
                  (len(back_points), ", ".join(back_points)))
    h_file.append("}")

    # Add the Make*Path functions.
    h_file.append("")
    cc_file.append("")
    for name, segment in list(enumerate(graph_paths.unnamed_segments)) + [
        (x.name, x) for x in graph_paths.named_segments
    ]:
        h_file.append("::std::unique_ptr<Path> %s();" %
                      path_function_name(name))
        cc_file.append("::std::unique_ptr<Path> %s() {" %
                       path_function_name(name))
        cc_file.append("  return ::std::unique_ptr<Path>(new Path(CosSpline{")
        cc_file.append("      NSpline<4, 2>((Eigen::Matrix<double, 2, 4>() <<")
        points = [
            segment.start, segment.control1, segment.control2, segment.end
        ]
        cc_file.append("             " +
                       " ".join(["%.12f," % (p[0]) for p in points]))
        cc_file.append("             " +
                       ", ".join(["%.12f" % (p[1]) for p in points]))

        cc_file.append("      ).finished()), {")
        for alpha, roll in segment.alpha_rolls:
            cc_file.append(
                "       CosSpline::AlphaTheta{.alpha = %.12f, .theta = %.12f},"
                % (alpha, roll))
        cc_file.append("  }}));")
        cc_file.append("}")
        cc_file.append("")

    # Matrix of nodes
    h_file.append("::std::vector<::Eigen::Matrix<double, 3, 1>> PointList();")

    cc_file.append(
        "::std::vector<::Eigen::Matrix<double, 3, 1>> PointList() {")
    cc_file.append("  ::std::vector<::Eigen::Matrix<double, 3, 1>> points;")
    for key in sorted(graph_paths.points.keys()):
        point = graph_paths.points[key]
        cc_file.append(
            "  points.push_back((::Eigen::Matrix<double, 3, 1>() << %.12s, %.12s, %.12s).finished());"
            % (point[0], point[1], point[2]))
    cc_file.append("  return points;")
    cc_file.append("}")

    # Now create the MakeSearchGraph function.
    h_file.append("")
    h_file.append("// Builds a search graph.")
    h_file.append("SearchGraph MakeSearchGraph("
                  "const frc971::control_loops::arm::Dynamics *dynamics, "
                  "::std::vector<TrajectoryAndParams> *trajectories,")
    h_file.append(
        "                            const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer,"
    )
    h_file.append("                            double vmax,")
    h_file.append(
        "const StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>, HybridKalman<3, 1, 1>> *hybrid_roll_joint_loop);"
    )
    cc_file.append("")
    cc_file.append("SearchGraph MakeSearchGraph(")
    cc_file.append("    const frc971::control_loops::arm::Dynamics *dynamics,")
    cc_file.append("    std::vector<TrajectoryAndParams> *trajectories,")
    cc_file.append(
        "    const ::Eigen::Matrix<double, 3, 3> &alpha_unitizer, double vmax,"
    )
    cc_file.append(
        "    const StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,"
    )
    cc_file.append(
        "                            HybridKalman<3, 1, 1>> *hybrid_roll_joint_loop) {"
    )
    cc_file.append("  ::std::vector<SearchGraph::Edge> edges;")

    index = 0
    segments_and_names = list(enumerate(graph_paths.unnamed_segments)) + [
        (x.name, x) for x in graph_paths.named_segments
    ]

    for name, segment in segments_and_names:
        add_edge(cc_file, name, segment, index, False)
        index += 1
        add_edge(cc_file, name, segment, index, True)
        index += 1

    cc_file.append("  return SearchGraph(%d, ::std::move(edges));" %
                   len(graph_paths.points))
    cc_file.append("}")

    h_file.append("")
    h_file.append("}  // namespace arm")
    h_file.append("}  // namespace superstructure")
    h_file.append("}  // namespace control_loops")
    h_file.append("}  // namespace y2023")
    h_file.append("")
    h_file.append(
        "#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")

    cc_file.append("}  // namespace arm")
    cc_file.append("}  // namespace superstructure")
    cc_file.append("}  // namespace control_loops")
    cc_file.append("}  // namespace y2023")

    if len(argv) == 3:
        with open(argv[1], "w") as hfd:
            hfd.write("\n".join(h_file))

        with open(argv[2], "w") as ccfd:
            ccfd.write("\n".join(cc_file))
    else:
        print("\n".join(h_file))
        print("\n".join(cc_file))


if __name__ == '__main__':
    main(sys.argv)
