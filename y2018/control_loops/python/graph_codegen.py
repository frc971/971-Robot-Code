from __future__ import print_function
import sys
import numpy
import graph_generate


def index_function_name(name):
    return "%sIndex" % name


def path_function_name(name):
    return "Make%sPath" % name


def add_edge(cc_file, name, segment, index, reverse):
    cc_file.append("  // Adding edge %d" % index)
    vmax = "vmax"
    if segment.vmax:
        vmax = "::std::min(vmax, %f)" % segment.vmax

    alpha_unitizer = "alpha_unitizer"
    if segment.alpha_unitizer is not None:
        alpha_unitizer = "(::Eigen::Matrix<double, 2, 2>() << %f, %f, %f, %f).finished()" % (
            segment.alpha_unitizer[0, 0], segment.alpha_unitizer[0, 1],
            segment.alpha_unitizer[1, 0], segment.alpha_unitizer[1, 1])
    cc_file.append( "  trajectories->emplace_back(%s," % (vmax))
    cc_file.append( "                             %s," % (alpha_unitizer))
    if reverse:
        cc_file.append(
            "                             Trajectory(Path::Reversed(%s()), 0.005));"
            % (path_function_name(str(name))))
    else:
        cc_file.append(
            "                             Trajectory(%s(), 0.005));"
            % (path_function_name(str(name))))

    start_index = None
    end_index = None
    for point, name in graph_generate.points:
        if (point == segment.start).all():
            start_index = name
        if (point == segment.end).all():
            end_index = name

    if reverse:
        start_index, end_index = end_index, start_index

    cc_file.append("  edges.push_back(SearchGraph::Edge{%s(), %s()," %
                   (index_function_name(start_index),
                    index_function_name(end_index)))
    cc_file.append(
        "                     (trajectories->back().trajectory.path().length() + 0.2)});")

    # TODO(austin): Allow different vmaxes for different paths.
    cc_file.append(
        "  trajectories->back().trajectory.OptimizeTrajectory(")
    cc_file.append("      trajectories->back().alpha_unitizer,")
    cc_file.append("      trajectories->back().vmax);")
    cc_file.append("")


def main(argv):
    cc_file = []
    cc_file.append(
        "#include \"y2018/control_loops/superstructure/arm/generated_graph.h\""
    )
    cc_file.append("")
    cc_file.append("#include <memory>")
    cc_file.append("")
    cc_file.append(
        "#include \"y2018/control_loops/superstructure/arm/trajectory.h\"")
    cc_file.append(
        "#include \"y2018/control_loops/superstructure/arm/graph.h\"")
    cc_file.append("")
    cc_file.append("namespace y2018 {")
    cc_file.append("namespace control_loops {")
    cc_file.append("namespace superstructure {")
    cc_file.append("namespace arm {")

    h_file = []
    h_file.append(
        "#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")
    h_file.append(
        "#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")
    h_file.append("")
    h_file.append("#include <memory>")
    h_file.append("")
    h_file.append(
        "#include \"y2018/control_loops/superstructure/arm/trajectory.h\"")
    h_file.append(
        "#include \"y2018/control_loops/superstructure/arm/graph.h\"")
    h_file.append("")
    h_file.append("namespace y2018 {")
    h_file.append("namespace control_loops {")
    h_file.append("namespace superstructure {")
    h_file.append("namespace arm {")

    h_file.append("")
    h_file.append("struct TrajectoryAndParams {")
    h_file.append("  TrajectoryAndParams(double new_vmax,")
    h_file.append(
        "                      const ::Eigen::Matrix<double, 2, 2> &new_alpha_unitizer,"
    )
    h_file.append("                      Trajectory &&new_trajectory)")
    h_file.append("      : vmax(new_vmax),")
    h_file.append("        alpha_unitizer(new_alpha_unitizer),")
    h_file.append("        trajectory(::std::move(new_trajectory)) {}")
    h_file.append("  double vmax;")
    h_file.append("  ::Eigen::Matrix<double, 2, 2> alpha_unitizer;")
    h_file.append("  Trajectory trajectory;")
    h_file.append("};")
    h_file.append("")

    # Now dump out the vertices and associated constexpr vertex name functions.
    for index, point in enumerate(graph_generate.points):
        h_file.append("")
        h_file.append("constexpr uint32_t %s() { return %d; }" %
                      (index_function_name(point[1]), index))
        h_file.append(
            "inline ::Eigen::Matrix<double, 2, 1> %sPoint() {" % point[1])
        h_file.append(
            "  return (::Eigen::Matrix<double, 2, 1>() << %f, %f).finished();"
            % (numpy.pi / 2.0 - point[0][0], numpy.pi / 2.0 - point[0][1]))
        h_file.append("}")

    front_points = [
        index_function_name(point[1]) + "()" for point in graph_generate.front_points
    ]
    h_file.append("")
    h_file.append("constexpr ::std::array<uint32_t, %d> FrontPoints() {" %
                  len(front_points))
    h_file.append("  return ::std::array<uint32_t, %d>{{%s}};" %
                  (len(front_points), ", ".join(front_points)))
    h_file.append("}")

    back_points = [
        index_function_name(point[1]) + "()" for point in graph_generate.back_points
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
    for name, segment in list(enumerate(graph_generate.unnamed_segments)) + [
        (x.name, x) for x in graph_generate.named_segments
    ]:
        h_file.append(
            "::std::unique_ptr<Path> %s();" % path_function_name(name))
        cc_file.append(
            "::std::unique_ptr<Path> %s() {" % path_function_name(name))
        cc_file.append("  return ::std::unique_ptr<Path>(new Path({")
        for point in segment.ToThetaPoints():
            cc_file.append("      {{%.12f, %.12f, %.12f," %
                           (numpy.pi / 2.0 - point[0],
                            numpy.pi / 2.0 - point[1], -point[2]))
            cc_file.append("        %.12f, %.12f, %.12f}}," %
                           (-point[3], -point[4], -point[5]))
        cc_file.append("  }));")
        cc_file.append("}")

    # Matrix of nodes
    h_file.append("::std::vector<::Eigen::Matrix<double, 2, 1>> PointList();")

    cc_file.append(
        "::std::vector<::Eigen::Matrix<double, 2, 1>> PointList() {")
    cc_file.append("  ::std::vector<::Eigen::Matrix<double, 2, 1>> points;")
    for point in graph_generate.points:
        cc_file.append(
            "  points.push_back((::Eigen::Matrix<double, 2, 1>() << %.12s, %.12s).finished());"
            % (numpy.pi / 2.0 - point[0][0], numpy.pi / 2.0 - point[0][1]))
    cc_file.append("  return points;")
    cc_file.append("}")

    # Now create the MakeSearchGraph function.
    h_file.append("")
    h_file.append("// Builds a search graph.")
    h_file.append("SearchGraph MakeSearchGraph("
                  "::std::vector<TrajectoryAndParams> *trajectories,")
    h_file.append("                            "
                  "const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer,")
    h_file.append("                            double vmax);")
    cc_file.append("SearchGraph MakeSearchGraph("
                   "::std::vector<TrajectoryAndParams> *trajectories,")
    cc_file.append("                            "
                   "const ::Eigen::Matrix<double, 2, 2> &alpha_unitizer,")
    cc_file.append("                            " "double vmax) {")
    cc_file.append("  ::std::vector<SearchGraph::Edge> edges;")

    index = 0
    segments_and_names = list(enumerate(graph_generate.unnamed_segments)) + [
        (x.name, x) for x in graph_generate.named_segments
    ]

    for name, segment in segments_and_names:
        add_edge(cc_file, name, segment, index, False)
        index += 1
        add_edge(cc_file, name, segment, index, True)
        index += 1

    cc_file.append("  return SearchGraph(%d, ::std::move(edges));" % len(
        graph_generate.points))
    cc_file.append("}")

    h_file.append("")
    h_file.append("}  // namespace arm")
    h_file.append("}  // namespace superstructure")
    h_file.append("}  // namespace control_loops")
    h_file.append("}  // namespace y2018")
    h_file.append("")
    h_file.append(
        "#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_ARM_GENERATED_GRAPH_H_")

    cc_file.append("}  // namespace arm")
    cc_file.append("}  // namespace superstructure")
    cc_file.append("}  // namespace control_loops")
    cc_file.append("}  // namespace y2018")

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
