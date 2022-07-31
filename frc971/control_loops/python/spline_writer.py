import os
import numpy as np


class SplineWriter(object):

    def __init__(self, namespaces=None, filename="auto_splines.cc"):
        if namespaces:
            self._namespaces = namespaces
        else:
            self._namespaces = ['frc971', 'autonomous']

        self._namespace_start = '\n'.join(
            ['namespace %s {' % name for name in self._namespaces])

        self._namespace_end = '\n'.join([
            '}  // namespace %s' % name for name in reversed(self._namespaces)
        ])

        self.filename_ = filename

    def make_string_list(self, list):
        string = "{{"
        for i in range(0, len(list)):
            if i == len(list) - 1:
                string += str(list[i])
            else:
                string += str(list[i]) + ", "
        return string + "}}"

    def Write(self, spline_name, spline_idx, control_points):
        """Writes the cc file to the file named cc_file."""
        xs = control_points[:, 0]
        ys = control_points[:, 1]
        spline_count = (len(xs) - 6) / 5 + 1
        with open(self.filename_, 'a') as fd:
            fd.write(self._namespace_start)
            #write the name
            fd.write("\n\n::frc971::MultiSpline " + spline_name + "() {\n")
            # write the objs, at the moment assumes a single constraint, needs fixing
            fd.write(
                "\t::frc971::MultiSpline spline;\n\t::frc971::Constraint constraints;\n"
            )
            fd.write(
                "\tconstraints.constraint_type = 0;\n\tconstraints.value = 0;\n"
            )
            fd.write(
                "\tconstraints.start_distance = 0;\n\tconstraints.end_distance = 0;\n"
            )
            fd.write('\n')
            fd.write("\tspline.spline_idx = " + str(spline_idx) +
                     ";\n\tspline.spline_count = " + str(spline_count) + ";\n")
            fd.write("\tspline.spline_x = " + self.make_string_list(xs) +
                     ";\n\tspline.spline_y = " + self.make_string_list(ys) +
                     ";\n")
            fd.write(
                "\tspline.constraints = {{constraints}};\n\treturn spline;\n")
            fd.write("}")
            fd.write("\n\n")
            fd.write(self._namespace_end)
            fd.write("\n\n")


def main():
    writer = SplineWriter()
    points = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0],
                       [9.0, 10.0], [11.0, 12.0]])
    spline_name = "test_spline"
    spline_idx = 1
    writer.Write(spline_name, spline_idx, points)


main()
