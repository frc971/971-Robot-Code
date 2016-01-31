#ifndef AOS_COMMON_CONTROLS_POLYTOPE_H_
#define AOS_COMMON_CONTROLS_POLYTOPE_H_

#include "Eigen/Dense"
#include "third_party/cddlib/lib-src/setoper.h"
#include "third_party/cddlib/lib-src/cdd.h"

#include "aos/common/logging/logging.h"
#include "aos/common/logging/matrix_logging.h"

namespace aos {
namespace controls {

// A number_of_dimensions dimensional polytope.
// This represents the region consisting of all points X such that H * X <= k.
// The vertices are calculated at construction time because we always use those
// and libcdd is annoying about calculating vertices. In particular, for some
// random-seeming polytopes it refuses to calculate the vertices completely. To
// avoid issues with that, using the "shifting" constructor is recommended
// whenever possible.
template <int number_of_dimensions>
class HPolytope {
 public:
  // Constructs a polytope given the H and k matrices.
  // Do NOT use this to calculate a polytope derived from another one in a way
  // another constructor can be used instead.
  HPolytope(
      const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> &H,
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &k)
      : H_(H), k_(k), vertices_(CalculateVertices(H, k)) {}

  // Constructs a polytope with H = other.H() * H_multiplier and
  // k = other.k() + other.H() * k_shifter.
  // This avoids calculating the vertices for the new H and k directly because
  // that sometimes confuses libcdd depending on what exactly the new H and k
  // are.
  //
  // This currently has the following restrictions (CHECKed internally) because
  // we don't have uses for anything else:
  //   If number_of_dimensions is not 1, it must be 2, other.H() *
  //   H_multiplier.inverse() * k_shifter must have its first 2 columns and last
  //   2 columns as opposites, and the 1st+2nd and 3rd+4th vertices must be
  //   normal to each other.
  HPolytope(const HPolytope<number_of_dimensions> &other,
            const Eigen::Matrix<double, number_of_dimensions,
                                number_of_dimensions> &H_multiplier,
            const Eigen::Matrix<double, number_of_dimensions, 1> &k_shifter)
      : H_(other.H() * H_multiplier),
        k_(other.k() + other.H() * k_shifter),
        vertices_(
            ShiftVertices(CalculateVertices(H_, other.k()),
                          other.H() * H_multiplier.inverse() * k_shifter)) {}

  // This is an initialization function shared across all instantiations of this
  // template.
  // This must be called at least once before creating any instances. It is
  // not thread-safe.
  static void Init() {
    dd_set_global_constants();
  }

  // Returns a reference to H.
  const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> &H() const {
    return H_;
  }

  // Returns a reference to k.
  const Eigen::Matrix<double, Eigen::Dynamic, 1> &k() const { return k_; }

  // Returns the number of dimensions in the polytope.
  int ndim() const { return number_of_dimensions; }

  // Returns the number of constraints currently in the polytope.
  int num_constraints() const { return k_.rows(); }

  // Returns true if the point is inside the polytope.
  bool IsInside(Eigen::Matrix<double, number_of_dimensions, 1> point) const;

  // Returns the list of vertices inside the polytope.
  const Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> &Vertices()
      const {
    return vertices_;
  }

 private:
  static Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic>
  CalculateVertices(
      const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> &H,
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &k);

  static Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic>
  ShiftVertices(const Eigen::Matrix<double, number_of_dimensions,
                                    Eigen::Dynamic> &vertices,
                const Eigen::Matrix<double, Eigen::Dynamic, 1> &shift) {
    static_assert(number_of_dimensions <= 2, "not implemented yet");
    if (vertices.cols() != number_of_dimensions * 2) {
      LOG(FATAL,
          "less vertices not supported yet: %zd vertices vs %d dimensions\n",
          vertices.cols(), number_of_dimensions);
    }
    if (number_of_dimensions == 2) {
      if ((shift.row(0) + shift.row(1)).norm() > 0.0001) {
        LOG_MATRIX(FATAL, "bad shift amount", shift.row(0) + shift.row(1));
      }
      if ((shift.row(2) + shift.row(3)).norm() > 0.0001) {
        LOG_MATRIX(FATAL, "bad shift amount", shift.row(2) + shift.row(3));
      }
      if (vertices.col(0).dot(vertices.col(1)) > 0.0001) {
        ::Eigen::Matrix<double, number_of_dimensions, 2> bad;
        bad.col(0) = vertices.col(0);
        bad.col(1) = vertices.col(1);
        LOG_MATRIX(FATAL, "bad vertices", bad);
      }
      if (vertices.col(2).dot(vertices.col(3)) > 0.0001) {
        ::Eigen::Matrix<double, number_of_dimensions, 2> bad;
        bad.col(0) = vertices.col(2);
        bad.col(1) = vertices.col(3);
        LOG_MATRIX(FATAL, "bad vertices", bad);
      }
    }

    Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> r(
        number_of_dimensions, vertices.cols());
    Eigen::Matrix<double, number_of_dimensions, 1> real_shift;
    real_shift(0, 0) = shift(0, 0);
    if (number_of_dimensions == 2) real_shift(1, 0) = shift(2, 0);
    for (int i = 0; i < vertices.cols(); ++i) {
      r.col(i) = vertices.col(i) + real_shift;
    }
    return r;
  }

  const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> H_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> k_;

  const Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> vertices_;
};

template <int number_of_dimensions>
bool HPolytope<number_of_dimensions>::IsInside(
    Eigen::Matrix<double, number_of_dimensions, 1> point) const {
  auto ev = H_ * point;
  for (int i = 0; i < num_constraints(); ++i) {
    if (ev(i, 0) > k_(i, 0)) {
      return false;
    }
  }
  return true;
}

template <int number_of_dimensions>
Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic>
HPolytope<number_of_dimensions>::CalculateVertices(
    const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> &H,
    const Eigen::Matrix<double, Eigen::Dynamic, 1> &k) {
  dd_MatrixPtr matrix = dd_CreateMatrix(k.rows(), number_of_dimensions + 1);

  // Copy the data over. TODO(aschuh): Is there a better way?  I hate copying...
  for (int i = 0; i < k.rows(); ++i) {
    dd_set_d(matrix->matrix[i][0], k(i, 0));
    for (int j = 0; j < number_of_dimensions; ++j) {
      dd_set_d(matrix->matrix[i][j + 1], -H(i, j));
    }
  }

  matrix->representation = dd_Inequality;
  matrix->numbtype = dd_Real;

  dd_ErrorType error;
  dd_PolyhedraPtr polyhedra = dd_DDMatrix2Poly(matrix, &error);
  if (error != dd_NoError || polyhedra == NULL) {
    dd_WriteErrorMessages(stderr, error);
    dd_FreeMatrix(matrix);
    LOG_MATRIX(ERROR, "bad H", H);
    LOG_MATRIX(ERROR, "bad k_", k);
    LOG(FATAL, "dd_DDMatrix2Poly failed\n");
  }

  dd_MatrixPtr vertex_matrix = dd_CopyGenerators(polyhedra);

  int num_vertices = 0;
  int num_rays = 0;
  for (int i = 0; i < vertex_matrix->rowsize; ++i) {
    if (dd_get_d(vertex_matrix->matrix[i][0]) == 0) {
      num_rays += 1;
    } else {
      num_vertices += 1;
    }
  }

  // Rays are unsupported right now.  This may change in the future.
  CHECK_EQ(0, num_rays);

  Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> vertices(
      number_of_dimensions, num_vertices);

  int vertex_index = 0;
  for (int i = 0; i < vertex_matrix->rowsize; ++i) {
    if (dd_get_d(vertex_matrix->matrix[i][0]) != 0) {
      for (int j = 0; j < number_of_dimensions; ++j) {
        vertices(j, vertex_index) = dd_get_d(vertex_matrix->matrix[i][j + 1]);
      }
      ++vertex_index;
    }
  }
  dd_FreeMatrix(vertex_matrix);
  dd_FreePolyhedra(polyhedra);
  dd_FreeMatrix(matrix);

  return vertices;
}

}  // namespace controls
}  // namespace aos

#endif  // AOS_COMMON_CONTROLS_POLYTOPE_H_
