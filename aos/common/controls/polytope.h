#ifndef AOS_COMMON_CONTROLS_POLYTOPE_H_
#define AOS_COMMON_CONTROLS_POLYTOPE_H_

#include "Eigen/Dense"
#include "third_party/cddlib/lib-src/setoper.h"
#include "third_party/cddlib/lib-src/cdd.h"

namespace aos {
namespace controls {

// A n dimension polytope.
template <int number_of_dimensions>
class HPolytope {
 public:
  // Constructs a polytope given the H and k matricies.
  HPolytope(Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> H,
            Eigen::Matrix<double, Eigen::Dynamic, 1> k)
      : H_(H),
        k_(k) {
  }

  // This is an initialization function shared across all instantiations of this
  // template.
  // This must be called at least once before calling any of the methods. It is
  // not thread-safe.
  static void Init() {
    dd_set_global_constants();
  }

  // Returns a reference to H.
  const Eigen::Matrix<double, Eigen::Dynamic,
                      number_of_dimensions> &H() const {
    return H_;
  }

  // Returns a reference to k.
  const Eigen::Matrix<double, Eigen::Dynamic,
                      1> &k() const {
    return k_;
  }

  // Returns the number of dimensions in the polytope.
  int ndim() const { return number_of_dimensions; }

  // Returns the number of constraints currently in the polytope.
  int num_constraints() const { return k_.rows(); }

  // Returns true if the point is inside the polytope.
  bool IsInside(Eigen::Matrix<double, number_of_dimensions, 1> point) const;

  // Returns the list of vertices inside the polytope.
  Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> Vertices() const;

 private:
  Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> H_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> k_;
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
    HPolytope<number_of_dimensions>::Vertices() const {
  dd_MatrixPtr matrix = dd_CreateMatrix(num_constraints(), ndim() + 1);

  // Copy the data over. TODO(aschuh): Is there a better way?  I hate copying...
  for (int i = 0; i < num_constraints(); ++i) {
    dd_set_d(matrix->matrix[i][0], k_(i, 0));
    for (int j = 0; j < ndim(); ++j) {
      dd_set_d(matrix->matrix[i][j + 1], -H_(i, j));
    }
  }

  matrix->representation = dd_Inequality;
  matrix->numbtype = dd_Real;

  dd_ErrorType error;
  dd_PolyhedraPtr polyhedra = dd_DDMatrix2Poly(matrix, &error);
  if (error != dd_NoError || polyhedra == NULL) {
    dd_WriteErrorMessages(stderr, error);
    dd_FreeMatrix(matrix);
    Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> ans(0, 0);
    return ans;
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
