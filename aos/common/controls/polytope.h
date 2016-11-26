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
class Polytope {
 public:
  virtual ~Polytope() {}

  // Returns a reference to H.
  virtual Eigen::Ref<
      const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions>>
  H() const = 0;

  // Returns a reference to k.
  virtual Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> k()
      const = 0;

  // Returns the number of dimensions in the polytope.
  constexpr int ndim() const { return number_of_dimensions; }

  // Returns the number of constraints currently in the polytope.
  int num_constraints() const { return k().rows(); }

  // Returns true if the point is inside the polytope.
  bool IsInside(Eigen::Matrix<double, number_of_dimensions, 1> point) const;

  // Returns the list of vertices inside the polytope.
  virtual Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> Vertices()
      const = 0;
};

template <int number_of_dimensions, int num_vertices>
Eigen::Matrix<double, number_of_dimensions, num_vertices> ShiftPoints(
    const Eigen::Matrix<double, number_of_dimensions, num_vertices> &vertices,
    const Eigen::Matrix<double, number_of_dimensions, 1> &offset) {
  Eigen::Matrix<double, number_of_dimensions, num_vertices> ans = vertices;
  for (int i = 0; i < num_vertices; ++i) {
    ans.col(i) += offset;
  }
  return ans;
}

template <int number_of_dimensions, int num_constraints, int num_vertices>
class HVPolytope : public Polytope<number_of_dimensions> {
 public:
  // Constructs a polytope given the H and k matrices.
  HVPolytope(Eigen::Ref<const Eigen::Matrix<double, num_constraints,
                                            number_of_dimensions>> H,
             Eigen::Ref<const Eigen::Matrix<double, num_constraints, 1>> k,
             Eigen::Ref<const Eigen::Matrix<double, number_of_dimensions,
                                            num_vertices>> vertices)
      : H_(H), k_(k), vertices_(vertices) {}

  Eigen::Ref<const Eigen::Matrix<double, num_constraints, number_of_dimensions>>
  static_H() const {
    return H_;
  }

  Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions>>
  H() const override {
    return H_;
  }

  Eigen::Ref<const Eigen::Matrix<double, num_constraints, 1>> static_k()
      const {
    return k_;
  }
  Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> k()
      const override {
    return k_;
  }

  Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> Vertices()
      const override {
    return vertices_;
  }

  Eigen::Matrix<double, number_of_dimensions, num_vertices>
  StaticVertices() const {
    return vertices_;
  }

 private:
  const Eigen::Matrix<double, num_constraints, number_of_dimensions> H_;
  const Eigen::Matrix<double, num_constraints, 1> k_;
  const Eigen::Matrix<double, number_of_dimensions, num_vertices> vertices_;
};



template <int number_of_dimensions>
class HPolytope : public Polytope<number_of_dimensions> {
 public:
  // Constructs a polytope given the H and k matrices.
  HPolytope(Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic,
                                           number_of_dimensions>> H,
            Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> k)
      : H_(H), k_(k), vertices_(CalculateVertices(H, k)) {}

  // This is an initialization function shared across all instantiations of this
  // template.
  // This must be called at least once before creating any instances. It is
  // not thread-safe.
  static void Init() { dd_set_global_constants(); }

  Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions>>
  H() const override {
    return H_;
  }
  Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> k()
      const override {
    return k_;
  }

  Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> Vertices()
      const override {
    return vertices_;
  }

 private:
  const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions> H_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> k_;
  const Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic> vertices_;

  static Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic>
  CalculateVertices(
      Eigen::Ref<
          const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions>> &H,
      Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> &k);
};

template <int number_of_dimensions>
bool Polytope<number_of_dimensions>::IsInside(
    Eigen::Matrix<double, number_of_dimensions, 1> point) const {
  auto k_ptr = k();
  auto ev = H() * point;
  for (int i = 0; i < k_ptr.rows(); ++i) {
    if (ev(i, 0) > k_ptr(i, 0)) {
      return false;
    }
  }
  return true;
}

template <int number_of_dimensions>
Eigen::Matrix<double, number_of_dimensions, Eigen::Dynamic>
HPolytope<number_of_dimensions>::CalculateVertices(
    Eigen::Ref<
        const Eigen::Matrix<double, Eigen::Dynamic, number_of_dimensions>> &H,
    Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>> &k) {
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
