#ifndef FRC971_MATH_FLATBUFFERS_MATRIX_H_
#define FRC971_MATH_FLATBUFFERS_MATRIX_H_
// This library provides utilities for converting between a frc971.fbs.Matrix
// flatbuffer type and an Eigen::Matrix.
// The interesting methods are ToEigen(), ToEigenOrDie(), and FromEigen().
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "tl/expected.hpp"
#include <Eigen/Core>

#include "frc971/math/matrix_static.h"

namespace frc971 {
inline constexpr Eigen::StorageOptions ToEigenStorageOrder(
    fbs::StorageOrder storage_order, int Rows, int Cols) {
  // Eigen only implements one *Major version of the Matrix class for vectors.
  // See https://eigen.tuxfamily.org/bz/show_bug.cgi?id=416
  if (Rows == 1) {
    return Eigen::RowMajor;
  }
  if (Cols == 1) {
    return Eigen::ColMajor;
  }
  return storage_order == fbs::StorageOrder::ColMajor ? Eigen::ColMajor
                                                      : Eigen::RowMajor;
}

template <int Rows, int Cols, fbs::StorageOrder StorageOrder>
struct EigenMatrix {
  typedef Eigen::Matrix<double, Rows, Cols,
                        ToEigenStorageOrder(StorageOrder, Rows, Cols)>
      type;
};

inline std::ostream &operator<<(std::ostream &os, fbs::MatrixField field) {
  os << fbs::EnumNameMatrixField(field);
  return os;
}

inline std::ostream &operator<<(std::ostream &os, fbs::FieldError error) {
  os << fbs::EnumNameFieldError(error);
  return os;
}

struct ConversionFailure {
  fbs::MatrixField field;
  fbs::FieldError error;
  bool operator==(const ConversionFailure &) const = default;
};

inline std::ostream &operator<<(std::ostream &os, ConversionFailure failure) {
  os << "(" << failure.field << ", " << failure.error << ")";
  return os;
}

template <int Rows, int Cols,
          fbs::StorageOrder StorageOrder = fbs::StorageOrder::ColMajor>
tl::expected<typename EigenMatrix<Rows, Cols, StorageOrder>::type,
             ConversionFailure>
ToEigen(const fbs::Matrix &matrix) {
  if (!matrix.has_rows()) {
    return tl::unexpected(
        ConversionFailure{fbs::MatrixField::kRows, fbs::FieldError::kMissing});
  }
  if (!matrix.has_cols()) {
    return tl::unexpected(
        ConversionFailure{fbs::MatrixField::kCols, fbs::FieldError::kMissing});
  }
  if (!matrix.has_data()) {
    return tl::unexpected(
        ConversionFailure{fbs::MatrixField::kData, fbs::FieldError::kMissing});
  }
  if (matrix.rows() != Rows) {
    return tl::unexpected(ConversionFailure{
        fbs::MatrixField::kRows, fbs::FieldError::kInconsistentWithTemplate});
  }
  if (matrix.cols() != Cols) {
    return tl::unexpected(ConversionFailure{
        fbs::MatrixField::kCols, fbs::FieldError::kInconsistentWithTemplate});
  }
  if (matrix.storage_order() != StorageOrder) {
    return tl::unexpected(
        ConversionFailure{fbs::MatrixField::kStorageOrder,
                          fbs::FieldError::kInconsistentWithTemplate});
  }
  if (matrix.data()->size() != Rows * Cols) {
    return tl::unexpected(ConversionFailure{
        fbs::MatrixField::kData, fbs::FieldError::kInconsistentWithTemplate});
  }
  return typename EigenMatrix<Rows, Cols, StorageOrder>::type(
      matrix.data()->data());
}

template <int Rows, int Cols,
          fbs::StorageOrder StorageOrder = fbs::StorageOrder::ColMajor>
typename EigenMatrix<Rows, Cols, StorageOrder>::type ToEigenOrDie(
    const fbs::Matrix &matrix) {
  auto result = ToEigen<Rows, Cols, StorageOrder>(matrix);
  if (!result.has_value()) {
    LOG(FATAL) << "Failed to convert to matrix with error " << result.error()
               << ".";
  }
  return result.value();
}

template <int Rows, int Cols,
          fbs::StorageOrder StorageOrder = fbs::StorageOrder::ColMajor>
bool FromEigen(
    const typename EigenMatrix<Rows, Cols, StorageOrder>::type &matrix,
    fbs::MatrixStatic *flatbuffer) {
  constexpr size_t kSize = Rows * Cols;
  auto data = flatbuffer->add_data();
  if (!data->reserve(kSize)) {
    return false;
  }
  // TODO(james): Use From*() methods once they get upstreamed...
  data->resize(kSize);
  std::copy(matrix.data(), matrix.data() + kSize, data->data());
  flatbuffer->set_rows(Rows);
  flatbuffer->set_cols(Cols);
  flatbuffer->set_storage_order(StorageOrder);
  return true;
}

template <int Rows, int Cols,
          fbs::StorageOrder StorageOrder = fbs::StorageOrder::ColMajor>
flatbuffers::Offset<fbs::Matrix> FromEigen(
    const typename EigenMatrix<Rows, Cols, StorageOrder>::type &matrix,
    flatbuffers::FlatBufferBuilder *fbb) {
  constexpr size_t kSize = Rows * Cols;
  flatbuffers::Offset<flatbuffers::Vector<double>> data_offset =
      fbb->CreateVector(matrix.data(), kSize);
  fbs::Matrix::Builder builder(*fbb);
  builder.add_rows(Rows);
  builder.add_cols(Cols);
  builder.add_storage_order(StorageOrder);
  builder.add_data(data_offset);
  return builder.Finish();
}

template <typename T>
bool FromEigen(const T &matrix, fbs::MatrixStatic *flatbuffer) {
  return FromEigen<T::RowsAtCompileTime, T::ColsAtCompileTime,
                   (T::IsRowMajor ? fbs::StorageOrder::RowMajor
                                  : fbs::StorageOrder::ColMajor)>(matrix,
                                                                  flatbuffer);
}

}  // namespace frc971
#endif  // FRC971_MATH_FLATBUFFERS_MATRIX_H_
