#define PY_SSIZE_T_CLEAN

#include <Python.h>

#include <optional>

#include "frc971/control_loops/dare.h"

namespace frc971::controls {
namespace {
template <int N, int M>
std::optional<Eigen::Matrix<double, N, M>> ToEigen(PyObject *list) {
  Eigen::Matrix<double, N, M> result;
  for (long row_i = 0; row_i < N; ++row_i) {
    PyObject *row_list = PyList_GetItem(list, row_i);
    if (!PyList_Check(row_list)) {
      PyErr_SetString(PyExc_ValueError, "Input lists should be list of list.");
      return std::nullopt;
    }
    for (long col_i = 0; col_i < M; ++col_i) {
      PyObject *element = PyList_GetItem(row_list, col_i);
      if (!PyFloat_Check(element)) {
        PyErr_SetString(
            PyExc_ValueError,
            ("Input lists should be list of list of floats. Current type: " +
             std::string(Py_TYPE(element)->tp_name))
                .c_str());
        return std::nullopt;
      }
      result(row_i, col_i) = PyFloat_AsDouble(element);
    }
  }
  return result;
}

PyObject *dare(PyObject * /*self*/, PyObject *args) {
  PyObject *A;
  PyObject *B;
  PyObject *Q;
  PyObject *R;

  if (!PyArg_ParseTuple(args, "OOOO", &A, &B, &Q, &R)) {
    PyErr_SetString(PyExc_ValueError, "Input arguments should be 4 lists.");
    return nullptr;
  }

  if (!PyList_Check(A)) {
    PyErr_SetString(PyExc_ValueError, "A should be a list");
    return nullptr;
  }

  if (!PyList_Check(B)) {
    PyErr_SetString(PyExc_ValueError, "B should be a list");
    return nullptr;
  }

  if (!PyList_Check(Q)) {
    PyErr_SetString(PyExc_ValueError, "Q should be a list");
    return nullptr;
  }

  if (!PyList_Check(R)) {
    PyErr_SetString(PyExc_ValueError, "R should be a list");
    return nullptr;
  }

  // check list shapes
  constexpr Py_ssize_t num_states = 8;
  constexpr Py_ssize_t num_inputs = 5;

  if (PyList_Size(A) != num_states ||
      PyList_Size(PyList_GetItem(A, 0)) != num_states) {
    PyErr_SetString(PyExc_ValueError,
                    "num_states is not consistent across parameters");
    return nullptr;
  }

  if (PyList_Size(Q) != num_states ||
      PyList_Size(PyList_GetItem(Q, 0)) != num_states) {
    PyErr_SetString(PyExc_ValueError,
                    "num_states is not consistent across parameters");
    return nullptr;
  }

  if (PyList_Size(R) != num_inputs ||
      PyList_Size(PyList_GetItem(R, 0)) != num_inputs) {
    PyErr_SetString(PyExc_ValueError,
                    "num_inputs is not consistent across parameters");
    return nullptr;
  }

  // convert PyList to Eigen
  std::optional<Eigen::Matrix<double, num_states, num_states>> A_eig =
      ToEigen<num_states, num_states>(A);

  if (!A_eig.has_value()) {
    return nullptr;
  }

  std::optional<Eigen::Matrix<double, num_states, num_inputs>> B_eig =
      ToEigen<num_states, num_inputs>(B);

  if (!B_eig.has_value()) {
    return nullptr;
  }

  std::optional<Eigen::Matrix<double, num_states, num_states>> Q_eig =
      ToEigen<num_states, num_states>(Q);

  if (!Q_eig.has_value()) {
    return nullptr;
  }

  std::optional<Eigen::Matrix<double, num_inputs, num_inputs>> R_eig =
      ToEigen<num_inputs, num_inputs>(R);

  if (!R_eig.has_value()) {
    return nullptr;
  }

  // calculate DARE
  auto ret = frc971::controls::dare<double, num_states, num_inputs>(
      A_eig.value(), B_eig.value(), Q_eig.value(), R_eig.value());

  if (!ret) {
    PyErr_SetString(PyExc_ValueError, "error with dare solver");
    return nullptr;
  }

  // return DARE
  auto P = ret.value();

  PyObject *result = PyList_New(num_states);
  for (size_t row_i = 0; row_i < num_states; ++row_i) {
    PyObject *row = PyList_New(num_states);
    for (size_t col_i = 0; col_i < num_states; ++col_i) {
      if (PyList_SetItem(row, col_i, PyFloat_FromDouble(P(row_i, col_i))) !=
          0) {
        return nullptr;
      }
    }
    if (PyList_SetItem(result, row_i, row) != 0) {
      return nullptr;
    }
  }

  return result;
}

static PyMethodDef methods[] = {
    {"dare", dare, METH_VARARGS, "P = dare(A, B, Q, R), all types are lists."},
    {NULL, NULL, 0, NULL}  // Sentinel
};

static PyModuleDef cpp_dare_module = {
    .m_base = PyModuleDef_HEAD_INIT,
    .m_name = "cpp_dare",
    .m_doc =
        "Wraps the DARE solver in order to support convenient "
        "testing.",
    .m_size = -1,
    .m_methods = methods,
};

PyObject *InitModule() { return PyModule_Create(&cpp_dare_module); }
}  // namespace
}  // namespace frc971::controls

PyMODINIT_FUNC PyInit_cpp_dare(void) { return frc971::controls::InitModule(); }
