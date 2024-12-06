#define PY_SSIZE_T_CLEAN
// Note that Python.h needs to be included before anything else.
#include <Python.h>

#include <iostream>
#include <optional>

#include "frc971/control_loops/swerve/dynamics.h"

namespace frc971::control_loops::swerve {
namespace {
template <int N>
std::optional<Eigen::Matrix<double, N, 1>> ToEigen(PyObject *list) {
  Eigen::Matrix<double, N, 1> result;
  for (size_t index = 0; index < N; ++index) {
    PyObject *element = PyList_GetItem(list, index);
    if (!PyFloat_Check(element)) {
      PyErr_SetString(PyExc_ValueError,
                      "Input lists should be lists of floats.");
      return std::nullopt;
    }
    result(index) = PyFloat_AsDouble(element);
  }
  return result;
}
PyObject *swerve_dynamics(PyObject * /*self*/, PyObject *args) {
  PyObject *X;
  PyObject *U;
  if (!PyArg_ParseTuple(args, "OO", &X, &U)) {
    PyErr_SetString(PyExc_ValueError, "Input arguments should be two lists.");
    return nullptr;
  }

  if (!PyList_Check(X)) {
    PyErr_SetString(PyExc_ValueError, "X should be a list.");
    return nullptr;
  }
  if (!PyList_Check(U)) {
    PyErr_SetString(PyExc_ValueError, "U should be a list.");
    return nullptr;
  }

  if (PyList_Size(X) != kNumFullDynamicsStates) {
    PyErr_SetString(PyExc_ValueError,
                    "X should have kNumFullDynamicsStates elements.");
    return nullptr;
  }

  if (PyList_Size(U) != kNumInputs) {
    PyErr_SetString(PyExc_ValueError, "U should have kNumInputs elements.");
    return nullptr;
  }

  std::optional<Eigen::Matrix<double, kNumFullDynamicsStates, 1>> X_eig =
      ToEigen<kNumFullDynamicsStates>(X);

  if (!X_eig.has_value()) {
    return nullptr;
  }

  std::optional<Eigen::Matrix<double, kNumInputs, 1>> U_eig =
      ToEigen<kNumInputs>(U);

  if (!U_eig.has_value()) {
    return nullptr;
  }

  Eigen::Matrix<double, kNumFullDynamicsStates, 1> Xdot =
      SwervePhysics(X_eig.value(), U_eig.value());

  PyObject *result = PyList_New(kNumFullDynamicsStates);
  for (size_t index = 0; index < kNumFullDynamicsStates; ++index) {
    if (PyList_SetItem(result, index, PyFloat_FromDouble(Xdot(index))) != 0) {
      return nullptr;
    }
  }

  return result;
}

static PyMethodDef methods[] = {
    {"swerve_dynamics", swerve_dynamics, METH_VARARGS,
     "Xdot = swerve_dynamics(X, U), all types are lists."},
    {NULL, NULL, 0, NULL}  // Sentinel
};

static PyModuleDef cpp_dynamics_module = {
    .m_base = PyModuleDef_HEAD_INIT,
    .m_name = "cpp_dynamics",
    .m_doc =
        "Wraps the generated C++ dynamics in order to support convenient "
        "testing.",
    .m_size = -1,
    .m_methods = methods,
};

PyObject *InitModule() { return PyModule_Create(&cpp_dynamics_module); }
}  // namespace
}  // namespace frc971::control_loops::swerve

PyMODINIT_FUNC PyInit_cpp_dynamics(void) {
  return frc971::control_loops::swerve::InitModule();
}
