#!/usr/bin/python

"""Wrapper around libcdd, a polytope manipulation library."""

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

import ctypes
import sys

# Wrapper around PyFile_AsFile so that we can print out the error messages.
# Set the arg type and return types of the function call.
class FILE(ctypes.Structure):
  pass

ctypes.pythonapi.PyFile_AsFile.argtypes = [ctypes.py_object]
ctypes.pythonapi.PyFile_AsFile.restype = ctypes.POINTER(FILE)

# Load and init libcdd.  libcdd is a C library that implements algorithm to
# manipulate half space and vertex representations of polytopes.
# Unfortunately, the library was compiled with C++ even though it has a lot of C
# code in it, so all the symbol names are mangled.  Ug.
libcdd = ctypes.cdll.LoadLibrary('libcdd.so')
libcdd._Z23dd_set_global_constantsv()

# The variable type mytype that libcdd defines (double[1])
# See http://docs.python.org/2/library/ctypes.html#arrays for the documentation
# explaining why ctypes.c_double * 1 => double[1]
# libcdd defines mytype to various things so it can essentially template its
# functions.  What a weird library.
mytype = ctypes.c_double * 1


# Forward declaration for the polyhedra data structure.
class dd_polyhedradata(ctypes.Structure):
  pass


# Definition of dd_matrixdata
class dd_matrixdata(ctypes.Structure):
  _fields_ = [
      ("rowsize", ctypes.c_long),
      ("linset", ctypes.POINTER(ctypes.c_ulong)),
      ("colsize", ctypes.c_long),
      ("representation", ctypes.c_int),
      ("numbtype", ctypes.c_int),
      ("matrix", ctypes.POINTER(ctypes.POINTER(mytype))),
      ("objective", ctypes.c_int),
      ("rowvec", ctypes.POINTER(mytype)),
  ]

# Define the input and output types for a bunch of libcdd functions.
libcdd._Z15dd_CreateMatrixll.restype = ctypes.POINTER(dd_matrixdata)
libcdd._Z9ddd_get_dPd.argtypes = [mytype]
libcdd._Z9ddd_get_dPd.restype = ctypes.c_double

libcdd._Z17dd_CopyGeneratorsP16dd_polyhedradata.argtypes = [
    ctypes.POINTER(dd_polyhedradata)
]
libcdd._Z17dd_CopyGeneratorsP16dd_polyhedradata.restype = ctypes.POINTER(dd_matrixdata)

libcdd._Z16dd_DDMatrix2PolyP13dd_matrixdataP12dd_ErrorType.argtypes = [
    ctypes.POINTER(dd_matrixdata),
    ctypes.POINTER(ctypes.c_int)
]
libcdd._Z16dd_DDMatrix2PolyP13dd_matrixdataP12dd_ErrorType.restype = (
  ctypes.POINTER(dd_polyhedradata))

libcdd._Z13dd_FreeMatrixP13dd_matrixdata.argtypes = [
    ctypes.POINTER(dd_matrixdata)
]

libcdd._Z16dd_FreePolyhedraP16dd_polyhedradata.argtypes = [
  ctypes.POINTER(dd_polyhedradata)
]

libcdd._Z9ddd_set_dPdd.argtypes = [
  mytype,
  ctypes.c_double
]


# Various enums.
DD_INEQUALITY = 1
DD_REAL = 1
DD_NO_ERRORS = 17


def dd_CreateMatrix(rows, cols):
  return libcdd._Z15dd_CreateMatrixll(
      ctypes.c_long(rows),
      ctypes.c_long(cols))


def dd_set_d(mytype_address, double_value):
  libcdd._Z9ddd_set_dPdd(mytype_address,
      ctypes.c_double(double_value))


def dd_CopyGenerators(polyhedraptr):
  return libcdd._Z17dd_CopyGeneratorsP16dd_polyhedradata(polyhedraptr)


def dd_get_d(mytype_address):
  return libcdd._Z9ddd_get_dPd(mytype_address)


def dd_FreeMatrix(matrixptr):
  libcdd._Z13dd_FreeMatrixP13dd_matrixdata(matrixptr)


def dd_FreePolyhedra(polyhedraptr):
  libcdd._Z16dd_FreePolyhedraP16dd_polyhedradata(polyhedraptr)


def dd_DDMatrix2Poly(matrixptr):
  error = ctypes.c_int()
  polyhedraptr = libcdd._Z16dd_DDMatrix2PolyP13dd_matrixdataP12dd_ErrorType(
      matrixptr,
      ctypes.byref(error))

  # Return None on error.
  # The error values are enums, so they aren't exposed.
  if error.value != DD_NO_ERRORS:
    # Dump out the errors to stderr
    libcdd._Z21dd_WriteErrorMessagesP8_IO_FILE12dd_ErrorType(
        ctypes.pythonapi.PyFile_AsFile(ctypes.py_object(sys.stdout)),
        error)
    dd_FreePolyhedra(polyhedraptr)
    return None
  return polyhedraptr
