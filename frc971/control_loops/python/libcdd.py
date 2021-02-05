#!/usr/bin/python3
"""Wrapper around libcdd, a polytope manipulation library."""

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

import ctypes
import os
import sys

# Load and init libcdd.  libcdd is a C library that implements algorithm to
# manipulate half space and vertex representations of polytopes.
# Unfortunately, the library was compiled with C++ even though it has a lot of C
# code in it, so all the symbol names are mangled.  Ug.
libcdd = None
for path in os.environ.get('PYTHONPATH').split(':'):
    try:
        libcdd = ctypes.cdll.LoadLibrary(
            os.path.join(path, 'third_party/cddlib/_cddlib.so'))
    except OSError:
        pass

assert libcdd is not None, 'Failed to find _cddlib.so'

libcdd.dd_set_global_constants()

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
libcdd.dd_CreateMatrix.restype = ctypes.POINTER(dd_matrixdata)
libcdd.ddd_get_d.argtypes = [mytype]
libcdd.ddd_get_d.restype = ctypes.c_double

libcdd.dd_CopyGenerators.argtypes = [ctypes.POINTER(dd_polyhedradata)]
libcdd.dd_CopyGenerators.restype = ctypes.POINTER(dd_matrixdata)

libcdd.dd_DDMatrix2Poly.argtypes = [
    ctypes.POINTER(dd_matrixdata),
    ctypes.POINTER(ctypes.c_int)
]
libcdd.dd_DDMatrix2Poly.restype = (ctypes.POINTER(dd_polyhedradata))

libcdd.dd_FreeMatrix.argtypes = [ctypes.POINTER(dd_matrixdata)]

libcdd.dd_FreePolyhedra.argtypes = [ctypes.POINTER(dd_polyhedradata)]

libcdd.ddd_set_d.argtypes = [mytype, ctypes.c_double]

# Various enums.
DD_INEQUALITY = 1
DD_REAL = 1
DD_NO_ERRORS = 17


def dd_CreateMatrix(rows, cols):
    return libcdd.dd_CreateMatrix(ctypes.c_long(rows), ctypes.c_long(cols))


def dd_set_d(mytype_address, double_value):
    libcdd.ddd_set_d(mytype_address, ctypes.c_double(double_value))


def dd_CopyGenerators(polyhedraptr):
    return libcdd.dd_CopyGenerators(polyhedraptr)


def dd_get_d(mytype_address):
    return libcdd.ddd_get_d(mytype_address)


def dd_FreeMatrix(matrixptr):
    libcdd.dd_FreeMatrix(matrixptr)


def dd_FreePolyhedra(polyhedraptr):
    libcdd.dd_FreePolyhedra(polyhedraptr)


def dd_DDMatrix2Poly(matrixptr):
    error = ctypes.c_int()
    polyhedraptr = libcdd.dd_DDMatrix2Poly(matrixptr, ctypes.byref(error))

    # Return None on error.
    # The error values are enums, so they aren't exposed.
    if error.value != DD_NO_ERRORS:
        # TODO(austin): Dump out the errors to stderr
        #libcdd.dd_WriteErrorMessages(
        #    ctypes.pythonapi.PyFile_AsFile(ctypes.py_object(sys.stdout)),
        #    error)
        dd_FreePolyhedra(polyhedraptr)
        return None
    return polyhedraptr
