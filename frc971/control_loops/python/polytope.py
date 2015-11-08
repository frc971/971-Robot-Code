#!/usr/bin/python

"""
Polyhedral set library.

This library implements convex regions of the form H x <= k, where H, x, and k
are matricies.  It also provides convenient methods to find all the verticies.
"""

__author__ = 'Austin Schuh (austin.linux@gmail.com)'


from frc971.control_loops.python import libcdd
import numpy
import string
import sys


def _PiecewiseConcat(*args):
  """Concatenates strings inside lists, elementwise.

  Given ['a', 's'] and ['d', 'f'], returns ['ad', 'sf']
  """
  return map(''.join, zip(*args))


def _SplitAndPad(s):
  """Splits a string on newlines, and pads the lines to be the same width."""
  split_string = s.split('\n')
  width = max(len(stringpiece) for stringpiece in split_string) + 1

  padded_strings = [string.ljust(stringpiece, width, ' ')
                        for stringpiece in split_string]
  return padded_strings


def _PadHeight(padded_array, min_height):
  """Adds lines of spaces to the top and bottom of an array symmetrically."""
  height = len(padded_array)
  if height < min_height:
    pad_array = [' ' * len(padded_array[0])]
    height_error = min_height - height
    return (pad_array * ((height_error) / 2) +
            padded_array +
            pad_array * ((height_error + 1) / 2))
  return padded_array


class HPolytope(object):
  """This object represents a H-polytope.

  Polytopes are convex regions in n-dimensional space.
  For H-polytopes, this is represented as the intersection of a set of half
  planes.  The mathematic equation that represents this is H x <= k.
  """

  def __init__(self, H, k):
    """Constructs a H-polytope from the H and k matricies.

    Args:
      H: numpy.Matrix (n by k), where n is the number of constraints, and k is
        the number of dimensions in the space.  Does not copy the matrix.
      k: numpy.Matrix (n by 1).  Does not copy the matrix.
    """
    self._H = H
    self._k = k

  @property
  def k(self):
    """Returns the k in H x <= k."""
    return self._k

  @property
  def H(self):
    """Returns the H in H x <= k."""
    return self._H

  @property
  def ndim(self):
    """Returns the dimension of the set."""
    return self._H.shape[1]

  @property
  def num_constraints(self):
    """Returns the number of constraints defining the set."""
    return self._k.shape[0]

  def IsInside(self, point):
    """Returns true if the point is inside the polytope, edges included."""
    return (self._H * point <= self._k).all()

  def Vertices(self):
    """Returns a matrix with the vertices of the set in its rows."""
    # TODO(aschuh): It would be better to write some small C/C++ function that
    # does all of this and takes in a numpy array.
    # The copy is expensive in Python and cheaper in C.

    # Create an empty matrix with the correct size.
    matrixptr = libcdd.dd_CreateMatrix(self.num_constraints, self.ndim + 1)
    matrix = matrixptr.contents

    try:
      # Copy the data into the matrix.
      for i in xrange(self.num_constraints):
        libcdd.dd_set_d(matrix.matrix[i][0], self._k[i, 0])
        for j in xrange(self.ndim):
          libcdd.dd_set_d(matrix.matrix[i][j + 1], -self._H[i, j])

      # Set enums to the correct values.
      matrix.representation = libcdd.DD_INEQUALITY
      matrix.numbtype = libcdd.DD_REAL

      # TODO(aschuh): Set linearity if it is useful.
      # This would be useful if we had any constraints saying B - A x = 0

      # Build a Polyhedra
      polyhedraptr = libcdd.dd_DDMatrix2Poly(matrixptr)

      if not polyhedraptr:
        return None

      try:
        # Return None on error.
        # The error values are enums, so they aren't exposed.

        # Magic happens here.  Computes the vertices
        vertex_matrixptr = libcdd.dd_CopyGenerators(
            polyhedraptr)
        vertex_matrix = vertex_matrixptr.contents

        try:
          # Count the number of vertices and rays in the result.
          num_vertices = 0
          num_rays = 0
          for i in xrange(vertex_matrix.rowsize):
            if libcdd.dd_get_d(vertex_matrix.matrix[i][0]) == 0:
              num_rays += 1
            else:
              num_vertices += 1

          # Build zeroed matricies for the new vertices and rays.
          vertices = numpy.matrix(numpy.zeros((num_vertices,
                                               vertex_matrix.colsize - 1)))
          rays = numpy.matrix(numpy.zeros((num_rays,
                                           vertex_matrix.colsize - 1)))

          ray_index = 0
          vertex_index = 0

          # Copy the data out of the matrix.
          for index in xrange(vertex_matrix.rowsize):
            if libcdd.dd_get_d(vertex_matrix.matrix[index][0]) == 0.0:
              for j in xrange(vertex_matrix.colsize - 1):
                rays[ray_index, j] = libcdd.dd_get_d(
                    vertex_matrix.matrix[index][j + 1])
              ray_index += 1
            else:
              for j in xrange(vertex_matrix.colsize - 1):
                vertices[vertex_index, j] = libcdd.dd_get_d(
                    vertex_matrix.matrix[index][j + 1])
              vertex_index += 1
        finally:
          # Free everything.
          libcdd.dd_FreeMatrix(vertex_matrixptr)

      finally:
        libcdd.dd_FreePolyhedra(polyhedraptr)

    finally:
      libcdd.dd_FreeMatrix(matrixptr)

    # Rays are unsupported right now.  This may change in the future.
    assert(rays.shape[0] == 0)

    return vertices


  def __str__(self):
    """Returns a formatted version of the polytope.

    The dump will look something like the following, which prints out the matrix
    comparison.

    [[ 1  0]            [[12]
     [-1  0]  [[x0]  <=  [12]
     [ 0  1]   [x1]]     [12]
     [ 0 -1]]            [12]]
    """
    height = max(self.ndim, self.num_constraints)

    # Split the print up into 4 parts and concatenate them all.
    H_strings = _PadHeight(_SplitAndPad(str(self.H)), height)
    v_strings = _PadHeight(_SplitAndPad(str(self.k)), height)
    x_strings = _PadHeight(self._MakeXStrings(), height)
    cmp_strings = self._MakeCmpStrings(height)

    return '\n'.join(_PiecewiseConcat(H_strings, x_strings,
                                      cmp_strings, v_strings))

  def _MakeXStrings(self):
    """Builds an array of strings with constraint names in it for printing."""
    x_strings = []
    if self.ndim == 1:
      x_strings = ["[[x0]] "]
    else:
      for index in xrange(self.ndim):
        if index == 0:
          x = "[[x%d]  " % index
        elif index == self.ndim - 1:
          x = " [x%d]] " % index
        else:
          x = " [x%d]  " % index
        x_strings.append(x)
    return x_strings

  def _MakeCmpStrings(self, height):
    """Builds an array of strings with the comparison in it for printing."""
    cmp_strings = []
    for index in xrange(height):
      if index == (height - 1) / 2:
        cmp_strings.append("<= ")
      else:
        cmp_strings.append("   ")
    return cmp_strings
