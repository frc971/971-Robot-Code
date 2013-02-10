#!/usr/bin/python

import numpy
from numpy.testing import *
import polytope
import unittest

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

def MakePoint(*args):
  """Makes a point from a set of arguments."""
  return numpy.matrix([[arg] for arg in args])

class TestHPolytope(unittest.TestCase):
  def setUp(self):
    """Builds a simple box polytope."""
    self.H = numpy.matrix([[1, 0],
                           [-1, 0],
                           [0, 1],
                           [0, -1]])
    self.k = numpy.matrix([[12],
                           [12],
                           [12],
                           [12]])
    self.p = polytope.HPolytope(self.H, self.k)

  def test_Hk(self):
    """Tests that H and k are saved correctly."""
    assert_array_equal(self.p.H, self.H)
    assert_array_equal(self.p.k, self.k)

  def test_IsInside(self):
    """Tests IsInside for various points."""
    inside_points = [
      MakePoint(0, 0),
      MakePoint(6, 6),
      MakePoint(12, 6),
      MakePoint(-6, 10)]
    outside_points = [
      MakePoint(14, 0),
      MakePoint(-14, 0),
      MakePoint(0, 14),
      MakePoint(0, -14),
      MakePoint(14, -14)]

    for inside_point in inside_points:
      self.assertTrue(self.p.IsInside(inside_point),
                      msg='Point is' + str(inside_point))

    for outside_point in outside_points:
      self.assertFalse(self.p.IsInside(outside_point),
                       msg='Point is' + str(outside_point))

  def AreVertices(self, p, vertices):
    """Checks that all the vertices are on corners of the set."""
    for i in xrange(vertices.shape[0]):
      # Check that all the vertices have the correct number of active
      # constraints.
      lmda = p.H * vertices[i,:].T - p.k
      num_active_constraints = 0
      for j in xrange(lmda.shape[0]):
        # Verify that the constraints are either active, or not violated.
        if numpy.abs(lmda[j, 0]) <= 1e-9:
          num_active_constraints += 1
        else:
          self.assertLessEqual(lmda[j, 0], 0.0)

      self.assertEqual(p.ndim, num_active_constraints)

  def HasSamePoints(self, expected, actual):
    """Verifies that the points in expected are in actual."""
    found_points = set()
    self.assertEqual(expected.shape, actual.shape)
    for index in xrange(expected.shape[0]):
      expected_point = expected[index, :]
      for actual_index in xrange(actual.shape[0]):
        actual_point = actual[actual_index, :]
        if numpy.abs(expected_point - actual_point).max() <= 1e-4:
          found_points.add(actual_index)
          break

    self.assertEqual(len(found_points), actual.shape[0],
        msg="Expected:\n" + str(expected) + "\nActual:\n" + str(actual))

  def test_Skewed_Nonsym_Vertices(self):
    """Tests the vertices of a severely skewed space."""
    self.H = numpy.matrix([[10, -1],
                           [-1, -1],
                           [-1, 10],
                           [10, 10]])
    self.k = numpy.matrix([[2],
                           [2],
                           [2],
                           [2]])
    self.p = polytope.HPolytope(self.H, self.k)
    vertices = self.p.Vertices()
    self.AreVertices(self.p, vertices)

    self.HasSamePoints(
        numpy.matrix([[0., 0.2],
                      [0.2, 0.],
                      [-2., 0.],
                      [0., -2.]]),
        vertices)

  def test_Vertices_Nonsym(self):
    """Tests the vertices of a nonsymetric space."""
    self.k = numpy.matrix([[6],
                           [12],
                           [2],
                           [10]])
    self.p = polytope.HPolytope(self.H, self.k)
    vertices = self.p.Vertices()
    self.AreVertices(self.p, vertices)

    self.HasSamePoints(
        numpy.matrix([[6., 2.],
                      [6., -10.],
                      [-12., -10.],
                      [-12., 2.]]),
        vertices)

  def test_Vertices(self):
    """Tests the vertices of a nonsymetric space."""
    self.HasSamePoints(self.p.Vertices(),
                       numpy.matrix([[12., 12.],
                                     [12., -12.],
                                     [-12., -12.],
                                     [-12., 12.]]))

  def test_concat(self):
    """Tests that the concat function works for simple inputs."""
    self.assertEqual(["asd", "qwe"],
                     polytope._PiecewiseConcat(["a", "q"],
                                               ["s", "w"],
                                               ["d", "e"]))

  def test_str(self):
    """Verifies that the str method works for the provided p."""
    self.assertEqual('[[ 1  0]            [[12]  \n'
                     ' [-1  0]  [[x0]  <=  [12]  \n'
                     ' [ 0  1]   [x1]]     [12]  \n'
                     ' [ 0 -1]]            [12]] ',
                     str(self.p))

  def MakePWithDims(self, num_constraints, num_dims):
    """Makes a zeroed out polytope with the correct size."""
    self.p = polytope.HPolytope(
        numpy.matrix(numpy.zeros((num_constraints, num_dims))),
        numpy.matrix(numpy.zeros((num_constraints, 1))))

  def test_few_constraints_odd_constraint_even_dims_str(self):
    """Tests printing out the set with odd constraints and even dimensions."""
    self.MakePWithDims(num_constraints=5, num_dims=2)
    self.assertEqual('[[ 0.  0.]            [[ 0.]  \n'
                     ' [ 0.  0.]  [[x0]      [ 0.]  \n'
                     ' [ 0.  0.]   [x1]] <=  [ 0.]  \n'
                     ' [ 0.  0.]             [ 0.]  \n'
                     ' [ 0.  0.]]            [ 0.]] ',
                     str(self.p))

  def test_few_constraints_odd_constraint_small_dims_str(self):
    """Tests printing out the set with odd constraints and odd dimensions."""
    self.MakePWithDims(num_constraints=5, num_dims=1)
    self.assertEqual('[[ 0.]            [[ 0.]  \n'
                     ' [ 0.]             [ 0.]  \n'
                     ' [ 0.]  [[x0]] <=  [ 0.]  \n'
                     ' [ 0.]             [ 0.]  \n'
                     ' [ 0.]]            [ 0.]] ',
                     str(self.p))

  def test_few_constraints_odd_constraint_odd_dims_str(self):
    """Tests printing out the set with odd constraints and odd dimensions."""
    self.MakePWithDims(num_constraints=5, num_dims=3)
    self.assertEqual('[[ 0.  0.  0.]            [[ 0.]  \n'
                     ' [ 0.  0.  0.]  [[x0]      [ 0.]  \n'
                     ' [ 0.  0.  0.]   [x1]  <=  [ 0.]  \n'
                     ' [ 0.  0.  0.]   [x2]]     [ 0.]  \n'
                     ' [ 0.  0.  0.]]            [ 0.]] ',
                     str(self.p))

  def test_many_constraints_even_constraint_odd_dims_str(self):
    """Tests printing out the set with even constraints and odd dimensions."""
    self.MakePWithDims(num_constraints=2, num_dims=3)
    self.assertEqual('[[ 0.  0.  0.]  [[x0]     [[ 0.]  \n'
                     ' [ 0.  0.  0.]]  [x1]  <=  [ 0.]] \n'
                     '                 [x2]]            ',
                     str(self.p))


if __name__ == '__main__':
  unittest.main()
