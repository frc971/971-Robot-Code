#!/usr/bin/python2

# This file checks our math in extended_lqr.tex.

from __future__ import print_function
from __future__ import division

import sys
import random

import sympy

'''
* `x_t1` means `x_{t + 1}`. Using `'x_t + 1'` as the symbol name makes the non-
  latex output really confusing, so not doing that.
* `xb` means `\\boldsymbol{x}`. Written as `'xbold'` in the symbol name because
  that's what sympy.latex recognizes.
* `xb_tb` means `\\boldsymbol{\\bar x}_t. Written as `'xboldbar'` in the symbol
  name so sympy.latex recognizes it.
'''

# sympy.latex

# n in the 2013 paper
number_of_states = sympy.Symbol('states', integer=True)
# m in the 2013 paper
number_of_inputs = sympy.Symbol('inputs', integer=True)
number_of_outputs = sympy.Symbol('outputs', integer=True)

DIMENSIONS = set([number_of_states, number_of_inputs, number_of_outputs])

A_t = sympy.MatrixSymbol('A_t', number_of_states, number_of_states)
B_t = sympy.MatrixSymbol('B_t', number_of_states, number_of_inputs)
cb_t = sympy.MatrixSymbol('cbold_t', number_of_states, 1)
A_tb = sympy.MatrixSymbol('Abar_t', number_of_states, number_of_states)
B_tb = sympy.MatrixSymbol('Bbar_t', number_of_states, number_of_inputs)
cb_tb = sympy.MatrixSymbol('cboldbar_t', number_of_states, 1)

Q_t = sympy.MatrixSymbol('Q_t', number_of_states, number_of_states)
P_t = sympy.MatrixSymbol('P_t', number_of_inputs, number_of_states)
R_t = sympy.MatrixSymbol('R_t', number_of_inputs, number_of_inputs)
qb_t = sympy.MatrixSymbol('qbold_t', number_of_states, 1)
rb_t = sympy.MatrixSymbol('rbold_t', number_of_inputs, 1)
q_t = sympy.MatrixSymbol('q_t', 1, 1)

S_t1 = sympy.MatrixSymbol('S_t1', number_of_states, number_of_states)
sb_t1 = sympy.MatrixSymbol('sbold_t1', number_of_states, 1)
s_t1 = sympy.MatrixSymbol('s_t1', 1, 1)
S_tb = sympy.MatrixSymbol('Sbar_t', number_of_states, number_of_states)
sb_tb = sympy.MatrixSymbol('sboldbar_t', number_of_states, 1)
s_tb = sympy.MatrixSymbol('sbar_t', 1, 1)

xb_t = sympy.MatrixSymbol('xbold_t', number_of_states, 1)
ub_t = sympy.MatrixSymbol('ubold_t', number_of_inputs, 1)
xb_t1 = sympy.MatrixSymbol('xbold_t1', number_of_states, 1)
ub_t1 = sympy.MatrixSymbol('ubold_t1', number_of_inputs, 1)

half = sympy.Integer(1) / sympy.Integer(2)
xb = sympy.MatrixSymbol('xbold', number_of_states, 1)
ub = sympy.MatrixSymbol('ubold', number_of_inputs, 1)

CONSTANTS = set([
    A_t, B_t, cb_t,
    S_t1, sb_t1, s_t1,
    A_tb, B_tb, cb_tb,
    S_tb, sb_tb, s_tb,
    P_t, Q_t, R_t, qb_t, rb_t, q_t,
    ])

SYMMETRIC_CONSTANTS = set([
    S_t1, S_tb,
    Q_t, R_t,
    ])

def verify_equivalent(a, b, inverses={}):
  def get_matrices(m):
    matrices = m.atoms(sympy.MatrixSymbol)
    new_matrices = set()
    for matrix in matrices:
      if matrix in inverses:
        new_matrices.update(inverses[matrix].atoms(sympy.MatrixSymbol))
    matrices.update(new_matrices)
  a_matrices, b_matrices = get_matrices(a), get_matrices(b)
  if a_matrices != b_matrices:
    raise RuntimeError('matrices different: %s vs %s' % (a_matrices,
                                                         b_matrices))
  a_symbols, b_symbols = a.atoms(sympy.Symbol), b.atoms(sympy.Symbol)
  if a_symbols != b_symbols:
    raise RuntimeError('symbols different: %s vs %s' % (a_symbols, b_symbols))
  if not a_symbols < DIMENSIONS:
    raise RuntimeError('not sure what to do with %s' % (a_symbols - DIMENSIONS))

  if a.shape != b.shape:
    raise RuntimeError('Different shapes: %s and %s' % (a.shape, b.shape))

  for _ in range(10):
    subs_symbols = {s: random.randint(1, 5) for s in a_symbols}
    for _ in range(10):
      diff = a - b
      subs_matrices = {}
      def get_replacement(*args):
        try:
          m = sympy.MatrixSymbol(*args)
          if m not in subs_matrices:
            if m in inverses:
              i = inverses[m].replace(sympy.MatrixSymbol, get_replacement, simultaneous=False)
              i_evaled = sympy.ImmutableMatrix(i.rows, i.cols,
                                               lambda x,y: i[x, y].evalf())
              subs_matrices[m] = i_evaled.I
            else:
              rows = m.rows.subs(subs_symbols)
              cols = m.cols.subs(subs_symbols)
              new_m = sympy.ImmutableMatrix(rows, cols,
                                            lambda i,j: random.uniform(-5, 5))
              if m in SYMMETRIC_CONSTANTS:
                if rows != cols:
                  raise RuntimeError('Non-square symmetric matrix %s' % m)
                def calculate_cell(i, j):
                  if i > j:
                    return new_m[i, j]
                  else:
                    return new_m[j, i]
                new_m = sympy.ImmutableMatrix(rows, cols, calculate_cell)
              subs_matrices[m] = new_m
          return subs_matrices[m]
        except AttributeError as e:
          # Stupid sympy silently eats AttributeErrors and treats them as
          # "no replacement"...
          raise RuntimeError(e)
      # subs fails when it tries doing matrix multiplies between fixed-size ones
      # and the rest of the equation which still has the symbolic-sized ones.
      # subs(simultaneous=True) wants to put dummies in for everything first,
      # and Dummy().transpose() is broken.
      # replace() has the same issue as subs with simultaneous being True.
      # lambdify() has no idea what to do with the transposes if you replace all
      # the matrices with ones of random sizes full of dummies.
      diff = diff.replace(sympy.MatrixSymbol, get_replacement,
                          simultaneous=False)
      for row in range(diff.rows):
        for col in range(diff.cols):
          result = diff[row, col].evalf()
          if abs(result) > 1e-7:
            raise RuntimeError('difference at (%s, %s) is %s' % (row, col,
                                                                 result))

def verify_arguments(f, *args):
  matrix_atoms = f.atoms(sympy.MatrixSymbol) - CONSTANTS
  if matrix_atoms != set(args):
    print('Arguments expected to be %s, but are %s, in:\n%s' % (
        sorted(args), sorted(list(matrix_atoms)), f), file=sys.stderr)
    raise RuntimeError

def make_c_t():
  x_and_u = sympy.BlockMatrix(((xb,), (ub,)))
  c_t = (half * x_and_u.transpose() *
         sympy.BlockMatrix(((Q_t, P_t.T), (P_t, R_t))) * x_and_u +
         x_and_u.transpose() * sympy.BlockMatrix(((qb_t,), (rb_t,))) +
         q_t)
  verify_arguments(c_t, xb, ub)
  return c_t

def check_backwards_cost():
  g_t = A_t * xb_t + B_t * ub_t + cb_t
  verify_arguments(g_t, xb_t, ub_t)
  v_t1 = half * xb.transpose() * S_t1 * xb + xb.transpose() * sb_t1 + s_t1
  verify_arguments(v_t1, xb)
  v_t = (v_t1.subs(xb, g_t) + make_c_t()).subs({xb_t: xb, ub_t: ub})
  verify_arguments(v_t, xb, ub)

  v_t_for_cost = (
      half * (
          xb.transpose() * (A_t.transpose() * S_t1 * A_t + Q_t) * xb +
          ub.transpose() * (B_t.transpose() * S_t1 * A_t + P_t) * xb +
          xb.transpose() * (A_t.transpose() * S_t1 * B_t + P_t.T) * ub +
          ub.transpose() * (B_t.transpose() * S_t1 * B_t + R_t) * ub) +
      xb.transpose() * (A_t.transpose() * sb_t1 + qb_t) +
      ub.transpose() * (B_t.transpose() * sb_t1 + rb_t) +
      cb_t.transpose() * sb_t1 +
      s_t1 + q_t +
      half * (cb_t.transpose() * S_t1 * cb_t +
    xb.transpose() * A_t.transpose() * S_t1 * cb_t +
    ub.transpose() * B_t.transpose() * S_t1 * cb_t +
    cb_t.transpose() * S_t1 * B_t * ub +
              cb_t.transpose() * S_t1 * A_t * xb))
  verify_equivalent(v_t, v_t_for_cost)

  v_t_now = (
      half * (xb.T * (A_t.T * S_t1 * A_t + Q_t) * xb +
              ub.T * (B_t.T * S_t1 * A_t + P_t) * xb +
              xb.T * (A_t.T * S_t1 * B_t + P_t.T) * ub +
              ub.T * (B_t.T * S_t1 * B_t + R_t) * ub) +
      xb.T * (A_t.T * sb_t1 + qb_t) +
      ub.T * (B_t.T * sb_t1 + rb_t) +
      cb_t.T * sb_t1 + s_t1 + q_t +
      half * (cb_t.T * S_t1 * cb_t +
              xb.T * A_t.T * S_t1 * cb_t +
              ub.T * B_t.T * S_t1 * cb_t +
              cb_t.T * S_t1 * B_t * ub +
              cb_t.T * S_t1 * A_t * xb))

  v_t_now = (
      half * (xb.T * (A_t.T * S_t1 * A_t + Q_t) * xb +
              ub.T * (B_t.T * S_t1 * A_t + P_t) * xb +
              xb.T * (A_t.T * S_t1 * B_t + P_t.T) * ub +
              ub.T * (B_t.T * S_t1 * B_t + R_t) * ub) +
      xb.T * (A_t.T * sb_t1 + qb_t + half * A_t.T * S_t1 * cb_t) +
      ub.T * (B_t.T * sb_t1 + rb_t + half * B_t.T * S_t1 * cb_t) +
      half * cb_t.T * S_t1 * (A_t * xb + B_t * ub + cb_t) +
      cb_t.T * sb_t1 + s_t1 + q_t)

  v_t_now = (
      half * (xb.T * (A_t.T * S_t1 * A_t + Q_t) * xb +
              ub.T * (B_t.T * S_t1 * A_t + P_t) * xb +
              xb.T * (A_t.T * S_t1 * B_t + P_t.T) * ub +
              ub.T * (B_t.T * S_t1 * B_t + R_t) * ub) +
      xb.T * (A_t.T * sb_t1 + qb_t + A_t.T * S_t1 * cb_t) +
      ub.T * (B_t.T * sb_t1 + rb_t + B_t.T * S_t1 * cb_t) +
      half * cb_t.T * S_t1 * cb_t +
      cb_t.T * sb_t1 + s_t1 + q_t)

  C_t = B_t.T * S_t1 * A_t + P_t
  E_t = B_t.T * S_t1 * B_t + R_t
  E_t_I = sympy.MatrixSymbol('E_t^-1', E_t.cols, E_t.rows)
  L_t = -E_t_I * C_t
  eb_t = B_t.T * S_t1 * cb_t + B_t.T * sb_t1 + rb_t
  lb_t = -E_t_I * eb_t
  D_t = A_t.T * S_t1 * A_t + Q_t
  db_t = A_t.T * S_t1 * cb_t + A_t.T * sb_t1 + qb_t

  v_t_now = (
      half * (xb.T * D_t * xb + ub.T * C_t * xb +
              xb.T * C_t.T * ub + ub.T * E_t * ub) +
      xb.T * db_t + ub.T * eb_t +
      half * cb_t.T * S_t1 * cb_t +
      cb_t.T * sb_t1 + s_t1 + q_t)

  v_t_final = (
      half * xb.T * (D_t + L_t.T * C_t + C_t.T * L_t + L_t.T * E_t * L_t) * xb +
      xb.T * (C_t.T * lb_t + L_t.T * E_t * lb_t + db_t + L_t.T * eb_t) +
      half * lb_t.T * E_t * lb_t +
      lb_t.T * eb_t +
      cb_t.T * sb_t1 + s_t1 + q_t + half * cb_t.T * S_t1 * cb_t
      )
  verify_arguments(v_t_final, xb, E_t_I)
  verify_equivalent(v_t.subs(ub, L_t * xb + lb_t), v_t_final, {E_t_I: E_t})

  def v_t_from_s(this_S_t, this_sb_t, this_s_t):
    return half * xb.T * this_S_t * xb + xb.T * this_sb_t + this_s_t

  S_t_new_first = D_t + L_t.T * C_t + C_t.T * L_t + L_t.T * E_t * L_t
  sb_t_new_first = db_t - C_t.T * E_t_I * eb_t
  s_t_new_first = (half * lb_t.T * E_t * lb_t +
                   lb_t.T * eb_t +
                   cb_t.T * sb_t1 +
                   s_t1 + q_t +
                   half * cb_t.T * S_t1 * cb_t)
  verify_equivalent(v_t_from_s(S_t_new_first, sb_t_new_first, s_t_new_first),
                    v_t_final, {E_t_I: E_t})

  S_t_new_end = D_t - C_t.T * E_t_I * C_t
  sb_t_new_end = db_t - C_t.T * E_t_I * eb_t
  s_t_new_end = (q_t - half * eb_t.T * E_t_I * eb_t +
                 half * cb_t.T * S_t1 * cb_t + cb_t.T * sb_t1 + s_t1)
  verify_equivalent(v_t_from_s(S_t_new_end, sb_t_new_end, s_t_new_end),
                    v_t_final, {E_t_I: E_t})

def check_forwards_cost():
  v_tb = half * xb.T * S_tb * xb + xb.T * sb_tb + s_tb
  verify_arguments(v_tb, xb)
  g_tb = A_tb * xb_t1 + B_tb * ub + cb_tb
  verify_arguments(g_tb, xb_t1, ub)
  c_t1b = make_c_t().subs(xb, g_tb)
  verify_arguments(c_t1b, xb_t1, ub)
  v_t1b = v_tb.subs(xb, g_tb) + c_t1b
  verify_arguments(v_t1b, xb_t1, ub)

  v_t1b_now = (
      half * g_tb.T * S_tb * g_tb +
      g_tb.T * sb_tb + s_tb +
      half * (g_tb.T * Q_t * g_tb +
              ub.T * P_t * g_tb +
              g_tb.T * P_t.T * ub +
              ub.T * R_t * ub) +
      g_tb.T * qb_t + ub.T * rb_t + q_t)

  v_t1b_for_cost = (
      half * (xb_t1.T * A_tb.T * (S_tb + Q_t) * A_tb * xb_t1 +
              xb_t1.T * A_tb.T * (S_tb + Q_t) * B_tb * ub +
              xb_t1.T * A_tb.T * (S_tb + Q_t) * cb_tb +
              ub.T * B_tb.T * (S_tb + Q_t) * A_tb * xb_t1 +
              ub.T * B_tb.T * (S_tb + Q_t) * B_tb * ub +
              ub.T * B_tb.T * (S_tb + Q_t) * cb_tb +
              cb_tb.T * (S_tb + Q_t) * A_tb * xb_t1 +
              cb_tb.T * (S_tb + Q_t) * B_tb * ub +
              cb_tb.T * (S_tb + Q_t) * cb_tb) +
      xb_t1.T * A_tb.T * sb_tb +
      ub.T * B_tb.T * sb_tb +
      cb_tb.T * sb_tb +
      s_tb +
      half * (ub.T * P_t * A_tb * xb_t1 +
              ub.T * P_t * B_tb * ub +
              ub.T * P_t * cb_tb) +
      half * (xb_t1.T * A_tb.T * P_t.T * ub +
              ub.T * B_tb.T * P_t.T * ub +
              cb_tb.T * P_t.T * ub) +
      half * ub.T * R_t * ub +
      xb_t1.T * A_tb.T * qb_t + ub.T * B_tb.T * qb_t + cb_tb.T * qb_t +
      ub.T * rb_t + q_t)
  verify_equivalent(v_t1b, v_t1b_for_cost)

  S_and_Q = S_tb + Q_t

  v_t1b_now = (
      half * (xb_t1.T * A_tb.T * S_and_Q * A_tb * xb_t1 +
              xb_t1.T * A_tb.T * S_and_Q * B_tb * ub +
              xb_t1.T * A_tb.T * S_and_Q * cb_tb +
              ub.T * B_tb.T * S_and_Q * A_tb * xb_t1 +
              ub.T * B_tb.T * S_and_Q * B_tb * ub +
              ub.T * B_tb.T * S_and_Q * cb_tb +
              cb_tb.T * S_and_Q * A_tb * xb_t1 +
              cb_tb.T * S_and_Q * B_tb * ub +
              cb_tb.T * S_and_Q * cb_tb) +
      xb_t1.T * A_tb.T * sb_tb +
      ub.T * B_tb.T * sb_tb +
      cb_tb.T * sb_tb +
      s_tb +
      half * (ub.T * P_t * A_tb * xb_t1 +
              ub.T * P_t * B_tb * ub +
              ub.T * P_t * cb_tb) +
      half * (xb_t1.T * A_tb.T * P_t.T * ub +
              ub.T * B_tb.T * P_t.T * ub +
              cb_tb.T * P_t.T * ub) +
      half * ub.T * R_t * ub +
      xb_t1.T * A_tb.T * qb_t +
      ub.T * B_tb.T * qb_t +
      cb_tb.T * qb_t +
      ub.T * rb_t +
      q_t)

  C_tb = B_tb.T * S_and_Q * A_tb + P_t * A_tb
  E_tb = B_tb.T * S_and_Q * B_tb + B_tb.T * P_t.T + P_t * B_tb + R_t
  E_tb_I = sympy.MatrixSymbol('Ebar_t^-1', E_tb.cols, E_tb.rows)
  L_tb = -E_tb_I * C_tb
  eb_tb = B_tb.T * S_and_Q * cb_tb + B_tb.T * sb_tb + P_t * cb_tb + B_tb.T * qb_t + rb_t
  lb_tb = -E_tb_I * eb_tb
  D_tb = A_tb.T * S_and_Q * A_tb
  db_tb = A_tb.T * S_and_Q * cb_tb + A_tb.T * (sb_tb + qb_t)

  v_t1b_now = (
      half * (xb_t1.T * D_tb * xb_t1 +
              xb_t1.T * C_tb.T * ub +
              ub.T * C_tb * xb_t1 +
              ub.T * E_tb * ub) +
      xb_t1.T * db_tb +
      ub.T * eb_tb +
      half * cb_tb.T * S_and_Q * cb_tb +
      cb_tb.T * sb_tb +
      cb_tb.T * qb_t +
      s_tb + q_t)

  v_t1b_final = (
      half * xb_t1.T * (D_tb - C_tb.T * E_tb_I * C_tb) * xb_t1 +
      xb_t1.T * (db_tb - C_tb.T * E_tb_I * eb_tb) +
      -half * eb_tb.T * E_tb_I * eb_tb +
      half * cb_tb.T * S_and_Q * cb_tb +
      cb_tb.T * sb_tb +
      cb_tb.T * qb_t +
      s_tb + q_t)
  verify_arguments(v_t1b_final, xb_t1, E_tb_I)
  verify_equivalent(v_t1b.subs(ub, -E_tb_I * C_tb * xb_t1 - E_tb_I * eb_tb),
                    v_t1b_final, {E_tb_I: E_tb})

def check_forwards_u():
  S_and_Q = S_tb + Q_t

  diff_start = (
      half * (xb_t1.T * A_tb.T * S_and_Q * B_tb +
              (B_tb.T * S_and_Q * A_tb * xb_t1).T +
              2 * ub.T * B_tb.T * S_and_Q * B_tb +
              (B_tb.T * S_and_Q * cb_tb).T +
              cb_tb.T * S_and_Q * B_tb) +
      sb_tb.T * B_tb +
      half * (P_t * A_tb * xb_t1).T +
      half * xb_t1.T * A_tb.T * P_t.T +
      half * ub.T * (P_t * B_tb + B_tb.T * P_t.T) +
      half * ub.T * (B_tb.T * P_t.T + P_t * B_tb) +
      half * (P_t * cb_tb).T +
      half * cb_tb.T * P_t.T +
      ub.T * R_t +
      (B_tb.T * qb_t).T + rb_t.T)
  verify_arguments(diff_start, xb_t1, ub)

  diff_end = (
      xb_t1.T * (A_tb.T * S_and_Q * B_tb + A_tb.T * P_t.T) +
      ub.T * (B_tb.T * S_and_Q * B_tb + B_tb.T * P_t.T + P_t * B_tb + R_t) +
      cb_tb.T * S_and_Q * B_tb +
      sb_tb.T * B_tb +
      cb_tb.T * P_t.T +
      qb_t.T * B_tb +
      rb_t.T)
  verify_equivalent(diff_start, diff_end)

def main():
  sympy.init_printing(use_unicode=True)
  check_backwards_cost()
  check_forwards_cost()
  check_forwards_u()

if __name__ == '__main__':
  main()
