import casadi
import pylab

# From https://gist.github.com/jgillis/dec56fa16c90a8e4a69465e8422c5459

# Point this to where the files generated by running casadi with solver options of
# {"debug": True}
root = "./"

actual = casadi.Sparsity.from_file(root + "debug_fatrop_actual.mtx")

A = casadi.Sparsity.from_file(root + "debug_fatrop_A.mtx")
B = casadi.Sparsity.from_file(root + "debug_fatrop_B.mtx")
C = casadi.Sparsity.from_file(root + "debug_fatrop_C.mtx")
D = casadi.Sparsity.from_file(root + "debug_fatrop_D.mtx")
I = casadi.Sparsity.from_file(root + "debug_fatrop_I.mtx")
errors = casadi.Sparsity.from_file(root + "debug_fatrop_errors.mtx").row()

pylab.figure()
pylab.spy(A,
          marker='o',
          color='r',
          markersize=5,
          label="expected A",
          markerfacecolor="white")
pylab.spy(B,
          marker='o',
          color='b',
          markersize=5,
          label="expected B",
          markerfacecolor="white")
pylab.spy(C,
          marker='o',
          color='g',
          markersize=5,
          label="expected C",
          markerfacecolor="white")
pylab.spy(D,
          marker='o',
          color='y',
          markersize=5,
          label="expected D",
          markerfacecolor="white")
pylab.spy(I,
          marker='o',
          color='k',
          markersize=5,
          label="expected I",
          markerfacecolor="white")
pylab.spy(actual, marker='o', color='k', markersize=2, label="actual")

pylab.hlines(errors,
             0,
             A.shape[1],
             color='gray',
             linestyle='-',
             label="offending rows")

pylab.title("Debug view of fatrop interface structure detection")
pylab.legend()
pylab.show()
