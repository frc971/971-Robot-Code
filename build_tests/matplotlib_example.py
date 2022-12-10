# This is the introductory example from
# https://matplotlib.org/stable/tutorials/introductory/pyplot.html

import matplotlib
import matplotlib.pyplot as plt

# Set up the gtk backend before running matplotlib.
matplotlib.use("GTK3Agg")

plt.plot([1, 2, 3, 4])
plt.ylabel("some numbers")
plt.show()
