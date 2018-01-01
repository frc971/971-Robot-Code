#!/usr/bin/python3

import numpy
from matplotlib import pylab

data = numpy.loadtxt('/tmp/dump3.csv',
                     delimiter=',',
                     skiprows=1)
x = range(len(data))

pylab.subplot(1, 1, 1)
pylab.plot(x, [d[0] for d in data], 'ro', label='ia')
pylab.plot(x, [d[1] for d in data], 'go', label='ib')
pylab.plot(x, [d[2] for d in data], 'bo', label='ic')
pylab.plot(x, [d[3] for d in data], 'r--', label='ia_goal')
pylab.plot(x, [d[4] for d in data], 'g--', label='ib_goal')
pylab.plot(x, [d[5] for d in data], 'b--', label='ic_goal')
pylab.plot(x, [d[6] for d in data], 'rx', label='i_overall')
pylab.plot(x, [d[7] for d in data], 'gx', label='omega')
pylab.plot(x, [d[8] for d in data], 'r', label='van')
pylab.plot(x, [d[9] for d in data], 'g', label='vbn')
pylab.plot(x, [d[10] for d in data], 'b', label='vcn')
pylab.legend()

pylab.show()
