#!/usr/bin/python3

from logreader import CollectingLogReader
import matplotlib
from matplotlib import pylab
from matplotlib.font_manager import FontProperties

class Plotter(CollectingLogReader):
  """
  A CollectingLogReader that plots collected data.
  """

  def PlotFile(self, f, no_binary_in_legend=False):
    """
    Parses and plots all the data.

    Args:
      f: str, The filename of the log whose data to parse and plot.

    Returns:
      None
    """
    self.HandleFile(f)
    self.Plot(no_binary_in_legend)

  def Plot(self, no_binary_in_legend):
    """
    Plots all the data after it's parsed.

    This should only be called after `HandleFile` has been called so that there
    is actual data to plot.
    """
    for key in self.signal:
      value = self.signal[key]

      # Create a legend label using the binary name (optional), the structure
      # name and the data search path.
      label = key[1] + '.' + '.'.join(key[2])
      if not no_binary_in_legend:
        label = key[0] + ' ' + label

      pylab.plot(value.time, value.data, label=label)

    # Set legend font size to small and move it to the top center.
    fontP = FontProperties()
    fontP.set_size('small')
    pylab.legend(bbox_to_anchor=(0.5, 1.05), prop=fontP)

    pylab.show()

