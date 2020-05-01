import numpy as np
from scipy.interpolate import splev, splprep


class Path:
  """Wrapper for scipy.interpolate.BSpline."""


  def __init__(self, controls, closed):
    """Construct a spline through the given control points."""
    self.controls = controls
    self.closed = closed
    self.dists = cumulative_distances(controls)
    self.spline, _ = splprep(controls, u=self.dists, k=3, s=0, per=self.closed)
    self.length = self.dists[-1]


  def position(self, s=None):
    """Returns x-y coordinates of sample points."""
    if s is None: return self.controls
    x, y = splev(s, self.spline)
    return np.array([x, y])


  def curvature(self, s=None):
    """Returns sample curvatures, Kappa."""
    if s is None: s = self.dists
    ddx, ddy = splev(s, self.spline, 2)
    return np.sqrt(ddx**2 + ddy**2)


  def gamma2(self, s=None):
    """Returns the sum of the squares of sample curvatures, Gamma^2."""
    if s is None: s = self.dists
    ddx, ddy = splev(s, self.spline, 2)
    return np.sum(ddx**2 + ddy**2)


###############################################################################


def cumulative_distances(points):
  """Returns the cumulative linear distance at each point."""
  d = np.cumsum(np.linalg.norm(np.diff(points, axis=1), axis=0))
  return np.append(0, d)
