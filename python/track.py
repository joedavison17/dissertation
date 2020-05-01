import json
import numpy as np
from path import Path
from utils import define_corners, idx_modulo, is_closed


class Track:
  """Represents a track with boundaries defined by a series of cones."""


  def __init__(self, json_path=None, left=None, right=None):
    """Create track from cone coordinates."""
    if json_path is None:
      self.left = left
      self.right = right
    else:
      self.read_cones(json_path)
    self.closed = is_closed(self.left, self.right)
    self.size = self.left[0].size - int(self.closed)
    self.diffs = self.right - self.left
    self.mid = Path(self.control_points(np.full(self.size, 0.5)), self.closed)
    self.length = self.mid.dists[-1]
    

  def read_cones(self, path):
    """Read cone coordinates from a JSON file."""
    track_data = json.load(open(path))
    self.name = track_data["name"]
    self.left = np.array([track_data["left"]["x"], track_data["left"]["y"]])
    self.right = np.array([track_data["right"]["x"], track_data["right"]["y"]])
    print("[ Imported {} ]".format(self.name))


  def avg_curvature(self, s):
    """Return the average of curvatures at the given sample distances."""
    k = self.mid.curvature(s)
    return np.sum(k) / s.size


  def corners(self, s, k_min, proximity, length):
    """Determine location of corners on this track."""
    return define_corners(self.mid, s, k_min, proximity, length)


  def control_points(self, alphas):
    """Translate alpha values to control point coordinates."""
    if self.closed: alphas = np.append(alphas, alphas[0])
    i = np.nonzero(alphas != -1)[0]
    return self.left[:,i] + (alphas[i] * self.diffs[:,i])
