import numpy as np


def idx_modulo(a, b, n):
  """
  Return a sequence of indices from a to b, that wraps around to 0 after n
  elements.
  """
  i = a % n
  j = b % n
  if i < j: return np.arange(i, j, dtype=int)
  return np.append(np.arange(i, n, dtype=int), np.arange(0, j, dtype=int))


def is_closed(left, right):
  """
  Compares the first and last cones in each boundary to determine if a track is
  open ended or a closed loop.
  """
  return all(left[:,0]==left[:,-1]) and all(right[:,0]==right[:,-1])


def define_corners(path, s, k_min, proximity, length):
  """
  Analyse the track to find corners and straights.

  k_min: defines the minimum curvature for a corner
  proximity: corners within this distance are joined
  length: corners must exceed this length to be accepted

  Returns: an array of control point index pairs defining the track's corners
  """
  is_corner = path.curvature(s) > k_min
  is_corner = filter_corners(is_corner, s, length, proximity)
  corners = samples_to_controls(s, corner_idxs(is_corner), path.dists)
  return corners, is_corner


def filter_corners(is_corner, dists, length, proximity):
  """Update corner status according to length and proximity."""

  # Shift to avoid splitting a straight or corner
  shift = np.argwhere(is_corner != is_corner[0])[0][0]
  is_corner = np.roll(is_corner, -shift)
  # Remove short straights
  start = 0
  for i in range(1, is_corner.size):
    if is_corner[i-1]:
      if not is_corner[i]:
        # Corner to straight, record straight start
        start = i
    elif is_corner[i]:
      # Straight to corner, measure straight and convert if too short
      is_corner[start:i] = (dists[i] - dists[start]) < proximity
  # Remove short corners
  start = 0
  for i in range(1, is_corner.size):
    if is_corner[i-1]:
      if not is_corner[i]:
        # Corner to straight, measure corner and convert if too short
        is_corner[start:i] = (dists[i] - dists[start]) > length
    elif is_corner[i]:
      # Straight to corner, record corner start
      start = i
  return np.roll(is_corner, shift)


def corner_idxs(is_corner):
  """Determine the samples at which corner sequences start and end."""

  # Shift to avoid splitting a straight or corner
  shift = np.argwhere(is_corner != is_corner[0])[0][0]
  is_corner = np.roll(is_corner, -shift)
  # Search for corners
  corners = np.array([],dtype=int)
  n = len(is_corner)
  start = shift
  for j in range(1, n+1):
    i = j % n
    if is_corner[i-1]:
      if not is_corner[i]: # Corner -> straight
        end = (i + shift) % n
        if len(corners) > 0: corners = np.vstack((corners, [start, end]))
        else: corners = np.array([start, end])
    else:
      if is_corner[i]: # Straight -> corner
        start = (i + shift) % n
  return corners


def samples_to_controls(s_dist, s_idx, c_dist):
  """Convert sample distances to control point indices."""
  n = s_idx.size
  s_flat = s_idx.ravel()
  c_flat = np.zeros(n, dtype=int)
  for i in range(n):
    j = 0
    while s_dist[s_flat[i]] > c_dist[j]: j += 1
    c_flat[i] = j
  return c_flat.reshape(s_idx.shape)

