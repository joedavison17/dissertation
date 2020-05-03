import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.collections import LineCollection

if sys.version_info[0]==3: matplotlib.use('Agg')
plt.rc('font', family='serif')
plt.rc('text', usetex=True)

LCLR = 'tab:gray'
RCLR = 'tab:gray'

###############################################################################

def plot_path(dest, left, right, samples, control=None, show_cones=False):
  """
  Plot track and solid colour path.
  """
  plt.figure() 
  plt.plot(left[0], left[1], color=LCLR, linestyle='solid', zorder=1, linewidth=1)
  plt.plot(right[0], right[1], color=LCLR, linestyle='solid', zorder=1, linewidth=1)
  plt.plot(samples[0], samples[1], color='tab:green', linestyle='solid', zorder=2)
  
  if control is not None:
    plt.scatter(control[0], control[1], color='tab:green', marker='.')

  if show_cones:
    plt.scatter(left[0], left[1], color='tab:blue', marker='.')
    plt.scatter(right[0], right[1], color='tab:orange', marker='.')

  plt.gca().set_aspect('equal', adjustable='box')
  plt.axis('off')
  plt.savefig(dest, bbox_inches='tight')


def plot_corners(dest, left, right, samples, is_corner):
  """
  Plot track with corners highlighted.
  """
  plt.figure()
  plt.plot(left[0], left[1], color=LCLR, linestyle='solid', linewidth=1)
  plt.plot(right[0], right[1], color=LCLR, linestyle='solid', linewidth=1)
  
  p = samples.T.reshape(-1, 1, 2)
  segments = np.concatenate([p[:-1], p[1:]], axis=1)
  norm = plt.Normalize(0, 1.5)
  lc = LineCollection(
    segments, array=is_corner, cmap='Greens', norm=norm, linewidth=4
  )
  plt.gca().add_collection(lc)

  plt.gca().set_aspect('equal', adjustable='box')
  plt.axis('off')
  plt.savefig(dest, bbox_inches='tight')


def plot_trajectory(dest, left, right, samples, velocities):
  """
  Plot path with velocity colour map.
  """
  # Set up velocity gradient segments
  plt.figure() 
  plt.plot(left[0], left[1], color=LCLR, linestyle='solid', linewidth=1, zorder=1)
  plt.plot(right[0], right[1], color=LCLR, linestyle='solid', linewidth=1, zorder=1)
  p = samples.T.reshape(-1, 1, 2)
  segments = np.concatenate([p[:-1], p[1:]], axis=1)
  norm = plt.Normalize(10, 40)
  lc = LineCollection(
    segments, array=velocities, cmap="inferno", norm=norm, linewidth=2, zorder=2
  )
  plt.gca().add_collection(lc)
  plt.gcf().colorbar(
    lc, orientation="horizontal", label="Velocity (m/s)", pad=0.05, aspect=30
  )
  plt.gca().set_aspect('equal', adjustable='box')
  plt.axis('off')
  plt.savefig(dest, bbox_inches='tight')
