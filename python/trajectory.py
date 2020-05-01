import math
import numpy as np
import os
import time
from functools import partial
from multiprocessing import Pool
from path import Path
from plot import plot_path
from scipy.optimize import Bounds, minimize, minimize_scalar
from track import Track
from utils import define_corners, idx_modulo
from velocity import VelocityProfile

class Trajectory:
  """
  Stores the geometry and dynamics of a path, handling optimisation of the
  racing line. Samples are taken every metre.
  """
  

  def __init__(self, track, vehicle):
    """Store track and vehicle and initialise a centerline path."""
    self.track = track
    self.ns = math.ceil(track.length)
    self.update(np.full(track.size, 0.5))
    self.vehicle = vehicle
    self.velocity = None


  def update(self, alphas):
    """Update control points and the resulting path."""
    self.alphas = alphas
    self.path = Path(self.track.control_points(alphas), self.track.closed)
    # Sample every metre
    self.s = np.linspace(0, self.path.length, self.ns)


  def update_velocity(self):
    """Generate a new velocity profile for the current path."""
    s = self.s[:-1]
    s_max = self.path.length if self.track.closed else None
    k = self.path.curvature(s)
    self.velocity = VelocityProfile(self.vehicle, s, k, s_max)


  def lap_time(self):
    """Calculate lap time from the velocity profile."""
    return np.sum(np.diff(self.s) / self.velocity.v)


  def minimise_curvature(self):
    """Generate a path minimising curvature."""

    def objfun(alphas):
      self.update(alphas)
      return self.path.gamma2(self.s)

    t0 = time.time()
    res = minimize(
      fun=objfun,
      x0=np.full(self.track.size, 0.5),
      method='L-BFGS-B',
      bounds=Bounds(0.0, 1.0)
    )
    self.update(res.x)
    return time.time() - t0


  def minimise_compromise(self, eps):
    """
    Generate a path minimising a compromise between path curvature and path
    length. eps gives the weight for path length.
    """

    def objfun(alphas):
      self.update(alphas)
      k = self.path.gamma2(self.s)
      d = self.path.length
      return (1-eps)*k + eps*d

    t0 = time.time()
    res = minimize(
      fun=objfun,
      x0=np.full(self.track.size, 0.5),
      method='L-BFGS-B',
      bounds=Bounds(0.0, 1.0)
    )
    self.update(res.x)
    return time.time() - t0


  def minimise_optimal_compromise(self, eps_min=0, eps_max=0.2):
    """
    Determine the optimal compromise weight when using optimise_compromise to
    produce a path.
    """

    def objfun(eps):
      self.minimise_compromise(eps)
      self.update_velocity()
      t = self.lap_time()
      if self.epsilon_history.size > 0:
        self.epsilon_history = np.vstack((self.epsilon_history, [eps, t]))
      else:
        self.epsilon_history = np.array([eps, t])
      return t

    self.epsilon_history = np.array([])
    t0 = time.time()
    res = minimize_scalar(
      fun=objfun,
      method='bounded',
      bounds=(eps_min, eps_max)
    )
    self.epsilon = res.x
    self.minimise_compromise(self.epsilon)
    end = time.time()
    return end - t0


  def minimise_lap_time(self):
    """
    Generate a path that directly minimises lap time.
    """

    def objfun(alphas):
      self.update(alphas)
      self.update_velocity()
      return self.lap_time()

    t0 = time.time()
    res = minimize(
      fun=objfun,
      x0=np.full(self.track.size, 0.5),
      method='L-BFGS-B',
      bounds=Bounds(0.0, 1.0)
    )
    self.update(res.x)
    return time.time() - t0


  def optimise_sectors(self, k_min, proximity, length):
    """
    Generate a path that optimises the path through each sector, and merges
    the results along intervening straights.
    """

    # Define sectors
    t0 = time.time()
    corners, _ = self.track.corners(self.s, k_min, proximity, length)

    # Optimise path for each sector in parallel
    nc = corners.shape[0]
    pool = Pool(os.cpu_count() - 1)
    alphas = pool.map(
      partial(optimise_sector_compromise, corners=corners, traj=self),
      range(nc)
    )
    pool.close()

    # Merge sectors and update trajectory
    alphas = np.sum(alphas, axis=0)
    self.update(alphas)
    return time.time() - t0

###############################################################################

def optimise_sector_compromise(i, corners, traj):
  """
  Builds a new Track for the given corner sequence, and optimises the path
  through it by the compromise method.
  """
  
  # Represent sector as new Track
  nc = corners.shape[0]
  n = traj.track.size
  a = corners[(i-1)%nc,1]  # Sector start
  b = corners[i,0]         # Corner entry
  c = corners[i,1]         # Corner exit
  d = corners[(i+1)%nc,0]  # Sector end
  idxs = idx_modulo(a,d,n)
  sector = Trajectory(
    Track(left=traj.track.left[:,idxs], right=traj.track.right[:,idxs]),
    traj.vehicle
  )
  
  # Optimise path through sector
  sector.minimise_optimal_compromise()
  
  # Weight alphas for merging across straights
  weights = np.ones((d-a)%n)
  weights[:(b-a)%n] = np.linspace(0, 1, (b-a)%n)
  weights[(c-a)%n:] = np.linspace(1, 0, (d-c)%n)
  alphas = np.zeros(n)
  alphas[idxs] = sector.alphas * weights
  
  # Report and plot sector results
  print("  Sector {:d}: eps={:.4f}, run time={:.2f}s".format(
    i, sector.epsilon, rt
  ))
  # plot_path(
  #   "./plots/" + traj.track.name + "_sector" + str(i) + ".png",
  #   sector.track.left, sector.track.right, sector.path.position(sector.s)
  # )
  
  return alphas
