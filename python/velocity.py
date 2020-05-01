import numpy as np
from math import sqrt

GRAV = 9.81 # ms^-2

###############################################################################

class VelocityProfile:
  """
  Stores and generates a velocity profile for a given path and vehicle.
  """

  def __init__(self, vehicle, s, k, s_max=None):
    """
    Generate a velocity profile for the given vehicle and path parameters.
    :s: and :k: should NOT include the overlapping element for closed paths.
    The length of a closed path should be supplied in :s_max:
    """
    self.vehicle = vehicle
    self.s = s
    self.s_max = s_max
    self.limit_local_velocities(k)
    self.limit_acceleration(k)
    self.limit_deceleration(k)
    self.v = np.minimum(self.v_acclim, self.v_declim)


  def limit_local_velocities(self, k):
    self.v_local = np.sqrt(self.vehicle.cof * GRAV / k)

  
  def limit_acceleration(self, k_in):

    # Start at slowest point
    shift = -np.argmin(self.v_local)
    s = np.roll(self.s, shift)
    v = np.roll(self.v_local, shift)
    k = np.roll(k_in, shift)

    # Limit according to acceleration
    for i in range(s.size):
      wrap = i == (shift % s.size)
      if wrap and self.s_max is None: continue
      if v[i] > v[i-1]:
        traction = self.vehicle.traction(v[i-1], k[i-1])
        force = min(self.vehicle.engine_force(v[i-1]), traction)
        accel = force / self.vehicle.mass
        ds = self.s_max - s[i-1] if wrap else s[i] - s[i-1]
        vlim = sqrt(v[i-1]**2 + 2*accel*ds)
        v[i] = min(v[i], vlim)

    # Reset shift and return
    self.v_acclim = np.roll(v, -shift)
  

  def limit_deceleration(self, k_in):

    # Start at slowest point, move backwards
    shift = -np.argmin(self.v_local)
    s = np.flip(np.roll(self.s, shift), 0)
    k = np.flip(np.roll(k_in, shift), 0)
    v = np.flip(np.roll(self.v_local, shift), 0)

    # Limit according to deceleration
    for i in range(s.size):
      wrap = i == (-shift)
      if wrap and self.s_max is None: continue
      if v[i] > v[i-1]:
        traction = self.vehicle.traction(v[i-1], k[i-1])
        decel = traction / self.vehicle.mass
        ds = self.s_max - s[i] if wrap else s[i-1] - s[i]
        vlim = sqrt(v[i-1]**2 + 2*decel*ds)
        v[i] = min(v[i], vlim)

    # Reset shift/flip and return
    self.v_declim = np.roll(np.flip(v, 0), -shift)

