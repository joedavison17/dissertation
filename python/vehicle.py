from math import sqrt
import json
import numpy as np

GRAV = 9.81 # m/s^2

###############################################################################

class Vehicle:
  """Vehicle parameters and behaviour."""

  def __init__(self, path):
    """Load vehicle data from JSON file."""
    vehicle_data = json.load(open(path))
    self.name = vehicle_data["name"]
    self.mass = vehicle_data["mass"]
    self.cof = vehicle_data["frictionCoefficient"]
    self.engine_profile = [
      vehicle_data["engineMap"]["v"],
      vehicle_data["engineMap"]["f"]
    ]
    print("[ Imported {} ]".format(self.name))


  def engine_force(self, velocity, gear=None):
    """Map current velocity to force output by the engine."""
    return np.interp(velocity, self.engine_profile[0], self.engine_profile[1])


  def traction(self, velocity, curvature):
    """Determine remaining traction when negotiating a corner."""
    f = self.cof * self.mass * GRAV
    f_lat = self.mass * velocity**2 * curvature
    if f <= f_lat: return 0
    return sqrt(f**2 - f_lat**2)

