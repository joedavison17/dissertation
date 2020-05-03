import argparse
import os
from enum import IntEnum, unique
from plot import plot_corners, plot_path, plot_trajectory
from track import Track
from trajectory import Trajectory
from vehicle import Vehicle

###############################################################################
## Argument parsing

@unique
class Method(IntEnum):
  CURVATURE = 0
  COMPROMISE = 1
  DIRECT = 2
  COMPROMISE_SECTORS = 3
  COMPROMISE_ESTIMATED = 4

parser = argparse.ArgumentParser(description='Racing line optimisation')
parser.add_argument('track',
  nargs=1, type=str,
  help='path to JSON containing track data'
)
parser.add_argument('vehicle',
  nargs=1, type=str,
  help='path to JSON containing vehicle data'
)
methods = parser.add_argument_group('generation methods').add_mutually_exclusive_group(required=True)
methods.add_argument('--curvature',
  action='store_const', dest='method', const=Method.CURVATURE,
  help='minimise curvature'
)
methods.add_argument('--compromise',
  action='store_const', dest='method', const=Method.COMPROMISE,
  help='minimise an optimal length-curvature compromise'
)
methods.add_argument('--laptime',
  action='store_const', dest='method', const=Method.DIRECT,
  help='directly minimise lap time'
)
methods.add_argument('--sectors',
  action='store_const', dest='method', const=Method.COMPROMISE_SECTORS,
  help='optimise and merge sector paths'
)
methods.add_argument('--estimated',
  action='store_const', dest='method', const=Method.COMPROMISE_ESTIMATED,
  help='minimise a pre-computed length-curvature compromise'
)
parser.add_argument('--plot-corners',
  action='store_true', dest='plot_corners',
  help='plot detected corners'
)
parser.add_argument('--plot-path',
  action='store_true', dest='plot_path',
  help='plot the generated path'
)
parser.add_argument('--plot-trajectory',
  action='store_true', dest='plot_trajectory',
  help='plot the generated path with velocity gradient'
)
parser.add_argument('--plot-all',
  action='store_true', dest='plot_all',
  help='plot all relevant graphs'
)
parser.add_argument('--plot-format',
  type=str, dest='ext', default='png',
  help='file format used to save plots'
)
args = parser.parse_args()

###############################################################################
## Generation

track = Track(args.track[0])
vehicle = Vehicle(args.vehicle[0])
trajectory = Trajectory(track, vehicle)

# Corner detection parameters
K_MIN = 0.03
PROXIMITY = 40
LENGTH = 10

if args.method is Method.CURVATURE:
  print("[ Minimising curvature ]")
  run_time = trajectory.minimise_curvature()
elif args.method is Method.COMPROMISE:
  print("[ Minimising optimal compromise ]")
  run_time = trajectory.minimise_optimal_compromise()
  print("  epsilon = {:.4f}".format(trajectory.epsilon))
elif args.method is Method.DIRECT:
  print("[ Minimising lap time ]")
  run_time = trajectory.minimise_lap_time()
elif args.method is Method.COMPROMISE_SECTORS:
  print("[ Optimising sectors ]")
  run_time = trajectory.optimise_sectors(K_MIN, PROXIMITY, LENGTH)
elif args.method is Method.COMPROMISE_ESTIMATED:
  print("[ Minimising pre-computed compromise ]")
  mask = track.corners(trajectory.s, K_MIN, PROXIMITY, LENGTH)[1]
  epsilon = 0.406 * track.avg_curvature(trajectory.s[mask])
  print("  epsilon = {:.4f}".format(epsilon))
  run_time = trajectory.minimise_compromise(epsilon)
else:
  raise ValueError("Did not recognise args.method {}".format(args.method))

print("[ Computing lap time ]")
trajectory.update_velocity()
lap_time = trajectory.lap_time()

print()
print("=== Results ==========================================================")
print("Lap time = {:.3f}".format(lap_time))
print("Run time = {:.3f}".format(run_time))
print("======================================================================")
print()

###############################################################################
## Plotting

method_dirs = ['curvature', 'compromise', 'laptime', 'sectors', 'estimated']
plot_dir = os.path.join(
  os.path.dirname(__file__), '..', 'data', 'plots', track.name,
  method_dirs[args.method]
)
if not os.path.exists(plot_dir): os.makedirs(plot_dir)

if args.plot_corners or args.plot_all:
  plot_corners(
    os.path.join(plot_dir, "corners." + args.ext),
    track.left, track.right, track.mid.position(trajectory.s),
    track.corners(trajectory.s, K_MIN, PROXIMITY, LENGTH)[1]
  )

if args.plot_path or args.plot_all:
  plot_path(
    os.path.join(plot_dir, "path." + args.ext),
    track.left, track.right, trajectory.path.position(trajectory.s),
    trajectory.path.controls
  )

if args.plot_trajectory or args.plot_all:
  plot_trajectory(
    os.path.join(plot_dir, "trajectory." + args.ext),
    track.left, track.right, trajectory.path.position(trajectory.s),
    trajectory.velocity.v
  )