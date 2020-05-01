# Racing Line Optimisation for Autonomous Vehicles
  
  BSc Computer Science 2019-20, University of Bath.

---

## Racing Line Generation

See data/{tracks,vehicles}/*.json for input format.

```
usage: main.py [-h] [--track path/to/track.json]
               [--vehicle path/to/vehicle.json] [--plot-corners] [--plot-path]
               [--plot-trajectory] [--plot-all] [--plot-format EXT]
               (--curvature | --compromise | --laptime | --sectors | --estimated)

Racing line optimisation

optional arguments:
  -h, --help            show this help message and exit
  --track PATH          path to JSON containing track data
  --vehicle PATH        path to JSON containing vehicle data
  --plot-corners        plot detected corners
  --plot-path           plot the generated path
  --plot-trajectory     plot the generated path with velocity gradient
  --plot-all            plot all relevant graphs
  --plot-format EXT     file format used to save plots

generation methods:
  --curvature           minimise curvature
  --compromise          minimise an optimal length-curvature compromise
  --laptime             directly minimise lap time
  --sectors             optimise and merge sector paths
  --estimated           minimise a pre-computed length-curvature compromise
```

For example, to optimise a racing line for Buckmore Park by minimising curvature:

```
cd python
python main.py --track ../data/tracks/buckmore.json --vehicle ../data/vehicles/tbr18.json --curvature --plot-all
```
