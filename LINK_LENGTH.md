# Configurable link length

`PlanTrajectoryRequest.link_length` uses a `oneof` to distinguish a missing value
from an explicitly invalid zero. Missing requests use 0.7835 m; finite values in
the inclusive range [0.75, 1.5] m are accepted. Invalid values return a failed
response with no trajectory. Successful responses report the length actually used.

The request updates the existing IK under the same mutex used for planning.
The source matches deployed image revision `6c5bd7c`; its fitting, FMT selection,
weights and coordinate conventions are preserved. If only length changes, a
cached successful contact path with identical UV endpoints is retained and the
body position is recomputed as `surface + link_length * normal`. Fitting a new
surface invalidates that cache.

Build in the existing ARM64 image, preserving its installed PCL, NURBS and OMPL
dependencies:

```bash
cd /opt/grpc_planner/build
cmake ..
cmake --build . -j2
```

Regenerate the WebUI Python bindings from this directory's `planner.proto` and
deploy them with the server. The new server is backward compatible with omitted
length requests, but old servers cannot confirm length to the new WebUI bridge.
