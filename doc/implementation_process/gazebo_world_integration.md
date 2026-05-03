# Gazebo World Integration Notes

This note documents the current integration path for running the ACTEA planner
with a small Gazebo team-car scene.

## Imported Files

The local map folder is:

- `mbgazworld/world_config.json`
- `mbgazworld/json_to_world.py`
- `mbgazworld/output.world`

The JSON now describes a small ACTEA demo scene, not the original classmate
map.  Its scale is close to the 2D GIF demo: a team car starts near
`(0.35, 0.35)`, static boxes form a small gate, and periodic orange balls move
slowly through the route.  The SDF world is useful for Gazebo visualization,
while the JSON is the planner-facing source of truth.

## Dynamic Obstacle Compatibility

The planner's native dynamic obstacle model is a circular obstacle with
constant velocity:

```text
p(t) = p0 + v t
```

The Gazebo actors use looping waypoint trajectories.  To keep the planner core
unchanged, `src/integrations/gazebo_world.py` unfolds each looping waypoint
trajectory over a finite annotation horizon and converts it into time-gated
constant-velocity circle segments.

For example:

```text
0s -> 3s:  move right
3s -> 6s:  move left
6s -> 9s:  move right
...
```

becomes:

```text
DynamicCircleObstacle(..., active_start_time_s=0, active_end_time_s=3)
DynamicCircleObstacle(..., active_start_time_s=3, active_end_time_s=6)
DynamicCircleObstacle(..., active_start_time_s=6, active_end_time_s=9)
...
```

ACTEA then computes blocked and valid departure-time intervals against these
active-time-bounded constant-velocity segments.

## Controller-Facing Planner Function

`src/integrations/planner_api.py` exposes a reusable planning object:

```python
from src.integrations.gazebo_world import import_gazebo_world_config, method_run_config_from_gazebo_import
from src.integrations.planner_api import build_controller_planning_function
from src.models.state import Pose2D

imported = import_gazebo_world_config("mbgazworld/world_config.json", annotation_horizon_s=30.0)
config = method_run_config_from_gazebo_import(imported)
planner_fn = build_controller_planning_function(imported.dynamic_obstacles, config, mode="actea")

start = imported.transform_pose(Pose2D(0.35, 0.35, 0.0))
goal = imported.transform_pose(Pose2D(3.65, 1.65, 0.0))
route = planner_fn.plan(start, goal, start_time_s=6.0)
```

The returned route contains:

- path waypoints
- timed waypoints
- primitive trajectory segments
- per-segment command metadata
- traversal time and path cost

The closed-loop controller should track the timed waypoint sequence or execute
the primitive command sequence, depending on which interface is more convenient.

## Demo Script

Run one imported-map ACTEA query:

```bash
PYTHONPYCACHEPREFIX=/tmp/codex_pycache python3 scripts/run_mbgazworld_planner_demo.py
```

Output:

```text
outputs/gazebo_integration/mbgazworld_route.json
```

The ROS2 package also includes a packaged default route:

```text
ros2_ws/src/team_car_control/routes/mbgazworld_route.json
```

The default route is planned for simulation time `6.0s` and declares
`route_period_s = 24.0`.  The control node reads Gazebo simulation time and
waits for the next equivalent departure phase `6 + 24k` before moving, so
periodic obstacle positions and ACTEA departure-time reasoning stay aligned
even when scene and controller are launched separately.

`actea_control.launch.py` and `actea_bringup.launch.py` use the regenerated
output route when it exists and otherwise fall back to this packaged route.
This lets a fresh clone launch the Gazebo demo after a ROS2 build, while still
allowing planner changes to produce a new route.

## External Repo Import

External reference repositories should be cloned under `.external_repos/` or
`external_repos/`; both folders are ignored by git.  Once the source repo URL is
known, vehicle assets, Gazebo launch scripts, and controller files can be copied
or wrapped without committing the cloned reference repo itself.

The Group 4 Phase 2 repository was imported from:

```text
https://github.com/mahmed1510/ENPM661_Group4_Project3
```

The cloned reference copy stays ignored under:

```text
.external_repos/ENPM661_Group4_Project3/
```

The reusable ROS2/Gazebo packages are vendored into:

```text
ros2_ws/src/team_car_description/
ros2_ws/src/team_car_interfaces/
ros2_ws/src/team_car_control/
```

`team_car_description` provides the vehicle URDF, STL meshes, ros2_control
configuration, and the `mbgazworld_actea.sdf` world.  `team_car_control` now
includes `actea_route_follower`, a closed-loop pure-pursuit route follower that
reads an ACTEA route JSON and sends steering and rear-wheel velocity commands
to the team-car Gazebo controllers.

## ROS2/Gazebo Run Path

Generate the ACTEA route:

```bash
PYTHONPYCACHEPREFIX=/tmp/codex_pycache python3 scripts/run_mbgazworld_planner_demo.py
```

Then build the ROS2 workspace:

```bash
source /opt/ros/humble/setup.zsh
colcon build --base-paths ros2_ws/src \
  --packages-select team_car_description team_car_interfaces team_car_control
source install/setup.zsh
```

Recommended one-command launch:

```bash
ros2 launch team_car_control actea_bringup.launch.py
```

For debugging, the scene and control can also be run separately:

```bash
ros2 launch team_car_description actea_scene.launch.py
```

```bash
ros2 launch team_car_control actea_control.launch.py
```

The route JSON contains planner-frame waypoints and Gazebo-frame waypoints.
The Gazebo route follower uses `gazebo_waypoints`, so the controller tracks the
path in the same coordinate frame as the SDF world.

The Gazebo world in `mbgazworld_actea.sdf` uses an inline directional light and
an inline ground plane instead of `model://sun` and `model://ground_plane`.
This avoids environment-dependent failures when the standard Gazebo model
database is not installed or not on the resource path.
