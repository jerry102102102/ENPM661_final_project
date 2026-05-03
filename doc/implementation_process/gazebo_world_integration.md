# Gazebo World Integration Notes

This note documents the current integration path for importing the classmate
Gazebo map into the ACTEA planner.

## Imported Files

The local map folder is:

- `mbgazworld/world_config.json`
- `mbgazworld/json_to_world.py`
- `mbgazworld/output.world`

The JSON describes static box walls and looping orange-ball actors.  The SDF
world is useful for Gazebo visualization, while the JSON is the planner-facing
source of truth.

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

imported = import_gazebo_world_config("mbgazworld/world_config.json", annotation_horizon_s=20.0)
config = method_run_config_from_gazebo_import(imported)
planner_fn = build_controller_planning_function(imported.dynamic_obstacles, config, mode="actea")

start = imported.transform_pose(Pose2D(-4.25, -3.75, 0.0))
goal = imported.transform_pose(Pose2D(4.25, 3.25, 0.0))
route = planner_fn.plan(start, goal)
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
reads `outputs/gazebo_integration/mbgazworld_route.json` and sends steering and
rear-wheel velocity commands to the team-car Gazebo controllers.

## ROS2/Gazebo Run Path

Generate the ACTEA route first:

```bash
PYTHONPYCACHEPREFIX=/tmp/codex_pycache python3 scripts/run_mbgazworld_planner_demo.py
```

Then build and launch the ROS2 workspace:

```bash
source /opt/ros/humble/setup.zsh
colcon build --base-paths ros2_ws/src \
  --packages-select team_car_description team_car_interfaces team_car_control
source install/setup.zsh

ACTEA_PROJECT_ROOT=$PWD ros2 launch team_car_control actea_bringup.launch.py
```

The route JSON contains planner-frame waypoints and Gazebo-frame waypoints.
The Gazebo route follower uses `gazebo_waypoints`, so the controller tracks the
path in the same coordinate frame as the SDF world.
