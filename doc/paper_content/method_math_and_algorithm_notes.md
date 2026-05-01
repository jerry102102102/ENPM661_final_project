# Method Math and Algorithm Notes

This note summarizes the mathematical and algorithmic content that should be
kept in the paper-facing documentation. It combines the nonholonomic planning
model, temporal roadmap search, and ACTEA edge-interval derivation.

## 1. State Space

The robot state is:

```text
x = (px, py, theta)
```

where:

- `px, py` are the robot reference point in the world frame.
- `theta` is the robot heading in radians.

Angles are normalized to:

```text
theta in [-pi, pi)
```

using:

```text
normalize(theta) = ((theta + pi) mod 2pi) - pi
```

The roadmap is therefore not a pure XY graph. A node is a pose:

```text
v_i = (x_i, y_i, theta_i)
```

This matters because a nonholonomic robot may not be able to move directly
between two nearby XY positions if the heading is incompatible.

## 2. Motion Primitive Model

The planner uses a discrete set of forward motion primitives. A primitive is a
control command held for a fixed duration:

```text
u = (u_l, u_r)
```

for the differential-drive model, where `u_l` and `u_r` are wheel RPM commands.

Wheel angular velocities are:

```text
omega_l = 2pi u_l / 60
omega_r = 2pi u_r / 60
```

The corresponding wheel linear velocities are:

```text
v_l = r omega_l
v_r = r omega_r
```

where `r` is the wheel radius.

The body linear and angular velocities are:

```text
v     = (v_r + v_l) / 2
omega = (v_r - v_l) / L
```

where `L` is the track width.

The continuous-time differential-drive kinematics are:

```text
dx/dt     = v cos(theta)
dy/dt     = v sin(theta)
dtheta/dt = omega
```

In implementation, each primitive is integrated over a fixed action duration
with timestep `dt`, producing a sampled trajectory:

```text
tau_e = [q_0, q_1, ..., q_K]
```

Each roadmap edge stores:

- source node id
- target node id
- primitive label
- trajectory samples
- duration
- geometric cost
- static-valid flag

## 3. Static Roadmap Construction

The sampled nonholonomic roadmap is built in three stages.

First, sample query-independent XY points in free space. Second, attach a
discrete heading set:

```text
Theta = {theta_1, theta_2, ..., theta_m}
```

This creates candidate roadmap nodes:

```text
(x, y, theta), theta in Theta
```

Third, roll out motion primitives from each node. If the primitive endpoint is
near another sampled node in both position and heading, and the full sampled
trajectory is statically valid, an edge is added.

The static validity condition is:

```text
for every sampled pose q_k in tau_e:
    robot footprint at q_k is inside map bounds
    robot footprint at q_k does not intersect static obstacles
```

This produces a reusable nonholonomic roadmap. Unlike the seed-expansion graph,
this roadmap is not built for one particular start-goal pair.

## 4. Temporal Search Labels

The temporal planner searches over labels:

```text
label = (node_id, arrival_time, cost_to_come, parent, incoming_edge)
```

If a label reaches node `i` at time `t`, then expanding edge `e = (i, j)` gives:

```text
t_child = t + duration(e)
```

The edge must be dynamically valid at departure time `t`.

The planner maintains a dominance table:

```text
best_cost[(node_id, time_bin)]
```

where:

```text
time_bin = round(arrival_time / time_bin_size)
```

A new label is pruned if another label has already reached the same
`(node_id, time_bin)` with lower or equal cost.

## 5. Online Temporal Edge Validation

For sampled online validation, an edge is checked during query. Each trajectory
sample receives a timestamp:

```text
t_k = t_depart + alpha_k duration(e)
```

where `alpha_k` is the normalized sample time.

For a moving circular obstacle:

```text
o(t) = o_0 + v_o t
```

The online collision check evaluates:

```text
||p_k - o(t_k)|| <= R
```

where:

```text
R = robot_radius + obstacle_radius + clearance
```

If any sample collides, the edge is rejected.

This online check is correct for the sampled trajectory representation, but it
can become expensive when many queries repeatedly test many edge-time pairs.

## 6. ACTEA Edge Annotation

ACTEA precomputes blocked and valid departure-time intervals for each roadmap
edge. The goal is to answer:

```text
Is edge e valid if the robot departs at time t0?
```

without recomputing dynamic obstacle collision checks during every query.

For one piecewise-linear segment of an edge trajectory, the robot center is:

```text
r(s) = r_a + v_r s
```

where `s` is edge-relative time inside the segment.

A constant-velocity circular obstacle is:

```text
o(t0 + s) = o_0 + v_o (t0 + s)
```

Collision occurs when:

```text
||r(s) - o(t0 + s)|| <= R
```

Substitute the two linear expressions:

```text
||r_a + v_r s - o_0 - v_o(t0 + s)|| <= R
```

Rearrange:

```text
||A + B s + C t0|| <= R
```

where:

```text
A = r_a - o_0
B = v_r - v_o
C = -v_o
```

Squaring both sides gives:

```text
||A + B s + C t0||^2 <= R^2
```

ACTEA projects this condition onto the departure-time axis `t0`. It considers:

- endpoint cases of the segment,
- interior closest-point cases where the minimizing `s*(t0)` lies inside the
  segment interval.

The result is a set of blocked departure-time intervals:

```text
B_e = {[a_1, b_1], [a_2, b_2], ...}
```

The valid intervals are the complement over the annotation horizon:

```text
V_e = [T_start, T_end] \ B_e
```

These intervals are stored as first-class edge metadata:

```text
blocked_intervals_exact
valid_intervals_exact
temporal_horizon_s
temporal_annotation_mode = "actea"
blocked_intervals_by_obstacle
```

## 7. Edge-Time Cost

The temporal roadmap planner uses a time-dependent edge cost:

```text
c_e(t0) = geometric_cost(e), if t0 is in a valid interval
c_e(t0) = infinity,          if t0 is in a blocked interval
```

Equivalently:

```text
edge_is_valid_at_time(e, t0) -> bool
edge_cost_at_time(e, t0) -> c_e(t0)
```

This formulation makes ACTEA a temporal roadmap annotation method rather than
only a validator-side cache.

## 8. Heuristic Modes

The planner supports two heuristic modes.

The Euclidean heuristic is:

```text
h(q, goal) = ||p_q - p_goal||
```

The heading/time lower-bound heuristic is:

```text
h(q, goal) =
    distance_to_goal
    + w_heading * heading_error
    + w_time * optimistic_remaining_time
```

where:

```text
optimistic_remaining_time = distance_to_goal / max_primitive_speed
```

This heuristic can reduce label expansions, but the current experiments do not
show improved path quality. Therefore, the ACTEA-vs-online comparison should
use the same heuristic setting for both methods.

## 9. What Is Exact and What Is Not

ACTEA is exact for the implemented piecewise-linear interpolation of primitive
trajectory samples and constant-velocity circular obstacles.

It is not claiming exactness for:

- unsampled continuous differential-drive dynamics before primitive rollout,
- uncertain obstacle motion,
- non-circular moving obstacles,
- arbitrary acceleration profiles.

This distinction is important for the final report.
