# T-PRM Paper Notes and Implementation Plan

Project: TA-PRM* / T-PRM for Dynamic-Environment Path Planning
Primary paper: **T-PRM: Temporal Probabilistic Roadmap for Path Planning in Dynamic Environments**
Goal of this note: turn the paper reading into a concrete implementation plan for our final project.

---

## 1. One-Sentence Summary

Traditional PRM builds a reusable roadmap for static environments, but it does not know *when* each node is safe. T-PRM extends PRM by attaching **time availability intervals** to roadmap nodes and then uses a **time-aware A\*** search to find paths that are safe not only in space, but also at the time the robot reaches each node.

In simple terms:

```text
PRM:
Can this point be used?

T-PRM:
Can this point be used at the time the robot arrives there?
```

---

## 2. Problem Motivation

### 2.1 Static path planning is not enough

In a static environment, a point or edge is either collision-free or not. If a node is outside an obstacle, it stays valid forever.

However, in a dynamic environment, safety depends on time.

Example:

```text
Node v = (5, 5)

At t = 0: v is safe
At t = 4: a moving obstacle passes through v
At t = 7: v is safe again
```

So the planner cannot only ask:

```text
Is v collision-free?
```

It must ask:

```text
Will v be collision-free when the robot reaches it?
```

This is the main reason T-PRM adds time information to PRM.

---

## 3. Background: PRM

### 3.1 What is PRM?

PRM stands for **Probabilistic Roadmap**.

It builds a graph:

```text
G = (V, E)
```

where:

```text
V = sampled configurations / waypoints
E = feasible connections between nearby waypoints
```

### 3.2 PRM learning phase

The learning phase builds the roadmap.

Basic process:

```text
1. Randomly sample nodes in free space
2. Reject nodes inside static obstacles
3. Connect nearby nodes using a local planner
4. Add an edge if the local path is collision-free
5. Repeat until enough nodes are sampled
```

### 3.3 PRM query phase

The query phase searches the roadmap.

Basic process:

```text
1. Connect start to the nearest roadmap node
2. Connect goal to the nearest roadmap node
3. Use Dijkstra or A* to find a path through the graph
4. Return the sequence of nodes as the path
```

### 3.4 Why PRM is useful

PRM is good for **multi-query planning**.

That means:

```text
Build the roadmap once.
Reuse it for many start-goal queries.
```

This is the main property T-PRM wants to preserve.

---

## 4. Why Vanilla PRM Fails in Dynamic Environments

Vanilla PRM assumes the environment is static.

A roadmap node is treated as permanently valid if it is not inside a static obstacle. This becomes unsafe when obstacles move.

A path can be geometrically collision-free but temporally unsafe.

Example:

```text
Path:
S -> A -> B -> G

Arrival times:
A at t = 1
B at t = 3
G at t = 4

Problem:
B is occupied by a moving obstacle at t = 3.
```

Vanilla PRM would think the path is valid because B is spatially free in the static map.
T-PRM rejects the path because B is not safe at the arrival time.

---

## 5. T-PRM Core Idea

T-PRM keeps the normal PRM structure but adds time information to each node.

A normal PRM node stores:

```text
v = (x, y)
```

A T-PRM node stores:

```text
v = (x, y)
TA_v = time availability intervals
```

Example:

```text
v = (5, 5)

TA_v = [0, 2.5] ∪ [4.0, 10.0] ∪ [12.0, ∞)
```

Meaning:

```text
The node is safe from 0 to 2.5 seconds.
The node is unsafe from 2.5 to 4.0 seconds.
The node is safe again from 4.0 to 10.0 seconds.
The node is unsafe from 10.0 to 12.0 seconds.
The node is safe after 12.0 seconds.
```

---

## 6. Main Assumptions in the Paper

The paper makes several important assumptions.

### 6.1 Holonomic robot

The robot is assumed to be holonomic.

That means it can move directly in any direction.

Good examples:

```text
Drone approximation
Point robot in 2D
Omnidirectional robot
```

Bad examples:

```text
Car-like robot
Differential drive robot with strong turning constraints
```

For our project, we should start with a 2D point robot or circular robot.

---

### 6.2 Robot is modeled as a point

The paper simplifies the robot as a point with no physical size.

To keep safety, obstacles are inflated by a safety margin.

Example:

```text
Robot radius = 0.3 m

Instead of modeling the robot as a circle,
inflate all obstacles by 0.3 m
and plan with a point robot.
```

For our implementation, this is a good simplification.

---

### 6.3 Moving obstacle velocity is known

The paper assumes moving obstacles have known position and velocity.

The clean version assumes constant velocity.

Example:

```text
obstacle position at t = 0:
p0 = (2, 2)

obstacle velocity:
v = (1, 0) m/s

predicted position:
p(t) = p0 + v * t
```

This assumption lets T-PRM compute when each roadmap node will be blocked.

---

## 7. Time Availability

### 7.1 Definition

For each node `v`, T-PRM computes:

```text
TA_v = time intervals when v is safe
```

A node is safe at time `t` if no moving obstacle overlaps the node at that time.

---

### 7.2 Single moving obstacle

For one obstacle `o`, we compute:

```text
TA_v^o = times when node v is not blocked by obstacle o
```

Example:

```text
Obstacle crosses node v from t = 3.2 to t = 4.1

Then:
TA_v^o = [0, 3.2] ∪ [4.1, ∞)
```

---

### 7.3 Multiple moving obstacles

If there are multiple moving obstacles, the node must be safe from all of them.

So:

```text
TA_v = intersection of TA_v^o over all moving obstacles o
```

Meaning:

```text
A node is available only when no moving obstacle blocks it.
```

Example:

```text
Obstacle A blocks v during [2, 3]
Obstacle B blocks v during [5, 6]

Then v is unavailable during [2, 3] and [5, 6].
```

---

## 8. Important Design Choice: Node Availability Instead of Edge Availability

The paper assigns time availability to nodes, not edges.

So it checks:

```text
Is node w safe when the robot arrives at w?
```

It does not fully check:

```text
Is every point along edge v -> w safe at every time during traversal?
```

The paper justifies this by keeping edges shorter than the obstacle dimensions.

However, for our implementation, we should be more conservative.

Recommended project choice:

```text
Check both:
1. Node arrival-time safety
2. Dynamic collision samples along the edge
```

This will make our planner safer and easier to defend in the final report.

---

## 9. T-PRM Learning Phase

The learning phase builds the roadmap and computes time availability.

### 9.1 Input

```text
Static obstacles
Moving obstacles with initial position and velocity
Maximum number of roadmap nodes
Maximum edge length
Safety margin
```

### 9.2 Output

```text
Graph G = (V, E)

Each node v has:
- position
- time availability intervals TA_v

Each edge e has:
- endpoint nodes
- distance cost
- traversal duration
```

### 9.3 Algorithm

```text
while number of sampled nodes < max_nodes:
    sample candidate node v uniformly in workspace

    if v is inside a static obstacle:
        reject v
        continue

    compute TA_v based on moving obstacles

    add v to V

    find nearby existing nodes

    for each nearby node u:
        if distance(u, v) <= max_edge_length:
            if straight-line path u -> v avoids static obstacles:
                add edge (u, v)
```

### 9.4 Why uniform sampling?

Many PRM variants try to sample fewer points in open spaces.

The paper argues that in dynamic environments, this can be risky.

Reason:

```text
If the roadmap depends on a small number of key nodes,
a moving obstacle can temporarily block one key node
and disconnect a large part of the graph.
```

Uniform sampling creates more redundant connections, which helps when some nodes are temporarily unavailable.

---

## 10. T-PRM Query Phase

The query phase finds a timed path from start to goal.

### 10.1 Input

```text
Roadmap G
Start position s
Goal position g
Robot speed
```

### 10.2 Output

```text
A path from start to goal
A timing for each node along the path
```

Example output:

```text
S at t = 0.0
A at t = 1.2
B at t = 2.7
C at t = 4.1
G at t = 5.3
```

### 10.3 High-level process

```text
1. Connect start to nearest roadmap node
2. Connect goal to nearest roadmap node
3. Run time-aware A* on the roadmap
4. Return path with arrival times
```

---

## 11. Time-Aware A*

### 11.1 Why normal A* is not enough

Normal A* keeps track of:

```text
cost_so_far[v]
```

T-PRM needs to also track:

```text
arrival_time[v]
```

Because a node can be safe at one time and blocked at another time.

---

### 11.2 Core rule

When expanding from node `v` to neighbor `w`:

```text
arrival_time_w = arrival_time_v + duration(edge v -> w)
```

Then check:

```text
Is w available at arrival_time_w?
```

If yes, the expansion is allowed.
If no, the expansion is rejected.

---

### 11.3 Pseudocode

```text
open_queue = priority queue ordered by f_cost
arrival_time[start] = 0
cost_to_go[start] = 0
f_cost[start] = heuristic(start, goal)

push start into open_queue

if start is not available at t = 0:
    return failure

while open_queue is not empty:
    v = pop node with smallest f_cost

    if v == goal:
        return reconstructed path

    for each neighbor w of v:
        dt = duration(edge v -> w)
        t_w = arrival_time[v] + dt

        if w is not available at t_w:
            continue

        tentative_cost = cost_to_go[v] + edge_cost(v, w)

        if tentative_cost improves w:
            predecessor[w] = v
            arrival_time[w] = t_w
            cost_to_go[w] = tentative_cost
            f_cost[w] = tentative_cost + heuristic(w, goal)
            push or update w in open_queue

return failure
```

---

### 11.4 Heuristic

The paper uses Euclidean distance to the goal:

```text
h(v) = ||v - goal||
```

For our 2D implementation:

```python
h = np.linalg.norm(node.position - goal.position)
```

---

### 11.5 Time monotonicity

Time-aware A* automatically respects time monotonicity because:

```text
arrival_time[next] = arrival_time[current] + positive_duration
```

So time always moves forward.

This prevents impossible paths where the robot appears to move backward in time.

---

## 12. Multiple Queries and Replanning

### 12.1 Multiple queries

If obstacle velocities are known and constant, then the same roadmap can be reused.

The graph structure does not need to change.

```text
Nodes stay the same.
Edges stay the same.
TA_v stays valid as long as obstacle predictions stay valid.
```

This preserves the multi-query advantage of PRM.

---

### 12.2 Replanning when obstacle velocity changes

If an obstacle changes velocity, the old time availability intervals become invalid.

However, we do not need to rebuild the whole roadmap.

We can:

```text
1. Keep the same sampled nodes
2. Keep the same edges
3. Recompute TA_v for each node
4. Run time-aware A* again
```

This is much faster than resampling and rewiring the whole graph.

---

## 13. Complexity Summary

Let:

```text
|V| = number of roadmap nodes
|E| = number of edges
|O| = number of moving obstacles
```

### 13.1 Learning phase

Computing time availability:

```text
O(|V| * |O|)
```

Brute-force edge construction:

```text
O(|V|^2)
```

Total:

```text
O(|V|^2 + |V| * |O|)
```

In practice, edge construction usually dominates because:

```text
|O| << |V|
```

---

### 13.2 Query phase

Standard A*:

```text
O(|V| log |V| + |E|)
```

T-PRM A* adds time availability lookup.

If intervals are sorted, availability lookup can be done in:

```text
O(log |O|)
```

So query complexity becomes approximately:

```text
O(log |O| * (|V| log |V| + |E|))
```

---

### 13.3 Replanning

Recomputing all node time availability intervals:

```text
O(|V| * |O|)
```

---

## 14. Experiments in the Paper

The paper evaluates T-PRM in both simulation and real-world UAV experiments.

### 14.1 Simulation setup

The simulation environments are:

```text
2D: 10 m x 10 m
3D: 10 m x 10 m x 10 m
```

The task:

```text
Plan from one corner to the diagonally opposite corner.
```

Compared planners:

```text
1. T-PRM
2. OMPL PRM
3. OMPL RRT*
4. RRT*-FND
```

Metrics:

```text
Path length
Computation time
Success rate in dynamic scenes
```

---

### 14.2 Static environment experiments

Purpose:

```text
Check whether T-PRM still behaves well in normal static planning.
```

Main result:

```text
T-PRM usually produces short and smooth paths.
RRT* is often faster in static scenes.
```

Interpretation:

```text
T-PRM is not designed to be the fastest static planner.
Its advantage appears mainly in dynamic environments.
```

---

### 14.3 Narrow gap limitation

The paper shows that T-PRM inherits PRM's narrow passage weakness.

In a narrow gap scenario:

```text
1000 nodes:
T-PRM fails to find a path

5000 nodes:
T-PRM finds a path, but computation time increases
```

Takeaway:

```text
T-PRM helps with dynamic obstacles,
but it does not magically solve PRM's sampling-density limitations.
```

This should be included in our final report as a limitation.

---

### 14.4 Dynamic environment experiments

The paper tests dynamic scenes with many moving obstacles:

```text
50, 100, 500, 1000 moving obstacles
```

Main result:

```text
As the number of moving obstacles increases,
T-PRM keeps high success rate and short paths,
while static planners with replanning degrade.
```

Important intuition:

```text
T-PRM plans with future obstacle motion in mind.
Vanilla PRM / RRT* with replanning reacts after the path becomes blocked.
```

---

### 14.5 Real-world UAV experiments

The paper also tests T-PRM on a UAV.

Important details:

```text
UAV: Holybro X500
Onboard computer: Intel NUC
Low-level controller: Pixhawk
State estimation: IMU + VICON
```

Real-world tests include:

```text
1. Static obstacle avoidance
2. Virtual moving sphere avoidance
3. Real moving UAV obstacle avoidance
```

Main takeaway:

```text
T-PRM can recompute time availability and query paths fast enough for onboard real-time replanning.
```

We do not need to reproduce UAV experiments for our project.

---

## 15. What We Should Implement

Our project should implement a simplified but faithful 2D version of T-PRM.

### 15.1 Recommended scope

```text
2D square workspace
Point robot or circular robot via obstacle inflation
Static circular obstacles
Moving circular obstacles with known constant velocity
Uniform PRM sampling
Time availability per node
Time-aware A*
Baseline vanilla PRM + replanning
Visualization and quantitative experiments
```

### 15.2 Avoid for first version

```text
No ROS 2 required
No Gazebo required
No UAV model
No full robot dynamics
No nonholonomic constraints
No 3D simulation initially
```

This keeps the project realistic and focused.

---

## 16. Proposed Software Structure

Recommended repo structure:

```text
t_prm_project/
    README.md
    requirements.txt
    src/
        geometry.py
        obstacles.py
        roadmap.py
        time_availability.py
        planners/
            prm.py
            t_prm.py
            astar.py
            time_aware_astar.py
        simulation.py
        metrics.py
        visualization.py
    experiments/
        run_static_experiment.py
        run_dynamic_density_experiment.py
        run_obstacle_speed_experiment.py
        run_narrow_gap_experiment.py
    results/
        figures/
        tables/
    docs/
        t_prm_paper_notes_and_plan.md
```

---

## 17. Core Data Structures

### 17.1 Node

```python
@dataclass
class Node:
    id: int
    position: np.ndarray
    time_availability: list[tuple[float, float]]
```

Example:

```python
Node(
    id=3,
    position=np.array([5.0, 2.0]),
    time_availability=[(0.0, 3.2), (4.5, float("inf"))]
)
```

---

### 17.2 Edge

```python
@dataclass
class Edge:
    u: int
    v: int
    length: float
    duration: float
```

Duration can be computed as:

```python
duration = length / robot_speed
```

---

### 17.3 Static obstacle

```python
@dataclass
class StaticCircleObstacle:
    center: np.ndarray
    radius: float
```

---

### 17.4 Moving obstacle

```python
@dataclass
class MovingCircleObstacle:
    initial_center: np.ndarray
    velocity: np.ndarray
    radius: float

    def position_at(self, t: float) -> np.ndarray:
        return self.initial_center + self.velocity * t
```

---

## 18. Computing Time Availability

For each node and moving obstacle, we need to compute when the moving obstacle overlaps the node.

For a circular moving obstacle:

```text
obstacle_center(t) = p0 + v * t
node_position = x
collision if ||obstacle_center(t) - x|| <= radius + safety_margin
```

This becomes:

```text
||p0 + v t - x||^2 <= R^2
```

This is a quadratic inequality in `t`.

We solve it to find the blocked interval.

Then:

```text
availability interval = planning horizon minus blocked intervals
```

For multiple obstacles:

```text
blocked intervals from all obstacles are merged
available intervals are the complement of merged blocked intervals
```

Recommended simplification:

```text
Use a finite planning horizon, for example T = 30 seconds.
Represent availability inside [0, T].
```

Example:

```text
planning horizon: [0, 30]

blocked intervals:
[3, 5], [8, 10]

availability:
[0, 3], [5, 8], [10, 30]
```

---

## 19. Baseline Planner

We should compare against a simple baseline:

```text
Vanilla PRM + replanning
```

### 19.1 Baseline behavior

```text
1. Build normal PRM using static obstacles only
2. Find path from start to goal
3. Simulate robot moving along path
4. Check if moving obstacle collides with robot
5. If collision predicted or detected, replan from current position
```

### 19.2 Why this is a fair baseline

This baseline represents a static planner that does not know future obstacle timing.

It reacts when the path becomes unsafe.

T-PRM should perform better because it plans with obstacle timing from the beginning.

---

## 20. Proposed Experiments for Our Final Project

### Experiment 1: Static baseline sanity check

Purpose:

```text
Show that T-PRM still works in static environments.
```

Setup:

```text
Workspace: 10 x 10
Start: bottom-left
Goal: top-right
Static obstacles: 0, 5, 10, 20
Moving obstacles: 0
```

Compare:

```text
Vanilla PRM
T-PRM
```

Metrics:

```text
Path length
Planning time
Success rate
```

Expected result:

```text
T-PRM should behave similarly to PRM.
It may be slightly slower due to time-aware bookkeeping.
```

---

### Experiment 2: Dynamic obstacle density

Purpose:

```text
Show that T-PRM becomes more useful as moving obstacles increase.
```

Setup:

```text
Workspace: 10 x 10
Moving obstacles: 5, 10, 20, 50
Obstacle speed: fixed medium range
```

Compare:

```text
Vanilla PRM + replanning
T-PRM
```

Metrics:

```text
Success rate
Collision count
Path length
Planning time
Number of replans
```

Expected result:

```text
T-PRM should have fewer collisions and fewer replans.
```

---

### Experiment 3: Obstacle speed sensitivity

Purpose:

```text
Test whether faster dynamic obstacles make planning harder.
```

Setup:

```text
Moving obstacle count: fixed
Speed scale:
    slow
    medium
    fast
```

Metrics:

```text
Success rate
Collision count
Path length
Planning time
```

Expected result:

```text
Fast obstacles should hurt both planners,
but T-PRM should degrade more gracefully.
```

---

### Experiment 4: Narrow passage limitation

Purpose:

```text
Show the known weakness inherited from PRM.
```

Setup:

```text
Workspace with a narrow passage
Node counts:
    200
    500
    1000
    3000
```

Metrics:

```text
Success rate
Roadmap construction time
Query time
```

Expected result:

```text
More nodes improve success,
but construction time increases.
```

This experiment is useful because it shows we understand the method's limitation.

---

## 21. Visualizations to Generate

We should generate figures for the final report.

Recommended figures:

```text
1. Roadmap graph with static obstacles
2. T-PRM path with moving obstacle trajectories
3. Node time availability example
4. Comparison of vanilla PRM path vs T-PRM path
5. Success rate vs number of moving obstacles
6. Planning time vs number of moving obstacles
7. Path length vs number of moving obstacles
8. Narrow passage success vs number of nodes
```

For dynamic scenes, show:

```text
Moving obstacles as red circles
Obstacle trajectories as dashed lines
Robot path as blue or green line
Roadmap nodes as gray dots
Start and goal markers
```

---

## 22. Metrics

### 22.1 Path length

```text
Sum of distances between consecutive path waypoints.
```

### 22.2 Planning time

```text
Wall-clock time for the planner to return a path.
```

For T-PRM, separate:

```text
Roadmap construction time
Time availability computation time
Query time
```

### 22.3 Success rate

```text
Number of successful trials / total trials
```

A trial is successful if:

```text
The robot reaches the goal without collision within the time horizon.
```

### 22.4 Collision count

```text
Number of collisions during path execution simulation.
```

### 22.5 Number of replans

Especially for vanilla PRM + replanning:

```text
How many times the planner had to generate a new path.
```

---

## 23. Known Limitations to Mention

### 23.1 Known obstacle velocity assumption

T-PRM works best when moving obstacle velocities are known or can be estimated.

If obstacle motion is highly unpredictable, time availability must be recomputed frequently.

---

### 23.2 Node-based safety is approximate

The paper stores time availability at nodes.

This may miss collisions along edges.

Our implementation should reduce this problem by sampling along edges during execution or planning.

---

### 23.3 Narrow passage problem

T-PRM inherits PRM's weakness in narrow passages.

More samples can help, but increase computation time.

---

### 23.4 Holonomic simplification

The paper assumes holonomic robots and first-order dynamics.

It does not directly solve planning for car-like robots or systems with strong dynamics.

---

## 24. Minimum Viable Implementation

The minimum version we need for a successful project:

```text
1. Generate 2D workspace
2. Add static and moving circular obstacles
3. Sample PRM nodes uniformly
4. Build static-collision-free edges
5. Compute node time availability intervals
6. Implement time-aware A*
7. Simulate path execution
8. Implement vanilla PRM + replanning baseline
9. Run at least two experiments:
    - dynamic obstacle density
    - obstacle speed sensitivity
10. Generate plots and tables
```

---

## 25. Stretch Goals

If time allows:

```text
1. Edge-level dynamic collision checking
2. Animated visualization
3. Narrow passage experiment
4. 3D extension
5. ROS 2 / Gazebo demo
6. TA-PRM* comparison from the 2024 paper
```

Recommended stretch priority:

```text
1. Edge-level dynamic collision checking
2. Animated visualization
3. Narrow passage experiment
```

These are more useful for the final report than jumping into ROS 2 too early.

---

## 26. Suggested Implementation Order

### Step 1: Geometry and obstacles

Implement:

```text
Point-circle collision
Line-circle static collision
Moving circle position at time t
```

### Step 2: Vanilla PRM

Implement:

```text
Uniform node sampling
Neighbor connection
A* search
Path extraction
```

### Step 3: Dynamic obstacle simulation

Implement:

```text
Obstacle motion
Robot path execution
Collision checking over time
```

### Step 4: Time availability

Implement:

```text
Blocked interval computation
Blocked interval merging
Availability interval computation
```

### Step 5: Time-aware A*

Implement:

```text
Arrival time tracking
Availability check during expansion
Path reconstruction with timing
```

### Step 6: Baseline replanning

Implement:

```text
Vanilla PRM path execution
Collision prediction / detection
Replan from current robot position
```

### Step 7: Experiments

Implement:

```text
Static sanity check
Dynamic obstacle density
Obstacle speed sensitivity
Narrow passage if time allows
```

### Step 8: Report figures

Generate:

```text
Roadmap plots
Trajectory plots
Metric plots
Comparison tables
```

---

## 27. Final Report Angle

The final report should not claim that T-PRM solves all dynamic planning problems.

A strong and honest report angle:

```text
This project implements a simplified 2D version of T-PRM to evaluate whether adding time availability intervals to PRM improves planning performance in dynamic environments. We compare T-PRM against vanilla PRM with replanning under different moving obstacle densities and speeds. The results show when time-aware planning improves safety and reduces replanning, while also highlighting inherited PRM limitations such as sensitivity to sampling density in narrow passages.
```

---

## 28. Key Takeaway

The most important idea from the paper is:

```text
A path is not safe just because it avoids obstacles in space.
It is safe only if the robot reaches every part of the path at a time when that part is not occupied.
```

T-PRM makes PRM time-aware by storing safe time intervals for roadmap nodes and using a modified A* search that checks arrival-time safety.
