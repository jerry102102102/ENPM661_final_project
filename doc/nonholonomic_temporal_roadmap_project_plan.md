# Project Plan: A Nonholonomic Extension of Temporal Probabilistic Roadmaps for Dynamic-Environment Path Planning

## 1. Project Summary

We will reproduce the core idea of temporal roadmap planning for dynamic environments, then extend it to a more realistic robot model.

The original T-PRM framework is strong at reasoning about **time** in dynamic environments, but it makes a major simplification: the robot is modeled as a **holonomic, first-order point robot**. In practice, many robots cannot move freely in any direction and cannot follow arbitrary straight-line connections. This limitation makes the original method less realistic for car-like robots, Ackermann robots, and other platforms with turning constraints.

Our project will address this gap by extending temporal roadmap planning to a **nonholonomic robot model**. Instead of straight-line local connections, we will use **motion primitives** and validate each candidate transition against both **static obstacles** and **dynamic obstacles over time**.

In short, our goal is:

> Keep the time-aware planning idea from T-PRM, but replace the unrealistic holonomic point-robot assumption with a nonholonomic robot model that can be executed more realistically in simulation.

## 2. Main Objective

The main research question is:

> Can a temporal roadmap planner like T-PRM be extended from a holonomic point-robot model to a nonholonomic robot model while preserving safe dynamic-obstacle avoidance and practical runtime?

We want to show that:

1. The original holonomic temporal planner is useful, but not always executable on a realistic robot.
2. A nonholonomic temporal planner can preserve the advantage of time-aware planning while producing more realistic and feasible paths.
3. In dynamic environments, a nonholonomic temporal roadmap planner can outperform a purely reactive replanning baseline in terms of safety, executability, and consistency.

## 3. Why This Project Matters

This is not just a random add-on. It directly targets a real limitation in the original papers.

The original T-PRM method assumes:
- a holonomic robot
- first-order dynamics
- a point-robot model with inflated obstacles

That assumption makes the planner easier to analyze, but it also creates a realism gap. A path that is valid for a holonomic point robot may not be executable for a robot with turning constraints.

Our extension makes the planner more realistic and more relevant to robotics systems that must actually execute the planned path.

This also gives us a strong project narrative:

- **Original contribution of the papers:** time-aware roadmap planning
- **Our extension:** nonholonomic feasibility and executable dynamic-environment planning

## 4. Scope of Our Work

We are **not** trying to build a full kinodynamic planner or a full real-world autonomy stack.

We are focusing on a clean, well-scoped extension:

- 2D workspace
- static obstacles
- moving dynamic obstacles with known future motion
- nonholonomic robot state
- motion-primitive-based roadmap connections
- time-aware dynamic collision checking during search

This keeps the project focused on the planner itself.

## 5. Core Technical Idea

### Original temporal roadmap idea
The original temporal planner uses a roadmap and reasons about whether a path is still safe **when the robot reaches it**, not only whether it is geometrically collision-free.

### Our extension
We will keep this **time-aware planning logic**, but replace the robot model and local planner.

Instead of:
- state = `(x, y)`
- straight-line edge
- holonomic motion assumption

we will use:
- state = `(x, y, theta)`
- nonholonomic motion primitives
- trajectory validation over both **space and time**

This means every roadmap transition will represent a **feasible robot motion**, not just a geometric shortcut.

## 6. Proposed Method

### 6.1 State Representation

We will represent each roadmap node as:

```text
(x, y, theta)
```

This allows the planner to encode robot heading directly in the graph and makes the transitions meaningfully nonholonomic.

We will not begin with a full dynamic state such as `(x, y, theta, v, omega)`, because that would significantly increase complexity and scope.

### 6.2 Motion Primitives

We will use a discrete set of nonholonomic motion primitives for local transitions.

A practical first version is:

- forward straight
- forward left arc
- forward right arc

Optional extensions:
- multiple curvature levels
- different primitive durations
- reverse motions if needed

Each primitive will have:
- a control definition
- a fixed rollout duration
- a corresponding trajectory in `(x, y, theta)` space

This is the key change that makes the roadmap nonholonomic.

### 6.3 Roadmap Construction

We will build a roadmap in state space.

For each sampled node:
1. sample a candidate state `(x, y, theta)`
2. attempt connections using motion primitives
3. roll out each primitive trajectory
4. keep the transition only if it is collision-free with respect to static obstacles

This produces a graph where edges are **feasible nonholonomic transitions**, not straight-line links.

### 6.4 Dynamic-Obstacle Handling

Dynamic obstacles will be modeled as moving circles with known future motion, initially using constant velocity.

During search:
1. we know the arrival time at the current node
2. we know the duration of the candidate primitive
3. we can assign timestamps to points along the primitive trajectory
4. at each sampled time, we check whether the robot footprint collides with a moving obstacle

If collision occurs at any sampled time along the primitive, that transition is invalid for that expansion.

This gives us **spatiotemporal primitive validation**.

That is the main planner innovation.

### 6.5 Time-Aware Search

We will keep the temporal-search idea from the original planner.

The search state will include both:
- roadmap node
- arrival time

Conceptually:

```text
(node_id, arrival_time)
```

This is necessary because the same spatial state may be safe at one time and unsafe at another.

The planner therefore searches not only over geometry, but over geometry **plus time**.

A first practical heuristic can be Euclidean distance to the goal. If needed later, we can add heading-aware heuristics.

### 6.6 Practical Design Choice

We do **not** plan to fully precompute all temporal validity intervals for every nonholonomic edge in the first implementation.

Instead, we will likely use:

> **query-time spatiotemporal validation**

This means:
- the roadmap geometry is built offline
- dynamic-obstacle checking is performed when a candidate edge is expanded during search

Why this choice:
- nonholonomic edges are more complex than straight lines
- full interval precomputation is significantly harder
- query-time validation is easier to implement and debug
- it still preserves the essential temporal planning idea

This makes our method a **practical nonholonomic adaptation of temporal roadmap planning**.

## 7. Baselines

We want comparisons that make the results easy to interpret.

### Baseline 1: Holonomic Temporal Planner
This is the closest paper-style baseline.

Properties:
- point robot
- straight-line connections
- time-aware dynamic-obstacle checking

Purpose:
- show what the original temporal planning idea achieves under idealized assumptions

### Baseline 2: Nonholonomic A* with Reactive Replanning
This is the most natural engineering baseline.

Properties:
- motion primitives
- replans when future collision is detected or when the current plan becomes invalid
- does not explicitly reason about future dynamic-obstacle timing in the roadmap

Purpose:
- compare **reactive** nonholonomic planning vs **time-aware** nonholonomic planning

### Proposed Method: Nonholonomic Temporal Roadmap Planner
Properties:
- roadmap in `(x, y, theta)`
- primitive-based edges
- time-aware dynamic-obstacle validation

Purpose:
- combine nonholonomic feasibility with temporal awareness

## 8. Planned Experiments

### Experiment 1: Feasibility Gap

Goal:
- show the difference between holonomic temporal paths and nonholonomic executable paths

Questions:
- how often does the holonomic temporal planner produce paths that are hard or impossible to execute with a nonholonomic robot?
- does the proposed planner produce more realistic trajectories?

Metrics:
- execution success rate
- tracking or path-following success in simulation
- turning-radius or curvature violations
- final goal error

### Experiment 2: Dynamic Obstacle Density

Goal:
- test how each planner behaves as the number of moving obstacles increases

Setup:
- same map size
- same start and goal
- varying number of moving obstacles

Metrics:
- success rate
- collision count
- planning time
- path length
- traversal time

Main comparison:
- proposed planner vs reactive nonholonomic replanning baseline

### Experiment 3: Motion Primitive Resolution

Goal:
- understand how primitive resolution affects planner performance

Setup:
- coarse primitive set
- medium primitive set
- fine primitive set

Metrics:
- planning time
- path quality
- success rate
- graph size / branching factor

Purpose:
- measure the trade-off between path quality and computational cost

### Experiment 4: Simulation Execution Study

Goal:
- compare planned trajectories with executed trajectories in simulation

Metrics:
- final goal error
- tracking deviation
- actual collision outcomes
- mission completion rate

Purpose:
- demonstrate that the proposed planner is not only theoretically reasonable, but also practically executable

## 9. Evaluation Metrics

We should evaluate the project at three levels.

### Planning Quality
- path length
- traversal time
- path smoothness or curvature profile if useful

### Safety
- collision count
- minimum clearance to dynamic obstacles
- success rate

### Practical Executability
- execution success rate
- tracking deviation
- final goal error
- number of failed expansions due to dynamic obstacle timing

These metrics will help us compare:
- temporal intelligence
- nonholonomic realism
- computational cost

## 10. What We Expect to Show

We expect the following story:

### Holonomic temporal baseline
- plans intelligently in time
- may generate geometrically short paths
- but can be unrealistic or hard to execute for nonholonomic robots

### Reactive nonholonomic baseline
- generates executable paths
- but is more myopic
- may require more replanning
- may behave less consistently in dynamic scenes

### Proposed nonholonomic temporal planner
- generates executable motion-primitive paths
- reasons about future dynamic obstacles
- avoids some unnecessary replanning
- gives a better balance of safety, realism, and path quality

The most important expected result is not necessarily the shortest path.

The most important expected result is:

> **time-aware planning remains useful after adding nonholonomic constraints, and the resulting paths are more realistic than those from the original holonomic formulation.**

## 11. Key Design Decisions

To keep the project successful and manageable, we will use the following principles:

### Keep the robot model nonholonomic, but not fully dynamic
Use `(x, y, theta)` first.

### Use simple, interpretable motion primitives
Do not start with a very large primitive library.

### Use simple dynamic obstacle models first
Moving circles with known constant velocity are enough for the first version.

### Prioritize a clean planner implementation over too much infrastructure
The main value of the project is the planner, not a huge simulation stack.

### Treat Gazebo or higher-fidelity simulation as an execution demonstration, not as the only source of results
We should first prove the planner works in a controlled environment.

## 12. Non-Goals

To avoid scope creep, the following are **not** core goals of this project:

- full kinodynamic planning
- uncertainty-aware obstacle prediction
- learned heuristics or RL-guided planning
- full multi-robot coordination
- complete real-world deployment stack
- advanced time-dependent risk databases

These are interesting future directions, but not necessary for a strong final project.

## 13. Deliverables

### 1. Planner Implementation
A working nonholonomic temporal roadmap planner with:
- `(x, y, theta)` state representation
- motion primitive rollout
- static collision checking
- dynamic obstacle spatiotemporal validation
- time-aware graph search

### 2. Baseline Implementations
At least:
- a holonomic temporal planning baseline
- a nonholonomic reactive replanning baseline

### 3. Reproducible Simulation Environment
A controllable 2D dynamic planning environment with:
- configurable static obstacles
- configurable moving obstacles
- configurable start and goal
- repeatable random seeds

### 4. Quantitative Results
A set of experiments comparing:
- success rate
- planning time
- path length
- traversal time
- collision statistics
- execution feasibility

### 5. Visualizations
Figures showing:
- roadmap examples
- holonomic vs nonholonomic path differences
- motion primitive examples
- obstacle trajectories
- planned vs executed trajectory overlays

### 6. Final Report
A clear write-up describing:
- the limitation of the original temporal roadmap assumption
- our nonholonomic extension
- the planner design
- baselines
- experiments
- results
- limitations and future directions

### 7. Demo or Presentation Material
A concise demo showing:
- a clean dynamic-obstacle scenario
- the difference between holonomic and nonholonomic planning
- one successful example of the proposed planner

## 14. One-Paragraph Version for Teammates

We will extend temporal roadmap planning from a holonomic point-robot formulation to a nonholonomic robot model. The original T-PRM idea is strong at reasoning about time in dynamic environments, but it assumes straight-line local motion and a robot that can move freely in any direction. Our project will replace those assumptions with a state representation `(x, y, theta)`, motion-primitive-based roadmap edges, and time-aware collision checking along each primitive trajectory. We will compare the proposed planner against a holonomic temporal baseline and a reactive nonholonomic replanning baseline, with the main goal of showing that time-aware planning remains useful after adding realistic motion constraints and that the resulting paths are more executable in simulation.

## 15. One-Paragraph Version for Codex

Build a 2D nonholonomic temporal roadmap planner. Use roadmap nodes in `(x, y, theta)` and connect them with a small set of forward motion primitives instead of straight-line edges. During search, the planner must keep track of arrival time and validate each primitive edge against moving circular obstacles over the full primitive traversal interval. The project must also include two baselines: a holonomic time-aware planner with straight-line edges and a nonholonomic reactive replanning baseline. The final output should support reproducible experiments comparing path quality, planning time, collision rate, and execution feasibility in dynamic environments.
