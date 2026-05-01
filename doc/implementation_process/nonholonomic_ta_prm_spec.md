# Spec: Sampled Nonholonomic TA-PRM-Style Planner

## 1. Goal

We want to extend temporal roadmap planning from the original **holonomic / point-robot** setting to a **nonholonomic robot model** while preserving the main benefits of TA-PRM / T-PRM:

- sampled reusable roadmap
- multi-query planning
- time-aware query
- dynamic-obstacle foresight instead of purely reactive replanning

Our target planner is **not** just “nonholonomic A* with time.”
It should become a **sampled nonholonomic temporal roadmap planner**.

---

## 2. Problem We Are Solving

The original temporal roadmap papers are strong at reasoning about **time**, but their robot model and local connections are too idealized for realistic nonholonomic robots.

Our previous Part 1 implementation already gives us a strong base:

- state representation: `(x, y, theta)`
- nonholonomic motion primitives
- primitive rollout into sampled poses
- full-trajectory static collision checking
- nonholonomic A* baseline

However, that older system does **not** yet provide:

- sampled reusable roadmap construction
- temporal search labels such as `(node_id, arrival_time)`
- multi-query roadmap reuse
- dynamic-obstacle-aware edge validity over time

So the project gap is:

> Preserve the roadmap and temporal-query strengths of TA-PRM-style planning, but replace straight-line / holonomic local motion with nonholonomic motion primitives.

---

## 3. Core Benefits We Want to Preserve

### 3.1 Reusable sampled roadmap
A core PRM / T-PRM / TA-PRM benefit is that the roadmap is built once and reused across multiple queries.

We want:

- a base roadmap independent of a single start/goal pair
- online start/goal connection at query time
- the ability to reuse the same roadmap for different goals

### 3.2 Time-aware query
The planner should reason about:

- whether an edge is valid **when the robot traverses it**
- not just whether the edge is geometrically collision-free in static space

### 3.3 Non-reactive dynamic obstacle handling
We want to preserve the temporal look-ahead advantage:

- do not wait until collision is imminent
- incorporate known/predictable dynamic obstacle motion into query-time validity

### 3.4 Compatibility with nonholonomic motion
Roadmap edges must represent motions the robot can actually execute.

That means:

- no straight-line-only local planner
- no purely holonomic connection assumption
- use primitive-based transitions or equivalent nonholonomic local connectors

---

## 4. Main Design Challenges

### 4.1 Straight-line local connection no longer works
The original roadmap planners assume simple local connections.
For a nonholonomic robot, straight-line connections are not generally feasible.

### 4.2 Node meaning changes
For a nonholonomic robot, heading matters.

The roadmap node should therefore include heading:

```text
(x, y, theta)
```

### 4.3 Edge temporal legality becomes harder
Each edge is now a **primitive trajectory**, not a simple line segment.

So temporal validity must be checked along the full trajectory:

- sample poses along the primitive
- assign timestamps to those poses
- test dynamic collision at each timestamp

### 4.4 Full offline temporal interval construction is expensive
If we try to immediately precompute full continuous-time availability intervals for every primitive edge, complexity becomes high.

So temporal reasoning should be introduced in stages.

---

## 5. High-Level Final Design

The final planner family should have this structure:

### Offline
- build a sampled nonholonomic roadmap in `(x, y, theta)`
- connect nodes using nonholonomic motion primitives
- keep only statically feasible edges

### Online query
- connect start and goal to the roadmap
- search over `(node_id, arrival_time)`
- validate each candidate edge against dynamic obstacles over traversal time

### Later enhancement
- add cached temporal edge metadata to reduce repeated online dynamic validation

---

## 6. Three-Phase Implementation Plan

---

# Phase 1 — Sampled Nonholonomic Static Roadmap

## Objective
Replace the current seed-expansion graph with a true sampled reusable roadmap.

## Why this phase matters
Without a sampled roadmap, we do not really get the PRM / TA-PRM benefit of:

- reusable graph structure
- multi-query planning
- independence from a single start/goal pair

## Phase 1 design

### Node generation
Use sampled roadmap nodes, not seed-expansion-only growth.

Recommended first version:

1. sample `(x, y)` in free space
2. attach a small discrete heading set
3. create candidate nodes `(x, y, theta)`

Example heading set:

```text
theta in {0, ±45°, ±90°, 180°}
```

This is easier to control than fully random continuous `(x, y, theta)` sampling.

### Edge generation
For each node:

1. find nearby candidate neighbors
2. attempt a nonholonomic local connection using motion primitives
3. keep the edge only if the primitive rollout is statically valid

Each stored edge should contain at least:

- source node id
- target node id
- primitive/action label
- trajectory samples
- duration
- geometric cost

## Phase 1 output
A reusable **sampled nonholonomic static roadmap**.

## Phase 1 acceptance criteria
- roadmap is built from sampled nodes, not only graph growth from seeds
- roadmap can be reused for multiple start/goal queries
- start and goal are connected online, not hardwired into roadmap construction

---

# Phase 2 — Exact Online Temporal Validation

## Objective
Add time-aware query to the sampled nonholonomic roadmap, but keep temporal legality checking online and exact.

## Why this phase matters
This is the first fully working temporal planner version.
It establishes correctness before we optimize the temporal side.

## Phase 2 design

### Search label
Search should run over labels of the form:

```text
(node_id, arrival_time)
```

not just `node_id`.

### Edge expansion
For each outgoing roadmap edge:

1. current label gives departure time `t0`
2. edge duration gives `t_arrival = t0 + duration`
3. each trajectory sample gets a timestamp:
   ```text
   t_i = t0 + alpha_i * duration
   ```
4. test every sample against moving obstacles at its timestamp
5. if any sample collides, reject the edge
6. otherwise generate the child label

### Dynamic obstacle model
First version should use:

- moving circles
- known initial position
- known constant velocity
- known radius

### Dominance / pruning
Use a practical time-bin-based rule:

```text
best_cost[(node_id, time_bin)]
```

where:

```text
time_bin = round(arrival_time / time_bin_size)
```

If a new label reaches the same `(node_id, time_bin)` with worse or equal cost, discard it.

This is a practical first temporal pruning policy.

## What this phase preserves
- nonholonomic feasibility
- time-aware query
- exact edge legality checking
- reusable roadmap

## What this phase does not yet optimize
- repeated temporal checks across many queries
- full offline temporal annotation of edges

## Phase 2 output
A **Sampled Nonholonomic Temporal Roadmap Planner with Online Validation**.

## Phase 2 acceptance criteria
- planner finds valid time-aware paths on the sampled roadmap
- dynamic obstacle collisions can invalidate roadmap edges at query time
- roadmap is reusable across different goals
- temporal search over `(node_id, arrival_time)` is functional

---

# Phase 3 — Hybrid Cached Temporal Roadmap

## Objective
Recover more of the offline/query separation and multi-query benefit that make TA-PRM attractive.

## Why this phase matters
Phase 2 is correct and practical, but repeated queries still spend too much time rechecking dynamic legality online.

Phase 3 pushes temporal information partially offline.

## Phase 3 design

### Phase 3A: Edge–Obstacle Interaction Cache
For each edge, precompute which dynamic obstacles could ever matter.

Store:

- edge-to-obstacle candidate interaction list

At query time:
- skip checking obstacles that can never interact with the edge

This is the safest and easiest cache layer.

---

### Phase 3B: Coarse Temporal Bin Cache
For each edge, precompute coarse temporal metadata over departure-time bins.

Example:

```text
edge e:
0–1 s   -> free
1–2 s   -> blocked
2–3 s   -> maybe free
```

At query time:

- if the coarse bin says definitely blocked -> skip edge immediately
- if maybe free -> run exact online validation
- if definitely free -> optionally still run final safety validation, depending on conservativeness

This gives temporal reuse without requiring full continuous-time interval construction.

---

### Phase 3C: Optional Full Edge Temporal Annotation
Only attempt this if time and stability permit.

Possible final form:

- piecewise time-valid intervals for each edge
- or time-varying edge legality / cost function `g(t, e)`

This is the closest to TA-PRM-style edge annotation, but also the most complex for nonholonomic primitive trajectories.

## Phase 3 output
A **Sampled Nonholonomic Temporal Roadmap Planner with Cached Temporal Edge Metadata**.

## Phase 3 acceptance criteria
- at least one temporal cache mechanism is used during query
- repeated queries become noticeably cheaper than Phase 2
- path quality and success rate remain acceptable
- exact online validation can still be used as final safety check

---

## 7. Recommended Method Variants

At the end, we should have two main method variants:

### Method A
**Sampled Nonholonomic Temporal Roadmap with Online Validation**

This is the exact/practical version.

### Method B
**Sampled Nonholonomic Temporal Roadmap with Cached Temporal Edge Metadata**

This is the enhanced version that better preserves the PRM / TA-PRM multi-query advantage.

---

## 8. Relationship Between Current Code and Final Method

### What we already have from the old nonholonomic planner
- `(x, y, theta)` state representation
- motion primitive rollout
- static collision checking
- primitive cost computation
- A* baseline
- temporal validation infrastructure prototype

### What must change
- roadmap builder must become sampled PRM-style
- start/goal should connect online to a reusable roadmap
- temporal query must operate on roadmap labels
- temporal edge legality should be handled in phases

### What current Phase 2 code should become
The current seed-expansion temporal graph planner should be treated as:

- a temporal graph-search baseline
- a source of reusable temporal validation code
- not the final PRM-style main method

---

## 9. Comparison Structure

We should compare:

### Baseline 1
**Static nonholonomic A\***
- no temporal query
- no reusable roadmap

### Baseline 2
**Current expansion-built temporal graph planner**
- temporal search
- nonholonomic motion
- but no true sampled reusable roadmap

### Proposed Method A
**Sampled nonholonomic temporal roadmap with online validation**

### Proposed Method B
**Sampled nonholonomic temporal roadmap with cached temporal metadata**

This comparison lets us isolate:

- benefit of temporal reasoning
- benefit of sampled reusable roadmap
- benefit of cached temporal information

---

## 10. Key Design Decisions We Must Communicate Clearly

### Decision 1
Use a sampled roadmap, not only expansion-built graph growth.

### Decision 2
Use `(x, y, theta)` nodes because heading affects nonholonomic reachability.

### Decision 3
Use motion primitives as local connectors instead of straight-line links.

### Decision 4
Do not immediately require full continuous-time offline edge annotation.

### Decision 5
Keep exact online validation as the correctness anchor, even when adding temporal caches later.

---

## 11. Deliverables

### Phase 1 deliverables
- sampled static nonholonomic roadmap builder
- reusable base graph
- online start/goal connection

### Phase 2 deliverables
- temporal roadmap query over `(node_id, arrival_time)`
- online exact dynamic validation
- first working nonholonomic temporal roadmap planner

### Phase 3 deliverables
- cached temporal edge metadata
- faster repeated query support
- comparison against Phase 2

### Final project deliverables
- code for all baseline and proposed methods
- quantitative experiments
- visualizations
- final report
- presentation/demo

---

## 12. One-Sentence Summary

We will implement a **sampled nonholonomic TA-PRM-style planner** in three phases: first a reusable sampled nonholonomic roadmap, then an exact time-aware query planner with online dynamic validation, and finally a hybrid cached temporal version that recovers more of the offline/query separation and multi-query benefits of TA-PRM.
