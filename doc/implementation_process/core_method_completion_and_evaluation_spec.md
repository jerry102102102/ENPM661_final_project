# ACTEA Method and Evaluation Spec

This document is the current implementation and evaluation checklist for the
project. The core planner is not being redesigned from scratch. The existing
sampled nonholonomic roadmap, temporal query planner, online validation
fallback, and seed-expansion temporal graph baseline remain the foundation.

The formal contribution name is:

```text
ACTEA = Analytic Continuous-Time Edge Annotation
```

ACTEA is the analytic edge annotation layer that computes blocked and valid
departure-time intervals for nonholonomic primitive edges under known
constant-velocity circular dynamic obstacles.

## 1. Preserved Core Pipeline

The repository already contains the core sampled nonholonomic temporal roadmap:

- sampled nonholonomic roadmap builder
- roadmap nodes in `(x, y, theta)`
- motion-primitive roadmap edges
- full sampled static trajectory validation
- temporal roadmap query over `(node_id, arrival_time)`
- online spatiotemporal validation fallback
- analytic continuous-time edge interval logic in `src/core/temporal_cache.py`
- seed-expansion temporal graph builder as a weaker baseline
- static nonholonomic A* baseline

Do not remove working code. Refactor around it.

## 2. Proposed Method Framing

The proposed method should be described as:

```text
Sampled Nonholonomic Temporal Roadmap with ACTEA
```

The important distinction:

```text
sampled_temporal_online:
    Uses the same sampled roadmap, but checks edge validity online during query.

sampled_temporal_bin_cache:
    Uses time-bin cache and candidate filtering.

sampled_temporal_actea:
    Uses analytic continuous-time edge annotations before falling back to cache
    or online validation.
```

The main proposed method for report figures is:

```text
sampled_temporal_actea
```

The bin-cache method may remain as a repeated-query efficiency ablation, but it
is not the headline contribution.

## 3. ACTEA Definition

For each roadmap edge `e`, ACTEA annotates departure times `t0` for which the
robot can safely traverse the full primitive trajectory.

The first-class edge temporal annotation stores:

- `blocked_intervals_exact`
- `valid_intervals_exact`
- `temporal_horizon_s`
- `annotation_mode = "actea"`
- optional `blocked_intervals_by_obstacle`

The edge-time APIs are:

```python
edge_is_valid_at_time(edge, departure_time_s, ...)
edge_cost_at_time(edge, departure_time_s, ...)
```

Default ACTEA behavior:

```text
if departure time is blocked:
    valid = False
    cost = infinity

otherwise:
    valid = True
    cost = edge.geometric_cost
```

ACTEA is exact for the current piecewise-linear representation of sampled
primitive trajectories. It is not a symbolic solution of the original
continuous differential-drive or bicycle dynamics before trajectory sampling.

## 4. External Method Names

Use these names consistently in outputs, tables, figures, and report text.

Negative/reference:

- `static_astar`
- `reactive_replanning`

Temporal baselines:

- `expansion_temporal`
- `sampled_temporal_online`

Proposed and ablation methods:

- `sampled_temporal_actea`
- `sampled_temporal_bin_cache`

Internal compatibility aliases may exist for older code, but user-facing names
should use ACTEA and `actea`.

## 5. Annotation Modes

External modes:

```text
online
bin_cache
actea
```

Mode behavior:

```text
online:
    No temporal metadata required. Edge validity is checked online.

bin_cache:
    Uses edge-obstacle interaction filtering and time-bin validation cache.

actea:
    Uses analytic continuous-time blocked/valid edge intervals first, then
    falls back to bin cache or online validation when metadata is missing.
```

Older internal aliases may still be accepted by code for backward
compatibility, but docs, plots, CSV/JSON outputs, and report tables should use
only `online`, `bin_cache`, and `actea`.

## 6. Baseline Suite

The main comparison suite should contain:

- `reactive_replanning`
- `expansion_temporal`
- `sampled_temporal_online`
- `sampled_temporal_actea`

Optional/supporting:

- `sampled_temporal_bin_cache` for repeated-query efficiency ablation
- `static_astar` as appendix/sanity baseline

Baseline roles:

```text
static_astar:
    Negative control. Useful for sanity and appendix plots, not a headline
    hard-scene competitor.

expansion_temporal:
    Weaker temporal graph baseline. Shows why sampled reusable PRM-style
    roadmap matters.

reactive_replanning:
    Stronger engineering baseline. It replans from the current state/time when
    the remaining path becomes invalid or predicted invalid.

sampled_temporal_online:
    Same sampled roadmap as ACTEA, without analytic annotation. This is the
    direct ablation baseline.

sampled_temporal_actea:
    Final proposed method.
```

## 7. Evaluation Questions

The evaluation is organized around four questions:

```text
Q1. Is ACTEA correct?
Q2. When does ACTEA help repeated queries?
Q3. Does ACTEA help in harder dynamic scenes that require temporal foresight?
Q4. How does roadmap scale affect ACTEA's usefulness?
```

## 8. Experiments

### Experiment 1 - ACTEA Correctness

Goal:

Compare ACTEA interval lookup against brute-force online temporal validation.

Setup:

- sample many roadmap edges
- generate dynamic obstacle scenarios
- sample random departure times
- compare ACTEA interval membership with online validation

Metrics:

- agreement rate
- false blocked count
- false free count
- disagreement rate by edge
- disagreement rate by obstacle scenario

Outputs:

- correctness summary table
- mismatch bar chart

### Experiment 2 - Repeated-Query Scaling

Goal:

Show when ACTEA's offline annotation cost pays off across many queries.

Query counts:

```text
N = 1, 5, 10, 25, 50, 100
```

Methods:

- `sampled_temporal_online`
- `sampled_temporal_bin_cache`
- `sampled_temporal_actea`

Metrics:

- roadmap build time
- annotation time
- average query time
- total runtime over `N` queries
- amortized cost per query
- success rate
- interval lookup hits
- bin cache hits
- interaction cache hits

Main plot:

```text
x-axis = number of queries
y-axis = total wall-clock time
```

### Experiment 3 - Hard Dynamic-Scene Benchmark

Goal:

Evaluate ACTEA in genuinely harder dynamic scenes.

Scene families:

- `cross_traffic`
- `moving_bottleneck`
- `longitudinal_interaction`
- `tight_corridor_heading`

Methods:

- `reactive_replanning`
- `expansion_temporal`
- `sampled_temporal_online`
- `sampled_temporal_actea`

Metrics:

- success rate
- mean traversal time
- mean total planning/query time
- expanded labels
- rejected dynamic edges
- replans
- timeout rate
- collision/failure count

### Experiment 4 - Roadmap-Scale Ablation

Goal:

Show how roadmap size affects ACTEA usefulness.

Roadmap regimes:

- sparse
- medium
- dense

Controls:

- XY sample count
- heading set size
- outgoing edge cap
- endpoint matching tolerance

Methods:

- `sampled_temporal_online`
- `sampled_temporal_actea`

Metrics:

- nodes
- edges
- roadmap build time
- ACTEA annotation time
- average query time
- total repeated-query time
- success rate
- path quality

## 9. Logging Schema

Every experiment record should include:

- `experiment_name`
- `method`
- `map_id` or `scene_family`
- `seed`
- `start_pose`
- `goal_pose`
- `obstacle_count`
- `obstacle_speed_scale`
- `roadmap_nodes`
- `roadmap_edges`
- `success`
- `traversal_time`
- `planning_time`
- `query_time`
- `annotation_time`
- `total_runtime`
- `expanded_labels`
- `rejected_dynamic_edges`
- `replans`
- `cache_hits_free`
- `cache_hits_blocked`
- `interval_hits_free`
- `interval_hits_blocked`
- `interaction_cache_hits`

Write outputs under:

```text
outputs/experiments/
```

Use CSV for aggregated analysis and JSON for richer debugging.

## 10. Required Figures

The plotting pipeline should generate:

- Figure 1: ACTEA correctness summary
- Figure 2: repeated-query scaling curve
- Figure 3: success rate by scene family and method
- Figure 4: mean query/planning time by scene family and method
- Figure 5: roadmap-scale ablation
- Figure 6: representative ACTEA overlay visualization

The representative overlay should show:

- sampled roadmap
- chosen path
- moving obstacle trajectory
- one edge's blocked/valid departure intervals or a visual explanation of ACTEA

## 11. Report Tables

Required tables:

- method summary table
- ACTEA correctness table
- repeated-query scaling table
- hard scene benchmark table
- roadmap-scale ablation table

## 12. Implemented Commands

The current implementation should provide:

```bash
python3 scripts/run_experiment_actea_correctness.py
python3 scripts/run_experiment_repeated_query.py
python3 scripts/run_experiment_hard_scenes.py
python3 scripts/run_experiment_roadmap_scale_ablation.py
python3 scripts/plot_experiment_results.py
python3 scripts/visualize_representative_paths.py
```

Compatibility scripts may remain only if they do not appear in headline
documentation or outputs.
