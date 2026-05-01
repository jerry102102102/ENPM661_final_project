# Phase 3 ACTEA Temporal Roadmap

Phase 3 is now framed as the project core:

```text
Sampled Nonholonomic Temporal Roadmap with ACTEA
```

ACTEA means:

```text
Analytic Continuous-Time Edge Annotation
```

The idea is simple: build a reusable sampled nonholonomic roadmap once, then
annotate each roadmap edge with the exact continuous-time departure intervals
that are blocked or valid for a known dynamic-obstacle scenario.

## Implemented Layers

### 3A: Edge-Obstacle Interaction Cache

For each edge and dynamic-obstacle scenario, the planner can precompute which
obstacles could ever interact with that edge over a finite time horizon.

API:

```python
planner.annotate_edge_obstacle_interactions(...)
```

The cache uses swept AABB checks:

- edge trajectory AABB, inflated by robot radius and clearance
- dynamic obstacle swept AABB over the selected horizon

If an obstacle can never intersect an edge's swept region, it can be skipped
during validation.

### 3B: Time-Bin Cache

During query, edge validation results can be cached by:

```text
(edge_id, departure_time_bin, dynamic_obstacle_signature)
```

where:

```text
departure_time_bin = round(departure_time / time_bin_size)
```

This is a sampled cache. It is useful as an efficiency ablation, but it is not
the main contribution.

### 3C: ACTEA

ACTEA computes analytic blocked and valid departure-time intervals for each
roadmap edge under the current piecewise-linear primitive trajectory
representation.

API:

```python
planner.annotate_temporal_intervals(...)
```

Implementation:

- ACTEA annotations are represented as first-class `EdgeTemporalAnnotation` metadata
- annotations are stored in an `EdgeTemporalAnnotationStore`
- keys are edge id plus dynamic-obstacle signature
- the validator still keeps legacy interval lookup structures internally for compatibility

One edge can receive metadata like:

```text
edge e:
[0.00, 1.25] -> valid
[1.50, 2.00] -> blocked
```

## ACTEA Math

For one primitive edge segment, the robot center is linearly interpolated over
edge-relative time `s`:

```text
r(s) = r_c + v_r s,    s in [s0, s1]
```

A moving circular obstacle has:

```text
o(t0 + s) = o0 + v_o (t0 + s)
```

where `t0` is the edge departure time.

Collision occurs if:

```text
|| r(s) - o(t0 + s) || <= R
```

with:

```text
R = robot_radius + obstacle_radius + clearance
```

Substitution gives a quadratic condition in segment-relative time and departure
time:

```text
|| A + B s + C t0 ||^2 <= R^2
```

The implementation projects this condition onto the departure-time axis using:

- endpoint cases `s = s0` and `s = s1`
- interior closest-point cases where the minimizing `s*(t0)` lies inside `[s0, s1]`

Blocked intervals are merged across all trajectory segments and dynamic
obstacles. Valid intervals are the complement over the finite ACTEA horizon.

Scope: ACTEA is exact for the current piecewise-linear interpolation of sampled
primitive trajectories. It is not a closed-form symbolic solution of the
original continuous robot dynamics before sampling.

## Planner Integration

The temporal planner config exposes explicit annotation modes:

```python
temporal_annotation_mode="online"
temporal_annotation_mode="bin_cache"
temporal_annotation_mode="actea"
```

Mode behavior:

```text
online:
    Validate edges online during query.

bin_cache:
    Use edge-obstacle filtering and time-bin validation cache.

actea:
    Use analytic blocked/valid intervals first, then fall back to cache or
    online validation if annotation metadata is missing.
```

The planner exposes formal edge-time query APIs:

```python
planner.edge_is_valid_at_time(...)
planner.edge_cost_at_time(...)
```

Current edge-time cost model:

```text
g_e(t0) = edge.geometric_cost    if edge e is valid at departure time t0
g_e(t0) = infinity              otherwise
```

## Output Method Names

Use these method names in experiment outputs and report plots:

- `static_astar`
- `reactive_replanning`
- `expansion_temporal`
- `sampled_temporal_online`
- `sampled_temporal_bin_cache`
- `sampled_temporal_actea`

The final proposed method is:

```text
sampled_temporal_actea
```

## Debug Metrics

Planner result debug payload includes:

- `exact_checks`
- `dynamic_collision_rejections`
- `candidate_filter_skips`
- `temporal_cache_hits_free`
- `temporal_cache_hits_blocked`
- `interval_cache_hits_free`
- `interval_cache_hits_blocked`
- `interaction_cache_hits`
- `edge_temporal_annotation_entries`

The experiment CSV writer also flattens the important counters into:

- `cache_hits_free`
- `cache_hits_blocked`
- `interval_hits_free`
- `interval_hits_blocked`
- `interaction_cache_hits`

## Scripts

Core experiment commands:

```bash
python3 scripts/run_experiment_actea_correctness.py
python3 scripts/run_experiment_repeated_query.py
python3 scripts/run_experiment_hard_scenes.py
python3 scripts/run_experiment_roadmap_scale_ablation.py
python3 scripts/visualize_representative_paths.py
python3 scripts/plot_experiment_results.py
```

Small cache smoke benchmark:

```bash
python3 scripts/run_temporal_cache_benchmark.py
```

The smoke benchmark compares:

- `online`
- `bin_cache`
- `actea`

## Evaluation Spec

The full report-ready checklist is tracked in:

```text
doc/implementation_process/core_method_completion_and_evaluation_spec.md
```
