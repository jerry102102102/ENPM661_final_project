# Phase 3 Cached Temporal Roadmap

This phase adds cached temporal edge metadata on top of the sampled nonholonomic temporal roadmap.

## Implemented Layers

### 3A: Edge-Obstacle Interaction Cache

For each edge and dynamic-obstacle scenario, the planner can precompute which obstacles could ever interact with that edge over a time horizon.

API:

```python
planner.annotate_edge_obstacle_interactions(...)
```

The cache uses swept AABB checks:

- edge trajectory AABB, inflated by robot radius and clearance
- dynamic obstacle swept AABB over the selected horizon

If an obstacle can never intersect an edge's swept region, it can be skipped during exact validation.

### 3B: Temporal Time-Bin Cache

During query, edge validation results are cached by:

```text
(edge_id, departure_time_bin, dynamic_obstacle_signature)
```

where:

```text
departure_time_bin = round(departure_time / time_bin_size)
```

This avoids repeated exact validation for the same edge/time bin.

### 3C: Analytic Temporal Validity Intervals

The planner now computes analytic blocked / valid departure-time intervals for each dynamic edge-obstacle interaction under the current piecewise-linear primitive trajectory representation.

API:

```python
planner.annotate_temporal_intervals(...)
```

This produces interval metadata such as:

```text
edge e:
[0.00, 1.25] -> valid
[1.50, 2.00] -> blocked
```

## 3C Math

For one primitive edge segment, the robot center is linearly interpolated over edge-relative time `s`:

```text
r(s) = r_c + v_r s,    s in [s0, s1]
```

A moving circular obstacle has:

```text
o(d + s) = o0 + v_o (d + s)
```

where `d` is the edge departure time.

Collision occurs if:

```text
|| r(s) - o(d + s) || <= R
```

with:

```text
R = robot_radius + obstacle_radius + clearance
```

Substituting gives:

```text
|| A + B s + C d ||^2 <= R^2
```

The implementation projects this quadratic condition onto the departure-time axis by considering:

- endpoint cases `s = s0` and `s = s1`
- interior closest-point cases where the minimizing `s*(d)` lies inside `[s0, s1]`

The resulting blocked intervals are merged across all trajectory segments and all dynamic obstacles. Valid intervals are the complement of blocked intervals over the finite planning horizon.

Important caveat: the intervals are exact for the current piecewise-linear interpolation of sampled primitive trajectories. They are not a symbolic solution of the original continuous bicycle dynamics before sampling.

## Planner Integration

The temporal planner config now supports:

```python
use_temporal_cache=True
use_temporal_intervals=True
```

When cache is enabled, edge expansion uses:

1. analytic interval lookup, if available
2. time-bin validation cache, if available
3. edge-obstacle candidate filtering
4. exact temporal validation as fallback

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

## Benchmark Script

```bash
python3 scripts/run_temporal_cache_benchmark.py
```

This compares:

- `online`
- `bin_cache`
- `interval_cache`

The current smoke benchmark shows cache reuse across repeated queries, while preserving successful path finding.
