# Phase 1 Refactor Architecture Summary

This phase preserves the static nonholonomic A* planner as a baseline and adds reusable infrastructure for future dynamic and temporal-roadmap work.

## Layer 1: Preserved Baseline

- `src/planners/baseline_nonholonomic_astar.py`
  - static nonholonomic A*
  - state key: `(ix, iy, itheta)`
  - successor generation through motion primitives
  - full sampled static trajectory collision checking
- `scripts/run_baseline_astar.py`
  - new CLI entry point for the baseline
- `Part01/main.py`
  - compatibility wrapper that forwards to the new baseline CLI

## Layer 2: Reusable Motion, Collision, and Time Infrastructure

- `src/models/state.py`
  - `Pose2D`, `TimedPose2D`, `OrientedBox`, `TrajectorySegment`
- `src/models/primitives.py`
  - `MotionPrimitive`
  - team-car and turtlebot primitive action sets
- `src/core/rollout.py`
  - primitive rollout
  - differential-drive integration
  - rear-drive-steered / team-car integration
  - calibrated team-car primitive support
- `src/core/static_collision.py`
  - bounds checks
  - circle footprint collision
  - oriented-box footprint collision
  - full static trajectory validation
- `src/core/dynamic_collision.py`
  - constant-velocity circular dynamic obstacles
  - dynamic pose and trajectory collision helpers
- `src/core/time_parameterization.py`
  - step-based sample timestamps
  - normalized sample timestamps
- `src/core/temporal_validation.py`
  - combined static and dynamic spatiotemporal trajectory validation
- `src/core/costs.py`, `src/core/heuristics.py`, `src/core/discretization.py`, `src/core/search_utils.py`
  - reusable search math and bookkeeping helpers

## Layer 3: Future Temporal Roadmap Scaffolding

- `src/models/roadmap.py`
  - `RoadmapNode`, `RoadmapEdge`, `Roadmap`
- `src/models/labels.py`
  - `TemporalSearchLabel`
- `src/builders/roadmap_builder.py`
  - static-valid sampled roadmap node generation
  - primitive edge rollout and static feasibility filtering
- `src/planners/temporal_roadmap_planner.py`
  - scaffold-only temporal planner boundary
- `src/planners/reactive_replanning_baseline.py`
  - scaffold-only reactive replanning baseline boundary

## Debug Scripts

- `scripts/run_temporal_validation_debug.py`
- `scripts/run_roadmap_builder_debug.py`

## Tests

- `tests/test_rollout.py`
- `tests/test_static_collision.py`
- `tests/test_dynamic_temporal.py`
- `tests/test_roadmap_builder.py`
- `tests/test_baseline_astar.py`

## Next Phase

The next phase should implement:

- roadmap temporal query search
- label expansion over `(node_id, arrival_time)`
- wait actions or time discretization policy
- dynamic-obstacle experiments
- comparison against reactive replanning
