# Sampled Nonholonomic Roadmap Phase 1 Status

This update implements the Phase 1 direction from `nonholonomic_ta_prm_spec.md`: a reusable sampled nonholonomic static roadmap.

## Main Entry Point

```python
build_sampled_nonholonomic_roadmap(...)
```

in:

```text
src/builders/roadmap_builder.py
```

## What It Does

1. Samples query-independent `(x, y)` points in free space.
2. Attaches a discrete heading set to each sampled point.
3. Creates roadmap nodes in `(x, y, theta)`.
4. Rolls out nonholonomic motion primitives from each node.
5. Finds sampled roadmap nodes near primitive endpoints.
6. Keeps only edges whose full primitive trajectory is statically collision-free.

Each edge stores:

- source node id
- target node id
- primitive/action label
- trajectory samples
- primitive duration
- geometric cost
- static-valid flag

## Sampling Modes

Two sampling modes are available:

- `random`: random free-space XY points plus discrete headings
- `grid`: deterministic XY grid plus discrete headings

The grid mode is useful for reproducible demos and tests. The random mode is closer to classic PRM sampling.

## Relationship to Existing Builders

The previous primitive-expansion builder is still available:

```python
build_primitive_expansion_roadmap(...)
```

That builder grows a graph from seed poses and is better treated as a temporal graph-search baseline. The new sampled roadmap builder is the PRM-style proposed-method direction.

## Verified Behavior

The debug builder script currently reports:

```text
sampled_prm_nodes=324
sampled_prm_edges=1088
```

The temporal roadmap demo now runs on the sampled PRM-style roadmap and finds a path while rejecting dynamic-obstacle-invalid edges.
