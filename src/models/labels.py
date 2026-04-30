"""Temporal search labels for future roadmap query planning."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TemporalSearchLabel:
    """A future query-search label keyed by node and arrival time."""

    node_id: int
    arrival_time: float
    cost_to_come: float
    parent_label_id: int | None = None
    incoming_edge_id: int | None = None
