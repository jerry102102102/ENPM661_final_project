"""Roadmap data structures for future temporal query planning."""

from __future__ import annotations

from dataclasses import dataclass, field

from src.models.state import Pose2D, TrajectorySegment


@dataclass(frozen=True)
class RoadmapNode:
    """Sampled nonholonomic roadmap node."""

    node_id: int
    pose: Pose2D


@dataclass(frozen=True)
class RoadmapEdge:
    """Directed primitive edge between two sampled roadmap nodes."""

    edge_id: int
    source_id: int
    target_id: int
    primitive_label: str | None
    trajectory: TrajectorySegment
    duration_s: float
    geometric_cost: float
    static_valid: bool


@dataclass
class Roadmap:
    """Container for sampled nodes and directed primitive edges."""

    nodes: dict[int, RoadmapNode] = field(default_factory=dict)
    edges: dict[int, RoadmapEdge] = field(default_factory=dict)
    outgoing_edges: dict[int, list[int]] = field(default_factory=dict)

    def add_node(self, pose: Pose2D) -> RoadmapNode:
        node = RoadmapNode(len(self.nodes), pose)
        self.nodes[node.node_id] = node
        self.outgoing_edges.setdefault(node.node_id, [])
        return node

    def add_edge(
        self,
        source_id: int,
        target_id: int,
        primitive_label: str | None,
        trajectory: TrajectorySegment,
        duration_s: float,
        static_valid: bool,
        geometric_cost: float | None = None,
    ) -> RoadmapEdge:
        edge = RoadmapEdge(
            edge_id=len(self.edges),
            source_id=source_id,
            target_id=target_id,
            primitive_label=primitive_label,
            trajectory=trajectory,
            duration_s=duration_s,
            geometric_cost=trajectory.cost if geometric_cost is None else geometric_cost,
            static_valid=static_valid,
        )
        self.edges[edge.edge_id] = edge
        self.outgoing_edges.setdefault(source_id, []).append(edge.edge_id)
        return edge
