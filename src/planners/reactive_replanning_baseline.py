"""Module boundary for the future reactive nonholonomic replanning baseline."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReactiveReplanningConfig:
    """Configuration placeholder for future reactive replanning experiments."""

    replan_period_s: float = 1.0
    lookahead_time_s: float = 3.0


class ReactiveReplanningBaseline:
    """Scaffold for a dynamic-environment reactive baseline.

    The implementation is intentionally deferred. This class marks the future
    boundary where static nonholonomic A* can be repeatedly invoked as the
    dynamic scene changes.
    """

    def __init__(self, config: ReactiveReplanningConfig | None = None) -> None:
        self.config = config or ReactiveReplanningConfig()

    def plan(self, *args, **kwargs):
        raise NotImplementedError("Reactive replanning baseline will be implemented in the next phase.")
