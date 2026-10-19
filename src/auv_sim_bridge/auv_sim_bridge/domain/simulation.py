from dataclasses import dataclass
from typing import Protocol, Any, ContextManager, Callable

@dataclass(frozen=True)
class Command:
    values: tuple[float, ...]
    def __post_init__(self):
        object.__setattr__(self, 'values', tuple(self.values))

class SimulationSession(Protocol):
    def step(self, command: Command) -> Any: ...
    def set_ocean_currents(self, agent: str, velocity) -> None: ...

class SessionFactory(Protocol):
    def open(self) -> ContextManager[SimulationSession]: ...

class SensorDispatcher(Protocol):
    def dispatch(self, state, session: SimulationSession) -> None: ...

class CurrentField(Protocol):
    def __call__(self, location): ...
