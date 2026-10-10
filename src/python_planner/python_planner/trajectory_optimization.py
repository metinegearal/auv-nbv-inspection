"""Numerical composition entry point; acceptance policy stays in use_cases."""
from .use_cases.trajectory_optimization import estimate_velocity, continuity_cost
from .use_cases.trajectory_optimization import optimize_trajectory as _optimize

def optimize_trajectory(*args, **kwargs):
    from .adapters.optimization import ScipySolver
    return _optimize(*args, solver=ScipySolver(), **kwargs)
