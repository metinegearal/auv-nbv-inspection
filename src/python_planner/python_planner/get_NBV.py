"""Compatibility/composition entry point for standalone NBV callers."""
from .use_cases.nbv import *
from .use_cases.nbv import calculate_nbv_waypoints as _calculate

def calculate_nbv_waypoints(occupied_points, auv_position, auv_heading,
                           observation_history=None, parameters=None,
                           timing_callback=None):
    import time
    from rclpy.logging import get_logger
    from .adapters.point_cloud import Open3DPreprocessor
    return _calculate(occupied_points, auv_position, auv_heading,
                      observation_history, parameters, timing_callback,
                      preprocessor=Open3DPreprocessor(),
                      diagnostics=get_logger('nbv_waypoint_calculator'), clock=time)
