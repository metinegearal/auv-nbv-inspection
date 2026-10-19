"""Backward-compatible explicit composition facade for non-ROS callers."""
from auv_contracts.values import CameraCalibration
_default = None

def create_depth_use_case():
    import time
    from .adapters.bgnet import BGNetDisparityEstimator
    from .use_cases.depth import StereoDepthUseCase
    return StereoDepthUseCase(BGNetDisparityEstimator(clock=time), time)

def compute_stereo_depth(left_img, right_img, focal_length_px, baseline_meters):
    global _default
    if _default is None:
        _default = create_depth_use_case()
    return _default.compute(left_img, right_img,
                            CameraCalibration(focal_length_px, baseline_meters))

def get_last_depth_timings():
    return {} if _default is None else _default.timings()
