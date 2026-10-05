"""Metric depth policy over disparity values; no model or ROS dependencies."""
from auv_contracts.ports import Clock
from ..domain.depth import DisparityEstimator

class StereoDepthUseCase:
    def __init__(self, estimator: DisparityEstimator, clock: Clock):
        self.estimator = estimator
        self.clock = clock
        self.last_depth_timings = {}

    def compute(self, left_img, right_img, calibration):
        started = self.clock.perf_counter()
        (disparity_orig, preprocessing_ms, inference_ms,
         postprocessing_started) = self.estimator.estimate(left_img, right_img, started)
        # 7. Convert Disparity to Depth (Meters)
        # Filter invalid disparities to prevent divide-by-zero
        disparity_orig[disparity_orig <= 0.0] = 0.1

        depth_map = (calibration.focal_length_px * calibration.baseline_meters) / disparity_orig

        # Mask out bad depth
        depth_map[disparity_orig == 0.1] = 0.0 

        h, w = depth_map.shape
        cy, cx = h // 2, w // 2

        # 2. Extract the metric depth at the center
        center_depth = depth_map[cy, cx]
        postprocessing_ms = (self.clock.perf_counter() - postprocessing_started) * 1000.0
        self.last_depth_timings = {
            'preprocessing_ms': preprocessing_ms,
            'inference_ms': inference_ms,
            'postprocessing_ms': postprocessing_ms,
            'total_compute_ms': (self.clock.perf_counter() - started) * 1000.0,
            'center_depth_m': float(center_depth),
        }

        return depth_map

    def timings(self):
        return dict(self.last_depth_timings)
