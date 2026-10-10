from typing import Protocol
from auv_contracts.values import CameraCalibration

class DisparityEstimator(Protocol):
    def estimate(self, left, right, started):
        """Return disparity pixels, preprocessing/inference ms, postprocess start."""
        ...
