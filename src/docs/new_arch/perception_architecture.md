# `perception` Architecture

## Overview
The `perception` package provides stereo depth estimation using the vendored `BGNet` (Bilateral Grid Learning) neural network. It has been refactored to decouple the PyTorch model lifecycle from the ROS 2 image synchronization.

## Layer Breakdown

### 1. Domain (`perception/domain/`)
*   **`depth.py`:** Defines the `DisparityEstimator` Protocol, ensuring the core use case does not hard-depend on BGNet specifically.

### 2. Use Cases (`perception/use_cases/`)
*   **`depth.py` (`StereoDepthUseCase`):** Orchestrates the conversion from left/right images to a metric depth map. It takes raw disparity from the estimator, applies the `CameraCalibration` focal length/baseline math, masks out invalid pixels, and calculates execution timings.

### 3. Interface Adapters (`perception/adapters/` & `BGNet/`)
*   **`bgnet.py` (`BGNetDisparityEstimator`):** The concrete PyTorch adapter. Its constructor loads the packaged `Sceneflow-IRS-BGNet-Plus.pth` checkpoint onto CUDA when available or CPU otherwise, retains the `ToTensor` transform, and executes the forward pass inside `torch.no_grad()`. Importing the metric-depth policy does not load a model.
*   **`BGNet/` Submodule:** Vendored model internals are used behind the adapter. Dataset packages are excluded from the ROS distribution; checkout-only prediction scripts remain research entry points. `prediction_registry.py` owns model/output choices, and imports support both package and direct research-script execution.
*   **`depth_estimation.py` (`obstacleAvoidance`):** The ROS 2 Node. It uses `message_filters.ApproximateTimeSynchronizer` to align stereo image pairs, passes them to the Use Case, and publishes the resulting 32-bit float depth map using the exact timestamp of the original source image.

The existing camera conversion and reversed stereo-call order are retained. Calibration remains `CameraCalibration(256, 0.5)` rather than new ROS parameters. `PipelineFileSink` supplies timing storage from `telemetry_adapters`; perception no longer imports benchmarking code. Torch inference and checkpoint loading were not executed by the framework-free regression tests; see the [validation report](../architecture/refactoring_status.md).
