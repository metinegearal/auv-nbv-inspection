# Package Architecture Audit: perception

## Current Metrics

- Fan-in: 1 | Fan-out: 1
- Instability (I): 0.500
- Abstractness (A): 0.000 (Na=0; Nc=20)
- Distance (D): 0.500
- Status: Zone of Pain

Scope: ROS stereo depth adapter, eager global inference module and vendored BGNet research code (20 concrete classes total). BGNet is a pinned Git submodule with its own upstream provenance, but has no separate package.xml or setup.py release unit here; its Python subpackages are measured inside the enclosing ROS component.

Counts use the observed source/deployment graph, not ROS topic flow. See [Phase 1 definitions and denominator conventions](../phase_1_topology.md#metric-definitions-and-counting-rules). An asterisk denotes a mathematically undefined denominator replaced by zero only for display; the resulting D/status is conditional. Classification alone is not a violation. All file paths below are relative to `NBV_paper_ros2/src`.

## Violations Found
| ID | Rule Broken | File & Line / Symbol | Concrete Issue | Actor / Architectural Impact |
|---|---|---|---|---|
| <a id="v-01"></a>V-01 | DIP / SRP | `perception/perception/compute_depth.py:15` / `global inference initialization` | **Critical.** Importing compute_stereo_depth constructs BGNet_Plus, chooses the device, loads a concrete checkpoint and configures transforms at module scope (lines 18–24). | Depth computation cannot be imported without model-loading infrastructure; inference research, deployment paths and hardware startup share module lifecycle. |
| <a id="v-02"></a>V-02 | REP / DIP | `perception/perception/compute_depth.py:7` / `sys.path and checkpoint paths` | **Moderate.** Absolute workstation sys.path and model-file paths resolve top-level models/datasets imports outside the normal installed perception namespace. | Installation/reuse depends on source checkout layout; model packaging and application startup are coupled. |
| <a id="v-03"></a>V-03 | SRP / DIP | `perception/perception/depth_estimation.py:61` / `sync_cam_callback` | **Moderate.** ROS callback converts images, applies camera-specific inference arguments, invokes the concrete depth function, records file-backed telemetry and publishes output; logger is constructed at line 29. | ROS synchronization, model selection/calibration and experiment logging cannot change independently at a use-case boundary. |
| <a id="v-04"></a>V-04 | DIP / CRP / ADP | `perception/perception/depth_estimation.py:13` / `MidrunLogger import` | **Moderate.** Perception depends on benchmark_evaluator only to obtain a timing writer, pulling an experiment component into runtime depth processing and closing a deployment cycle through bringup. | Logging/storage changes and experiment component packaging affect the perception release. |
| <a id="v-05"></a>V-05 | CCP / CRP | `perception/setup.py:8` / `find_packages and BGNet subtree` | **Moderate.** Runtime ROS inference and vendored model variants, dataset classes and research prediction utilities occupy the same package tree/release boundary. | Consumers needing only depth inference inherit changes for dataset/research tooling they do not reuse. Keep framework model internals cohesive behind an adapter. |
| <a id="v-06"></a>V-06 | OCP | `perception/perception/BGNet/predict.py:36` / `model and dataset selection` | **Low.** Model selection and output-folder selection use literal if/elif branches; adding a model or dataset requires editing the prediction script in addition to model/dataset definitions. | Research variant authors modify the central runner; a registry or injected factory can localize extension. CLI composition is a low-severity boundary, not a demand for polymorphism in every branch. |
| <a id="v-07"></a>V-07 | REP (dependency metadata) | `perception/package.xml:11` / `dependency declarations` | **Moderate.** Manifest declares benchmark_evaluator and message_filters but omits rclpy, sensor/std messages, cv_bridge, NumPy, OpenCV, Torch, torchvision and PIL used by runtime source; setup.py declares only setuptools. | The release unit does not communicate its runtime installation requirements. |

## Proposed Structural Boundary
- Interfaces to extract: `StereoDepthEstimator`, `DepthOutput`, `PipelineTimingSink`, `ModelLoader`; pass StereoPair, CameraCalibration and DepthFrame values through a client-owned boundary.
- Dependencies to invert or decouple: Construct BGNet/checkpoint/device adapters explicitly at startup. Keep ROS synchronization outside depth policy and reuse only narrow telemetry contracts. Isolate vendored research dataset and prediction utilities from runtime inference packaging.

## Current Internal Package Diagram

Solid arrows are calls, construction, field ownership or source dependencies as labeled. These diagrams summarize source relationships, not measured runtime execution; dormant paths are labeled. Red nodes mix policy with details.

```mermaid
graph LR
 R[obstacleAvoidance.sync_cam_callback] --> CV[CvBridge / OpenCV / message_filters]
 R --> DEP[compute_stereo_depth]
 R --> LOG[benchmark_evaluator.MidrunLogger]
 DEP --> GLOBAL[global BGNet_Plus / device / checkpoint]
 GLOBAL --> FS[absolute model path and sys.path]
 DEP --> TRANS[get_transform / PIL / Torch]
 MODEL[BGNet and BGNet_Plus] --> FE[feature_extraction / BasicBlock / BasicConv / Conv2x]
 MODEL --> CO[CoeffsPredictor extends HourGlass]
 MODEL --> HG[HourglassRefinement]
 MODEL --> SL[Slice / GuideNN / correlation and disparity functions]
 HG --> WARP[disp_warp]
 CO --> SUB[SubModule and convolution factories]
 HG --> SUB
 PRED[predict.py and predict_sample.py] --> MODEL
 PRED --> DATA[KITTIDataset / KITTI_12_Dataset / data_io]
 classDef leak fill:#ffeded,stroke:#bf3030
 class R,DEP,GLOBAL leak
```

## Ideal Clean Internal Package Diagram

This is a proposed partition, not an implemented structure. Ports are policy-owned abstract interfaces (Python Protocol/ABC, C++ pure virtual classes); implementations depend on them. Entities cross ports as plain values. A grouped port box means separate role-specific interfaces, never one mandatory aggregate interface.

```mermaid
graph LR
 subgraph Adapters[Interface Adapters]
 R[ROS synchronization and CvBridge]
 B[BGNet adapter: Torch / model loader / preprocessor]
 T[File timing adapter]
 CLI[Separate research dataset and prediction entry points]
 end
 subgraph UseCases[Use Cases]
 U[EstimateStereoDepth]
 P[StereoDepthEstimator / TimingSink / DepthOutput interfaces]
 end
 subgraph Entities
 E[StereoPair / CameraCalibration / DepthFrame / StageTiming]
 end
 R --> U
 U --> P
 B -. implements .-> P
 T -. implements .-> P
 CLI --> B
 P --> E
 U --> E
```

## Principle Assessment and Non-Findings

PyTorch models legitimately depend on Torch as outer inference implementations; those imports are not independently labeled DIP violations. Dataset and Module subclasses show no demonstrated substitution-contract violation. The inconsistent dataset load_path tuple arity and conditionally defined get_last_depth_timings are ordinary contract/correctness concerns, recorded separately rather than mislabeled LSP. No abstract fat interface is implemented here; the concrete logger dependency is assessed under benchmark ISP. ADP membership comes from deployment plus telemetry, not BGNet cyclic imports.

Actor labels are inferred responsibilities, not verified organizational roles. SOLID findings require responsibility, client or contract evidence; line count, inheritance, ordinary guards, framework adapters and missing abstract classes alone do not prove a violation. REP findings establish incomplete reuse/deployment contracts; no release-history audit was performed.

## Module-by-Module Target Placement

| File | Current defined classes / top-level functions | Ideal placement / diagnostic disposition |
|---|---|---|
| `perception/package.xml` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `perception/perception/BGNet/__init__.py` | No classes/top-level Python functions | Empty package marker; no executable policy or class interaction to partition. |
| `perception/perception/BGNet/dataloader/__init__.py` | No classes/top-level Python functions | Empty package marker; no executable policy or class interaction to partition. |
| `perception/perception/BGNet/datasets/__init__.py` | No classes/top-level Python functions | Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform. |
| `perception/perception/BGNet/datasets/data_io.py` | `mean_std_transform:11`; `get_transform:42`; `read_all_lines:54`; `pfm_imread:61` | Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform. |
| `perception/perception/BGNet/datasets/kitti12_dataset.py` | `KITTI_12_Dataset:15` | Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform. |
| `perception/perception/BGNet/datasets/kitti_dataset.py` | `KITTIDataset:15` | Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform. |
| `perception/perception/BGNet/models/__init__.py` | No classes/top-level Python functions | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/bgnet.py` | `Slice:16`; `GuideNN:26`; `BGNet:67`; `groupwise_correlation:36`; `build_gwc_volume:45`; `correlation:56`; `disparity_regression:62` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/bgnet_plus.py` | `Slice:16`; `GuideNN:27`; `BGNet_Plus:70`; `groupwise_correlation:37`; `build_gwc_volume:47`; `correlation:59`; `disparity_regression:65` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/feature_extractor_fast.py` | `BasicBlock:23`; `BasicConv:44`; `Conv2x:72`; `feature_extraction:104`; `convbn_relu:11`; `convbn:17` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/submodules.py` | `BasicBlock:64`; `SubModule:86`; `convbn_2d_lrelu:9`; `convbn_2d_Tanh:16`; `deconvbn_2d_lrelu:23`; `convbn_3d_lrelu:30`; `conv_relu:38`; `convbn:43`; `convbn_relu:50`; `convbn_transpose_3d:58` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/submodules2d.py` | `BasicConv:24`; `Conv2x:52`; `HourglassRefinement:87`; `disparity_regression:12`; `conv2d:17` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/models/submodules3d.py` | `HourGlass:11`; `CoeffsPredictor:33` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/nets/warp.py` | `disp_warp:9` | Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate. |
| `perception/perception/BGNet/predict.py` | No classes/top-level Python functions | Research CLI/composition adapter; keep model selection and filesystem outside the reusable depth policy. |
| `perception/perception/BGNet/predict.sh` | No classes/top-level Python functions | Research CLI/composition adapter; keep model selection and filesystem outside the reusable depth policy. |
| `perception/perception/BGNet/predict_sample.py` | No classes/top-level Python functions | Research CLI/composition adapter; keep model selection and filesystem outside the reusable depth policy. |
| `perception/perception/__init__.py` | No classes/top-level Python functions | Empty package marker; no executable policy or class interaction to partition. |
| `perception/perception/compute_depth.py` | `compute_stereo_depth:43` | Depth use case plus constructed BGNet model adapter; remove import-time composition in a later migration. |
| `perception/perception/depth_estimation.py` | `obstacleAvoidance:18`; `main:127` | ROS synchronization/input-output adapter around depth use case. |
| `perception/setup.cfg` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `perception/setup.py` | No classes/top-level Python functions | Release/build metadata; reconcile dependencies with source. No domain policy extraction. |
| `perception/test/test_copyright.py` | `test_copyright:23` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |
| `perception/test/test_flake8.py` | `test_flake8:21` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |
| `perception/test/test_pep257.py` | `test_pep257:21` | Test-only consumer; excluded from production metrics. Static review only; no test execution claimed. |

## Class Counting Evidence

- `perception/perception/BGNet/datasets/kitti12_dataset.py`: `KITTI_12_Dataset` (line 15).
- `perception/perception/BGNet/datasets/kitti_dataset.py`: `KITTIDataset` (line 15).
- `perception/perception/BGNet/models/bgnet.py`: `Slice` (line 16), `GuideNN` (line 26), `BGNet` (line 67).
- `perception/perception/BGNet/models/bgnet_plus.py`: `Slice` (line 16), `GuideNN` (line 27), `BGNet_Plus` (line 70).
- `perception/perception/BGNet/models/feature_extractor_fast.py`: `BasicBlock` (line 23), `BasicConv` (line 44), `Conv2x` (line 72), `feature_extraction` (line 104).
- `perception/perception/BGNet/models/submodules.py`: `BasicBlock` (line 64), `SubModule` (line 86).
- `perception/perception/BGNet/models/submodules2d.py`: `BasicConv` (line 24), `Conv2x` (line 52), `HourglassRefinement` (line 87).
- `perception/perception/BGNet/models/submodules3d.py`: `HourGlass` (line 11), `CoeffsPredictor` (line 33).
- `perception/perception/depth_estimation.py`: `obstacleAvoidance` (line 18).

## Function and Class Interaction Detail

The following module diagrams are static Python AST call/name and class-containment maps. Dotted edges indicate containment, solid edges an explicit local call. Repeated method names are resolved within the containing class; nested functions belong to their lexical owner. Dynamic dispatch, callbacks registered for later execution and arbitrary imported calls are not inferred as exact call edges. Package diagrams above supply the cross-module and C++ relationships. See [source coverage](../source_coverage.md) for all imports, C++ includes and function symbols.

### `perception/BGNet/__init__.py`

Ideal placement: Empty package marker; no executable policy or class interaction to partition.

No executable local symbol interaction (empty marker or import/registry facade).

### `perception/BGNet/dataloader/__init__.py`

Ideal placement: Empty package marker; no executable policy or class interaction to partition.

No executable local symbol interaction (empty marker or import/registry facade).

### `perception/BGNet/datasets/__init__.py`

Ideal placement: Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform.

No executable local symbol interaction (empty marker or import/registry facade).

### `perception/BGNet/datasets/data_io.py`

Ideal placement: Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform.

```mermaid
graph LR
 s0["mean_std_transform : 11"]
 s1["get_transform : 42"]
 s2["read_all_lines : 54"]
 s3["pfm_imread : 61"]
```

### `perception/BGNet/datasets/kitti12_dataset.py`

Ideal placement: Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform.

```mermaid
graph LR
 s0["KITTI_12_Dataset : 15"]
 s1["KITTI_12_Dataset.__init__ : 16"]
 s2["KITTI_12_Dataset.load_path : 22"]
 s3["KITTI_12_Dataset.load_image : 36"]
 s4["KITTI_12_Dataset.load_disp : 39"]
 s5["KITTI_12_Dataset.__len__ : 44"]
 s6["KITTI_12_Dataset.__getitem__ : 47"]
 s0 -.-> s1
 s0 -.-> s2
 s0 -.-> s3
 s0 -.-> s4
 s0 -.-> s5
 s0 -.-> s6
 s1 --> s2
 s6 --> s3
 s6 --> s4
```

### `perception/BGNet/datasets/kitti_dataset.py`

Ideal placement: Vendored dataset/loading adapter. Isolate from production depth-policy release; no need to wrap each transform.

```mermaid
graph LR
 s0["KITTIDataset : 15"]
 s1["KITTIDataset.__init__ : 16"]
 s2["KITTIDataset.load_path : 22"]
 s3["KITTIDataset.load_image : 36"]
 s4["KITTIDataset.load_disp : 39"]
 s5["KITTIDataset.__len__ : 44"]
 s6["KITTIDataset.__getitem__ : 47"]
 s0 -.-> s1
 s0 -.-> s2
 s0 -.-> s3
 s0 -.-> s4
 s0 -.-> s5
 s0 -.-> s6
 s1 --> s2
 s6 --> s3
 s6 --> s4
```

### `perception/BGNet/models/__init__.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

No executable local symbol interaction (empty marker or import/registry facade).

### `perception/BGNet/models/bgnet.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["Slice : 16"]
 s1["Slice.__init__ : 17"]
 s2["Slice.forward : 19"]
 s3["GuideNN : 26"]
 s4["GuideNN.__init__ : 27"]
 s5["GuideNN.forward : 33"]
 s6["groupwise_correlation : 36"]
 s7["build_gwc_volume : 45"]
 s8["correlation : 56"]
 s9["disparity_regression : 62"]
 s10["BGNet : 67"]
 s11["BGNet.__init__ : 68"]
 s12["BGNet.forward : 88"]
 s0 -.-> s1
 s0 -.-> s2
 s10 -.-> s11
 s10 -.-> s12
 s11 --> s0
 s11 --> s3
 s12 --> s7
 s3 -.-> s4
 s3 -.-> s5
 s7 --> s6
```

### `perception/BGNet/models/bgnet_plus.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["Slice : 16"]
 s1["Slice.__init__ : 17"]
 s2["Slice.forward : 20"]
 s3["GuideNN : 27"]
 s4["GuideNN.__init__ : 28"]
 s5["GuideNN.forward : 34"]
 s6["groupwise_correlation : 37"]
 s7["build_gwc_volume : 47"]
 s8["correlation : 59"]
 s9["disparity_regression : 65"]
 s10["BGNet_Plus : 70"]
 s11["BGNet_Plus.__init__ : 71"]
 s12["BGNet_Plus.forward : 84"]
 s0 -.-> s1
 s0 -.-> s2
 s10 -.-> s11
 s10 -.-> s12
 s11 --> s0
 s11 --> s3
 s12 --> s7
 s3 -.-> s4
 s3 -.-> s5
 s7 --> s6
```

### `perception/BGNet/models/feature_extractor_fast.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["convbn_relu : 11"]
 s1["convbn : 17"]
 s2["BasicBlock : 23"]
 s3["BasicBlock.__init__ : 26"]
 s4["BasicBlock.forward : 34"]
 s5["BasicConv : 44"]
 s6["BasicConv.__init__ : 46"]
 s7["BasicConv.forward : 63"]
 s8["Conv2x : 72"]
 s9["Conv2x.__init__ : 74"]
 s10["Conv2x.forward : 92"]
 s11["feature_extraction : 104"]
 s12["feature_extraction.__init__ : 105"]
 s13["feature_extraction._make_layer : 139"]
 s14["feature_extraction.forward : 154"]
 s11 -.-> s12
 s11 -.-> s13
 s11 -.-> s14
 s12 --> s0
 s12 --> s13
 s12 --> s5
 s12 --> s8
 s2 -.-> s3
 s2 -.-> s4
 s3 --> s0
 s3 --> s1
 s5 -.-> s6
 s5 -.-> s7
 s8 -.-> s10
 s8 -.-> s9
 s9 --> s5
```

### `perception/BGNet/models/submodules.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["convbn_2d_lrelu : 9"]
 s1["convbn_2d_Tanh : 16"]
 s2["deconvbn_2d_lrelu : 23"]
 s3["convbn_3d_lrelu : 30"]
 s4["conv_relu : 38"]
 s5["convbn : 43"]
 s6["convbn_relu : 50"]
 s7["convbn_transpose_3d : 58"]
 s8["BasicBlock : 64"]
 s9["BasicBlock.__init__ : 67"]
 s10["BasicBlock.forward : 75"]
 s11["SubModule : 86"]
 s12["SubModule.__init__ : 87"]
 s13["SubModule.weight_init : 90"]
 s11 -.-> s12
 s11 -.-> s13
 s8 -.-> s10
 s8 -.-> s9
 s9 --> s5
 s9 --> s6
```

### `perception/BGNet/models/submodules2d.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["disparity_regression : 12"]
 s1["conv2d : 17"]
 s2["BasicConv : 24"]
 s3["BasicConv.__init__ : 26"]
 s4["BasicConv.forward : 43"]
 s5["Conv2x : 52"]
 s6["Conv2x.__init__ : 54"]
 s7["Conv2x.forward : 78"]
 s8["HourglassRefinement : 87"]
 s9["HourglassRefinement.__init__ : 90"]
 s10["HourglassRefinement.forward : 122"]
 s2 -.-> s3
 s2 -.-> s4
 s5 -.-> s6
 s5 -.-> s7
 s6 --> s2
 s8 -.-> s10
 s8 -.-> s9
 s9 --> s1
 s9 --> s2
 s9 --> s5
```

### `perception/BGNet/models/submodules3d.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["HourGlass : 11"]
 s1["HourGlass.__init__ : 12"]
 s2["CoeffsPredictor : 33"]
 s3["CoeffsPredictor.__init__ : 35"]
 s4["CoeffsPredictor.forward : 38"]
 s0 -.-> s1
 s2 -.-> s3
 s2 -.-> s4
```

### `perception/BGNet/nets/warp.py`

Ideal placement: Vendored Torch inference implementation behind StereoDepthEstimator; concrete internal network composition is appropriate.

```mermaid
graph LR
 s0["disp_warp : 9"]
```

### `perception/BGNet/predict.py`

Ideal placement: Research CLI/composition adapter; keep model selection and filesystem outside the reusable depth policy.

```mermaid
graph LR
 M[Module-level composition] --> I[Imported model / CLI / IO operations]
```

No local class/function definitions; operations run at module scope. See import-time lifecycle findings where applicable.

### `perception/BGNet/predict_sample.py`

Ideal placement: Research CLI/composition adapter; keep model selection and filesystem outside the reusable depth policy.

```mermaid
graph LR
 M[Module-level composition] --> I[Imported model / CLI / IO operations]
```

No local class/function definitions; operations run at module scope. See import-time lifecycle findings where applicable.

### `perception/__init__.py`

Ideal placement: Empty package marker; no executable policy or class interaction to partition.

No executable local symbol interaction (empty marker or import/registry facade).

### `perception/compute_depth.py`

Ideal placement: Depth use case plus constructed BGNet model adapter; remove import-time composition in a later migration.

```mermaid
graph LR
 s0["get_last_depth_timings : 35"]
 s1["compute_stereo_depth : 43"]
```

### `perception/depth_estimation.py`

Ideal placement: ROS synchronization/input-output adapter around depth use case.

```mermaid
graph LR
 s0["obstacleAvoidance : 18"]
 s1["obstacleAvoidance.__init__ : 19"]
 s2["obstacleAvoidance.sync_cam_callback : 61"]
 s3["obstacleAvoidance._publish_depth : 104"]
 s4["main : 127"]
 s0 -.-> s1
 s0 -.-> s2
 s0 -.-> s3
 s2 --> s3
 s4 --> s0
```
