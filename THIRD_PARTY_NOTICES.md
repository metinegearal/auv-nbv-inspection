# Third-party licensing and provenance

Project-authored code and documentation are Apache-2.0; see [LICENSE](LICENSE)
and [NOTICE](NOTICE). That grant does not replace the terms of the components
below. Existing upstream source notices are retained.

| Component | Location / version | Terms and status |
| --- | --- | --- |
| SciPy numerical routines | `src/auv_inspection_planner/vendor/scipy`, imported from SciPy 1.16.3 | BSD-3-Clause. Original license and per-source notices are preserved in `vendor/scipy/LICENSE.txt`. The L-BFGS-B copyright and notice are also in `LICENSES/SciPy-LBFGSB-BSD-3-Clause.txt`. |
| nlohmann/json | `src/auv_inspection_planner/vendor/json.hpp`, version 3.11.3 | MIT. Source copyright/SPDX notices are preserved; the upstream license text is in `LICENSES/nlohmann-json-MIT.txt`, with embedded-component attributions in `LICENSES/nlohmann-json-ATTRIBUTIONS.txt`. |
| BGNet | Git submodule `src/auv_stereo_depth/auv_stereo_depth/BGNet`, recorded commit `19b9c1bc17ebf653b344e52470ae6f85953c17a9` | No explicit license file or permission grant was found in this checkout. Upstream code, inherited third-party code, and local modifications are not relicensed by this project's Apache-2.0 declaration. Redistribution permission remains unverified. |
| Pretrained BGNet weights | In particular `Sceneflow-IRS-BGNet-Plus.pth` referenced by the stereo-depth package | No separate model-weight permission grant was verified. Availability for download or inclusion in a repository does not establish redistribution rights. Excluded from this project's Apache-2.0 grant. |
| Datasets, HoloOcean and simulator assets | External inputs and dependencies | Governed by their own provider terms. No rights to these materials are granted by this project's license. This also applies to derived maps or screenshots to the extent they contain protected third-party material. |

## Numerical source provenance

The SciPy import is documented in
[`vendor/scipy/README.md`](src/auv_inspection_planner/vendor/scipy/README.md).
FITPACK sources are unchanged. The cKDTree import replaces NumPy integer/branch
macros with C++ equivalents; L-BFGS-B removes the CPython wrapper and exports
`setulb`. Preserve the upstream attribution and license conditions when
redistributing these sources or binaries containing them.

Upstream sources:

- [SciPy 1.16.3](https://github.com/scipy/scipy/tree/v1.16.3)
- [nlohmann/json 3.11.3 license](https://github.com/nlohmann/json/blob/v3.11.3/LICENSE.MIT)
- [BGNet](https://github.com/3DCVdeveloper/BGNet)

## BGNet and model release status

Checked on October 19, 2026: the pinned BGNet checkout has no license file and
its README provides no explicit redistribution grant. The upstream GitHub
license lookup also returned 404. BGNet acknowledges code adopted from
DeepPruner, GwcNet, GANet and AANet, whose applicable terms also need review
before redistributing the affected code or model artifacts.

The Apache-2.0 metadata in `auv_stereo_depth` describes the project-authored
wrapper, not a grant for BGNet or its weights. Obtain and record the applicable
upstream permissions before distributing those components, including wheels,
ROS binary packages, containers or model bundles that include them. This
document records unresolved permissions; it is not a replacement license.

## Separately installed dependencies

ROS 2, Open3D, OctoMap, PCL, Eigen, OpenCV, NumPy, SciPy, PyTorch and other
dependencies keep their own licenses. Installing or linking a dependency does
not put that dependency under this repository's license. Preserve the notices
and meet the terms of the actual dependency versions shipped in a distribution.

## Repository-wide licensing

The repository is intended to be cloned and used as a whole. One root LICENSE
and NOTICE apply to project-authored code; package metadata records the license
identifiers without duplicating or installing those texts per package.
Third-party notices are centralized here and in the root LICENSES directory,
alongside the unchanged original vendor license. BGNet/model caveats above
remain applicable. Third-party source files, frozen incident fixtures and
historical migration evidence are not rewritten by this licensing update.
