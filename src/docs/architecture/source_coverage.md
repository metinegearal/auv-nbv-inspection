# Source Coverage and Evidence Inventory

Snapshot: 2026-10-05. Paths are relative to `NBV_paper_ros2/src`. This inventory enumerates every scoped source/script plus build/configuration file; no application imports were executed. Package reports contain rule findings and target placement for every row. Hashes identify the exact inspected content and are not security claims.

## Scope and Class Counting

Seven ROS package.xml release units; 92 text files in this inventory, of which 74 are Python/C++/header/shell sources. Excluded from Nc: tests, functions, aliases and imported types. Include C++ structs, dormant production classes and vendored BGNet. Na counts explicit abstract declarations, not inferred duck typing or inherited framework hooks. No local abstract declaration was identified. BGNet SubModule and HourGlass are concrete helper bases without a declared abstract interface.

## Per-File Review Ledger

The ledger covers architectural source relationships, not complete behavioral verification. “No independent finding” means a module has no separately evidenced rule violation beyond the containing package findings; it is not a blanket correctness assertion.

| File | Lines | Production classes / structs | Review disposition | SHA-256 |
|---|---:|---:|---|---|
| `active_inspection_planner/CMakeLists.txt` | 57 | 0 | [V-05](packages/active_inspection_planner_audit.md#v-05) | `c9637bcf9471f4adcd7efe8b29074d8840d4a2373c2993c7d5e1fddd4249718a` |
| `active_inspection_planner/package.xml` | 22 | 0 | [V-06](packages/active_inspection_planner_audit.md#v-06) | `5025c4a37cdaa6cae88b8eab1a18f4639c3f206db8435485c881057679de555a` |
| `active_inspection_planner/src/nbv_utility_node.cpp` | 205 | 1 | [V-04](packages/active_inspection_planner_audit.md#v-04) | `66ad8c45d4693e6ab58c65a8bcb19be0b68880a77ab8aecca49b5e21abd3a3a4` |
| `active_inspection_planner/src/realtime_stereo_mapper.cpp` | 572 | 0 | [V-02](packages/active_inspection_planner_audit.md#v-02), [V-03](packages/active_inspection_planner_audit.md#v-03) | `11d5859799d89f8323039a1467ab5bc9d64700d57281d4c568d78789b8ee8ac3` |
| `active_inspection_planner/src/realtime_stereo_mapper.hpp` | 71 | 2 | [V-01](packages/active_inspection_planner_audit.md#v-01) | `89bd1a46fecc9f2f77acaefa574243b6893381604ee9a61eb3aa5361e0467049` |
| `active_inspection_planner/src/search_algo.cpp` | 204 | 1 | [Package/module assessment](packages/active_inspection_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `34ce8bcf34e2a7b910c67b83a25611e70d854f08d995e17f0768cb85e39d724b` |
| `active_inspection_planner/src/search_algo.hpp` | 42 | 3 | [Package/module assessment](packages/active_inspection_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `b490af1baced7ffa40fb9593e973fb69dc5303f728039b7e31006c4d95b6256c` |
| `benchmark_evaluator/benchmark_evaluator/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `benchmark_evaluator/benchmark_evaluator/benchmark_evaluator.py` | 152 | 0 | [V-05](packages/benchmark_evaluator_audit.md#v-05) | `81946ea4fdcc0cd3813b44041b623bd401d8aabdd870eb32e9f77fb6c9a55f58` |
| `benchmark_evaluator/benchmark_evaluator/midrun_logger.py` | 270 | 1 | [V-02](packages/benchmark_evaluator_audit.md#v-02), [V-03](packages/benchmark_evaluator_audit.md#v-03) | `544af802203dd50ac8f5fca5dd83d7426a626b4bc62897b43cfe8e143b2f6859` |
| `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py` | 148 | 0 | [V-01](packages/benchmark_evaluator_audit.md#v-01), [V-06](packages/benchmark_evaluator_audit.md#v-06) | `7d1858535882a2861eec46e6aa67e685229a0c368ffd495b0060cf441e74df8f` |
| `benchmark_evaluator/benchmark_evaluator/plot_results.py` | 25 | 0 | [Package/module assessment](packages/benchmark_evaluator_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `22d3c1b4e93cc77785cdef1717883bae74121495cf7ca39d83052d48674cc96d` |
| `benchmark_evaluator/benchmark_evaluator/run_baselines.py` | 375 | 1 | [V-04](packages/benchmark_evaluator_audit.md#v-04), [V-07](packages/benchmark_evaluator_audit.md#v-07) | `a22f6488b38a5100537dd3b4cc8b076e3452cd495623a94eb52efd973b2b1dd5` |
| `benchmark_evaluator/package.xml` | 20 | 0 | [V-08](packages/benchmark_evaluator_audit.md#v-08) | `a99be271b13f0f6252107104cfea0fcef50e381262ce2fa1fdd5945a2326a58f` |
| `benchmark_evaluator/setup.cfg` | 4 | 0 | [Package/module assessment](packages/benchmark_evaluator_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `2e789e61556c42fffdfecd9d6c779b5ab4469f8081e00d71b3d9e369cde38ced` |
| `benchmark_evaluator/setup.py` | 31 | 0 | [Package/module assessment](packages/benchmark_evaluator_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `b3fa7e37fd318aaff228a6a483f5c75a6071800c1000349e54fb886f562b96e2` |
| `benchmark_evaluator/test/test_copyright.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ea71bbbc6247b5c14d176ed09f39ee274d563cc9b2ec355828ea451ceff31757` |
| `benchmark_evaluator/test/test_flake8.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `37a813a0507f0ce51a2d0b57312d2cc078a0307a3ca85580ea801b0d2f9755d2` |
| `benchmark_evaluator/test/test_pep257.py` | 23 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `e8b88d49ee296987b1b40069f81019b5fa09b1aa1bfee9a1faae45bb5cf53eda` |
| `bringup/bringup/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `bringup/config/holoocean_config copy.json` | 123 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `b3bfaf8d555824c9e6d601bd10d24016bb0f19ebe4d071b3f417ddaf37a35354` |
| `bringup/config/holoocean_config.json` | 123 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `75f527af94a5ff6e75bc3e748fd0b4e06d0e6d0c5b4e3026a4e34d3c4a26cba8` |
| `bringup/config/nbv_params.yaml` | 39 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `5b56cdce7c8d5708e2e1be9c1c19e156eae60555b0c3a8fb1116d038c65c10cf` |
| `bringup/config/params.yaml` | 80 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `22710a4c280eb6e10b6783d157ebd4134eb15426e3658577195c8950c9b0b9f4` |
| `bringup/launch/mapper.launch.py` | 82 | 0 | [V-01](packages/bringup_audit.md#v-01), [V-03](packages/bringup_audit.md#v-03) | `8cb9fd5e9fd90d926b14b97d8cb4c41dc96ac792f8ac60710f1f89ab0c23d6ac` |
| `bringup/package.xml` | 18 | 0 | [V-02](packages/bringup_audit.md#v-02) | `f8ecf67f6c4c402c4d79b95a204414611a5c45a18f811a216cbd6189b8739ec5` |
| `bringup/setup.cfg` | 4 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `79d609944ac6ef4241ff6ae0df631ef54c6b8fd385aaf370700a734d68d03335` |
| `bringup/setup.py` | 34 | 0 | [Package/module assessment](packages/bringup_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `a5ad8df77f272f7fb042a3c9a4ad2b90c482e8034f96d30e387faccba740f256` |
| `bringup/test/test_copyright.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ea71bbbc6247b5c14d176ed09f39ee274d563cc9b2ec355828ea451ceff31757` |
| `bringup/test/test_flake8.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `37a813a0507f0ce51a2d0b57312d2cc078a0307a3ca85580ea801b0d2f9755d2` |
| `bringup/test/test_pep257.py` | 23 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `e8b88d49ee296987b1b40069f81019b5fa09b1aa1bfee9a1faae45bb5cf53eda` |
| `motion_control/CMakeLists.txt` | 50 | 0 | [Package/module assessment](packages/motion_control_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `3fbbc721d78f51c1061627d90c71baeff9189f5cb417fade6dab4820e45af50d` |
| `motion_control/package.xml` | 18 | 0 | [V-06](packages/motion_control_audit.md#v-06) | `d38f0dd18fe69795adfdb56ed799c5ee6d9f57194d3d13a0ce198a67d8da0a66` |
| `motion_control/src/PathPlanning.py` | 201 | 1 | [V-05](packages/motion_control_audit.md#v-05) | `a59c2f287af8b2ad4706d1b165b5dd55695fee6f4f7d4cc07b23f12c8826a26b` |
| `motion_control/src/math_pid_utils.hpp` | 72 | 1 | [Package/module assessment](packages/motion_control_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `e15baa62b069085c6ade568416142b596cdc2bac965da914b8ff1e21d357475a` |
| `motion_control/src/point_iterator.cpp` | 204 | 1 | [V-03](packages/motion_control_audit.md#v-03), [V-04](packages/motion_control_audit.md#v-04) | `383e421ffa010573c6a0f87891f4d47803f3ed980b9a75e809725d3cac339e2b` |
| `motion_control/src/trajectory_smoother.cpp` | 10 | 0 | [Package/module assessment](packages/motion_control_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `1d7650edf02f44e5aee0c62e7b6a56233939ffb14e8ec439674a6fa4177e823f` |
| `motion_control/src/waypoint_control_node.cpp` | 432 | 1 | [V-01](packages/motion_control_audit.md#v-01), [V-02](packages/motion_control_audit.md#v-02) | `f3d01b6d4b6265ca9ea1e8dab9409a02f6aaf81e88fce9c971c2bc98300a6e65` |
| `perception/package.xml` | 21 | 0 | [V-07](packages/perception_audit.md#v-07) | `03ebdbf38baace8cd46cf0b0e51d791756d6ef9c656dfbdd2f919213d1c3886b` |
| `perception/perception/BGNet/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `perception/perception/BGNet/dataloader/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `perception/perception/BGNet/datasets/__init__.py` | 11 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `4e818e4ad3113091ce1280c1789820f7b83b46343b14f5924a449e55769e782f` |
| `perception/perception/BGNet/datasets/data_io.py` | 95 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `07ea7de53014b18d3c7101c73004708afcaa17ce82d769f94834f39bd90e53c2` |
| `perception/perception/BGNet/datasets/kitti12_dataset.py` | 113 | 1 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `496092579e622b8b6a833a955fd1387896a0e2e35263307cdaf03ff51e1eea52` |
| `perception/perception/BGNet/datasets/kitti_dataset.py` | 111 | 1 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `c1b5d984c9e31261a79abcde3f604faedcdf7614ca14730647dcf7bc80fd665a` |
| `perception/perception/BGNet/models/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `perception/perception/BGNet/models/bgnet.py` | 155 | 3 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `52a5218a355372424404780161a271f611a0cde507f160f98847dadb31d826ae` |
| `perception/perception/BGNet/models/bgnet_plus.py` | 161 | 3 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `a660158470368e5c5d1db2dbc2e99779bb723579e98d3fd46169cd7364830db7` |
| `perception/perception/BGNet/models/feature_extractor_fast.py` | 207 | 4 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `20fcfc3b29db83a86d35041fd2ec40438632eca3fe5554e7f55b776237ea4d4d` |
| `perception/perception/BGNet/models/submodules.py` | 105 | 2 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `ccb44e11affec23d5b6963e8b33d944371270557eced0230360f675aa7d9474b` |
| `perception/perception/BGNet/models/submodules2d.py` | 175 | 3 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `1e65a76b7c7c896e905e8368836ac5332e4cd414bb3872c4c550e741739dfbf7` |
| `perception/perception/BGNet/models/submodules3d.py` | 57 | 2 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `f275094ae7fb14879f9d932626177f23fec5d81e8d6ed5f2334bcdcc14b8d049` |
| `perception/perception/BGNet/nets/warp.py` | 37 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `abef0475439ea52d28fd7e6fb41dc5e5aef178a680ba1d2068355808df254401` |
| `perception/perception/BGNet/predict.py` | 70 | 0 | [V-06](packages/perception_audit.md#v-06) | `dc1c77cd1c5da2e63db9ad1cf2a820596372693160951fed1e3fa67244d833b5` |
| `perception/perception/BGNet/predict.sh` | 25 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `7c71691c23793562b4208d6752e288773c68619a1706586a3afc4f8bb2cafd71` |
| `perception/perception/BGNet/predict_sample.py` | 47 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `ff365c47f586c10a832a8191496e6d6a390a81f5bf09957619fddf824a5202a7` |
| `perception/perception/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `perception/perception/compute_depth.py` | 115 | 0 | [V-01](packages/perception_audit.md#v-01), [V-02](packages/perception_audit.md#v-02) | `1d1830a6adb3ec034e48d659cbc7959efb92ce544faacd2ef0b7b89d9ecece4f` |
| `perception/perception/depth_estimation.py` | 135 | 1 | [V-03](packages/perception_audit.md#v-03), [V-04](packages/perception_audit.md#v-04) | `6a75e97dec10b928bf390f8e3c8bb6b7d9e51f4d0c0bee4db586c850e7401667` |
| `perception/setup.cfg` | 4 | 0 | [Package/module assessment](packages/perception_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `d3f76fd34626be7130c677d75d42cb981e6091b7f466125a2836938d7302667e` |
| `perception/setup.py` | 30 | 0 | [V-05](packages/perception_audit.md#v-05) | `2d2607cf916cbf453d96eabf09ac9caff5673a2b91f9c0e78071c06a18a248ee` |
| `perception/test/test_copyright.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ea71bbbc6247b5c14d176ed09f39ee274d563cc9b2ec355828ea451ceff31757` |
| `perception/test/test_flake8.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `37a813a0507f0ce51a2d0b57312d2cc078a0307a3ca85580ea801b0d2f9755d2` |
| `perception/test/test_pep257.py` | 23 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `e8b88d49ee296987b1b40069f81019b5fa09b1aa1bfee9a1faae45bb5cf53eda` |
| `python_planner/package.xml` | 28 | 0 | [V-09](packages/python_planner_audit.md#v-09) | `acbb85b2531bce907fbcb4192c49fe317382b9f87cabfe7d57b54e8d88d8e0fb` |
| `python_planner/python_planner/Tools/get_point_cloud.py` | 471 | 1 | [V-04](packages/python_planner_audit.md#v-04) | `d704cfc6cbf7b27a283fc3d7539189cabe1d2ba76990c51454b52cd237b43cc1` |
| `python_planner/python_planner/Tools/mapping3D.py` | 112 | 0 | [V-06](packages/python_planner_audit.md#v-06) | `2235831b312c3ff5b65e4dc43d492bf8f81f2df8a30c234454331539be046abc` |
| `python_planner/python_planner/Tools/mapping_3d.py` | 170 | 1 | [V-05](packages/python_planner_audit.md#v-05) | `256badb86d0aec6abb613d1ff3c448d0d150db32f65c55e9e3c7b36a5b741b1e` |
| `python_planner/python_planner/Tools/search_algo.py` | 119 | 0 | [Package/module assessment](packages/python_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `3ebaa29326a520c41797c425a0329f89450135b0464ec0da233a521e2db54f1d` |
| `python_planner/python_planner/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `python_planner/python_planner/get_NBV.py` | 398 | 0 | [V-02](packages/python_planner_audit.md#v-02) | `77237686b5b18245f776b8fd541a3171be02a210d7b8366ba1da35da46d5b689` |
| `python_planner/python_planner/planner_node.py` | 958 | 1 | [V-01](packages/python_planner_audit.md#v-01), [V-03](packages/python_planner_audit.md#v-03), [V-08](packages/python_planner_audit.md#v-08) | `f1285132e43472fc896436cfc6ae318142a8d9b51fe14fa9c92b9b77106928ac` |
| `python_planner/python_planner/route_planning.py` | 265 | 0 | [Package/module assessment](packages/python_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `0d32c0ff048f760189a405af108559029286d9afeeacffd4f36aadb232ced257` |
| `python_planner/python_planner/trajectory_optimization.py` | 151 | 0 | [V-07](packages/python_planner_audit.md#v-07) | `f709d61c5be9b715848455e89fb6c2813148714f4aff4d08c886e6e8344dfcee` |
| `python_planner/setup.cfg` | 4 | 0 | [Package/module assessment](packages/python_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `bda8534b221d4e444815025d47271449275a694d1e71ed8148d3b0b7b84d780a` |
| `python_planner/setup.py` | 31 | 0 | [Package/module assessment](packages/python_planner_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `f63bfd35a586f24248fa46a97d270ffe276a762cfbb2c60e0652bcb8f74c911a` |
| `python_planner/test/test_copyright.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ea71bbbc6247b5c14d176ed09f39ee274d563cc9b2ec355828ea451ceff31757` |
| `python_planner/test/test_flake8.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `37a813a0507f0ce51a2d0b57312d2cc078a0307a3ca85580ea801b0d2f9755d2` |
| `python_planner/test/test_motion_continuity.py` | 146 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ee3acfc66f1fea817a027acc94b7a77bff33df34af1261c1872560cf7e051a45` |
| `python_planner/test/test_nbv_config.py` | 21 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `d0b8cc5c589596d95d17192d1ae0f7b779a4b0864cf0f7d260b1956503ce83b4` |
| `python_planner/test/test_nbv_filter_logging.py` | 413 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `4a70cc05fbceba6b5454ec212ffe93fcd10c9d7dfbea76c8e810b3d0f4f41487` |
| `python_planner/test/test_nbv_math.py` | 103 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `b6cff696fa1e0df41d21e90754dfd722023bfc6aa7f31e2c6866368106be4bfc` |
| `python_planner/test/test_pep257.py` | 23 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `e8b88d49ee296987b1b40069f81019b5fa09b1aa1bfee9a1faae45bb5cf53eda` |
| `python_planner/test/test_route_search_limits.py` | 53 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `c3c5e7fea4a4923532cb339a8702c00cbd8db670d11f1572a7177d60c888f287` |
| `sim_bridge/package.xml` | 18 | 0 | [V-04](packages/sim_bridge_audit.md#v-04) | `475f4d36102729258f16f481bee1ef9572161e73a4370c9082927f2a1b64e3a4` |
| `sim_bridge/setup.cfg` | 4 | 0 | [Package/module assessment](packages/sim_bridge_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `bdd509eb88c568a04168c1efff39aef05a050452294aae96244749564bb70819` |
| `sim_bridge/setup.py` | 30 | 0 | [Package/module assessment](packages/sim_bridge_audit.md#module-by-module-target-placement); no independent finding assigned to this file. | `d8b206d353b9227e95d332a2aebe246ad1da4ccd3747b4f6d67a7971de109e88` |
| `sim_bridge/sim_bridge/__init__.py` | 0 | 0 | Empty package marker; no independent finding. | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `sim_bridge/sim_bridge/holoocean_node.py` | 582 | 1 | [V-01](packages/sim_bridge_audit.md#v-01), [V-02](packages/sim_bridge_audit.md#v-02), [V-03](packages/sim_bridge_audit.md#v-03), [V-05](packages/sim_bridge_audit.md#v-05) | `3cb203ee333674f8e6bb26a0f1eea28933185ab1848bf72dfec32dca33247ac8` |
| `sim_bridge/test/test_copyright.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `ea71bbbc6247b5c14d176ed09f39ee274d563cc9b2ec355828ea451ceff31757` |
| `sim_bridge/test/test_flake8.py` | 25 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `37a813a0507f0ce51a2d0b57312d2cc078a0307a3ca85580ea801b0d2f9755d2` |
| `sim_bridge/test/test_pep257.py` | 23 | 0 | Test-only: assertions/dependencies inspected; not executed; excluded from metrics. | `e8b88d49ee296987b1b40069f81019b5fa09b1aa1bfee9a1faae45bb5cf53eda` |

## Declared Build and Release Dependencies

All declarations are retained here, including test/build-only dependencies. They do not all contribute to the production seven-package graph. Setup/build declarations alone do not account for undeclared executable imports or named launch resources.

| Component | Manifest dependency kind and target | Build/setup evidence |
|---|---|---|
| active_inspection_planner | buildtool_depend: ament_cmake; depend: pcl_conversions; depend: tf2; depend: octomap; depend: cv_bridge; test_depend: ament_lint_auto; test_depend: ament_lint_common | `13: find_package(ament_cmake REQUIRED)`; `14: find_package(Eigen3 REQUIRED)`; `15: find_package(PCL REQUIRED)`; `16: find_package(octomap REQUIRED)`; `17: find_package(cv_bridge REQUIRED)`; `18: find_package(tf2 REQUIRED)`; `19: find_package(nav_msgs REQUIRED)`; `21: add_executable(nbv_utility_node`; `34: target_link_libraries(nbv_utility_node`; `46: find_package(ament_lint_auto REQUIRED)` |
| benchmark_evaluator | exec_depend: python3-matplotlib; test_depend: ament_copyright; test_depend: ament_flake8; test_depend: ament_pep257; test_depend: python3-pytest | `3: package_name = 'benchmark_evaluator'`; `8: packages=find_packages(exclude=['test']),`; `14: install_requires=['setuptools', 'matplotlib'],`; `27: 'run_baselines = benchmark_evaluator.run_baselines:main',`; `28: 'plot_benchmark_results = benchmark_evaluator.plot_results:main',` |
| bringup | test_depend: ament_copyright; test_depend: ament_flake8; test_depend: ament_pep257; test_depend: python3-pytest | `5: package_name = 'bringup'`; `10: packages=find_packages(exclude=['test']),`; `19: install_requires=['setuptools'],` |
| motion_control | buildtool_depend: ament_cmake; test_depend: ament_lint_auto; test_depend: ament_lint_common | `9: find_package(ament_cmake REQUIRED)`; `10: find_package(nlohmann_json REQUIRED)`; `11: find_package(rclcpp REQUIRED)`; `12: find_package(sensor_msgs REQUIRED)`; `13: find_package(nav_msgs REQUIRED)`; `14: find_package(std_msgs REQUIRED)`; `19: add_executable(waypoint_control_node src/waypoint_control_node.cpp)`; `26: target_link_libraries(waypoint_control_node nlohmann_json::nlohmann_json)`; `32: add_executable(point_iterator src/point_iterator.cpp)`; `44: find_package(ament_lint_auto REQUIRED)` |
| perception | exec_depend: benchmark_evaluator; exec_depend: message_filters; test_depend: ament_copyright; test_depend: ament_flake8; test_depend: ament_pep257; test_depend: python3-pytest | `3: package_name = 'perception'`; `8: packages=find_packages(exclude=['test']),`; `14: install_requires=['setuptools'],`; `27: 'depth_estimation = perception.depth_estimation:main'` |
| python_planner | test_depend: ament_copyright; test_depend: ament_flake8; test_depend: ament_pep257; test_depend: python3-pytest; exec_depend: rclpy; exec_depend: benchmark_evaluator; exec_depend: cv_bridge; exec_depend: nav_msgs; exec_depend: sensor_msgs; exec_depend: std_msgs; exec_depend: python3-numpy; exec_depend: python3-scipy; exec_depend: python3-open3d | `3: package_name = 'python_planner'`; `8: packages=find_packages(exclude=['test']),`; `14: install_requires=[`; `28: 'planner_node = python_planner.planner_node:main'` |
| sim_bridge | test_depend: ament_copyright; test_depend: ament_flake8; test_depend: ament_pep257; test_depend: python3-pytest | `3: package_name = 'sim_bridge'`; `8: packages=find_packages(exclude=['test']),`; `14: install_requires=['setuptools'],`; `27: 'holoocean_node = sim_bridge.holoocean_node:main'` |

## Complete Source Import/Include and Function Inventory

Imports include standard-library, internal, external, conditional and unused imports; an import is not automatically a violation. Top-level `models`, `datasets`, `nets` in BGNet are intended internal vendored references reached through sys.path; `Tools` and `routeCalculation` have ambiguous checkout-dependent resolution. They are not invented additional scoped ROS packages. C++ member definitions may be inline; package diagrams show the principal interaction paths, and include lists expose compilation coupling. Dynamic dispatch and macros require compiler/runtime evidence for exact resolution.

### `active_inspection_planner/src/nbv_utility_node.cpp`

Dependencies: `#include <rclcpp/rclcpp.hpp>` (line 1); `#include <sensor_msgs/msg/image.hpp>` (line 2); `#include <sensor_msgs/msg/magnetic_field.hpp>` (line 3); `#include <nav_msgs/msg/odometry.hpp>` (line 4); `#include <std_msgs/msg/float32_multi_array.hpp>` (line 5); `#include <std_msgs/msg/bool.hpp>` (line 6); `#include <cv_bridge/cv_bridge.hpp>` (line 7); `#include <tf2/LinearMath/Quaternion.h>` (line 8); `#include <tf2/LinearMath/Matrix3x3.h>` (line 9); `#include "realtime_stereo_mapper.hpp"` (line 11); `#include <fstream>` (line 12); `#include <chrono>` (line 13).

Types: `NBVUtilityNode` : no explicit Python base / C++ definition (line 15).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 34: `rclcpp::QoS qos(10);`.
- Line 40: `std::bind(&NBVUtilityNode::depth_callback, this, std::placeholders::_1));`.
- Line 44: `std::bind(&NBVUtilityNode::rgb_callback, this, std::placeholders::_1));`.
- Line 48: `std::bind(&NBVUtilityNode::odom_callback, this, std::placeholders::_1));`.
- Line 52: `std::bind(&NBVUtilityNode::mag_callback, this, std::placeholders::_1));`.
- Line 59: `std::chrono::milliseconds(500),`.
- Line 60: `std::bind(&NBVUtilityNode::timer_callback, this));`.
- Line 89: `void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {`.
- Line 92: `std::abs(msg->pose.pose.position.y) < 0.01 ) {`.
- Line 108: `void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {`.
- Line 114: `void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {`.
- Line 118: `void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {`.
- Line 128: `void timer_callback() {`.
- Line 140: `static std::ofstream eval_file("evaluation_parameters.txt", std::ios::app);`.
- Line 200: `int main(int argc, char** argv) {`.
- Line 201: `rclcpp::init(argc, argv);`.
- Line 202: `rclcpp::spin(std::make_shared<NBVUtilityNode>());`.
- Line 203: `rclcpp::shutdown();`.

### `active_inspection_planner/src/realtime_stereo_mapper.cpp`

Dependencies: `#include "realtime_stereo_mapper.hpp"` (line 1); `#include <iostream>` (line 2); `#include <pcl/point_cloud.h>` (line 3); `#include <pcl/kdtree/kdtree_flann.h>` (line 4); `#include <algorithm>` (line 5); `#include <numeric>` (line 6); `#include "search_algo.hpp"` (line 7).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 9: `RealTimeStereoMapper::RealTimeStereoMapper(const MapperConfig& config)`.
- Line 20: `size_t RealTimeStereoMapper::get_current_leaves() const {`.
- Line 25: `void RealTimeStereoMapper::generate_global_point_cloud(`.
- Line 54: `Eigen::Vector3d local_pt(local_x, local_y, local_z);`.
- Line 68: `void RealTimeStereoMapper::process_frame(`.
- Line 73: `Eigen::Vector3d offset(0.35, 0.1125 + 0.275, -0.5);`.
- Line 112: `void RealTimeStereoMapper::save_map(const std::string& filename) {`.
- Line 119: `RealTimeStereoMapper::generate_candidate_viewpoints(const Eigen::Vector3d& current_position)`.
- Line 135: `Eigen::Vector3d pt(it.getX(), it.getY(), it.getZ());`.
- Line 184: `std::vector<int> pointIdx(11);`.
- Line 185: `std::vector<float> pointDistSq(11);`.
- Line 188: `std::vector<double> rhos(occ_cloud->size());`.
- Line 325: `Eigen::Vector3d dir_xy(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);`.
- Line 439: `RealTimeStereoMapper::plan_multi_waypoint_route(`.
- Line 493: `Eigen::Vector4d catmull_rom(const Eigen::Vector4d& p0, const Eigen::Vector4d& p1,`.
- Line 505: `double unwrap_angle(double previous_angle, double new_angle) {`.
- Line 511: `std::vector<Eigen::Vector4d> RealTimeStereoMapper::generate_seamless_mission_trajectory(`.

### `active_inspection_planner/src/realtime_stereo_mapper.hpp`

Dependencies: `#include <octomap/octomap.h>` (line 4); `#include <octomap/ColorOcTree.h>` (line 5); `#include <opencv2/opencv.hpp>` (line 6); `#include <Eigen/Dense>` (line 7); `#include <vector>` (line 8); `#include <string>` (line 9); `#include "search_algo.hpp"` (line 11).

Types: `MapperConfig` : no explicit Python base / C++ definition (line 13); `RealTimeStereoMapper` : no explicit Python base / C++ definition (line 26).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 30: `void process_frame(const Eigen::Vector3d& location,`.
- Line 34: `void save_map(const std::string& filename);`.
- Line 41: `size_t get_current_leaves() const;`.
- Line 53: `void generate_global_point_cloud(`.
- Line 65: `std::vector<Eigen::Vector4d> generate_seamless_mission_trajectory(`.

### `active_inspection_planner/src/search_algo.cpp`

Dependencies: `#include "search_algo.hpp"` (line 1); `#include <iostream>` (line 2).

Types: `AStarNode` : no explicit Python base / C++ definition (line 5).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 12: `inline double heuristic(const VoxelIdx &a, const VoxelIdx &b)`.
- Line 18: `bool find_nearest_free_voxel(`.
- Line 66: `std::vector<Eigen::Vector3d> SearchAlgo::astar_3d(`.
- Line 131: `std::reverse(path.begin(), path.end());`.
- Line 157: `bool SearchAlgo::check_line_of_sight(`.

### `active_inspection_planner/src/search_algo.hpp`

Dependencies: `#include <vector>` (line 4); `#include <unordered_set>` (line 5); `#include <tuple>` (line 6); `#include <cmath>` (line 7); `#include <queue>` (line 8); `#include <Eigen/Dense>` (line 9).

Types: `VoxelIdx` : no explicit Python base / C++ definition (line 12); `VoxelHash` : no explicit Python base / C++ definition (line 17); `SearchAlgo` : no explicit Python base / C++ definition (line 25).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 27: `static std::vector<Eigen::Vector3d> astar_3d(`.
- Line 34: `static bool check_line_of_sight(`.

### `benchmark_evaluator/benchmark_evaluator/benchmark_evaluator.py`

Dependencies: `import numpy as np` (line 1); `import open3d as o3d` (line 2); `import pyoctomap` (line 3); `import matplotlib.pyplot as plt` (line 4); `import copy` (line 5).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `evaluate_map_accuracy` | 7 | `cmap`; `ground_truth_file.split`; `gt_mesh.sample_points_uniformly`; `gt_mesh.scale`; `gt_pcd.compute_point_cloud_distance`; `gt_pcd.get_center`; `gt_pcd.paint_uniform_color`; `gt_pcd.translate`; `len`; `map_pcd.compute_point_cloud_distance`; `map_pcd.remove_statistical_outlier`; `map_pcd.transform`; `np.asarray`; `np.clip`; `np.eye`; `np.mean`; `np.sqrt`; `np.sum`; `o3d.geometry.PointCloud`; `o3d.io.read_triangle_mesh`; `o3d.pipelines.registration.ICPConvergenceCriteria`; `o3d.pipelines.registration.TransformationEstimationPointToPoint`; `o3d.pipelines.registration.registration_icp`; `o3d.utility.Vector3dVector`; `o3d.visualization.draw_geometries`; `octomap_file.split`; `plt.get_cmap`; `print`; `pyoctomap.ColorOcTree`; `tree.extractPointCloud`; `tree.read` |

### `benchmark_evaluator/benchmark_evaluator/midrun_logger.py`

Dependencies: `import csv` (line 3); `import json` (line 4); `import math` (line 5); `from datetime import datetime` (line 6); `from pathlib import Path` (line 7); `import matplotlib.pyplot as plt` (line 154); `from matplotlib.patches import Rectangle, Wedge` (line 155).

Types: `MidrunLogger` : no explicit Python base / C++ definition (line 10).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `generate_results_plot` | 150 | `Path`; `Path(results_dir).expanduser`; `Rectangle`; `Wedge`; `axis.add_patch`; `axis.annotate`; `axis.get_legend_handles_labels`; `axis.grid`; `axis.legend`; `axis.plot`; `axis.scatter`; `axis.set_aspect`; `axis.set_title`; `axis.set_xlabel`; `axis.set_ylabel`; `by_iteration.items`; `by_iteration.setdefault`; `by_iteration.setdefault(row['iteration'], []).append`; `candidate.get`; `candidates.extend`; `csv.DictReader`; `depth_path.exists`; `depth_path.open`; `dict`; `enumerate`; `figure.colorbar`; `figure.savefig`; `float`; `future_path.exists`; `future_path.open`; `iteration_path.exists`; `iteration_path.open`; `json.loads`; `line.strip`; `list`; `math.degrees`; `plt.close`; `plt.subplots`; `str`; `trajectory.append`; `trajectory_path.exists`; `trajectory_path.open`; `unique.keys`; `unique.values`; `zip` |
| `default_run_id` | 269 | `datetime.now`; `datetime.now().strftime` |
| `__init__` | 13 | `Path`; `Path(results_dir).expanduser`; `self._reset_files`; `self.results_dir.mkdir` |
| `_reset_files` | 28 | `csv.writer`; `csv.writer(stream).writerow`; `self.depth_pose_path.open`; `self.filter_path.write_text`; `self.future_path.open`; `self.iteration_path.write_text`; `self.pipeline_path.open`; `self.trajectory_path.open` |
| `record_filter` | 54 | `dict`; `int`; `json.dumps`; `self._json_safe`; `self.filter_path.open`; `stream.write` |
| `_json_safe` | 64 | `MidrunLogger._json_safe`; `hasattr`; `isinstance`; `math.isfinite`; `str`; `value.items`; `value.tolist` |
| `_vector` | 77 | `float` |
| `record_trajectory` | 80 | `csv.writer`; `csv.writer(stream).writerow`; `self._vector`; `self.trajectory_path.open` |
| `record_stage` | 86 | `csv.writer`; `csv.writer(stream).writerow`; `float`; `self.pipeline_path.open` |
| `record_depth_pose` | 98 | `csv.writer`; `csv.writer(stream).writerow`; `self._vector`; `self.depth_pose_path.open` |
| `record_iteration` | 105 | `any`; `bool`; `candidate.get`; `csv.writer`; `enumerate`; `float`; `int`; `json.dumps`; `len`; `list`; `record['candidates'].append`; `self._vector`; `self.future_path.open`; `self.iteration_path.open`; `stream.write`; `sum`; `writer.writerow`; `zip` |

### `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py`

Dependencies: `import optuna` (line 1); `import subprocess` (line 2); `import time` (line 3); `import signal` (line 4); `import os` (line 5); `from pathlib import Path` (line 6); `import yaml` (line 7); `from benchmark_evaluator import evaluate_map_accuracy` (line 9); `import pandas as pd` (line 145).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `write_params_yaml` | 18 | `BASE_PARAMS_PATH.open`; `TEMP_PARAMS_PATH.open`; `float`; `params_dict.items`; `str`; `yaml.safe_dump`; `yaml.safe_load` |
| `objective` | 38 | `MISSION_STATS_PATH.exists`; `MISSION_STATS_PATH.open`; `evaluate_map_accuracy`; `float`; `line.split`; `line.split(':')[1].replace`; `line.split(':')[1].replace('meters', '').strip`; `os.getpgid`; `os.killpg`; `output_path.unlink`; `print`; `process.wait`; `subprocess.Popen`; `time.sleep`; `time.time`; `trial.suggest_float`; `write_params_yaml` |

### `benchmark_evaluator/benchmark_evaluator/plot_results.py`

Dependencies: `import argparse` (line 1); `from .midrun_logger import generate_results_plot` (line 3).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `main` | 6 | `SystemExit`; `argparse.ArgumentParser`; `generate_results_plot`; `parser.add_argument`; `parser.parse_args`; `print` |

### `benchmark_evaluator/benchmark_evaluator/run_baselines.py`

Dependencies: `import rclpy` (line 1); `from rclpy.node import Node` (line 2); `from nav_msgs.msg import Odometry` (line 3); `from std_msgs.msg import Bool, Float32MultiArray` (line 4); `import math` (line 5); `import csv` (line 6); `import time` (line 7); `from datetime import datetime` (line 8); `from .midrun_logger import MidrunLogger, generate_results_plot` (line 10); `import matplotlib.pyplot as plt` (line 210).

Types: `BenchmarkLogger` : Node (line 12).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `main` | 363 | `BenchmarkLogger`; `node.destroy_node`; `node.get_logger`; `node.get_logger().info`; `rclpy.init`; `rclpy.shutdown`; `rclpy.spin` |
| `__init__` | 13 | `MidrunLogger`; `self.create_subscription`; `self.create_timer`; `self.declare_parameter`; `self.get_logger`; `self.get_logger().info`; `super`; `super().__init__` |
| `_wrap_angle` | 57 | `math.atan2`; `math.cos`; `math.sin` |
| `planned_trajectory_callback` | 60 | `float`; `len`; `range`; `tuple` |
| `_nearest_planned_point` | 69 | `min` |
| `odom_callback` | 79 | `abs`; `math.atan2`; `self.get_logger`; `self.get_logger().info`; `time.time` |
| `log_trajectory_callback` | 99 | `len`; `math.degrees`; `math.hypot`; `math.sqrt`; `max`; `self._nearest_planned_point`; `self._wrap_angle`; `self.diagnostic_log.append`; `self.telemetry.record_trajectory`; `self.trajectory_log.append`; `time.time` |
| `finish_callback` | 153 | `self.get_logger`; `self.get_logger().info`; `self.save_benchmark_data`; `time.time` |
| `_trajectory_rmse` | 165 | `errors.append`; `len`; `math.sqrt`; `sum` |
| `_save_diagnostics` | 178 | `axis.axhline`; `axis.grid`; `axis.legend`; `axis.plot`; `axis.set_aspect`; `axis.set_title`; `axis.set_xlabel`; `axis.set_ylabel`; `csv.DictWriter`; `diagnostics_path.open`; `figure.savefig`; `plt.close`; `plt.subplots`; `self._diagnostic_metrics`; `self.get_logger`; `self.get_logger().warning`; `writer.writeheader`; `writer.writerow` |
| `_diagnostic_metrics` | 283 | `abs`; `len`; `max`; `self._trajectory_rmse`; `sum` |
| `save_benchmark_data` | 299 | `csv.writer`; `datetime.now`; `datetime.now().strftime`; `f.write`; `generate_results_plot`; `len`; `open`; `self._save_diagnostics`; `self.get_logger`; `self.get_logger().info`; `writer.writerow` |
| `destroy_node` | 352 | `self.get_logger`; `self.get_logger().info`; `self.save_benchmark_data`; `super`; `super().destroy_node`; `time.time` |

### `benchmark_evaluator/setup.py`

Dependencies: `from setuptools import find_packages, setup` (line 1).


### `benchmark_evaluator/test/test_copyright.py`

Dependencies: `from ament_copyright.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_copyright` | 23 | `main`; `pytest.mark.skip` |

### `benchmark_evaluator/test/test_flake8.py`

Dependencies: `from ament_flake8.main import main_with_errors` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_flake8` | 21 | `'\n'.join`; `len`; `main_with_errors` |

### `benchmark_evaluator/test/test_pep257.py`

Dependencies: `from ament_pep257.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_pep257` | 21 | `main` |

### `bringup/launch/mapper.launch.py`

Dependencies: `import os` (line 1); `from ament_index_python.packages import get_package_share_directory` (line 2); `from launch import LaunchDescription` (line 3); `from launch.actions import DeclareLaunchArgument` (line 4); `from launch.substitutions import LaunchConfiguration` (line 5); `from launch_ros.actions import Node` (line 6).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `generate_launch_description` | 8 | `DeclareLaunchArgument`; `LaunchConfiguration`; `LaunchDescription`; `Node`; `get_package_share_directory`; `os.path.join` |

### `bringup/setup.py`

Dependencies: `import os` (line 1); `from setuptools import find_packages, setup` (line 3).


### `bringup/test/test_copyright.py`

Dependencies: `from ament_copyright.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_copyright` | 23 | `main`; `pytest.mark.skip` |

### `bringup/test/test_flake8.py`

Dependencies: `from ament_flake8.main import main_with_errors` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_flake8` | 21 | `'\n'.join`; `len`; `main_with_errors` |

### `bringup/test/test_pep257.py`

Dependencies: `from ament_pep257.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_pep257` | 21 | `main` |

### `motion_control/src/PathPlanning.py`

Dependencies: `import sys` (line 8); `from Tools.explore2D import addWaypoint, isItScanned` (line 13); `from Tools.mapping2D import diff` (line 14); `from routeCalculation import calculate_route` (line 15); `from std_msgs.msg import Float32, Header, Float32MultiArray` (line 16); `import rclpy` (line 17); `from rclpy.node import Node` (line 18); `from sensor_msgs.msg import Image, Imu, MagneticField, FluidPressure` (line 20); `from geometry_msgs.msg import TwistStamped, Point32` (line 21); `from std_msgs.msg import Bool` (line 22); `from nav_msgs.msg import Odometry` (line 23); `import numpy as np` (line 24); `import cv2` (line 25); `from cv_bridge import CvBridge` (line 26).

Types: `PathPlanner` : Node (line 61).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `boustrophedon_path_planning` | 29 | `enumerate`; `int`; `np.arange`; `np.linspace`; `waypoints.append` |
| `main` | 192 | `PathPlanner`; `node.destroy_node`; `rclpy.init`; `rclpy.shutdown`; `rclpy.spin` |
| `__init__` | 62 | `CvBridge`; `self.create_publisher`; `self.create_subscription`; `self.create_timer`; `self.get_logger`; `self.get_logger().info`; `super`; `super().__init__` |
| `areaSearch` | 86 | `addWaypoint`; `boustrophedon_path_planning`; `isItScanned`; `print`; `self.waypoints.append` |
| `point_callback` | 104 | `point.tolist`; `self.get_logger`; `self.get_logger().info` |
| `pointSegment_callback` | 109 | `len`; `np.array`; `np.array(msg.data).reshape`; `print`; `self.get_logger`; `self.get_logger().info`; `wps.append` |
| `timer_callback` | 137 | `Float32MultiArray`; `flat_waypoints.extend`; `len`; `self.get_logger`; `self.get_logger().info`; `self.publisher.publish` |
| `stop_callback` | 155 | `Float32MultiArray`; `addWaypoint`; `diff`; `msg.data[:3].tolist`; `print`; `self.get_logger`; `self.get_logger().info`; `self.timer_callback`; `self.waypoints.pop`; `self.waypoints.remove`; `tuple` |

### `motion_control/src/math_pid_utils.hpp`

Dependencies: `#include <cmath>` (line 3); `#include <vector>` (line 4); `#include <array>` (line 5).

Types: `PID` : no explicit Python base / C++ definition (line 45).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 10: `inline double normalize_angle_deg(double angle) {`.
- Line 14: `inline double quaternion_to_roll_deg(double x, double y, double z, double w) {`.
- Line 20: `inline double quaternion_to_pitch_rad(double x, double y, double z, double w) {`.
- Line 28: `inline double diff_2d(const std::vector<double>& a, const std::vector<double>& b) {`.
- Line 32: `inline double diff_3d(const std::vector<double>& a, const std::vector<double>& b) {`.
- Line 36: `inline std::vector<double> world_to_auv_coordinates(const std::vector<double>& world_pos, double yaw) {`.
- Line 37: `std::vector<double> auv_pos(3, 0.0);`.
- Line 49: `integral_(0.0), prev_error_(0.0) {}`.
- Line 51: `double update(double measurement) {`.

### `motion_control/src/point_iterator.cpp`

Dependencies: `#include <memory>` (line 1); `#include <vector>` (line 2); `#include <cmath>` (line 3); `#include <algorithm>` (line 4); `#include "rclcpp/rclcpp.hpp"` (line 6); `#include "std_msgs/msg/float32_multi_array.hpp"` (line 7); `#include "std_msgs/msg/bool.hpp"` (line 8).

Types: `PathPlanner` : no explicit Python base / C++ definition (line 10).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 60: `std::bind(&PathPlanner::timer_callback, this));`.
- Line 87: `void point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)`.
- Line 90: `std::vector<double> new_wp(msg->data.begin(), msg->data.end());`.
- Line 96: `void pointSegment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)`.
- Line 120: `void timer_callback()`.
- Line 150: `void stop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)`.
- Line 197: `int main(int argc, char * argv[])`.
- Line 199: `rclcpp::init(argc, argv);`.
- Line 201: `rclcpp::spin(node);`.
- Line 202: `rclcpp::shutdown();`.

### `motion_control/src/trajectory_smoother.cpp`

Dependencies: `#include <cstdio>` (line 1).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 3: `int main(int argc, char ** argv)`.

### `motion_control/src/waypoint_control_node.cpp`

Dependencies: `#include <algorithm>` (line 1); `#include <chrono>` (line 2); `#include <geometry_msgs/msg/point_stamped.hpp>` (line 3); `#include <nav_msgs/msg/odometry.hpp>` (line 4); `#include <optional>` (line 5); `#include <rclcpp/rclcpp.hpp>` (line 6); `#include <sensor_msgs/msg/magnetic_field.hpp>` (line 7); `#include <std_msgs/msg/bool.hpp>` (line 8); `#include <std_msgs/msg/float32.hpp>` (line 9); `#include <std_msgs/msg/float32_multi_array.hpp>` (line 10); `#include <std_msgs/msg/string.hpp>` (line 11); `#include "math_pid_utils.hpp"` (line 13).

Types: `WaypointControlNode` : no explicit Python base / C++ definition (line 17).

C++ declaration/definition candidates (textual, not a resolved call graph):

- Line 82: `std::bind(&WaypointControlNode::finished_execution_callback, this, _1));`.
- Line 146: `double get_time_now() { return this->get_clock()->now().seconds(); }`.
- Line 148: `double smooth_desired_yaw(double target_yaw)`.
- Line 162: `std::sin(target_yaw - smoothed_desired_yaw_),`.
- Line 163: `std::cos(target_yaw - smoothed_desired_yaw_));`.
- Line 167: `std::sin(smoothed_desired_yaw_), std::cos(smoothed_desired_yaw_));`.
- Line 171: `void publish_cmd(std::vector<double> motor_command)`.
- Line 182: `void finished_execution_callback(const std_msgs::msg::Bool::SharedPtr msg)`.
- Line 186: `void mode_callback(const std_msgs::msg::String::SharedPtr msg) { mode_ = msg->data; }`.
- Line 187: `void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)`.
- Line 192: `void dvl_reset_callback(const std_msgs::msg::Bool::SharedPtr msg)`.
- Line 201: `void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)`.
- Line 212: `void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)`.
- Line 221: `void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_depth_ = msg->data; }`.
- Line 222: `void dist_callback(const std_msgs::msg::Float32::SharedPtr msg)`.
- Line 227: `void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)`.
- Line 232: `void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {`.
- Line 244: `void move_towards_waypoint(std::vector<double> target)`.
- Line 253: `std::vector<double> direction(3);`.
- Line 263: `std::vector<float> f_target(target.begin(), target.end());`.
- Line 283: `std::vector<double> cmd(8, 0.0);`.
- Line 312: `std::abs(direction[2]) * kp_dist_, 0.0, max_forward_thrust_);`.
- Line 343: `std::vector<char> determine_keys(const std::vector<double> & dir)`.
- Line 369: `std::vector<double> parse_keys(`.
- Line 414: `std::vector<double> apply_yaw_control(double yaw_error, double val, std::vector<double> cmd)`.
- Line 426: `int main(int argc, char ** argv)`.
- Line 428: `rclcpp::init(argc, argv);`.
- Line 429: `rclcpp::spin(std::make_shared<WaypointControlNode>());`.
- Line 430: `rclcpp::shutdown();`.

### `perception/perception/BGNet/datasets/__init__.py`

Dependencies: `from .kitti_dataset import KITTIDataset` (line 6); `from .kitti12_dataset import KITTI_12_Dataset` (line 7).


### `perception/perception/BGNet/datasets/data_io.py`

Dependencies: `import torch` (line 6); `import numpy as np` (line 7); `import re` (line 8); `import torchvision.transforms as transforms` (line 9).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `mean_std_transform` | 11 | `np.ascontiguousarray`; `np.shape`; `np.zeros`; `torch.from_numpy` |
| `get_transform` | 42 | `transforms.Compose`; `transforms.ToTensor` |
| `read_all_lines` | 54 | `f.readlines`; `line.rstrip`; `open` |
| `pfm_imread` | 61 | `Exception`; `dim_match.groups`; `file.readline`; `file.readline().decode`; `file.readline().decode('utf-8').rstrip`; `file.readline().rstrip`; `float`; `map`; `np.flipud`; `np.fromfile`; `np.reshape`; `open`; `re.match` |

### `perception/perception/BGNet/datasets/kitti12_dataset.py`

Dependencies: `import os` (line 6); `import random` (line 7); `from torch.utils.data import Dataset` (line 8); `from PIL import Image` (line 9); `import numpy as np` (line 10); `from datasets.data_io import get_transform, read_all_lines` (line 11); `import cv2` (line 12); `import torchvision` (line 13); `from PIL import ImageFilter` (line 14).

Types: `KITTI_12_Dataset` : Dataset (line 15).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `__init__` | 16 | `self.load_path` |
| `load_path` | 22 | `len`; `line.split`; `read_all_lines` |
| `load_image` | 36 | `Image.open`; `Image.open(filename).convert` |
| `load_disp` | 39 | `Image.open`; `np.array` |
| `__len__` | 44 | `len` |
| `__getitem__` | 47 | `get_transform`; `left_img.convert`; `left_img.crop`; `np.asarray`; `np.ascontiguousarray`; `np.expand_dims`; `np.lib.pad`; `os.path.join`; `preprocess`; `random.randint`; `right_img.convert`; `right_img.crop`; `self.load_disp`; `self.load_image` |

### `perception/perception/BGNet/datasets/kitti_dataset.py`

Dependencies: `import os` (line 6); `import random` (line 7); `from torch.utils.data import Dataset` (line 8); `from PIL import Image` (line 9); `import numpy as np` (line 10); `from datasets.data_io import get_transform, read_all_lines` (line 11); `import cv2` (line 12); `import torchvision` (line 13); `from PIL import ImageFilter` (line 14).

Types: `KITTIDataset` : Dataset (line 15).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `__init__` | 16 | `self.load_path` |
| `load_path` | 22 | `len`; `line.split`; `read_all_lines` |
| `load_image` | 36 | `Image.open`; `Image.open(filename).convert` |
| `load_disp` | 39 | `Image.open`; `np.array` |
| `__len__` | 44 | `len` |
| `__getitem__` | 47 | `get_transform`; `left_img.convert`; `left_img.crop`; `np.asarray`; `np.ascontiguousarray`; `np.expand_dims`; `np.lib.pad`; `os.path.join`; `preprocess`; `random.randint`; `right_img.convert`; `right_img.crop`; `self.load_disp`; `self.load_image` |

### `perception/perception/BGNet/models/bgnet.py`

Dependencies: `from __future__ import print_function` (line 6); `from models.feature_extractor_fast import feature_extraction` (line 7); `from models.submodules3d import CoeffsPredictor` (line 8); `from models.submodules2d import HourglassRefinement` (line 9); `from models.submodules import SubModule, convbn_2d_lrelu, convbn_3d_lrelu, convbn_2d_Tanh` (line 10); `from nets.warp import disp_warp` (line 11); `import torch` (line 12); `import torch.nn as nn` (line 13); `import torch.nn.functional as F` (line 14); `import time` (line 15).

Types: `Slice` : SubModule (line 16); `GuideNN` : SubModule (line 26); `BGNet` : SubModule (line 67).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `groupwise_correlation` | 36 | `(fea1 * fea2).view`; `(fea1 * fea2).view([B, num_groups, channels_per_group, H, W]).mean` |
| `build_gwc_volume` | 45 | `groupwise_correlation`; `range`; `refimg_fea.new_zeros`; `volume.contiguous` |
| `correlation` | 56 | `(fea1 * fea2).mean` |
| `disparity_regression` | 62 | `disp_values.view`; `len`; `torch.arange`; `torch.sum` |
| `__init__` | 17 | `super`; `super(Slice, self).__init__` |
| `forward` | 19 | `F.grid_sample`; `coeff.squeeze`; `guidemap.permute`; `guidemap.permute(0, 2, 3, 1).contiguous`; `torch.cat`; `torch.cat([wg, hg, guidemap], dim=3).unsqueeze` |
| `__init__` | 27 | `convbn_2d_Tanh`; `convbn_2d_lrelu`; `super`; `super(GuideNN, self).__init__` |
| `forward` | 33 | `self.conv1`; `self.conv2` |
| `__init__` | 68 | `CoeffsPredictor`; `GuideNN`; `Slice`; `convbn_3d_lrelu`; `feature_extraction`; `nn.Sequential`; `nn.Softmax`; `self.weight_init`; `super`; `super(BGNet, self).__init__` |
| `forward` | 88 | `F.interpolate`; `F.interpolate(half_disp * 2.0, scale_factor=(2.0, 2.0), mode='bilinear', align_corners=False).squeeze`; `build_gwc_volume`; `disparity_samples.repeat`; `hg.float`; `hg.float().repeat`; `hg.float().repeat(N, 1, 1).unsqueeze`; `hg.to`; `list_coeffs[0].get_device`; `min`; `range`; `self.coeffs_disparity_predictor`; `self.dres0`; `self.feature_extraction`; `self.guide`; `self.slice`; `self.softmax`; `slice.size`; `slice_dict.append`; `slice_dict_a.append`; `slice_dict_b.append`; `torch.arange`; `torch.arange(0, 97, dtype=slice.dtype, device=slice.device).view`; `torch.cat`; `torch.clamp`; `torch.floor`; `torch.meshgrid`; `torch.split`; `torch.sum`; `torch.sum(disparity_samples * slice, dim=1).unsqueeze`; `wa.float`; `wa.to`; `wa.view`; `wb.float`; `wb.to`; `wb.view`; `wg.float`; `wg.float().repeat`; `wg.float().repeat(N, 1, 1).unsqueeze`; `wg.to` |

### `perception/perception/BGNet/models/bgnet_plus.py`

Dependencies: `from __future__ import print_function` (line 6); `from models.feature_extractor_fast import feature_extraction` (line 7); `from models.submodules3d import CoeffsPredictor` (line 8); `from models.submodules2d import HourglassRefinement` (line 9); `from models.submodules import SubModule, convbn_2d_lrelu, convbn_3d_lrelu, convbn_2d_Tanh` (line 10); `from nets.warp import disp_warp` (line 11); `import torch` (line 12); `import torch.nn as nn` (line 13); `import torch.nn.functional as F` (line 14); `import time` (line 15).

Types: `Slice` : SubModule (line 16); `GuideNN` : SubModule (line 27); `BGNet_Plus` : SubModule (line 70).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `groupwise_correlation` | 37 | `(fea1 * fea2).view`; `(fea1 * fea2).view([B, num_groups, channels_per_group, H, W]).mean` |
| `build_gwc_volume` | 47 | `groupwise_correlation`; `range`; `refimg_fea.new_zeros`; `volume.contiguous` |
| `correlation` | 59 | `(fea1 * fea2).mean` |
| `disparity_regression` | 65 | `disp_values.view`; `len`; `torch.arange`; `torch.sum` |
| `__init__` | 17 | `super`; `super(Slice, self).__init__` |
| `forward` | 20 | `F.grid_sample`; `coeff.squeeze`; `guidemap.permute`; `guidemap.permute(0, 2, 3, 1).contiguous`; `torch.cat`; `torch.cat([wg, hg, guidemap], dim=3).unsqueeze` |
| `__init__` | 28 | `convbn_2d_Tanh`; `convbn_2d_lrelu`; `super`; `super(GuideNN, self).__init__` |
| `forward` | 34 | `self.conv1`; `self.conv2` |
| `__init__` | 71 | `CoeffsPredictor`; `GuideNN`; `HourglassRefinement`; `Slice`; `convbn_3d_lrelu`; `feature_extraction`; `nn.Sequential`; `nn.Softmax`; `self.weight_init`; `super`; `super(BGNet_Plus, self).__init__` |
| `forward` | 84 | `F.interpolate`; `F.interpolate(half_disp * 2.0, scale_factor=(2.0, 2.0), mode='bilinear', align_corners=False).squeeze`; `F.interpolate(refinement_disp * 2.0, scale_factor=(2.0, 2.0), mode='bilinear', align_corners=False).squeeze`; `build_gwc_volume`; `disparity_samples.repeat`; `hg.float`; `hg.float().repeat`; `hg.float().repeat(N, 1, 1).unsqueeze`; `hg.to`; `list_coeffs[0].get_device`; `min`; `pow`; `range`; `self.coeffs_disparity_predictor`; `self.dres0`; `self.feature_extraction`; `self.guide`; `self.refinement_net`; `self.slice`; `self.softmax`; `slice.size`; `slice_dict.append`; `slice_dict_a.append`; `slice_dict_b.append`; `torch.arange`; `torch.arange(0, 97, dtype=slice.dtype, device=slice.device).view`; `torch.cat`; `torch.clamp`; `torch.floor`; `torch.meshgrid`; `torch.split`; `torch.sum`; `torch.sum(disparity_samples * slice, dim=1).unsqueeze`; `wa.float`; `wa.to`; `wa.view`; `wb.float`; `wb.to`; `wb.view`; `wg.float`; `wg.float().repeat`; `wg.float().repeat(N, 1, 1).unsqueeze`; `wg.to` |

### `perception/perception/BGNet/models/feature_extractor_fast.py`

Dependencies: `from __future__ import print_function` (line 6); `import torch` (line 7); `import torch.nn as nn` (line 8); `import torch.utils.data` (line 9); `import torch.nn.functional as F` (line 10).

Types: `BasicBlock` : nn.Module (line 23); `BasicConv` : nn.Module (line 44); `Conv2x` : nn.Module (line 72); `feature_extraction` : nn.Module (line 104).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `convbn_relu` | 11 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.ReLU`; `nn.Sequential` |
| `convbn` | 17 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.Sequential` |
| `__init__` | 26 | `convbn`; `convbn_relu`; `super`; `super(BasicBlock, self).__init__` |
| `forward` | 34 | `self.conv1`; `self.conv2`; `self.downsample` |
| `__init__` | 46 | `nn.BatchNorm2d`; `nn.BatchNorm3d`; `nn.Conv2d`; `nn.Conv3d`; `nn.ConvTranspose2d`; `nn.ConvTranspose3d`; `super`; `super(BasicConv, self).__init__` |
| `forward` | 63 | `F.relu`; `self.conv` |
| `__init__` | 74 | `BasicConv`; `super`; `super(Conv2x, self).__init__` |
| `forward` | 92 | `rem.size`; `self.conv1`; `self.conv2`; `torch.cat`; `x.size` |
| `__init__` | 105 | `BasicConv`; `Conv2x`; `convbn_relu`; `nn.Sequential`; `self._make_layer`; `super`; `super(feature_extraction, self).__init__` |
| `_make_layer` | 139 | `block`; `layers.append`; `nn.BatchNorm2d`; `nn.Conv2d`; `nn.Sequential`; `range` |
| `forward` | 154 | `self.conv1a`; `self.conv1b`; `self.conv2a`; `self.conv2b`; `self.conv3a`; `self.conv3b`; `self.deconv1a`; `self.deconv1b`; `self.deconv2a`; `self.deconv2b`; `self.deconv3a`; `self.deconv3b`; `self.firstconv`; `self.layer1`; `self.layer2`; `self.layer3`; `self.layer4`; `self.reduce`; `torch.cat` |

### `perception/perception/BGNet/models/submodules.py`

Dependencies: `from __future__ import print_function` (line 6); `import torch.nn as nn` (line 7); `import math` (line 8).

Types: `BasicBlock` : nn.Module (line 64); `SubModule` : nn.Module (line 86).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `convbn_2d_lrelu` | 9 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.LeakyReLU`; `nn.Sequential` |
| `convbn_2d_Tanh` | 16 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.Sequential`; `nn.Tanh` |
| `deconvbn_2d_lrelu` | 23 | `nn.BatchNorm2d`; `nn.ConvTranspose2d`; `nn.LeakyReLU`; `nn.Sequential` |
| `convbn_3d_lrelu` | 30 | `nn.BatchNorm3d`; `nn.Conv3d`; `nn.LeakyReLU`; `nn.Sequential` |
| `conv_relu` | 38 | `nn.Conv2d`; `nn.ReLU`; `nn.Sequential` |
| `convbn` | 43 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.Sequential` |
| `convbn_relu` | 50 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.ReLU`; `nn.Sequential` |
| `convbn_transpose_3d` | 58 | `nn.BatchNorm3d`; `nn.ConvTranspose3d`; `nn.Sequential` |
| `__init__` | 67 | `convbn`; `convbn_relu`; `super`; `super(BasicBlock, self).__init__` |
| `forward` | 75 | `self.conv1`; `self.conv2`; `self.downsample` |
| `__init__` | 87 | `super`; `super(SubModule, self).__init__` |
| `weight_init` | 90 | `isinstance`; `m.bias.data.zero_`; `m.weight.data.fill_`; `m.weight.data.normal_`; `math.sqrt`; `self.modules` |

### `perception/perception/BGNet/models/submodules2d.py`

Dependencies: `from __future__ import print_function` (line 6); `import torch.nn as nn` (line 7); `from models.submodules import SubModule, convbn_2d_lrelu, deconvbn_2d_lrelu` (line 8); `import torch` (line 9); `from nets.warp import disp_warp` (line 10); `import torch.nn.functional as F` (line 11).

Types: `BasicConv` : nn.Module (line 24); `Conv2x` : nn.Module (line 52); `HourglassRefinement` : SubModule (line 87).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `disparity_regression` | 12 | `disp_values.view`; `len`; `torch.arange`; `torch.sum` |
| `conv2d` | 17 | `nn.BatchNorm2d`; `nn.Conv2d`; `nn.LeakyReLU`; `nn.Sequential` |
| `__init__` | 26 | `nn.BatchNorm2d`; `nn.BatchNorm3d`; `nn.Conv2d`; `nn.Conv3d`; `nn.ConvTranspose2d`; `nn.ConvTranspose3d`; `super`; `super(BasicConv, self).__init__` |
| `forward` | 43 | `F.relu`; `self.bn`; `self.conv` |
| `__init__` | 54 | `BasicConv`; `super`; `super(Conv2x, self).__init__` |
| `forward` | 78 | `rem.size`; `self.conv1`; `self.conv2`; `torch.cat`; `x.size` |
| `__init__` | 90 | `BasicConv`; `Conv2x`; `conv2d`; `nn.Conv2d`; `super`; `super(HourglassRefinement, self).__init__` |
| `forward` | 122 | `F.interpolate`; `F.relu`; `disp_warp`; `left_img.size`; `low_disp.size`; `self.conv1`; `self.conv1a`; `self.conv1b`; `self.conv2`; `self.conv2a`; `self.conv2b`; `self.conv3a`; `self.conv3b`; `self.conv4a`; `self.conv4b`; `self.conv_start`; `self.deconv1a`; `self.deconv1b`; `self.deconv2a`; `self.deconv2b`; `self.deconv3a`; `self.deconv3b`; `self.deconv4a`; `self.deconv4b`; `self.final_conv`; `torch.cat` |

### `perception/perception/BGNet/models/submodules3d.py`

Dependencies: `from __future__ import print_function` (line 6); `import torch` (line 7); `import torch.nn as nn` (line 8); `import torch.utils.data` (line 9); `from models.submodules import SubModule, convbn_3d_lrelu, convbn_transpose_3d` (line 10).

Types: `HourGlass` : SubModule (line 11); `CoeffsPredictor` : HourGlass (line 33).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `__init__` | 12 | `convbn_3d_lrelu`; `convbn_transpose_3d`; `self.weight_init`; `super`; `super(HourGlass, self).__init__` |
| `__init__` | 35 | `super`; `super(CoeffsPredictor, self).__init__` |
| `forward` | 38 | `self.conv1`; `self.conv1_1`; `self.conv2`; `self.conv2_1`; `self.conv3`; `self.conv4`; `self.conv5`; `self.conv6`; `self.conv7`; `self.last_for_guidance`; `self.last_for_guidance(output1).permute`; `self.last_for_guidance(output1).permute(0, 2, 1, 3, 4).contiguous` |

### `perception/perception/BGNet/nets/warp.py`

Dependencies: `import torch` (line 6); `import torch.nn.functional as F` (line 7); `import time` (line 8).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `disp_warp` | 9 | `((right_y_coordinate_1 < 0) + (right_y_coordinate_1 > right_input.size()[3] - 1)).float`; `disparity_samples.float`; `disparity_samples.size`; `left_y_coordinate.expand`; `left_y_coordinate.expand(disparity_samples.size()[1], -1, -1, -1).permute`; `left_y_coordinate.view`; `right_input.expand`; `right_input.expand(disparity_samples.size()[1], -1, -1, -1, -1).permute`; `right_input.get_device`; `right_input.size`; `right_y_coordinate.expand`; `right_y_coordinate.expand(right_input.size()[1], -1, -1, -1, -1).permute`; `right_y_coordinate.expand(right_input.size()[1], -1, -1, -1, -1).permute([1, 0, 2, 3, 4]).long`; `right_y_coordinate_1.unsqueeze`; `torch.arange`; `torch.arange(0.0, right_input.size()[3], device=device).repeat`; `torch.clamp`; `torch.gather`; `torch.zeros_like`; `warped_right_feature_map.squeeze` |

### `perception/perception/BGNet/predict.py`

Dependencies: `from __future__ import print_function, division` (line 6); `import argparse` (line 7); `import os` (line 8); `import torch.backends.cudnn as cudnn` (line 9); `from torch.utils.data import DataLoader` (line 10); `import torch.utils.data` (line 11); `import time` (line 12); `from datasets import __datasets__` (line 13); `import gc` (line 14); `import skimage` (line 15); `import skimage.io` (line 16); `import skimage.transform` (line 17); `from PIL import Image` (line 18); `from models.bgnet import BGNet` (line 19); `from models.bgnet_plus import BGNet_Plus` (line 20).


### `perception/perception/BGNet/predict_sample.py`

Dependencies: `from __future__ import print_function, division` (line 6); `import argparse` (line 7); `import os` (line 8); `import torch.backends.cudnn as cudnn` (line 9); `from torch.utils.data import DataLoader` (line 10); `import torch.utils.data` (line 11); `import time` (line 12); `from datasets import __datasets__` (line 13); `import gc` (line 14); `import skimage` (line 15); `import skimage.io` (line 16); `import skimage.transform` (line 17); `import numpy as np` (line 18); `from PIL import Image` (line 19); `from datasets.data_io import get_transform` (line 20); `from models.bgnet import BGNet` (line 21); `from models.bgnet_plus import BGNet_Plus` (line 22).


### `perception/perception/compute_depth.py`

Dependencies: `import cv2` (line 1); `import numpy as np` (line 2); `import time` (line 3); `from PIL import Image` (line 4); `import torch` (line 5); `import sys` (line 6); `from datasets.data_io import get_transform` (line 8); `from models.bgnet_plus import BGNet_Plus` (line 9).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `compute_stereo_depth` | 43 | `Image.fromarray`; `bgnet_model`; `cv2.cvtColor`; `cv2.resize`; `float`; `int`; `left_pil.resize`; `np.ascontiguousarray`; `pred[0].data.cpu`; `pred[0].data.cpu().numpy`; `preprocess`; `preprocess(left_np).unsqueeze`; `preprocess(left_np).unsqueeze(0).to`; `preprocess(right_np).unsqueeze`; `preprocess(right_np).unsqueeze(0).to`; `right_pil.resize`; `time.perf_counter`; `torch.cuda.synchronize`; `torch.no_grad` |
| `get_last_depth_timings` | 35 | `dict` |

### `perception/perception/depth_estimation.py`

Dependencies: `from .compute_depth import compute_stereo_depth` (line 1); `from .compute_depth import get_last_depth_timings` (line 2); `from std_msgs.msg import Float32, Header, Float32MultiArray` (line 3); `import rclpy` (line 4); `from rclpy.node import Node` (line 5); `import time` (line 6); `from sensor_msgs.msg import Image` (line 8); `from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy` (line 9); `import numpy as np` (line 10); `import cv2` (line 11); `from cv_bridge import CvBridge` (line 12); `from benchmark_evaluator.midrun_logger import MidrunLogger` (line 13); `import message_filters` (line 16).

Types: `obstacleAvoidance` : Node (line 18).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `main` | 127 | `node.destroy_node`; `obstacleAvoidance`; `rclpy.init`; `rclpy.shutdown`; `rclpy.spin` |
| `__init__` | 19 | `CvBridge`; `MidrunLogger`; `QoSProfile`; `float`; `int`; `message_filters.ApproximateTimeSynchronizer`; `message_filters.Subscriber`; `self.create_publisher`; `self.declare_parameter`; `self.get_logger`; `self.get_logger().info`; `self.get_parameter`; `self.ts.registerCallback`; `super`; `super().__init__` |
| `sync_cam_callback` | 61 | `compute_stereo_depth`; `cv2.cvtColor`; `depth_timings.items`; `get_last_depth_timings`; `self._publish_depth`; `self.bridge.imgmsg_to_cv2`; `self.telemetry.record_stage`; `stage.endswith`; `time.perf_counter` |
| `_publish_depth` | 104 | `depth_np.astype`; `depth_np[:, :, 0].astype`; `np.array`; `self.bridge.cv2_to_imgmsg`; `self.depth_image_pub.publish`; `self.get_logger`; `self.get_logger().error`; `self.get_logger().warn`; `str` |

### `perception/setup.py`

Dependencies: `from setuptools import find_packages, setup` (line 1).


### `perception/test/test_copyright.py`

Dependencies: `from ament_copyright.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_copyright` | 23 | `main`; `pytest.mark.skip` |

### `perception/test/test_flake8.py`

Dependencies: `from ament_flake8.main import main_with_errors` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_flake8` | 21 | `'\n'.join`; `len`; `main_with_errors` |

### `perception/test/test_pep257.py`

Dependencies: `from ament_pep257.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_pep257` | 21 | `main` |

### `python_planner/python_planner/Tools/get_point_cloud.py`

Dependencies: `import cv2` (line 1); `import numpy as np` (line 2); `import math` (line 3); `import os` (line 4); `from datetime import datetime` (line 5); `import pyoctomap` (line 6); `import open3d as o3d` (line 7); `import matplotlib.pyplot as plt` (line 8); `from scipy.spatial import cKDTree` (line 9); `from Tools.mapping3D import generate_seamless_mission_trajectory, plan_multi_waypoint_route` (line 11); `from Tools.search_algo import astar_3d, check_line_of_sight` (line 12); `from scipy.spatial import cKDTree` (line 216).

Types: `RealTimeStereoMapper` : no explicit Python base / C++ definition (line 66).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `generate_global_point_cloud` | 14 | `cv2.cvtColor`; `depth_image.flatten`; `math.cos`; `math.radians`; `math.sin`; `math.tan`; `np.arange`; `np.dot`; `np.meshgrid`; `np.vstack`; `rgb_image.reshape`; `u.flatten`; `v.flatten` |
| `__init__` | 67 | `np.array`; `o3d.geometry.PointCloud`; `o3d.geometry.TriangleMesh.create_sphere`; `open`; `pyoctomap.ColorOcTree`; `self.vehicle_marker.paint_uniform_color` |
| `process_frame` | 96 | `generate_global_point_cloud`; `len`; `location.astype`; `math.cos`; `math.sin`; `np.array`; `raw_colors.astype`; `raw_points.astype`; `self.explore_file.write`; `self.tree.getNumLeafNodes`; `self.tree.insertPointCloud` |
| `update_visualization` | 174 | `current_location.copy`; `len`; `o3d.utility.Vector3dVector`; `plt.get_cmap`; `plt.get_cmap('ocean')`; `self.tree.extractPointCloud`; `self.vehicle_marker.translate`; `self.vis.poll_events`; `self.vis.update_geometry`; `self.vis.update_renderer`; `z_vals.max`; `z_vals.min` |
| `save_map` | 201 | `print`; `self.tree.write` |
| `generate_candidate_viewpoints` | 210 | `adjusted_scores[final_mask].tolist`; `cKDTree`; `candidate_centers[final_mask].tolist`; `candidates.append`; `check_line_of_sight`; `covered_frontiers.add`; `free_tree.query_ball_point`; `generate_seamless_mission_trajectory`; `inflated_occ_set.update`; `int`; `len`; `map`; `np.arccos`; `np.argwhere`; `np.array`; `np.ceil`; `np.clip`; `np.cos`; `np.dot`; `np.exp`; `np.floor`; `np.floor(occupied_points / res).astype`; `np.floor(viewpoint / res).astype`; `np.linalg.norm`; `np.mean`; `np.radians`; `np.round`; `np.sin`; `occ_tree.query`; `optimized_candidates.append`; `plan_multi_waypoint_route`; `print`; `rho.max`; `rho.min`; `self.tree.extractPointCloud`; `self.tree.getResolution`; `set`; `tuple`; `zip` |
| `get_path_to_viewpoint` | 411 | `astar_3d`; `map`; `np.floor`; `np.floor(occupied_points / res).astype`; `self.tree.extractPointCloud`; `self.tree.getResolution`; `set` |
| `validate_current_path` | 422 | `check_line_of_sight`; `map`; `np.floor`; `np.floor(occupied_points / res).astype`; `p.astype`; `print`; `self.tree.extractPointCloud`; `self.tree.getResolution`; `self.tree.isNodeOccupied`; `self.tree.search`; `set` |

### `python_planner/python_planner/Tools/mapping3D.py`

Dependencies: `import math` (line 4); `import numpy as np` (line 5); `from scipy.interpolate import splprep, splev` (line 6); `from Tools.search_algo import astar_3d` (line 8).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `rotate_around_xy_line` | 10 | `math.radians`; `np.cos`; `np.sin`; `np.sqrt` |
| `plan_multi_waypoint_route` | 22 | `astar_3d`; `candidates.copy`; `full_path.extend`; `len`; `np.allclose`; `np.argmin`; `np.array`; `np.linalg.norm`; `ordered_candidates.append`; `print`; `target_tracking.extend`; `unvisited.pop` |
| `generate_seamless_mission_trajectory` | 68 | `len`; `np.arctan2`; `np.linspace`; `np.unwrap`; `np.vstack`; `splev`; `splprep` |

### `python_planner/python_planner/Tools/mapping_3d.py`

Dependencies: `from datetime import datetime` (line 8); `import sys` (line 9); `from Tools.get_point_cloud import RealTimeStereoMapper` (line 12); `from std_msgs.msg import Float32, Header, Float32MultiArray` (line 14); `import rclpy` (line 15); `from rclpy.node import Node` (line 16); `from sensor_msgs.msg import Image, Imu, MagneticField, FluidPressure` (line 18); `from geometry_msgs.msg import TwistStamped` (line 19); `from std_msgs.msg import Float32, Bool` (line 20); `from nav_msgs.msg import Odometry` (line 21); `import numpy as np` (line 22); `import pandas as pd` (line 23); `import cv2` (line 24); `from cv_bridge import CvBridge` (line 25); `from collections import deque` (line 28).

Types: `objectDetection` : Node (line 30).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `main` | 161 | `node.destroy_node`; `objectDetection`; `rclpy.init`; `rclpy.shutdown`; `rclpy.spin` |
| `__init__` | 31 | `CvBridge`; `RealTimeStereoMapper`; `np.array`; `open`; `self.create_publisher`; `self.create_subscription`; `self.create_timer`; `self.get_logger`; `self.get_logger().info`; `super`; `super().__init__` |
| `position_callback` | 66 | `np.arcsin`; `np.arctan2`; `np.array`; `np.clip` |
| `magnetometer_callback` | 80 | `np.arctan2` |
| `cam_callback` | 87 | `cv2.cvtColor`; `self.bridge.imgmsg_to_cv2`; `self.get_logger`; `self.get_logger().info` |
| `depth_cam_callback` | 93 | `cv2.applyColorMap`; `cv2.imshow`; `cv2.normalize`; `cv2.waitKey`; `self.bridge.imgmsg_to_cv2` |
| `timer_callback` | 105 | `(datetime.now() - now).total_seconds`; `Bool`; `Float32MultiArray`; `datetime.now`; `len`; `print`; `self.eval_parameters.write`; `self.file.write`; `self.file_heading.write`; `self.finished_mapping_pub.publish`; `self.get_logger`; `self.get_logger().info`; `self.mapper.generate_candidate_viewpoints`; `self.mapper.process_frame`; `self.mapper.save_map`; `self.pointSegment_pub.publish`; `smooth_traj.flatten`; `smooth_traj.flatten().tolist` |

### `python_planner/python_planner/Tools/search_algo.py`

Dependencies: `import heapq` (line 1); `import numpy as np` (line 2).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `astar_3d` | 4 | `abs`; `heapq.heappop`; `heapq.heappush`; `heuristic`; `np.array`; `np.floor`; `np.floor(goal_pos / res).astype`; `np.floor(start_pos / res).astype`; `path.append`; `path.reverse`; `print`; `tuple` |
| `check_line_of_sight` | 80 | `int`; `np.floor`; `np.floor(p / res).astype`; `np.linalg.norm`; `offsets.append`; `range` |
| `heuristic` | 21 | `abs` |

### `python_planner/python_planner/get_NBV.py`

Dependencies: `import numpy as np` (line 1); `import time` (line 2); `import open3d as o3d` (line 3); `from rclpy.logging import get_logger` (line 4); `from scipy.spatial import KDTree` (line 5).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `calculate_pca_complexity` | 42 | `KDTree`; `eigenvalues.sum`; `int`; `len`; `min`; `neighbors.mean`; `np.asarray`; `np.divide`; `np.einsum`; `np.linalg.eigvalsh`; `np.zeros`; `np.zeros_like`; `range`; `tree.query` |
| `detect_frontiers` | 66 | `(start + np.flatnonzero(has_neighbors & (largest > angle_threshold))).tolist`; `KDTree`; `angles.sort`; `frontier_indices.extend`; `gaps.max`; `int`; `len`; `max`; `min`; `np.arange`; `np.arctan2`; `np.asarray`; `np.cross`; `np.einsum`; `np.flatnonzero`; `np.linalg.norm`; `np.maximum`; `np.radians`; `np.subtract`; `np.zeros`; `range`; `tree.query`; `valid.sum` |
| `cluster_frontiers` | 106 | `clusters.append`; `indices[~accepted].tolist`; `len`; `list`; `np.arccos`; `np.array`; `np.asarray`; `np.clip`; `np.linalg.norm`; `np.mean`; `np.radians`; `range` |
| `generate_cone_viewpoints` | 153 | `candidates.append`; `np.array`; `np.cos`; `np.cross`; `np.linalg.norm`; `np.linspace`; `np.radians`; `np.sin` |
| `evaluate_viewpoint` | 180 | `float`; `np.asarray`; `np.clip`; `np.linalg.norm`; `np.sum` |
| `calculate_kinematic_cost` | 190 | `np.arccos`; `np.clip`; `np.dot`; `np.linalg.norm`; `pcd_tree.query` |
| `normalize_array` | 225 | `arr.max`; `arr.min`; `np.array`; `np.zeros_like` |
| `calculate_nbv_waypoints` | 235 | `KDTree`; `LOGGER.info`; `all_candidates.append`; `calculate_kinematic_cost`; `calculate_pca_complexity`; `cluster_frontiers`; `cluster_winners.items`; `detect_frontiers`; `enumerate`; `evaluate_viewpoint`; `final_waypoints.append`; `final_waypoints.sort`; `float`; `generate_cone_viewpoints`; `int`; `len`; `normalize_array`; `np.abs`; `np.any`; `np.arctan2`; `np.asarray`; `np.asarray([sample[0] for sample in observation_history], dtype=float).reshape`; `np.cos`; `np.linalg.norm`; `np.radians`; `np.sin`; `o3d.geometry.KDTreeSearchParamHybrid`; `o3d.geometry.PointCloud`; `o3d.utility.Vector3dVector`; `operation`; `pcd.estimate_normals`; `pcd.orient_normals_towards_camera_location`; `pcd.remove_statistical_outlier`; `pcd.voxel_down_sample`; `time.perf_counter`; `timed`; `timing_callback` |
| `timed` | 243 | `operation`; `time.perf_counter`; `timing_callback` |

### `python_planner/python_planner/planner_node.py`

Dependencies: `import math` (line 1); `import json` (line 2); `import time` (line 3); `from threading import Lock` (line 4); `from pathlib import Path` (line 5); `import numpy as np` (line 7); `import pyoctomap` (line 8); `import rclpy` (line 9); `from cv_bridge import CvBridge` (line 10); `from nav_msgs.msg import Odometry` (line 11); `from rclpy.callback_groups import MutuallyExclusiveCallbackGroup` (line 12); `from rclpy.executors import MultiThreadedExecutor` (line 13); `from rclpy.node import Node` (line 14); `from sensor_msgs.msg import Image, MagneticField` (line 15); `from std_msgs.msg import Bool, Float32MultiArray` (line 16); `from collections import deque` (line 17); `from benchmark_evaluator.midrun_logger import MidrunLogger` (line 19); `from .get_NBV import DEFAULT_NBV_PARAMETERS, calculate_nbv_waypoints` (line 21); `from .route_planning import generate_smooth_trajectory, inflate_occupied, path_is_collision_free, plan_multi_waypoint_route, resample_trajectory_by_distance, reconnect_trajectory` (line 22); `from .trajectory_optimization import continuity_cost, estimate_velocity, optimize_trajectory` (line 30).

Types: `NBVUtilityNode` : Node (line 58).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `body_rotation` | 35 | `math.cos`; `math.sin`; `np.array` |
| `vehicle_viewpoint` | 46 | `abs`; `body_rotation`; `math.atan2`; `math.cos`; `math.sin`; `range` |
| `main` | 943 | `MultiThreadedExecutor`; `NBVUtilityNode`; `executor.add_node`; `executor.shutdown`; `executor.spin`; `node.destroy_node`; `rclpy.init`; `rclpy.shutdown` |
| `__init__` | 61 | `CvBridge`; `DEFAULT_NBV_PARAMETERS.items`; `Lock`; `MidrunLogger`; `MutuallyExclusiveCallbackGroup`; `Path`; `deque`; `np.array`; `np.empty`; `np.zeros`; `open`; `pyoctomap.ColorOcTree`; `rclpy.qos.QoSProfile`; `self.create_publisher`; `self.create_subscription`; `self.create_timer`; `self.declare_parameter`; `self.get_logger`; `self.get_logger().info`; `set`; `super`; `super().__init__`; `{'continuity_weight': 8.0, 'reversal_weight': 4.0, 'history_weight': 2.0, 'history_radius': 0.5}.items` |
| `request_nbv_callback` | 202 | `self.get_logger`; `self.get_logger().info` |
| `_motion_snapshot` | 208 | `list`; `self.position.copy` |
| `odom_callback` | 213 | `self._update_odometry` |
| `_update_odometry` | 218 | `abs`; `math.asin`; `math.atan2`; `np.array`; `np.clip`; `np.copy`; `self._log_nbv_filter`; `self.odom_history.append`; `self.odom_history.clear` |
| `mag_callback` | 257 | `math.atan2` |
| `rgb_callback` | 265 | None |
| `depth_callback` | 270 | `abs`; `latest[1].copy`; `len`; `min`; `np.copy`; `self._motion_snapshot`; `self.bridge.imgmsg_to_cv2`; `self.get_logger`; `self.get_logger().warning`; `self.telemetry.record_depth_pose` |
| `_depth_points` | 305 | `body_rotation`; `cols[valid].astype`; `depth.astype`; `finite_distances.max`; `finite_distances.min`; `float`; `int`; `list`; `math.cos`; `math.radians`; `math.sin`; `math.tan`; `max`; `np.any`; `np.asarray`; `np.column_stack`; `np.count_nonzero`; `np.empty`; `np.isfinite`; `rows[valid].astype`; `self._log_nbv_filter` |
| `_accumulate_depth` | 357 | `abs`; `float`; `len`; `map`; `math.atan2`; `math.cos`; `math.radians`; `math.sin`; `new_voxels.difference_update`; `np.copy`; `np.floor`; `np.floor(points / self.resolution).astype`; `np.linalg.norm`; `np.random.choice`; `points.astype`; `self._depth_points`; `self._log_nbv_filter`; `self.get_logger`; `self.get_logger().info`; `self.get_logger().warning`; `self.observation_history.append`; `self.occupied_voxels.update`; `self.octree.insertPointCloud`; `self.pending_occupied_voxels.update`; `sensor_origin.astype`; `set`; `voxel_indices.tolist` |
| `_occupied_points` | 407 | `np.asarray`; `np.empty`; `points.reshape`; `self.octree.extractPointCloud`; `tuple` |
| `_publish_trajectory` | 422 | `Float32MultiArray`; `len`; `np.asarray`; `np.asarray(trajectory, dtype=np.float64).copy`; `path_is_collision_free`; `self._log_nbv_filter`; `self.get_logger`; `self.get_logger().info`; `self.waypoint_pub.publish`; `wire.astype`; `wire.flatten`; `wire.flatten().tolist` |
| `_plan_and_publish` | 446 | `candidate.get`; `continuity_cost`; `estimate_velocity`; `float`; `generate_smooth_trajectory`; `int`; `len`; `min`; `np.any`; `np.asarray`; `np.copy`; `np.flatnonzero`; `np.linalg.norm`; `np.linspace`; `np.linspace(0, len(recent_positions) - 1, 64).astype`; `np.unwrap`; `optimize_trajectory`; `option.copy`; `path_is_collision_free`; `plan_multi_waypoint_route`; `reconnect_trajectory`; `resample_trajectory_by_distance`; `route_options.append`; `safe_options.append`; `self._log_nbv_filter`; `self._motion_snapshot`; `self._publish_trajectory`; `self.get_logger`; `self.get_logger().info`; `self.get_logger().warning`; `self.telemetry.record_stage`; `time.perf_counter`; `valid_candidates.sort`; `vehicle_viewpoint` |
| `_expire_target_blacklist` | 622 | `retained.append`; `self._log_nbv_filter`; `self.blacklist_expiries.get`; `self.blacklist_expiries.pop`; `time.perf_counter`; `tuple` |
| `_blacklist_failed_target` | 634 | `next`; `np.asarray`; `np.asarray(self._failed_route_target, dtype=float).copy`; `np.linalg.norm`; `self._log_nbv_filter`; `self.blacklisted_targets.append`; `self.get_logger`; `self.get_logger().warning`; `target.tolist`; `time.perf_counter`; `tuple` |
| `_log_nbv_filter` | 656 | `MidrunLogger._json_safe`; `details.get`; `json.dumps`; `self.get_logger`; `self.get_logger().info`; `self.telemetry.record_filter` |
| `_active_trajectory_is_safe` | 670 | `len`; `path_is_collision_free` |
| `_cancel_active_trajectory` | 680 | `Float32MultiArray`; `len`; `np.empty`; `self._log_nbv_filter`; `self.waypoint_pub.publish` |
| `timer_callback` | 690 | `Bool`; `any`; `body_rotation`; `bool`; `calculate_nbv_waypoints`; `cand.get`; `float`; `inflate_occupied`; `int`; `len`; `map`; `math.cos`; `math.sin`; `max`; `np.array`; `np.floor`; `np.floor(occupied_points / self.resolution).astype`; `np.linalg.norm`; `safe_candidates.append`; `self._accumulate_depth`; `self._active_trajectory_is_safe`; `self._blacklist_failed_target`; `self._cancel_active_trajectory`; `self._expire_target_blacklist`; `self._log_nbv_filter`; `self._motion_snapshot`; `self._occupied_points`; `self._plan_and_publish`; `self.collision_voxels.update`; `self.evaluation_file.flush`; `self.evaluation_file.write`; `self.finished_pub.publish`; `self.get_logger`; `self.get_logger().error`; `self.get_logger().info`; `self.get_logger().warning`; `self.octree.getNumLeafNodes`; `self.octree.write`; `self.telemetry.record_iteration`; `self.telemetry.record_stage`; `set`; `str`; `time.perf_counter`; `tuple`; `type` |
| `destroy_node` | 931 | `self.evaluation_file.close`; `self.get_logger`; `self.get_logger().error`; `self.get_logger().info`; `self.octree.write`; `super`; `super().destroy_node` |

### `python_planner/python_planner/route_planning.py`

Dependencies: `import heapq` (line 1); `import numpy as np` (line 3); `from scipy.interpolate import splprep, splev` (line 4).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `astar_3d` | 7 | `abs`; `costs.get`; `float`; `heapq.heappop`; `heapq.heappush`; `heuristic`; `np.asarray`; `np.floor`; `np.floor(goal_pos / resolution).astype`; `np.floor(start_pos / resolution).astype`; `path.append`; `path.reverse`; `range`; `sum`; `tuple` |
| `inflate_occupied` | 54 | `inflated.update`; `int`; `np.ceil`; `range`; `set` |
| `path_is_collision_free` | 79 | `int`; `len`; `max`; `np.all`; `np.asarray`; `np.ceil`; `np.floor`; `np.floor(np.asarray(path[0]) / resolution).astype`; `np.floor(point / resolution).astype`; `np.isfinite`; `np.linalg.norm`; `np.linspace`; `tuple`; `zip` |
| `reconnect_trajectory` | 99 | `int`; `len`; `np.all`; `np.argmin`; `np.asarray`; `np.clip`; `np.diff`; `np.empty`; `np.isfinite`; `np.linalg.norm`; `np.maximum`; `np.sum`; `np.unwrap`; `np.vstack`; `path.copy`; `path[segment + 1:].copy`; `path_is_collision_free` |
| `plan_multi_waypoint_route` | 137 | `astar_3d`; `full_path.extend`; `int`; `len`; `list`; `max`; `np.allclose`; `np.argmin`; `np.asarray`; `np.concatenate`; `np.diff`; `np.dot`; `np.empty`; `np.linalg.norm`; `np.vstack`; `np.zeros`; `ordered.append`; `path_is_collision_free`; `remaining.pop`; `scores.append`; `target_tracking.extend` |
| `generate_smooth_trajectory` | 194 | `int`; `len`; `max`; `np.arctan2`; `np.asarray`; `np.ceil`; `np.column_stack`; `np.concatenate`; `np.cumsum`; `np.diff`; `np.empty`; `np.interp`; `np.linalg.norm`; `np.linalg.norm(np.diff(path, axis=0), axis=1).sum`; `np.linspace`; `np.unwrap`; `splev`; `splprep` |
| `resample_trajectory_by_distance` | 234 | `int`; `len`; `np.append`; `np.arange`; `np.asarray`; `np.column_stack`; `np.concatenate`; `np.cumsum`; `np.diff`; `np.interp`; `np.linalg.norm`; `np.linspace`; `np.linspace(0, len(result) - 1, int(max_points)).round`; `np.unique`; `np.unique(indices).astype`; `np.unwrap`; `range`; `trajectory[[0]].copy` |
| `heuristic` | 22 | `abs`; `range`; `sum` |

### `python_planner/python_planner/trajectory_optimization.py`

Dependencies: `import numpy as np` (line 7); `from scipy.optimize import minimize` (line 8); `from .route_planning import path_is_collision_free` (line 10).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `estimate_velocity` | 13 | `len`; `np.all`; `np.asarray`; `np.dot`; `np.isfinite`; `np.sum`; `np.zeros`; `positions.mean`; `times.mean` |
| `continuity_cost` | 32 | `abs`; `float`; `history.reshape`; `len`; `max`; `min`; `np.abs`; `np.abs(yaw_steps).sum`; `np.arctan2`; `np.array`; `np.asarray`; `np.cos`; `np.cross`; `np.cumsum`; `np.diff`; `np.linalg.norm`; `np.linalg.norm(trajectory[1:, None, :3] - history[None, :, :], axis=2).min`; `np.maximum`; `np.sin`; `np.sum`; `np.unwrap` |
| `optimize_trajectory` | 83 | `continuity_cost`; `decode`; `dict`; `int`; `len`; `min`; `minimize`; `np.all`; `np.arange`; `np.asarray`; `np.asarray(trajectory, dtype=np.float64).copy`; `np.column_stack`; `np.diff`; `np.empty`; `np.interp`; `np.isfinite`; `np.linalg.norm`; `np.linspace`; `np.linspace(0, len(reference) - 1, min(32, len(reference))).round`; `np.linspace(0, len(reference) - 1, min(32, len(reference))).round().astype`; `np.maximum`; `np.mean`; `np.sum`; `np.unique`; `np.unwrap`; `np.zeros`; `objective`; `offsets.reshape`; `path_is_collision_free`; `range` |
| `decode` | 112 | `np.column_stack`; `np.interp`; `offsets.reshape`; `range` |
| `objective` | 118 | `continuity_cost`; `decode`; `np.diff`; `np.linalg.norm`; `np.maximum`; `np.mean`; `np.sum` |

### `python_planner/setup.py`

Dependencies: `from setuptools import find_packages, setup` (line 1).


### `python_planner/test/test_copyright.py`

Dependencies: `from ament_copyright.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_copyright` | 23 | `main`; `pytest.mark.skip` |

### `python_planner/test/test_flake8.py`

Dependencies: `from ament_flake8.main import main_with_errors` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_flake8` | 21 | `'\n'.join`; `len`; `main_with_errors` |

### `python_planner/test/test_motion_continuity.py`

Dependencies: `import numpy as np` (line 3); `from python_planner.route_planning import generate_smooth_trajectory, path_is_collision_free, plan_multi_waypoint_route` (line 5); `from python_planner.trajectory_optimization import continuity_cost, estimate_velocity, optimize_trajectory` (line 8); `from python_planner.route_planning import reconnect_trajectory` (line 112); `from python_planner.route_planning import reconnect_trajectory` (line 123); `from python_planner.route_planning import reconnect_trajectory` (line 133); `from python_planner.route_planning import reconnect_trajectory` (line 142).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `trajectory` | 13 | `len`; `np.asarray`; `np.broadcast_to`; `np.column_stack` |
| `test_velocity_uses_world_pose_history_and_irregular_timestamps` | 18 | `estimate_velocity`; `np.array`; `np.testing.assert_allclose`; `np.testing.assert_array_equal`; `np.zeros` |
| `test_candidate_order_prefers_forward_motion_but_allows_required_return` | 27 | `np.array`; `np.testing.assert_allclose`; `plan_multi_waypoint_route`; `set` |
| `test_reversal_and_full_yaw_turn_have_higher_cost` | 40 | `continuity_cost`; `forward.copy`; `np.array`; `np.linspace`; `trajectory` |
| `test_recent_retracing_is_penalized_without_penalizing_departure` | 51 | `continuity_cost`; `np.zeros`; `trajectory` |
| `test_gradual_spatial_loop_is_penalized_even_with_constant_yaw` | 58 | `continuity_cost`; `len`; `np.column_stack`; `np.cos`; `np.linspace`; `np.sin`; `np.zeros`; `trajectory` |
| `test_spline_positions_are_independent_of_target_heading` | 65 | `generate_smooth_trajectory`; `np.array`; `np.testing.assert_allclose` |
| `test_optimizer_improves_entry_and_preserves_boundary_and_step_lengths` | 74 | `continuity_cost`; `np.array`; `np.linalg.norm`; `np.testing.assert_allclose`; `optimize_trajectory`; `range`; `set`; `trajectory` |
| `test_optimizer_keeps_collision_safe_route_in_narrow_corridor` | 86 | `len`; `np.array`; `optimize_trajectory`; `path_is_collision_free`; `range`; `trajectory` |
| `test_collision_check_rejects_single_occupied_point_and_nan` | 96 | `np.array`; `path_is_collision_free`; `set` |
| `test_yaw_wrap_is_shortest_rotation_and_stationary_velocity_is_safe` | 101 | `np.abs`; `np.all`; `np.deg2rad`; `np.diff`; `np.isfinite`; `np.max`; `np.zeros`; `optimize_trajectory`; `range`; `set`; `trajectory` |
| `test_delayed_plan_starts_at_latest_pose_and_drops_passed_prefix` | 111 | `np.all`; `np.array`; `np.diff`; `np.testing.assert_allclose`; `range`; `reconnect_trajectory`; `set`; `trajectory` |
| `test_delayed_entry_rejects_collision_on_new_connector` | 122 | `path_is_collision_free`; `reconnect_trajectory`; `trajectory` |
| `test_delayed_entry_preserves_required_turn_and_short_yaw_wrap` | 132 | `np.abs`; `np.deg2rad`; `np.diff`; `np.max`; `np.testing.assert_allclose`; `reconnect_trajectory`; `set`; `trajectory` |
| `test_delayed_entry_handles_stationary_single_and_duplicate_samples` | 141 | `np.empty`; `np.testing.assert_allclose`; `reconnect_trajectory`; `set`; `trajectory` |

### `python_planner/test/test_nbv_config.py`

Dependencies: `import ast` (line 3); `from pathlib import Path` (line 4); `import yaml` (line 6).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_split_nbv_config_contains_supported_tuning_parameters` | 9 | `(config / 'nbv_params.yaml').read_text`; `(config / 'params.yaml').read_text`; `(source_root / 'python_planner/python_planner/get_NBV.py').read_text`; `Path`; `any`; `ast.literal_eval`; `ast.parse`; `isinstance`; `next`; `set`; `set(tuning).intersection`; `yaml.safe_load` |

### `python_planner/test/test_nbv_filter_logging.py`

Dependencies: `import ast` (line 3); `import importlib.util` (line 4); `import json` (line 5); `import inspect` (line 6); `import math` (line 7); `from pathlib import Path` (line 8); `from types import SimpleNamespace` (line 9); `import time` (line 10); `import numpy as np` (line 12); `import pytest` (line 13); `from scipy.spatial import KDTree` (line 14).

Types: `Cloud` : no explicit Python base / C++ definition (line 42).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `load_file` | 20 | `importlib.util.module_from_spec`; `importlib.util.spec_from_file_location`; `spec.loader.exec_module` |
| `pipeline` | 34 | `SimpleNamespace`; `any`; `ast.Module`; `ast.parse`; `compile`; `dict`; `exec`; `isinstance`; `len`; `list`; `np.arange`; `np.array`; `np.column_stack`; `np.empty`; `np.linspace`; `np.ones`; `np.tile`; `np.zeros`; `range`; `source.read_text`; `str` |
| `test_empty_pipeline_returns_no_candidates` | 88 | `np.array`; `np.empty`; `np.zeros`; `ns['calculate_nbv_waypoints']`; `pytest.mark.parametrize` |
| `test_timing_preserves_candidates` | 113 | `all`; `events.append`; `len`; `np.array`; `np.testing.assert_equal`; `np.zeros`; `ns['calculate_nbv_waypoints']` |
| `test_pose_history_removes_matching_frontier` | 126 | `np.array`; `np.zeros`; `ns['calculate_nbv_waypoints']` |
| `test_cluster_rejects_groups_below_minimum_sample_count` | 134 | `(ROOT / 'python_planner/get_NBV.py').read_text`; `ast.Module`; `ast.parse`; `compile`; `exec`; `isinstance`; `next`; `np.array`; `np.tile`; `ns['cluster_frontiers']` |
| `test_node_nbv_call_matches_restored_signature` | 146 | `(ROOT / 'python_planner/planner_node.py').read_text`; `ast.parse`; `ast.walk`; `inspect.signature`; `inspect.signature(ns['calculate_nbv_waypoints']).bind`; `isinstance`; `object` |
| `test_pipeline_passes_complexity_weight_to_gain_calculation` | 157 | `ast.Module`; `ast.parse`; `compile`; `exec`; `isinstance`; `len`; `next`; `np.array`; `np.zeros`; `ns['calculate_nbv_waypoints']`; `pytest.approx`; `source.read_text`; `str` |
| `test_astar_return_matches_occupied_endpoints_and_expansion_limit` | 177 | `bool`; `np.array`; `pytest.mark.parametrize`; `routes.astar_3d`; `set` |
| `test_collision_rejects_occupied_voxel_between_endpoints` | 182 | `np.array`; `routes.path_is_collision_free` |
| `test_route_skips_occupied_viewpoint` | 188 | `len`; `np.array`; `routes.plan_multi_waypoint_route` |
| `test_jsonl_logs_iteration_and_handles_nonfinite_values` | 196 | `json.loads`; `logger.filter_path.read_text`; `logger.record_filter`; `np.float64`; `telemetry.MidrunLogger` |
| `node_method` | 207 | `SimpleNamespace`; `ast.Module`; `ast.parse`; `compile`; `dict`; `exec`; `isinstance`; `namespace.update`; `next`; `source.read_text`; `str` |
| `test_node_reports_utility_removing_all_candidates` | 223 | `SimpleNamespace`; `dict`; `events.append`; `node_method`; `node_method('_plan_and_publish')`; `np.ones`; `np.zeros`; `set` |
| `test_depth_range_diagnostics_count_disjoint_rejections` | 233 | `SimpleNamespace`; `events.append`; `len`; `node_method`; `node_method('_depth_points')`; `np.array`; `np.zeros` |
| `test_disabled_node_logging_does_not_access_logger_or_files` | 247 | `SimpleNamespace`; `node_method`; `node_method('_log_nbv_filter')` |
| `test_blocked_route_cancellation_reaches_downstream_queue` | 252 | `SimpleNamespace`; `events.append`; `len`; `node_method`; `node_method('_cancel_active_trajectory')`; `np.ones` |
| `test_depth_sensor_mount_offset_rotates_with_vehicle` | 268 | `SimpleNamespace`; `math.sqrt`; `node_method`; `node_method('_depth_points')`; `np.array`; `np.full`; `np.testing.assert_allclose` |
| `test_vehicle_viewpoint_places_rotated_mount_at_requested_camera_position` | 281 | `ast.Module`; `ast.parse`; `compile`; `dict`; `exec`; `isinstance`; `math.atan2`; `namespace['body_rotation']`; `namespace['vehicle_viewpoint']`; `np.array`; `np.testing.assert_allclose`; `source.read_text`; `str` |
| `test_rejected_combined_spline_keeps_checked_first_structural_target` | 297 | `SimpleNamespace`; `all`; `dict`; `events.append`; `len`; `method`; `node_method`; `np.array`; `np.array_equal`; `np.column_stack`; `np.testing.assert_allclose`; `np.testing.assert_equal`; `np.vstack`; `np.zeros`; `published.append`; `pytest.mark.parametrize`; `routes.generate_smooth_trajectory`; `routes.path_is_collision_free` |
| `test_vehicle_route_is_checked_instead_of_camera_offset_route` | 344 | `SimpleNamespace`; `node_method`; `node_method('_publish_trajectory')`; `np.array`; `routes.path_is_collision_free` |
| `test_wire_validation_rejects_float32_rounding_into_occupied_voxel` | 356 | `SimpleNamespace`; `node_method`; `node_method('_publish_trajectory')`; `np.array`; `routes.path_is_collision_free` |
| `test_failed_target_blacklist_expires_and_requests_another_target` | 368 | `SimpleNamespace`; `blacklist`; `events.append`; `expire`; `len`; `node_method`; `np.array`; `np.testing.assert_equal`; `tuple` |
| `test_failed_target_blacklist_deduplicates_nearby_cluster_drift` | 394 | `SimpleNamespace`; `len`; `method`; `node_method`; `np.array` |
| `test_global_failure_or_disabled_cooldown_does_not_ban_a_target` | 409 | `SimpleNamespace`; `node_method`; `node_method('_blacklist_failed_target')`; `np.ones`; `pytest.mark.parametrize` |
| `spline` | 314 | `len`; `np.column_stack`; `np.vstack`; `np.zeros`; `routes.generate_smooth_trajectory` |
| `__init__` | 43 | `np.empty` |
| `voxel_down_sample` | 46 | None |
| `remove_statistical_outlier` | 49 | `len`; `list`; `range` |
| `estimate_normals` | 52 | `len`; `np.tile` |
| `orient_normals_towards_camera_location` | 55 | None |

### `python_planner/test/test_nbv_math.py`

Dependencies: `import ast` (line 3); `from pathlib import Path` (line 4); `from types import SimpleNamespace` (line 5); `import numpy as np` (line 7); `import pytest` (line 8); `from scipy.spatial import KDTree` (line 9).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_viewpoint_gain_uses_configured_complexity_weight` | 24 | `math_functions['evaluate_viewpoint']`; `np.array`; `np.tile`; `np.zeros`; `pytest.approx`; `pytest.mark.parametrize` |
| `test_pca_matches_original_covariance_formula` | 33 | `KDTree`; `SimpleNamespace`; `expected.append`; `math_functions['calculate_pca_complexity']`; `neighbor_tree.query`; `neighbors.mean`; `np.cov`; `np.linalg.eigvalsh`; `np.random.default_rng`; `np.random.default_rng(12).normal`; `np.testing.assert_allclose`; `pytest.mark.parametrize`; `values.sum` |
| `test_frontiers_match_original_angular_gap_formula` | 47 | `KDTree`; `SimpleNamespace`; `enumerate`; `expected.append`; `math_functions['detect_frontiers']`; `neighbor_tree.query`; `np.arctan2`; `np.cross`; `np.diff`; `np.dot`; `np.linalg.norm`; `np.max`; `np.radians`; `np.random.default_rng`; `pytest.mark.parametrize`; `rng.normal`; `sorted`; `zip` |
| `test_frontiers_handle_small_and_duplicate_clouds` | 72 | `SimpleNamespace`; `math_functions['detect_frontiers']`; `np.tile`; `np.zeros`; `pytest.mark.parametrize` |
| `test_greedy_clustering_keeps_order_and_discards_small_clusters` | 78 | `cluster['global_indices'].tolist`; `len`; `list`; `math_functions['cluster_frontiers']`; `np.array`; `np.testing.assert_allclose`; `np.tile`; `range` |
| `test_viewpoint_gain_matches_original_and_skips_coincident_point` | 89 | `math_functions['evaluate_viewpoint']`; `np.array`; `np.clip`; `np.dot`; `np.linalg.norm`; `np.random.default_rng`; `np.testing.assert_allclose`; `rng.normal`; `rng.uniform`; `zip` |

### `python_planner/test/test_pep257.py`

Dependencies: `from ament_pep257.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_pep257` | 21 | `main` |

### `python_planner/test/test_route_search_limits.py`

Dependencies: `import ast` (line 3); `import inspect` (line 4); `from pathlib import Path` (line 5); `import numpy as np` (line 7); `from python_planner import route_planning as routes` (line 9).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_runtime_callers_match_restored_route_planning_signatures` | 12 | `AssertionError`; `Path`; `all`; `ast.parse`; `ast.walk`; `getattr`; `imports.get`; `inspect.signature`; `isinstance`; `object`; `package.glob`; `signature.bind`; `source.read_text` |
| `test_astar_preserves_detours_around_obstacles` | 39 | `len`; `np.array`; `np.asarray`; `routes.astar_3d`; `routes.path_is_collision_free` |
| `test_resampling_handles_duplicate_and_stationary_points` | 49 | `np.testing.assert_allclose`; `np.tile`; `routes.resample_trajectory_by_distance` |

### `sim_bridge/setup.py`

Dependencies: `from setuptools import find_packages, setup` (line 1).


### `sim_bridge/sim_bridge/holoocean_node.py`

Dependencies: `import sys` (line 19); `import rclpy` (line 21); `from rclpy.node import Node` (line 22); `from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data` (line 23); `import numpy as np` (line 25); `import time` (line 26); `import traceback` (line 27); `from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField` (line 29); `from std_msgs.msg import Float32, Header, Float32MultiArray` (line 30); `from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point` (line 31); `from nav_msgs.msg import Odometry` (line 32); `from cv_bridge import CvBridge` (line 35); `import math` (line 38); `import json` (line 39); `import holoocean` (line 566).

Types: `HoloOceanROS2Bridge` : Node (line 59).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `quaternion_from_euler` | 41 | `math.cos`; `math.sin` |
| `vortex_field` | 48 | `np.cos` |
| `main` | 565 | `HoloOceanROS2Bridge`; `bridge_node.destroy_node`; `bridge_node.get_logger`; `bridge_node.get_logger().info`; `rclpy.init`; `rclpy.shutdown`; `rclpy.spin` |
| `__init__` | 60 | `CameraInfo`; `CvBridge`; `QoSProfile`; `json.load`; `open`; `self._run_holoocean_loop`; `self.create_publisher`; `self.create_subscription`; `self.declare_parameter`; `self.get_logger`; `self.get_logger().error`; `self.get_logger().info`; `self.get_parameter`; `str`; `super`; `super().__init__`; `traceback.format_exc` |
| `cmd_callback` | 144 | `list`; `print` |
| `_stamp` | 148 | `self.get_clock`; `self.get_clock().now`; `self.get_clock().now().to_msg` |
| `_publish_camera` | 152 | `CameraInfo`; `Header`; `np.clip`; `np.clip(np_img, 0, 255).astype`; `self._stamp`; `self.bridge.cv2_to_imgmsg`; `self.cam_info_pub.publish`; `self.camera_pub.publish`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `str`; `traceback.format_exc` |
| `_publish_camera_down` | 198 | `Header`; `np.clip`; `np.clip(np_img, 0, 255).astype`; `self._stamp`; `self.bridge.cv2_to_imgmsg`; `self.cameraDown_pub.publish`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `str`; `traceback.format_exc` |
| `_publish_depth` | 247 | `Float32`; `depth_np.astype`; `depth_np[:, :, 0].astype`; `float`; `isinstance`; `np.array`; `self._stamp`; `self.bridge.cv2_to_imgmsg`; `self.depth_image_pub.publish`; `self.depth_scalar_pub.publish`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.get_logger().warn`; `str`; `traceback.format_exc` |
| `_publish_imu` | 290 | `Imu`; `arr[0:3].tolist`; `arr[3:6].tolist`; `float`; `imu_data.get`; `isinstance`; `len`; `list`; `np.array`; `np.array(imu_data).flatten`; `ori.get`; `self._stamp`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.imu_pub.publish`; `str`; `traceback.format_exc` |
| `_publish_mag` | 337 | `MagneticField`; `float`; `isinstance`; `mag_data.get`; `np.array`; `np.array(mag_data).flatten`; `self._stamp`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.mag_pub.publish`; `str`; `traceback.format_exc` |
| `_publish_dvl` | 361 | `Float32`; `TwistStamped`; `float`; `np.array`; `np.average`; `self._stamp`; `self.depth_distance_pub.publish`; `self.dvl_pub.publish`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `str`; `traceback.format_exc` |
| `_publish_range_finder` | 391 | `Float32`; `float`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.range_finder_pub.publish`; `str`; `traceback.format_exc` |
| `_publish_location` | 406 | `Odometry`; `arr[0:3].tolist`; `arr[3:6].tolist`; `float`; `isinstance`; `len`; `list`; `loc_data.get`; `np.array`; `ori.get`; `self._stamp`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.odom_pub.publish`; `str`; `traceback.format_exc` |
| `_run_holoocean_loop` | 459 | `env.set_ocean_currents`; `env.step`; `getattr`; `hasattr`; `isinstance`; `np.array`; `rclpy.spin_once`; `self._publish_camera`; `self._publish_camera_down`; `self._publish_depth`; `self._publish_dvl`; `self._publish_imu`; `self._publish_location`; `self._publish_mag`; `self._publish_range_finder`; `self.get_logger`; `self.get_logger().debug`; `self.get_logger().error`; `self.get_logger().info`; `self.holoocean_make_fn`; `state.get`; `str`; `traceback.format_exc`; `vortex_field` |

### `sim_bridge/test/test_copyright.py`

Dependencies: `from ament_copyright.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_copyright` | 23 | `main`; `pytest.mark.skip` |

### `sim_bridge/test/test_flake8.py`

Dependencies: `from ament_flake8.main import main_with_errors` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_flake8` | 21 | `'\n'.join`; `len`; `main_with_errors` |

### `sim_bridge/test/test_pep257.py`

Dependencies: `from ament_pep257.main import main` (line 15); `import pytest` (line 16).

| Function/method | Line | Syntactic calls (including nested bodies; not runtime frequency) |
|---|---:|---|
| `test_pep257` | 21 | `main` |

## Contract Concerns Kept Separate from SOLID Violations

- `perception/perception/compute_depth.py:32–37`: timing state/accessor is indented under the PIL AttributeError branch, yet depth_estimation imports that accessor unconditionally. This is a version-sensitive initialization/contract concern, not evidence of a subtype violating LSP. The architectural lifecycle issue is already perception/V-01.
- Both KITTI dataset implementations unpack four values from load_path (`:18`), while the two-column branch returns three (`:28`). That is a local API inconsistency for that input, not proof of a Dataset substitution-contract violation. No dataset execution was performed.
- `benchmark_evaluator/benchmark_evaluator/optuna_optimizer.py:9` uses `from benchmark_evaluator import evaluate_map_accuracy`; the package __init__.py is empty. Direct script execution can resolve the sibling module differently from package import. The diagram records intended evaluation use; installed-module execution is not verified.
- Alternative C++ A* can relocate occupied endpoints (`active_inspection_planner/src/search_algo.cpp:85`); active Python A* has a different endpoint rejection policy. These implementations have no declared common substitutability contract. Define that contract before consolidation; do not label the difference an established LSP violation.

These observations are diagnostic only; none was fixed. Binary model weights, training data and third-party installed library implementations were not semantically audited.
