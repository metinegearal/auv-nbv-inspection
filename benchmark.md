| Method                          | Purpose                         |
| ------------------------------- | ------------------------------- |
| Frontier                        | Classical exploration baseline  |
| Lawnmower/orbit                 | Pre-scripted survey baseline    |
| Gain-only NBV                   | Tests whether A* cost matters   |
| Gain/cost NBV without normals   | Tests your candidate constraint |
| Gain/cost + normals without LoD | Tests complexity weighting      |
| Full method                     | Complete proposed planner       |

## Experiments
Experiment A — viewpoint efficiency

Measure:

$$ N_{\text{candidate}} $$

and computation time.

Hypothesis:

$$ \text{normal-constrained candidates} < \text{generic spherical candidate sampling}. $$
Experiment B — coverage

Measure:

$$ \text{Surface Coverage}= \frac{A_{\text{observed}}}{A_{\text{ground truth}}}. $$
Experiment C — reconstruction quality

For known simulated ground truth:

$$ \text{Chamfer Distance} $$

or

$$ \text{Hausdorff Distance}. $$

Also measure completeness.

Experiment D — mission efficiency

Measure:

$$ L_{\text{trajectory}}, \quad T_{\text{mission}}, \quad N_{\text{views}}. $$
Experiment E — safety

Measure:

$$ d_{\min} $$

between trajectory and occupied geometry.

Experiment F — complexity prioritization

Partition the environment into:

planar,
moderately structured,
highly structured

regions.

Then show whether the method spends more observation effort on the highly structured regions.

## Scenes
Scene 1 — Flat wall

Tests whether you waste observations.

Scene 2 — Convex pipe/tank

Tests normal estimation.

Scene 3 — Concave structure

Tests the failure mode of normal-only viewpoints.

Scene 4 — Highly irregular reef/rock

Tests geometric complexity weighting.

Scene 5 — Industrial-looking subsea structure

Tests genuine inspection behavior.

Scene 6 — Occlusion-heavy scene

Tests whether raycasting and viewpoint generation recover hidden surfaces.