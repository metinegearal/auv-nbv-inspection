Problem

Conventional frontier exploration maximizes exploration of unknown space but does not explicitly optimize surface observation quality.

Limitation

Pre-scripted lawnmower/orbit trajectories provide coverage but waste sensing effort on geometrically simple regions and have limited adaptability to complex structures.

Proposed idea

Use the local surface geometry discovered online to generate observation-aware candidate viewpoints at a controlled standoff distance.

Key insight

The NBV search space can be dramatically reduced by exploiting surface normals, while geometric-complexity weighting focuses observations where additional detail is most valuable.

Closed loop

After every observation:

$$ \text{Map} \rightarrow \text{surface extraction} \rightarrow \text{normal estimation} \rightarrow \text{candidate generation} \rightarrow \text{utility} \rightarrow \text{A*} \rightarrow \text{motion} \rightarrow \text{map update}. $$

That is a good research story.


## Use
Map B — Surface/reconstruction map

Voxblox/TSDF/mesh:

$$ M_{\text{surf}} $$

contains:

surface geometry,
normals,
curvature,
uncertainty,
observed area,
reconstruction quality.