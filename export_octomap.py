#!/usr/bin/env python3
"""Export occupied OctoMap leaves as a block-shaped Wavefront OBJ mesh.

Uses the existing pyoctomap dependency. Coordinates and voxel sizes are in
meters. Each occupied leaf becomes a closed cube, including internal faces;
this is a voxel visualization, rather than a smoothed reconstruction.

Examples:
    python3 export_octomap.py
    python3 export_octomap.py results/final_map_ros2.bt -o results/map.obj --center
    python3 export_octomap.py --z-min -298.5
"""

import argparse
import math
from pathlib import Path


# Counterclockwise faces viewed from outside the cube.
FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
CORNERS = ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
           (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))


def export_obj(input_path, output_path, *, center=False, z_min=None):
    import pyoctomap

    if input_path.resolve() == output_path.resolve():
        raise ValueError('Input and output must be different files')
    if not input_path.is_file():
        raise ValueError(f'OctoMap file does not exist: {input_path}')
    tree = pyoctomap.ColorOcTree(0.2).read(str(input_path))
    if tree is None:
        raise ValueError(f'Could not read OctoMap: {input_path}')
    voxels = []
    for leaf in tree.begin_leafs():
        if tree.isNodeOccupied(leaf.current_node):
            position = tuple(leaf.getCoordinate())
            if z_min is None or position[2] >= z_min:
                voxels.append((position, leaf.getSize()))
    if not voxels:
        raise ValueError('No occupied voxels remain after filtering')

    origin = (0.0, 0.0, 0.0)
    if center:
        lower = [min(p[a] - size / 2 for p, size in voxels) for a in range(3)]
        upper = [max(p[a] + size / 2 for p, size in voxels) for a in range(3)]
        origin = tuple((lo + hi) / 2 for lo, hi in zip(lower, upper))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write(f'# Occupied OctoMap voxels from {input_path.name}\n')
        stream.write('# Units: meters; Z points upward\n')
        stream.write('# Subtracted world origin: ' + ' '.join(map(str, origin)) + '\n')
        stream.write('o octomap\n')
        for index, (position, size) in enumerate(voxels):
            for corner in CORNERS:
                vertex = [position[a] - origin[a] + corner[a] * size / 2
                          for a in range(3)]
                stream.write('v ' + ' '.join(f'{v:.9f}' for v in vertex) + '\n')
            base = index * 8 + 1
            for a, b, c, d in FACES:
                stream.write(f'f {base+a} {base+b} {base+c}\n')
                stream.write(f'f {base+a} {base+c} {base+d}\n')
    return len(voxels), origin


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', nargs='?', type=Path,
                        default=Path(__file__).resolve().parent / 'results/final_map_ros2.bt')
    parser.add_argument('-o', '--output', type=Path, help='Output OBJ path (default: input with .obj extension)')
    parser.add_argument('--center', action='store_true', help='Center the mesh bounding box at the origin')
    parser.add_argument('--z-min', type=float, help='Keep occupied leaves whose center Z is at least this value')
    args = parser.parse_args()
    if args.z_min is not None and not math.isfinite(args.z_min):
        parser.error('--z-min must be finite')
    output = args.output or args.input.with_suffix('.obj')
    try:
        count, origin = export_obj(args.input, output, center=args.center, z_min=args.z_min)
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f'Export failed: {error}\n')
    print(f'Saved {output}: {count} occupied voxels, {count * 12} triangles')
    if args.center:
        print(f'Subtracted world origin: {origin}')


if __name__ == '__main__':
    main()
