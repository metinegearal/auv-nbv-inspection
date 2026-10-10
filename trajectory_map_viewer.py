"""Display saved benchmark trajectories over the known inspection objects."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


PIPE_OBJECTS = (
    ((-14.1, -81.1), (-15.7, -78.5)),
    ((-12.1, -81.1), (-11.0, -78.5)),
    ((-14.1, -88.4), (-15.7, -91.0)),
    ((-12.1, -88.4), (-11.0, -91.0)),
)


def load_trajectory(path):
    """Read the XY path and final cumulative distance from one CSV file."""
    points = []
    final_distance = None
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            points.append((float(row["X_m"]), float(row["Y_m"])))
            if row.get("Cumulative_Distance_m"):
                final_distance = float(row["Cumulative_Distance_m"])
    return points, final_distance


def add_objects(axis):
    """Draw the rectangle and pipe objects used by the benchmark plot."""
    axis.add_patch(Rectangle(
        (-14.1, -82.7),
        2.1,
        1.6,
        facecolor="#2563eb",
        edgecolor="#1e3a8a",
        alpha=0.16,
        linewidth=1.2,
        label="Rectangle object",
    ))
    axis.add_patch(Rectangle(
        (-14.1, -87.4),
        2.1,
        1.6,
        facecolor="#2563eb",
        edgecolor="#1e3a8a",
        alpha=0.16,
        linewidth=1.2,
        label="Mirrored rectangle object",
    ))
    for index, (start, end) in enumerate(PIPE_OBJECTS):
        axis.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#a16207",
            linewidth=7,
            alpha=0.22,
            solid_capstyle="round",
                 label=("Pipe objects" if index == 0 else
                     "Mirrored pipe objects" if index == 2 else None),
        )
        axis.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            color="#92400e",
            linewidth=1.2,
            alpha=0.9,
            solid_capstyle="round",
        )


def plot_trajectory(path, save_path=None):
    points, final_distance = load_trajectory(path)
    if not points:
        print(f"Skipping empty trajectory: {path.name}")
        return

    figure, axis = plt.subplots(figsize=(12, 9), constrained_layout=True)
    x_values, y_values = zip(*points)
    axis.plot(
        x_values,
        y_values,
        color="#111827",
        linewidth=1.5,
        label="Vehicle trajectory",
        zorder=2,
    )
    axis.scatter(
        x_values[0], y_values[0], color="#16a34a", s=50, label="Start", zorder=4
    )
    axis.scatter(
        x_values[-1], y_values[-1], color="#dc2626", s=50, label="End", zorder=4
    )
    add_objects(axis)

    distance_text = "unknown" if final_distance is None else f"{final_distance:.2f} m"
    axis.set_title(f"{path.name}\nDistance: {distance_text}")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(save_path, dpi=160)
        print(f"Saved {save_path}")
        plt.close(figure)
    else:
        print(f"Showing {path.name}. Close the plot window for the next trajectory.")
        plt.show()
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Show benchmark trajectories one by one with the inspection objects."
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
        help="Directory containing trajectory_benchmark_*.csv files.",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        help="Save one PNG per trajectory instead of opening plot windows.",
    )
    args = parser.parse_args()

    trajectory_files = sorted(args.results_dir.glob("trajectory_benchmark_*.csv"))
    if not trajectory_files:
        parser.error(f"No trajectory_benchmark_*.csv files found in {args.results_dir}")

    print(f"Found {len(trajectory_files)} trajectories in {args.results_dir}")
    for trajectory_path in trajectory_files:
        save_path = None
        if args.save_dir is not None:
            save_path = args.save_dir / f"{trajectory_path.stem}.png"
        plot_trajectory(trajectory_path, save_path)


if __name__ == "__main__":
    main()
