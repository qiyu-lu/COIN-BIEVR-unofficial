#!/usr/bin/env python3
"""Renders a top-down view of a map saved with `debug.map_path` (PCD with x, y, z, intensity).

  scripts/render_map.py map.pcd map.png --resolution 0.05 --trajectory traj.txt

Every image pixel shows the mean intensity of the map points above it; pixels without map points
stay black. Points without intensity (0) are ignored.
"""
import argparse

import numpy as np


def load_pcd(path):
    """Loads a binary PCD whose points are four float32 values (x, y, z, intensity)."""
    with open(path, "rb") as f:
        n_points = 0
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("POINTS"):
                n_points = int(line.split()[1])
            if line.startswith("DATA"):
                if line.split()[1] != "binary":
                    raise ValueError("only binary PCD files are supported")
                break
        return np.fromfile(f, dtype=np.float32, count=4 * n_points).reshape(-1, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map", help="map saved by the pipeline (PCD)")
    parser.add_argument("output", help="image to write")
    parser.add_argument("--resolution", type=float, default=0.05, help="pixel size [m]")
    parser.add_argument("--trajectory", default=None, help="TUM trajectory to draw on top")
    parser.add_argument("--crop", type=float, nargs=4, default=None,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"), help="region to render [m]")
    parser.add_argument("--z-range", type=float, nargs=2, default=None, metavar=("ZMIN", "ZMAX"),
                        help="only use points with a height inside this range [m]")
    args = parser.parse_args()

    points = load_pcd(args.map)
    points = points[points[:, 3] > 0]
    if args.z_range:
        points = points[(points[:, 2] >= args.z_range[0]) & (points[:, 2] <= args.z_range[1])]
    if args.crop:
        x0, x1, y0, y1 = args.crop
        keep = ((points[:, 0] >= x0) & (points[:, 0] <= x1) & (points[:, 1] >= y0) &
                (points[:, 1] <= y1))
        points = points[keep]
    else:
        x0, y0 = points[:, :2].min(axis=0)
        x1, y1 = points[:, :2].max(axis=0)
    if len(points) == 0:
        raise SystemExit("no map points with intensity in the selected region")

    width = int(np.ceil((x1 - x0) / args.resolution)) + 1
    height = int(np.ceil((y1 - y0) / args.resolution)) + 1
    u = ((points[:, 0] - x0) / args.resolution).astype(np.int64)
    v = ((points[:, 1] - y0) / args.resolution).astype(np.int64)
    flat = v * width + u
    total = np.bincount(flat, weights=points[:, 3], minlength=width * height)
    count = np.bincount(flat, minlength=width * height)
    image = np.where(count > 0, total / np.maximum(count, 1), 0.0).reshape(height, width)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    scale = min(1.0, 4000.0 / max(width, height))
    fig = plt.figure(figsize=(width * scale / 100.0, height * scale / 100.0), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(image, cmap="gray", vmin=0, vmax=255, origin="lower", extent=(x0, x1, y0, y1),
              interpolation="nearest")
    if args.trajectory:
        trajectory = np.loadtxt(args.trajectory)
        ax.plot(trajectory[:, 1], trajectory[:, 2], color="tab:orange", lw=1.0)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_axis_off()
    fig.savefig(args.output, dpi=100, facecolor="black")
    print("Rendered %d points into a %d x %d image (%s)" % (len(points), width, height,
                                                           args.output))


if __name__ == "__main__":
    main()
