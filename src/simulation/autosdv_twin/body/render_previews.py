#!/usr/bin/env python3
"""Orthographic previews of car A's body beside the photos they were estimated from.

    python3 render_previews.py OUT.png [--photos DIR]

Draws the body (make_body.py's parts, in base_link) from the right side, the left side, the
front and the rear, with a 5 cm grid, and, with --photos, the matching photo of car A next
to each view: 78ce8998 (right side), 23aca77f (left side), c1aff61f (front), 59a9c2e8 (rear).
The photos are perspective and the previews are not, so compare positions by eye, against
the grid: the wheels, the lidar's height and the ZED's.

Needs make_body.py's dependencies plus matplotlib and Pillow.
"""
import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_body  # noqa: E402

# view name -> (horizontal axis, its sign, depth axis, sign toward the camera, photo id)
VIEWS = {
    "right side": (0, 1, 1, -1, "78ce8998"),
    "left side": (0, -1, 1, 1, "23aca77f"),
    "front": (1, 1, 0, 1, "c1aff61f"),
    "rear": (1, -1, 0, -1, "59a9c2e8"),
}


def draw(ax, h, hs, d, ds):
    polys, cols, depth = [], [], []
    for _, p, m in make_body.all_parts_base_link():
        pts = p.points()
        for f in p.faces:
            q = pts[f]
            polys.append(np.stack([hs * q[:, h], q[:, 2]], 1))
            cols.append(make_body.MATERIALS[m][0])
            depth.append(ds * q[:, d].mean())
    order = np.argsort(depth)                     # far first: the painter's algorithm
    ax.add_collection(PolyCollection([polys[i] for i in order], facecolors=[cols[i] for i in order],
                                     edgecolors="none"))
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(-0.4, 0.45, 0.05), minor=True)
    ax.set_yticks(np.arange(0, 0.45, 0.05), minor=True)
    ax.grid(True, which="both", lw=0.3, alpha=0.5)
    ax.autoscale_view()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out")
    ap.add_argument("--photos")
    a = ap.parse_args()
    cols = 2 if a.photos else 1
    fig, axes = plt.subplots(len(VIEWS), cols, figsize=(6 * cols, 4.5 * len(VIEWS)), squeeze=False)
    for row, (name, (h, hs, d, ds, photo)) in enumerate(VIEWS.items()):
        draw(axes[row, 0], h, hs, d, ds)
        axes[row, 0].set_title(f"body, {name} (m, base_link)")
        if a.photos:
            from PIL import Image, ImageOps
            im = ImageOps.exif_transpose(Image.open(Path(a.photos) / f"{photo}-image.jpg"))
            im.thumbnail((900, 900))
            axes[row, 1].imshow(im)
            axes[row, 1].set_title(f"photo {photo}")
            axes[row, 1].axis("off")
    fig.tight_layout()
    fig.savefig(a.out, dpi=90)
    print(a.out)


if __name__ == "__main__":
    main()
