# Internal technical documents

Typst sources. PDFs are build artefacts and are not tracked — compile locally:

```bash
cd docs/tech && typst compile 2d-mcl-localization.typ
```

| Document | Contents |
|---|---|
| `2d-mcl-localization.typ` | AutoSDV 2D-MCL: post-integration node diagram, MCL theory, the six revisions against upstream Roboracer/nav2, and measured five-seed results against NDT |

Figures in `assets/` are generated, not hand-drawn. Regenerate the trajectory
set with `scripts/2dlidar/plot_trajectories.py` (see the document's
"Reproducing" section for the exact invocations).
