
<p align="center">
  <img src="logo/logo_brand_gray.png" width=""/>
  <br>
  <a href="https://newslabntu.github.io/autosdv-book/">
    <strong>Read the Book »</strong>
  </a>
</p>

AutoSDV project provides a recommended build for a small-scale
autonomous vehicle, equipped with practical, industry-standard sensors
and running Autoware, the leading open-source autonomous driving
platform. Designed for research, development, and education, the
platform offers an affordable, modular solution that enables realistic
experimentation and rapid prototyping in autonomous driving
technologies.

<table align="center" border="0">
  <tr>
    <td align="center" valign="bottom">
      <img src="figures/model_robin-w.webp" alt="Robin-W Solid-State LiDAR Kit" width="80%"/>
    </td>
    <td align="center" valign="bottom">
      <img src="figures/model_velodyne_32c.webp" alt="Velodyne 32C LiDAR Kit" width="80%"/>
    </td>
    <td align="center" valign="bottom">
      <img src="figures/model_cube1_moxa-5g.webp" alt="Blickfeld Cube1 + MOXA 5G Kit" width="80%"/>
    </td>
  </tr>
  <tr>
    <td align="center">
      <b>Robin-W Solid-State LiDAR Kit</b>
    </td>
    <td align="center">
      <b>Velodyne 32C LiDAR Kit</b>
    </td>
    <td align="center">
      <b>Cube1 LiDAR + MOXA 5G Kit</b>
    </td>
  </tr>
</table>

## Releases

AutoSDV uses [Semantic Versioning](https://semver.org/). All version dependencies are defined in [`versions.yaml`](versions.yaml).

| Version                                                             | Autoware | CUDA (x86) | JetPack | Status      |
|---------------------------------------------------------------------|----------|------------|---------|-------------|
| [v0.1.0](https://github.com/NEWSLabNTU/AutoSDV/releases/tag/v0.1.0) | 2025.02  | 12.3       | 6.0     | Stable      |
| [develop](https://github.com/NEWSLabNTU/AutoSDV/tree/develop)       | 2025.02  | 12.3       | 6.0     | Development |

### Installation

**Stable release:**
```sh
git clone -b v0.1.0 --recurse-submodules git@github.com:NEWSLabNTU/AutoSDV.git
```

**Development version:**
```sh
git clone -b develop --recurse-submodules git@github.com:NEWSLabNTU/AutoSDV.git
```

### Legacy Releases

Previous releases based on Autoware version naming:
- [2025.02](https://github.com/NEWSLabNTU/F1EIGHTH/tree/2025.02)
- [2024.11](https://github.com/NEWSLabNTU/F1EIGHTH/tree/2024.11)
- [2024.02](https://github.com/NEWSLabNTU/F1EIGHTH/tree/2024.02)

## License

This project is distributed under Apache 2.0 license in the [license
file](LICENSE.txt). If you use this project in your work, please cite
it as follows:

```latex
@misc{autosdv2025,
  author = {Hsiang-Jui Lin, Chi-Sheng Shih},
  title = {AutoSDV: A Software-Defined Vehicle Platform for Research and Education},
  year = {2025},
  institution = {National Taiwan University},
  url = {https://github.com/NEWSLabNTU/AutoSDV},
  note = {Accessed: 2025-04-28}
}
```
