# 17. Autoware 2.1.0

**Status:** started 2026-11-17 on branch `feature/autoware-2.1.0`. Phases 0-4 done on the
workstation (one checkout builds either version; perception, the simulators and the lab
graders match 1.5.0); phase 5 blocked on car access. The SDV course runs Autoware 1.5.0 from
`develop`, so the default does not switch on `develop` until the course ends (phase 6).

**Goal:** AutoSDV builds, launches and drives on Autoware 2.1.0, with 1.5.0 still selectable
until the cars and the labs move, and the default switched only when every phase's exit holds.

**Sources.** The golf cart migrated first (NEWSLabNTU/2026-golf-cart, branch
`autoware-2.1.0`): `docs/research/system/autoware-1.5.0-to-2.1.0-changes.md`,
`docs/roadmaps/11-autoware-2.1.0-migration.md`, and the step-10 validation report when it
lands. Our own survey of the two installs is in [0-host-and-install.md](0-host-and-install.md).

## Phases

| # | phase | needs | exit, in one line |
|---|---|---|---|
| 0 | [Host and install](0-host-and-install.md) | — | both versions under `/opt/autoware/<ver>`, models and play_launch ready, 1.5.0 libraries load |
| 1 | [Versioned build](1-versioned-build.md) | 0 | `just build` on either version from one tree, separate workspaces, submodules bumped |
| 2 | [Launch layer](2-launch-layer.md) | 1 | main launch, logging and planning simulations resolve on 2.1.0 with no dead include or missing parameter |
| 3 | [Perception and models](3-perception-and-models.md) | 2 | the perception presets run or refuse with a named error; engines built per version |
| 4 | [Simulation regression](4-simulation-regression.md) | 2, 3 | COSS demo, RC sim, Lab 2 and Lab 3 simulators within 1.5.0's numbers |
| 5 | [On the cars](5-on-the-cars.md) | 4 | a jetpack62 2.1.0 install on one Orin; localization and a lap on car A or B |
| 6 | [Switch the default](6-switch-default.md) | 5, course end | develop merged with 1.5.0 still the default; 2.1.0 the default after the course |

## Global constraints

- **develop stays on 1.5.0** for as long as the SDV course runs: students and TAs use
  `develop`, not only the lab tags, so its default is the course's Autoware. Work lands on `feature/autoware-2.1.0`, rebased
  onto develop (linear history, no merge commits).
- **Both versions build from one tree** until phase 6: the version is chosen before
  anything is sourced, never by mixing underlays. Files that differ per version are selected,
  not duplicated wholesale.
- **The labs are not touched.** Lab 2 and Lab 3 run on 1.5.0 from their released tags
  (`lab2-v2.1`, `lab3-v1`) this semester; released tags are never moved. The cars stay on
  1.5.0 until phase 5 says otherwise and no lab slot is running.
- **Installed Autoware stays official.** Every adaptation is in this repository or its
  submodules (CLAUDE.md, "Important Notes").
- **`just test` passes at the end of every phase**, on both versions from phase 1 on.
