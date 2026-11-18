# Phase 6: switch the default

The SDV course runs Autoware 1.5.0 from `develop` this semester, so the switch is two steps.

**6a. Merge, 1.5.0 still the default** (after phase 5's exit):

- `feature/autoware-2.1.0` rebased onto develop and fast-forwarded in.
- `versions.yaml` default stays 1.5.0; 2.1.0 is opt-in (`.autoware-version` or
  `AUTOSDV_AUTOWARE_VERSION`). Nothing a course user runs changes: phase 4 showed the
  1.5.0 path unchanged (Lab 2 Part 1 grades identical, RC smoke, Lab 3 lap, integration test).
- Exit: `just test` green on both versions; a fresh 1.5.0 checkout of develop builds and
  runs the lab recipes as before.

**6b. Switch the default** (after the course ends):

- `versions.yaml` default 2.1.0; CLAUDE.md, the guides and the book name 2.1.0; setup's
  default install.
- 1.5.0 stays selectable while any lab tag needs it.
- Exit: develop builds and runs on 2.1.0 by default, `just test` green on both.
