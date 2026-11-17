# Phase 6: switch the default

- `versions.yaml` default 2.1.0; CLAUDE.md, the guides and the book name 2.1.0; setup's
  default install.
- 1.5.0 stays selectable while any lab tag needs it.
- `feature/autoware-2.1.0` rebased onto develop and fast-forwarded in.

**Exit:** develop builds and runs on 2.1.0 by default, `just test` green on both.
