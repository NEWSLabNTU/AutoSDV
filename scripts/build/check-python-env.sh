#!/usr/bin/env bash
# Fails fast, with one short message, when a user-site numpy shadows the apt
# python3-numpy that apt's scipy (and colcon-built packages like range_libc)
# were built against. Left uncaught, this surfaces as a multi-page distutils/
# scipy traceback from deep inside a package's build_ext step, which reads
# like a bug in that package rather than a site-packages ordering problem.
set -euo pipefail

numpy_file=$(python3 -c "
import sys
try:
    import numpy
except ImportError:
    sys.exit(1)
print(numpy.__file__)
" 2>/dev/null) || exit 0

case "$numpy_file" in
    "$HOME"/.local/*)
        echo "error: a user-site numpy shadows the system python3-numpy package." >&2
        echo "  found: $numpy_file" >&2
        echo "  this breaks colcon builds that link against apt scipy/ROS (e.g. range_libc)." >&2
        echo "  fix: pip uninstall numpy   (repeat until none remain under ~/.local)" >&2
        exit 1
        ;;
esac
