#!/usr/bin/env bash
# Serve the exported image files over HTTP, and print the address to put on the
# board.
#
#   cd /srv/autosdv && /path/to/serve-images.sh
#   PORT=8080 ./serve-images.sh
#   ./serve-images.sh /srv/autosdv        serve that directory instead of $PWD
#
# Ctrl+C stops it.

set -euo pipefail

PORT="${PORT:-8000}"
DIR="${1:-$PWD}"

[ -d "$DIR" ] || { echo "error: no such directory: $DIR" >&2; exit 1; }
cd "$DIR"

command -v python3 >/dev/null || {
    echo "error: python3 not found; it is what serves the files." >&2
    exit 1
}

# Every global IPv4 address, because the machine handing out the files usually
# has several -- the classroom link, wifi, docker0 -- and only one of them is
# the address a student can reach. Printing them all and letting a human pick
# beats guessing wrong and having fifty people type a dead address.
mapfile -t ADDRS < <(
    ip -4 -o addr show scope global 2>/dev/null \
        | awk '{split($4,a,"/"); print $2, a[1]}' \
        | grep -vE '^(docker|br-|veth|virbr|tun|tap)' \
        || true
)

echo
echo "  Serving: $PWD"
echo

shopt -s nullglob
files=(*.tar.gz *.tar *.txt)
if [ ${#files[@]} -eq 0 ]; then
    echo "  WARNING: no .tar.gz files here. Did export-images.sh run?"
    echo
else
    for f in "${files[@]}"; do
        printf '    %8s  %s\n' "$(du -h "$f" | cut -f1)" "$f"
    done
    echo
fi

if [ ${#ADDRS[@]} -eq 0 ]; then
    echo "  No global IPv4 address found. Is the network up?"
    echo "  Serving anyway on port ${PORT}."
else
    echo "  Students open:"
    echo
    for entry in "${ADDRS[@]}"; do
        iface="${entry%% *}"
        addr="${entry##* }"
        printf '      http://%s:%s        (%s)\n' "$addr" "$PORT" "$iface"
    done
    echo
    echo "  Use the one on the interface the classroom switch is plugged into."
fi

echo
echo "  Ctrl+C to stop."
echo "  ------------------------------------------------------------"
echo

# 0.0.0.0 rather than a chosen address: the point is that every interface
# works, so a student on whichever network reaches it either way.
exec python3 -m http.server "$PORT" --bind 0.0.0.0
