"""Per-lanelet speed limits of a Lab 3 loop: check them, list them, change them.

The lanelet2 file's `speed_limit` tag (km/h) is Autoware's per-segment speed. Every
road lanelet of the loop must carry one, above 0 and at most MAX_KMH (the speed-limits
requirement of `lab3_check_lanelet`).

A team tunes the segment speeds in week 2, after the map is handed in, so the speeds are
the one part of the lanelet file allowed to change: `normalized_osm` drops every
`speed_limit` tag before the map is hashed (mapdir.folder_hash), and `set_speeds` edits
the tags in place, text only, leaving every other byte of the file as it was.

    lab3_lanelet_speed MAP_DIR                       list the lanelets and their limits
    lab3_lanelet_speed MAP_DIR --corners 4           every corner lanelet to 4 km/h
    lab3_lanelet_speed MAP_DIR --straights 8 --lanelet 176=6

A corner is a lanelet whose turn_direction is left or right (Add Joint Lanelet sets it).
Autoware loads the map at launch, so a change applies after the stack is restarted.
"""

import argparse
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

from autosdv_lab3 import mapdir
from autosdv_speed_limit.speed_limit import MAX_KMH

TAG = 'speed_limit'
normalized_osm = mapdir.normalized_osm


def parse_kmh(value) -> Optional[float]:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if v == v else None   # NaN is no number


def refusal(kmh) -> Optional[str]:
    if kmh is None:
        return 'no number'
    if not kmh > 0:
        return f'{kmh:g} km/h is not above 0'
    if kmh > MAX_KMH:
        return f'{kmh:g} km/h is above the {MAX_KMH:g} km/h maximum'
    return None


def is_corner(tags: dict) -> bool:
    return tags.get('turn_direction') in ('left', 'right')


def check(lanelets) -> Tuple[bool, str, str]:
    """(ok, value, reason) for the speed-limits row: every lanelet's limit in range."""
    bad = []
    speeds = []
    for lid in sorted(lanelets):
        raw = lanelets[lid].tags.get(TAG)
        if raw is None:
            bad.append(f'lanelet {lid}: no speed_limit')
            continue
        v = parse_kmh(raw)
        why = refusal(v)
        if why:
            bad.append(f'lanelet {lid}: speed_limit {raw!r}: {why}')
        else:
            speeds.append(v)
    if bad:
        more = f' (+{len(bad) - 3} more)' if len(bad) > 3 else ''
        return False, f'{len(bad)} of {len(lanelets)} lanelets', '; '.join(bad[:3]) + more
    return True, f'{min(speeds):g}-{max(speeds):g} km/h', ''


def _relation_span(text: str, lid: int) -> Tuple[int, int]:
    m = re.search(r'<relation\s+id="%d"[^>]*>' % lid, text)
    if not m:
        raise KeyError(f'no relation {lid}')
    end = text.find('</relation>', m.end())
    if end < 0:
        raise KeyError(f'relation {lid} is not closed')
    return m.start(), end


def set_speeds(text: str, changes: Dict[int, float]) -> str:
    """The file with each lanelet's speed_limit set (replaced, or added before </relation>)."""
    for lid, kmh in changes.items():
        start, end = _relation_span(text, lid)
        block = text[start:end]
        value = f'{kmh:g}'
        new, n = re.subn(r'(<tag\s+k="speed_limit"\s+v=")[^"]*(")', r'\g<1>%s\g<2>' % value, block)
        if n == 0:
            # indent like the relation's other tags
            ind = re.search(r'\n([ \t]*)<tag', block)
            pad = ind.group(1) if ind else '    '
            close_pad = re.search(r'\n([ \t]*)$', block)
            stripped = block[:close_pad.start()] if close_pad else block
            tail = block[close_pad.start():] if close_pad else '\n'
            new = f'{stripped}\n{pad}<tag k="speed_limit" v="{value}"/>{tail}'
        text = text[:start] + new + text[end:]
    return text


def plan_changes(lanelets, corners=None, straights=None, every=None,
                 single: Optional[List[Tuple[int, float]]] = None) -> Dict[int, float]:
    out = {}
    for lid, ll in lanelets.items():
        if every is not None:
            out[lid] = every
        if corners is not None and is_corner(ll.tags):
            out[lid] = corners
        if straights is not None and not is_corner(ll.tags):
            out[lid] = straights
    for lid, kmh in single or []:
        if lid not in lanelets:
            raise KeyError(f'no road lanelet {lid}')
        out[lid] = kmh
    return out


def describe(lanelets) -> str:
    from autosdv_lab3.route import Loop, find_chain
    order, _ = find_chain(lanelets)
    ids = order if order else sorted(lanelets)
    rows = ['lanelet  kind      length  speed_limit']
    for lid in ids:
        ll = lanelets[lid]
        try:
            length = Loop.from_chain(lanelets, [lid]).length
            ls = f'{length:5.1f} m'
        except Exception:   # noqa: BLE001 - a listing, not a check
            ls = '   ?   '
        rows.append(f'{lid:>7}  {"corner" if is_corner(ll.tags) else "straight":<8}  {ls}  '
                    f'{ll.tags.get(TAG, "-")}')
    return '\n'.join(rows)


def _pair(s: str) -> Tuple[int, float]:
    try:
        a, b = s.split('=', 1)
        return int(a), float(b)
    except ValueError:
        raise argparse.ArgumentTypeError(f'{s!r}: expected ID=KMH')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog='lab3_lanelet_speed',
        description='List or set the speed limit of the loop\'s lanelets (km/h, at most '
                    f'{MAX_KMH:g}). Restart the stack afterwards: Autoware loads the map at launch.')
    ap.add_argument('map_dir')
    ap.add_argument('--all', type=float, dest='every', metavar='KMH', help='every lanelet')
    ap.add_argument('--corners', type=float, metavar='KMH', help='every corner lanelet')
    ap.add_argument('--straights', type=float, metavar='KMH', help='every other lanelet')
    ap.add_argument('--lanelet', type=_pair, action='append', metavar='ID=KMH',
                    help='one lanelet (repeatable; applied last)')
    args = ap.parse_args(argv)
    from autosdv_lab3.route import MapError, read_lanelets
    path = os.path.join(args.map_dir, mapdir.LANELET)
    try:
        lanelets = read_lanelets(path)
    except MapError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    try:
        changes = plan_changes(lanelets, args.corners, args.straights, args.every, args.lanelet)
    except KeyError as e:
        print(f'error: {e.args[0]}', file=sys.stderr)
        return 2
    if not changes:
        print(describe(lanelets))
        return 0
    for lid, kmh in sorted(changes.items()):
        why = refusal(kmh)
        if why:
            print(f'error: lanelet {lid}: {why}', file=sys.stderr)
            return 2
    with open(path, encoding='utf-8') as f:
        text = f.read()
    new = set_speeds(text, changes)
    assert normalized_osm(new) == normalized_osm(text), 'only speed_limit tags may change'
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(new)
    os.replace(tmp, path)
    print(describe(read_lanelets(path)))
    print(f'\n{len(changes)} lanelet(s) changed. Restart the stack (terminal 1) to load them; '
          'the map hash is unchanged.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
