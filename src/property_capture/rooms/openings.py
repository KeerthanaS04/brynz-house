"""Doors, windows and other openings in walls, from camera rays.

A missing patch of wall cannot tell an opening from a part of the wall that was
never seen, and the fused cloud cannot either: on a shared wall the neighbouring
room's points sit behind the whole wall. Rays can: for frames with the camera
inside a room, each depth pixel is a ray from the camera to the surface it hit.
For every wall of that room, an elevation grid (position along the wall x height)
counts
  * wall evidence: the ray ended at the wall surface (|distance to wall| <= band),
  * through evidence: the ray crossed the wall plane and ended beyond it,
    which is only possible through an opening.
A cell is open when it was seen through at least min_through times and at least
through_ratio times as often as it was seen as wall. Open cells are grouped into
candidate openings; a candidate is kept only if there is wall on both sides of it
(jambs), which rejects stretches where the room simply continues into unscanned
space. Openings are classified from their sill and head heights.

Wall frame: start point a, unit direction t along the wall, outward normal n
(room polygons are counter-clockwise, so n = (t_y, -t_x)); the room interior is
at negative distance.
"""
import cv2
import numpy as np
from shapely.geometry import Point, Polygon
from shapely.prepared import prep

from ..geometry.transforms import backproject, transform


class WallGrid:
    """Elevation grid of wall and through evidence for one wall edge."""

    def __init__(self, a, b, h_max, cfg):
        """The grid extends `jamb_extend_m` past both ends of the edge: wall snapping often ends an
        edge at a doorway, so the jamb on the far side lies on the next, collinear stretch."""
        self.a = np.asarray(a, float)
        d = np.asarray(b, float) - self.a
        self.length = float(np.linalg.norm(d))
        self.t = d / self.length
        self.n = np.array([self.t[1], -self.t[0]])
        self.cell = cfg['cell_m']
        self.h_max = h_max
        self.ext = cfg['jamb_extend_m']
        self.shape = (int(np.ceil(h_max / self.cell)), int(np.ceil((self.length + 2 * self.ext) / self.cell)))
        self.wall = np.zeros(self.shape, np.int32)
        self.through = np.zeros(self.shape, np.int32)
        self.cfg = cfg

    def _add(self, grid, s, h):
        i = np.floor(h / self.cell).astype(int)
        j = np.floor((s + self.ext) / self.cell).astype(int)
        ok = (i >= 0) & (i < self.shape[0]) & (j >= 0) & (j < self.shape[1])
        np.add.at(grid, (i[ok], j[ok]), 1)

    def accumulate(self, cam2d, cam_h, p2d, p_h):
        """Add one frame's rays: camera (2D floor position, height) to points (N x 2, N)."""
        dc = (cam2d - self.a) @ self.n
        if dc >= 0:                                   # camera must be on the room side
            return
        sc = (cam2d - self.a) @ self.t
        dp = (p2d - self.a) @ self.n
        sp = (p2d - self.a) @ self.t
        band = self.cfg['wall_band_m']
        hit = np.abs(dp) <= band
        self._add(self.wall, sp[hit], p_h[hit])
        beyond = dp > self.cfg['through_min_beyond_m']
        if beyond.any():
            f = -dc / (dp[beyond] - dc)               # where the ray crosses the wall plane
            self._add(self.through, sc + f * (sp[beyond] - sc), cam_h + f * (p_h[beyond] - cam_h))

    def _through_fraction(self, r0, r1, j):
        if not 0 <= j < self.shape[1]:
            return 0.0
        th, wa = self.through[r0:r1, j].sum(), self.wall[r0:r1, j].sum()
        return float(th / (th + wa)) if th + wa else 0.0

    def openings(self):
        cfg = self.cfg
        open_ = (self.through >= cfg['min_through']) & (self.through >= cfg['through_ratio'] * self.wall)
        k = max(1, int(round(cfg['close_m'] / self.cell)))
        open_ = cv2.morphologyEx(open_.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(open_, connectivity=8)
        wall_seen = self.wall > 0
        out = []
        for c in range(1, n):
            comp = lab == c
            rows = np.flatnonzero(comp.any(axis=1))
            cols_h = comp.sum(axis=0)
            core = np.flatnonzero(cols_h >= cfg['core_column_fraction'] * cols_h.max())
            j0, j1 = int(core.min()), int(core.max())
            r0, r1 = rows.min(), rows.max() + 1
            # Edge cells straddle the jamb (part seen through, part wall) and fail the open test;
            # the through fraction of the column beyond each edge places the edge within that cell.
            ext0 = self._through_fraction(r0, r1, j0 - 1) * self.cell
            ext1 = self._through_fraction(r0, r1, j1 + 1) * self.cell
            # positions along the edge (the grid starts jamb_extend_m before the edge start)
            s_start = j0 * self.cell - ext0 - self.ext
            s_end = (j1 + 1) * self.cell + ext1 - self.ext
            width = s_end - s_start
            height = (rows.max() - rows.min() + 1) * self.cell
            if width < cfg['min_width_m'] or height < cfg['min_height_m']:
                continue
            if not 0.0 <= (s_start + s_end) / 2 <= self.length:
                continue                     # centred on the neighbouring edge: reported there
            sill = rows.min() * self.cell
            head = (rows.max() + 1) * self.cell
            jw = max(1, int(round(cfg['jamb_check_m'] / self.cell)))

            def jamb(lo, hi):
                lo, hi = max(lo, 0), min(hi, self.shape[1])
                return float(wall_seen[r0:r1, lo:hi].any(axis=1).mean()) if hi > lo else 0.0
            left, right = jamb(j0 - jw, j0), jamb(j1 + 1, j1 + 1 + jw)
            bounded = left >= cfg['jamb_min_fraction'] and right >= cfg['jamb_min_fraction']
            if sill <= cfg['door_max_sill_m'] and head >= self.h_max - cfg['passage_head_margin_m']:
                kind = 'passage'             # floor-to-ceiling: an open passage, not a door
            elif sill <= cfg['door_max_sill_m'] and head >= cfg['door_min_head_m']:
                kind = 'door'
            elif sill >= cfg['window_min_sill_m']:
                kind = 'window'
            else:
                kind = 'opening'
            centre_s = (s_start + s_end) / 2
            out.append({
                'type': kind, 'width_m': width, 'height_m': height, 'sill_m': sill, 'head_m': head,
                's_start_m': s_start, 's_end_m': s_end,
                'centre': (self.a + centre_s * self.t).tolist(),
                'start': (self.a + s_start * self.t).tolist(),
                'end': (self.a + s_end * self.t).tolist(),
                'jamb_wall_fraction': [left, right], 'bounded': bounded,
                'through_rays': int(self.through[comp].sum()), 'cells': int(comp.sum()),
                'width_resolution_m': self.cell,
            })
        return out


TYPE_PRIORITY = ('door', 'passage', 'window', 'opening')


def fuse_physical(members, cfg):
    """One physical opening from its detections (one per room it was seen from).

    Each end of a detection is a jamb only if wall was seen beside it; otherwise the opening
    there just ran until the evidence ran out, and its position says nothing. An end's
    position is the mean of the estimates from detections that saw a jamb there; the width
    is measurable only if both ends have at least one. The spread of estimates at an end
    is a consistency check between rooms (no ground truth).
    """
    ref = members[0]
    a = np.asarray(ref['start'], float)
    u = np.asarray(ref['end'], float) - a
    u /= np.linalg.norm(u)
    lo, hi = [], []
    for m in members:
        ends = [((np.asarray(m['start'], float) - a) @ u, m['jamb_wall_fraction'][0]),
                ((np.asarray(m['end'], float) - a) @ u, m['jamb_wall_fraction'][1])]
        ends.sort(key=lambda e: e[0])
        if ends[0][1] >= cfg['jamb_min_fraction']:
            lo.append(ends[0][0])
        if ends[1][1] >= cfg['jamb_min_fraction']:
            hi.append(ends[1][0])
    candidate = float(np.median([m['width_m'] for m in members]))
    kind = next(t for t in TYPE_PRIORITY if any(m['type'] == t for m in members) or t == 'opening')
    out = {'members': [m['opening_id'] for m in members], 'type': kind, 'candidate_width_m': candidate,
           'jamb_estimates': {'low_end': len(lo), 'high_end': len(hi)},
           'end_spread_m': float(max([np.ptp(lo) if len(lo) > 1 else 0.0, np.ptp(hi) if len(hi) > 1 else 0.0])),
           'centre': np.mean([m['centre'] for m in members], axis=0).tolist(),
           'sill_m': float(min(m['sill_m'] for m in members)), 'head_m': float(max(m['head_m'] for m in members))}
    if lo and hi:
        out.update({'width_m': float(np.mean(hi) - np.mean(lo)), 'width_status': 'measured'})
    else:
        missing = 'both ends' if not lo and not hi else 'one end'
        out.update({'width_m': None,
                    'width_status': f'not_measurable: jamb not observed at {missing} (from any room)'})
    return out


def _wall_height(room, cfg):
    cand = (room.get('ceiling') or {}).get('candidate_height_m')
    return cand if cand is not None and 2.0 <= cand <= 6.0 else cfg['default_height_m']


def detect_openings(cap, T, axes, plan, cfg, dcfg):
    """Openings per room from rays of frames whose camera is inside that room."""
    basis = plan['basis']
    e1, e2, up, origin = (np.asarray(basis[k], float) for k in ('e1', 'e2', 'up', 'origin'))
    rooms = plan['rooms']
    grids, shapes = {}, {}
    for r in rooms:
        shapes[r['room_id']] = prep(Polygon(r['polygon']).buffer(0))
        poly = np.asarray(r['polygon'], float)
        hmax = _wall_height(r, cfg)
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            if np.linalg.norm(b - a) >= cfg['min_edge_length_m']:
                grids.setdefault(r['room_id'], {})[i] = WallGrid(a, b, hmax, cfg)

    step = cfg['pixel_step']
    H, W = cap.depth_size[1], cap.depth_size[0]
    v, u = np.mgrid[0:H:step, 0:W:step]
    u, v = u.ravel().astype(float), v.ravel().astype(float)
    ui, vi = u.astype(int), v.astype(int)
    used = {rid: 0 for rid in grids}
    for f in range(0, len(cap), cfg['frame_stride']):
        cam = T[f, :3, 3]
        c2 = np.array([cam @ e1, cam @ e2])
        rid = next((k for k, s in shapes.items() if s.contains(Point(c2))), None)
        if rid is None or rid not in grids:
            continue
        d = cap.load_depth_raw(f)[vi, ui] * dcfg['unit_scale_m']
        ok = (cap.load_confidence(f)[vi, ui] >= cfg['confidence_min']) & (d > dcfg['min_m']) & (d < dcfg['max_m'])
        Pw = transform(T[f], backproject(u[ok], v[ok], d[ok], cap.depth_intrinsics(f), axes))
        p2 = np.column_stack([Pw @ e1, Pw @ e2])
        ph = (Pw - origin) @ up
        ch = float((cam - origin) @ up)
        for g in grids[rid].values():
            g.accumulate(c2, ch, p2, ph)
        used[rid] += 1

    # A candidate is kept if wall is seen on both sides (jambs), or if room segmentation found an
    # opening between rooms at the same place from independent evidence.
    found, unbounded = [], []
    for rid, edges in grids.items():
        for i, g in edges.items():
            for o in g.openings():
                near = [c for c in plan.get('connections') or [] if c['type'] == 'opening'
                        and rid in c['room_ids']
                        and np.linalg.norm(np.subtract(c['location'], o['centre'])) <= cfg['match_connection_m']]
                o.update({'room_id': rid, 'edge': i, 'matches_connection': bool(near),
                          'accepted_by': 'jambs' if o['bounded'] else ('room connection' if near else None)})
                (found if o['accepted_by'] else unbounded).append(o)

    # the same doorway seen from both rooms is one physical opening
    for k, o in enumerate(found):
        o['opening_id'] = f"{o['room_id']}-opening-{k:02d}"
    for o in found:
        mates = [m for m in found if m['room_id'] != o['room_id'] and
                 np.linalg.norm(np.subtract(m['centre'], o['centre'])) <= cfg['match_connection_m']]
        o['same_as'] = [m['opening_id'] for m in mates]
    groups, seen = [], set()
    for o in found:
        if o['opening_id'] in seen:
            continue
        g = [o['opening_id'], *o['same_as']]
        seen.update(g)
        groups.append(g)
    # an opening seen from both rooms gives two independent widths: their difference is an
    # internal consistency check (no ground truth)
    by_id = {o['opening_id']: o for o in found}
    both_sides = [{'openings': g, 'widths_m': [by_id[i]['width_m'] for i in g],
                   'difference_m': float(max(by_id[i]['width_m'] for i in g) - min(by_id[i]['width_m'] for i in g))}
                  for g in groups if len(g) > 1]
    physical = []
    for k, g in enumerate(groups):
        p = fuse_physical([by_id[i] for i in g], cfg)
        p['physical_id'] = f'opening-{k:02d}'
        for i in g:
            by_id[i]['physical_id'] = p['physical_id']
        physical.append(p)
    return found, unbounded, {'frames_used_per_room': used, 'walls_searched': sum(len(e) for e in grids.values()),
                              'physical_openings': len(groups), 'seen_from_both_sides': both_sides,
                              'physical': physical}
