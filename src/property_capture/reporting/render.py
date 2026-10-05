"""Dimensioned floor plan PNG (OpenCV only)."""
import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
ROOM_COLOURS = [(180, 119, 31), (14, 127, 255), (44, 160, 44), (40, 39, 214), (189, 103, 148),
                (75, 86, 140), (194, 119, 227), (34, 189, 188), (207, 190, 23), (127, 127, 127)]


def _centroid(poly):
    x, y = poly[:, 0], poly[:, 1]
    cross = x * np.roll(y, -1) - np.roll(x, -1) * y
    a = cross.sum() / 2
    if abs(a) < 1e-9:
        return poly.mean(axis=0)
    return np.array([((x + np.roll(x, -1)) * cross).sum(), ((y + np.roll(y, -1)) * cross).sum()]) / (6 * a)


def _text(img, text, centre, scale, colour, thickness=1):
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, thickness)
    org = (int(centre[0] - tw / 2), int(centre[1] + th / 2))
    cv2.putText(img, text, org, FONT, scale, (255, 255, 255), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, colour, thickness, cv2.LINE_AA)


def render_floorplan(path, plan, title_lines, px_per_m=120, margin=110):
    rooms = plan.get('rooms') or []
    space = plan['room']
    cam = plan['camera_2d']
    polys = [r['polygon'] for r in rooms] or [space['polygon']]
    allp = np.vstack(polys + [space['polygon'], cam])
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    top_pad = 22 * len(title_lines) + 20
    w = int((hi[0] - lo[0]) * px_per_m) + 2 * margin
    h = int((hi[1] - lo[1]) * px_per_m) + 2 * margin + top_pad
    img = np.full((h, w, 3), 255, np.uint8)

    def to_px(p):
        p = np.atleast_2d(p)
        return np.stack([margin + (p[:, 0] - lo[0]) * px_per_m,
                         top_pad + margin + (hi[1] - p[:, 1]) * px_per_m], axis=1)

    g = space['grid']
    for counts, colour in ((space['floor_counts'], (235, 235, 235)), (space['wall_counts'], (170, 170, 170))):
        r_, c_ = np.nonzero(counts)
        cells = (np.stack([c_, r_], axis=1) + 0.5) * g['cell_m'] + g['origin']
        px = np.rint(to_px(cells)).astype(int)
        ok = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
        img[px[ok, 1], px[ok, 0]] = colour

    for wl in plan.get('wall_detection', {}).get('wall_lines_geometry', []):
        a, b = np.rint(to_px(np.array([wl['start'], wl['end']]))).astype(int)
        cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (200, 200, 245), 3, cv2.LINE_AA)
    cv2.polylines(img, [np.rint(to_px(cam)).astype(np.int32).reshape(-1, 1, 2)], False, (200, 140, 60), 1, cv2.LINE_AA)
    if len(rooms) > 1:
        cv2.polylines(img, [np.rint(to_px(space['polygon'])).astype(np.int32).reshape(-1, 1, 2)], True,
                      (190, 190, 190), 1, cv2.LINE_AA)

    for k, r in enumerate(rooms or [{'polygon': space['polygon'], 'walls': space['walls'], 'room_id': 'room-00',
                                     'area_m2': space['area_m2'], 'visited': True, 'ceiling': {}}]):
        colour = ROOM_COLOURS[k % len(ROOM_COLOURS)]
        ppoly = np.rint(to_px(r['polygon'])).astype(np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, [ppoly.reshape(-1, 1, 2)], colour)
        img = cv2.addWeighted(overlay, 0.10, img, 0.90, 0)
        cv2.polylines(img, [ppoly.reshape(-1, 1, 2)], True, colour, 2, cv2.LINE_AA)
        for wall in r['walls']:
            a, b = np.array(wall['start']), np.array(wall['end'])
            if wall['length_m'] < 0.3:
                continue
            d = b - a
            n_out = np.array([d[1], -d[0]]) / np.linalg.norm(d)
            inferred = wall['evidence'] != 'observed'
            _text(img, f"{wall['length_m']:.2f}" + ('*' if inferred else ''), to_px((a + b) / 2 + n_out * 0.15)[0],
                  0.4, (0, 110, 220) if inferred else (0, 0, 0))
        c = to_px(_centroid(r['polygon']))[0]
        lines = [r['room_id'], f"{r['area_m2']:.1f} m2"]
        ch = (r.get('ceiling') or {}).get('height_m')
        if ch is not None:
            lines.append(f"ceil {ch:.2f} m ({(r['ceiling'].get('coverage') or 0):.0%})")
        if not r['visited']:
            lines.append('not entered')
        for i, line in enumerate(lines):
            _text(img, line, (c[0], c[1] + 18 * (i - (len(lines) - 1) / 2)), 0.5, colour, 1)

    for sw in plan.get('shared_walls') or []:
        for cl in sw['centrelines']:
            a, b = np.rint(to_px(np.array(cl))).astype(int)
            cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (60, 60, 60), 1, cv2.LINE_AA)
        longest = max(sw['centrelines'], key=lambda cl: np.linalg.norm(np.subtract(cl[1], cl[0])))
        label = (f"t {100 * sw['thickness_m']:.0f} cm" if sw['thickness_m'] is not None
                 else f"t ~{100 * sw['candidate_thickness_m']:.0f} cm?")
        _text(img, label, to_px(np.mean(longest, axis=0))[0], 0.38, (60, 60, 60))

    for conn in plan.get('connections') or []:
        p = to_px(conn['location'])[0].astype(int)
        if conn['type'] == 'opening':
            cv2.circle(img, (int(p[0]), int(p[1])), 7, (0, 0, 200), 2, cv2.LINE_AA)
            width = conn['opening_width_m']
            _text(img, f"{width:.2f} m" if width is not None else 'passage', (p[0], p[1] - 16), 0.42, (0, 0, 200))
        else:
            cv2.drawMarker(img, (int(p[0]), int(p[1])), (120, 120, 120), cv2.MARKER_TILTED_CROSS, 10, 2)

    for k, line in enumerate(title_lines):
        cv2.putText(img, line, (12, 24 + 22 * k), FONT, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    x0, y0 = 12, h - 40
    cv2.line(img, (x0, y0), (x0 + px_per_m, y0), (0, 0, 0), 2)
    cv2.putText(img, '1 m', (x0 + px_per_m + 6, y0 + 5), FONT, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, '* inferred wall. Red circle: opening between rooms (width). Grey x: shared wall. '
                     'Blue: camera path. All values UNCALIBRATED.', (12, h - 14), FONT, 0.42, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
