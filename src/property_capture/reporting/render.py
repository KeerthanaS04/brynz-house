"""Dimensioned floor plan PNG (OpenCV only)."""
import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX


def render_floorplan(path, plan, title_lines, px_per_m=120, margin=110):
    room = plan['room']
    poly = room['polygon']
    cam = plan['camera_2d']
    lo = np.minimum(poly.min(axis=0), cam.min(axis=0))
    hi = np.maximum(poly.max(axis=0), cam.max(axis=0))
    top_pad = 22 * len(title_lines) + 20
    w = int((hi[0] - lo[0]) * px_per_m) + 2 * margin
    h = int((hi[1] - lo[1]) * px_per_m) + 2 * margin + top_pad
    img = np.full((h, w, 3), 255, np.uint8)

    def to_px(p):
        p = np.atleast_2d(p)
        x = margin + (p[:, 0] - lo[0]) * px_per_m
        y = top_pad + margin + (hi[1] - p[:, 1]) * px_per_m
        return np.stack([x, y], axis=1)

    g = room['grid']
    for counts, colour in ((room['floor_counts'], (235, 235, 235)), (room['wall_counts'], (170, 170, 170))):
        r, c = np.nonzero(counts)
        cells = (np.stack([c, r], axis=1) + 0.5) * g['cell_m'] + g['origin']
        px = np.rint(to_px(cells)).astype(int)
        ok = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
        img[px[ok, 1], px[ok, 0]] = colour

    for wl in plan.get('wall_detection', {}).get('wall_lines_geometry', []):
        a, b = np.rint(to_px(np.array([wl['start'], wl['end']]))).astype(int)
        cv2.line(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (150, 150, 240), 3, cv2.LINE_AA)
    cv2.polylines(img, [np.rint(to_px(cam)).astype(np.int32).reshape(-1, 1, 2)], False, (200, 140, 60), 1, cv2.LINE_AA)
    ppoly = np.rint(to_px(poly)).astype(np.int32)
    cv2.polylines(img, [ppoly.reshape(-1, 1, 2)], True, (20, 20, 20), 2, cv2.LINE_AA)

    for wall in room['walls']:
        a, b = np.array(wall['start']), np.array(wall['end'])
        d = b - a
        n_out = np.array([d[1], -d[0]]) / np.linalg.norm(d)
        mid = to_px((a + b) / 2 + n_out * 0.18)[0]
        inferred = wall['evidence'] != 'observed'
        text = f"{wall['length_m']:.2f} m" + ('*' if inferred else '')
        (tw, th), _ = cv2.getTextSize(text, FONT, 0.45, 1)
        org = (int(mid[0] - tw / 2), int(mid[1] + th / 2))
        cv2.putText(img, text, org, FONT, 0.45, (0, 110, 220) if inferred else (0, 0, 0), 1, cv2.LINE_AA)

    for k, line in enumerate(title_lines):
        cv2.putText(img, line, (12, 24 + 22 * k), FONT, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    x0, y0 = 12, h - 40
    cv2.line(img, (x0, y0), (x0 + px_per_m, y0), (0, 0, 0), 2)
    cv2.putText(img, '1 m', (x0 + px_per_m + 6, y0 + 5), FONT, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, '* inferred wall (low point support). Blue: camera path. Pink: detected wall lines. '
                     'All values UNCALIBRATED.',
                (12, h - 14), FONT, 0.42, (60, 60, 60), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
