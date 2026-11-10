#!/usr/bin/env python3
"""Wrist pitch angle from a photo: the angle between the shaft and the instrument's distal body.

    python3 photo_pitch_angle.py [--crop x0,y0,x1,y1] out.csv photo1.jpg photo2.jpg ...

For each photo, click 8 points, two per edge, in this order:
  1-2 shaft, one edge      3-4 shaft, other edge
  5-6 distal body, one edge  7-8 distal body, other edge
Each pair is a line; a part's direction is the mean of its two edge directions. The result is the
signed angle from the shaft to the distal body in the image plane (degrees, counter-clockwise
positive in the image). Only differences between photos mean anything: the camera must not move,
and the pitch plane should be perpendicular to the view (foreshortening shrinks the angle).
Keys: r = redo this photo, n = accept and go to the next, q = quit. Rows are appended to out.csv
with every click in full-resolution pixels, so the angle can be recomputed. --crop shows only
that region (full-resolution pixels) so the wrist fills the screen.
"""
import csv
import math
import os
import sys

import cv2

WIN = 'photo_pitch_angle'
SCREEN_H = 1000
LABELS = ['shaft edge A', 'shaft edge A', 'shaft edge B', 'shaft edge B',
          'distal edge A', 'distal edge A', 'distal edge B', 'distal edge B']
COLORS = [(255, 128, 0)] * 4 + [(0, 200, 255)] * 4


def direction(p, q):
    # Edge direction as a unit vector, pointing distally (down the image, +y) for averaging.
    dx, dy = q[0] - p[0], q[1] - p[1]
    if dy < 0 or (dy == 0 and dx < 0):
        dx, dy = -dx, -dy
    n = math.hypot(dx, dy)
    return dx / n, dy / n


def part_angle(pts):
    a, b = direction(pts[0], pts[1]), direction(pts[2], pts[3])
    return math.atan2(a[1] + b[1], a[0] + b[0])


def measure(path, crop):
    full = cv2.imread(path)
    if full is None:
        sys.exit(f'cannot read {path}')
    x0, y0, x1, y1 = crop or (0, 0, full.shape[1], full.shape[0])
    img = full[y0:y1, x0:x1]
    scale = SCREEN_H / img.shape[0]
    pts = []

    def redraw():
        view = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        for i, (x, y) in enumerate(pts):
            c = (int(x * scale), int(y * scale))
            cv2.circle(view, c, 4, COLORS[i], -1)
            if i % 2 == 1:
                p = pts[i - 1]
                cv2.line(view, (int(p[0] * scale), int(p[1] * scale)), c, COLORS[i], 1)
        msg = (f'click {len(pts) + 1}/8: {LABELS[len(pts)]}' if len(pts) < 8
               else 'n = accept, r = redo')
        cv2.putText(view, f'{os.path.basename(path)}  {msg}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow(WIN, view)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 8:
            pts.append((x / scale, y / scale))
            redraw()

    cv2.setMouseCallback(WIN, on_mouse)
    redraw()
    while True:
        k = cv2.waitKey(50) & 0xFF
        if k == ord('q'):
            sys.exit('quit')
        if k == ord('r'):
            pts.clear()
            redraw()
        if k == ord('n') and len(pts) == 8:
            shaft, distal = part_angle(pts[:4]), part_angle(pts[4:])
            ang = math.degrees(math.atan2(math.sin(distal - shaft), math.cos(distal - shaft)))
            # image y points down: negate for counter-clockwise positive; clicks back to full res
            return -ang, [(x + x0, y + y0) for x, y in pts]


def main():
    args = sys.argv[1:]
    crop = None
    if args[:1] == ['--crop']:
        crop = tuple(int(v) for v in args[1].split(','))
        args = args[2:]
    if len(args) < 2:
        sys.exit(__doc__)
    out = args[0]
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    fresh = not os.path.exists(out)
    with open(out, 'a', newline='') as f:
        w = csv.writer(f)
        if fresh:
            w.writerow(['photo', 'angle_deg'] + [f'{a}{i}' for i in range(1, 9) for a in 'xy'])
        for path in args[1:]:
            ang, pts = measure(path, crop)
            print(f'{os.path.basename(path)}: {ang:+.2f} deg')
            w.writerow([os.path.basename(path), f'{ang:.3f}'] +
                       [f'{v:.1f}' for p in pts for v in p])
            f.flush()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
