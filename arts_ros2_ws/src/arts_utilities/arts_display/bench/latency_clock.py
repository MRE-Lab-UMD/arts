#!/usr/bin/env python3
"""Clock for the glass-to-glass latency test (E7, docs/modules/console_display.md).

Closed loop, no external camera: this clock and the surgeon display's tower view sit side by
side on the screen the scope looks at. Each camera frame then holds two readings: the clock
now, and the clock as the display showed it after one trip through camera, USB, driver,
display node, window system and panel. Their difference is glass-to-glass latency; the
camera's own exposure timing is common to both and cancels. Side by side, not stacked, so the
rolling shutter reads both on the same rows.

The reading is machine-readable: a Gray code of k = floor(t / unit) mod 2^bits, drawn as
2 rows of squares, MSB top-left, white = 1. The copy inside the tower view is small (the
view shows the whole field), so squares survive where digits did not, and 4 bits (2x2, the
default) beat 8 (2x4): the copy squares are twice as wide, and 4 bits x 20 ms still span
320 ms, far above the latency (2026-10-28, 8 bits misread 20-32 % per copy bit). The unit
(20 ms) is longer than one 60 Hz refresh, so consecutive refreshes differ by at most one
step, i.e. one bit: a frame that straddles a refresh reads one of the two, never a mix.
A single reading is quantized to one unit; the display and the cameras are not synchronized,
so the phase is uniform and the MEAN of the difference is unbiased.

  latency_clock.py --x X --y Y [--bits 4] [--square 80] [--unit-ms 20]        q quits
"""
import argparse
import time

import cv2
import numpy as np

WIN = 'ARTS latency clock'


def gray(k):
    return k ^ (k >> 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--x', type=int, default=0, help='window position on the desktop')
    ap.add_argument('--y', type=int, default=0)
    ap.add_argument('--bits', type=int, default=4, choices=(2, 4, 6, 8))
    ap.add_argument('--square', type=int, default=80, help='square size, px')
    ap.add_argument('--gap', type=int, default=10)
    ap.add_argument('--unit-ms', type=float, default=20.0)
    args = ap.parse_args()

    sq, gap, bits = args.square, args.gap, args.bits
    cols = bits // 2
    w, h = cols * sq + (cols + 1) * gap, 2 * sq + 3 * gap
    img = np.zeros((h, w), np.uint8)
    cells = [(gap + c * (sq + gap), gap + r * (sq + gap)) for r in range(2) for c in range(cols)]
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
    placed, n, t_rate = False, 0, time.monotonic()
    while True:
        g = gray(int(time.time() * 1000.0 / args.unit_ms) % (1 << bits))
        img[:] = 0
        for i, (x, y) in enumerate(cells):
            if (g >> (bits - 1 - i)) & 1:
                img[y:y + sq, x:x + sq] = 255
        cv2.imshow(WIN, img)
        if not placed:
            # GNOME ignores the placement of a window with no content (same as arts_display).
            cv2.waitKey(1)
            cv2.moveWindow(WIN, args.x, args.y)
            cv2.resizeWindow(WIN, w, h)
            placed = True
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        n += 1
        if time.monotonic() - t_rate >= 5.0:
            print('clock redraws %.0f /s, window %dx%d' % (n / (time.monotonic() - t_rate),
                                                          w, h), flush=True)
            n, t_rate = 0, time.monotonic()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
