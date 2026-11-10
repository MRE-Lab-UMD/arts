#!/usr/bin/env python3
"""E6 step 1: track the two ArUco markers in a 120 fps video, one row per frame.

    python3 e6_track_markers.py VIDEO.MOV OUT.csv

Markers: DICT_4X4_50, ID 0 on the Haply handle, ID 1 on the arm's adapter, 40 mm sides
(~/arts_local/e6/e6_aruco_markers_letter.pdf). Each row: frame index, time = index / the
container's frame rate (s), then per marker the centre in pixels and the mean side length in
pixels (the mm-per-pixel scale at that marker), NaN where the marker is not found.

Time comes from the frame index, not CAP_PROP_POS_MSEC, which OpenCV 4.6 reports as 0 for
the last frames of an iPhone .MOV. The iPhone records constant frame rate: packet times of
IMG_8593 step 8.337 ms on average, 7.92-8.75 ms (timebase rounding), 2026-10-30.
"""
import csv
import sys

import cv2
import numpy as np

IDS = (0, 1)


def main():
    video, out = sys.argv[1], sys.argv[2]
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    rows, found, i = [], np.zeros(len(IDS), int), 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = i / fps
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), dictionary, parameters=params)
        row = [i, t]
        for k, mid in enumerate(IDS):
            hit = None if ids is None else np.where(ids.ravel() == mid)[0]
            if hit is not None and len(hit):
                c = corners[hit[0]].reshape(4, 2)
                side = np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1))
                row += [*c.mean(axis=0), side]
                found[k] += 1
            else:
                row += [np.nan] * 3
        rows.append(row)
        i += 1
    with open(out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['frame', 't', 'u0', 'v0', 'side0', 'u1', 'v1', 'side1'])
        w.writerows(rows)
    print(f'{video}: {i} frames, marker 0 in {100 * found[0] / i:.1f} %, '
          f'marker 1 in {100 * found[1] / i:.1f} %')


if __name__ == '__main__':
    main()
