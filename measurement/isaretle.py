"""
isaretle.py -- Hand-annotate the true target centre in saved frames
(ground truth for the detector-accuracy number in the paper).

Draws a random sample of the JPEGs saved by takip_denemesi.py. For each frame:
    left click   = centre of the red disc
    n            = target not visible in this frame
    u            = undo last
    q            = save and quit (you can resume later; done frames are skipped)

USAGE
    python isaretle.py --kok veri_takip --adet 200
Output: veri_takip/isaretler.csv  (jpg, true_x, true_y, visible)
"""

import argparse
import csv
import glob
import os
import random

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kok', default='veri_takip')
    ap.add_argument('--adet', type=int, default=200)
    ap.add_argument('--tohum', type=int, default=1)
    a = ap.parse_args()

    allj = sorted(glob.glob(os.path.join(a.kok, 'kareler', '*', '*.jpg')))
    random.Random(a.tohum).shuffle(allj)
    sample = allj[:a.adet]
    out = os.path.join(a.kok, 'isaretler.csv')
    done = {}
    if os.path.exists(out):
        for r in csv.DictReader(open(out)):
            done[r['jpg']] = r
    todo = [j for j in sample if os.path.relpath(j, a.kok) not in done]
    print(f"{len(allj)} kare var, orneklem {len(sample)}, kalan {len(todo)}")

    click = {}
    cv2.namedWindow('isaretle')
    cv2.setMouseCallback('isaretle', lambda ev, x, y, fl, p:
                         click.update(xy=(x, y)) if ev == cv2.EVENT_LBUTTONDOWN else None)
    rows = list(done.values())
    i = 0
    while i < len(todo):
        img = cv2.imread(todo[i])
        click.clear()
        while True:
            show = img.copy()
            cv2.putText(show, f"{i + 1}/{len(todo)}  tikla=merkez  n=gorunmuyor  u=geri  q=cik",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if 'xy' in click:
                cv2.drawMarker(show, click['xy'], (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
            cv2.imshow('isaretle', show)
            k = cv2.waitKey(30) & 0xFF
            rel = os.path.relpath(todo[i], a.kok)
            if 'xy' in click:
                rows.append(dict(jpg=rel, true_x=click['xy'][0], true_y=click['xy'][1], visible=1))
                i += 1; break
            if k == ord('n'):
                rows.append(dict(jpg=rel, true_x=-1, true_y=-1, visible=0)); i += 1; break
            if k == ord('u') and rows:
                rows.pop(); i = max(0, i - 1); break
            if k == ord('q'):
                i = len(todo); break
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['jpg', 'true_x', 'true_y', 'visible'])
        w.writeheader(); w.writerows(rows)
    print(f"yazildi: {out} ({len(rows)} kare)")


if __name__ == '__main__':
    main()
