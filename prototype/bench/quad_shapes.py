"""Area shortfall and coherence of multicontact selections on three kinds of contact polygons.

random: convex polygons with edge points and near-duplicates (quad_check.random_polygon).
symmetric: a regular 16-gon (a cylinder cap); two equal 16-gons rotated by 1e-4..1e-2 (32 pts).
plates: two equal rectangles (#1798's plates) rotated by 1e-5..1e-3 rad and offset by up to 1 um,
clipped against each other, so corners are cut by micrometres and edges cross at shallow angles.
Coherence: selected points move by more than 1e-4 m under 1e-8 noise on every vertex.

Usage: quad_shapes.py [ntrial]
"""

import sys

import numpy as np
from quad_check import area4
from quad_check import diagonal_scan
from quad_check import exhaustive
from quad_check import hill_climb
from quad_check import random_polygon
from quad_coherence import ear_decimation
from quad_coherence import greedy
from quad_coherence import moved
from quad_coherence import points

METHODS = (("hill climb", hill_climb), ("diagonal scan", diagonal_scan), ("greedy", greedy), ("ear decimation", ear_decimation))


def regular(n, r, phase):
  t = phase + 2 * np.pi * np.arange(n) / n
  return np.stack([r * np.cos(t), r * np.sin(t), np.zeros(n)], 1)


def symmetric(rng):
  phase = rng.uniform(0, 2 * np.pi)
  if rng.random() < 0.5:
    return regular(16, 0.05, phase)
  pts = np.concatenate([regular(16, 0.05, phase), regular(16, 0.05, phase + rng.uniform(1e-4, 1e-2))])
  return pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))]


def clip(subject, clipper):
  """Sutherland-Hodgman clip of a convex CCW polygon by another (2D, float64)."""
  out = subject
  for i in range(len(clipper)):
    e0, e1 = clipper[i], clipper[(i + 1) % len(clipper)]
    side = lambda p: (e1[0] - e0[0]) * (p[1] - e0[1]) - (e1[1] - e0[1]) * (p[0] - e0[0])
    pts, out = out, []
    for j in range(len(pts)):
      p, q = pts[j], pts[(j + 1) % len(pts)]
      sp, sq = side(p), side(q)
      if sp >= 0:
        out.append(p)
      if (sp >= 0) != (sq >= 0):
        out.append(p + sp / (sp - sq) * (q - p))
  return np.array(out)


def plates(rng):
  half = np.array([0.105, 0.148])
  rect = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half
  t = rng.uniform(1e-5, 1e-3) * rng.choice([-1, 1])
  rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
  top = rect @ rot.T + rng.uniform(-1e-6, 1e-6, 2)
  poly = clip(top, rect)
  return np.concatenate([poly, np.zeros((len(poly), 1))], 1)


def main():
  ntrial = int(sys.argv[1]) if len(sys.argv) > 1 else 500
  rng = np.random.default_rng(0)
  print(f"{'shapes':10s} {'selection':14s} {'mean short':>10s} {'max short':>10s} {'<4 pts':>7s} {'moved':>6s}  (n)")
  for label, make in (("random", random_polygon), ("symmetric", symmetric), ("plates", plates)):
    stats = {name: [[], 0, 0] for name, _ in METHODS}
    sizes = []
    for _ in range(ntrial):
      p = (make(rng) + 0.4).astype(np.float32)
      if len(p) < 5:
        continue
      sizes.append(len(p))
      q = p + rng.normal(scale=1e-8, size=p.shape).astype(np.float32) * np.array([1, 1, 0], np.float32)
      best = area4(p, *exhaustive(p)) if len(p) <= 20 else area4(p, *diagonal_scan(p))
      for name, method in METHODS:
        s1, s2 = method(p), method(q)
        st = stats[name]
        st[0].append((best - area4(p, *s1)) / best if len(set(s1)) == 4 else 1.0)
        st[1] += len(set(s1)) < 4
        st[2] += moved(points(p, sorted(set(s1))), points(q, sorted(set(s2))))
    total = len(sizes)
    for name, (short, few, mv) in stats.items():
      short = np.array(short)
      print(
        f"{label:10s} {name:14s} {short.mean():10.2e} {short.max():10.2e} {few / total * 100:6.1f}% "
        f"{mv / total * 100:5.1f}%  ({np.mean(sizes):.1f})"
      )


if __name__ == "__main__":
  main()
