"""Compare multicontact point selections by what the solver needs, not by exact maximum area.

For random float32 contact polygons (edge points and near-duplicates as clipping produces), each
selection is scored on: area shortfall from the exhaustive maximum, picking the same vertex
twice, and coherence, i.e. whether the selected points move by more than 1e-4 when every vertex is
perturbed by rounding-level noise (1e-8). Selections: the original hill climbing, the diagonal
scan, an engine-style greedy (support point along a fixed direction, the point farthest from it,
then the farthest point on each side of that segment, as in Jolt's PruneContactPoints), and ear
decimation (drop the vertex with the smallest triangle with its neighbors until 4 remain, as in
Visvalingam-Whyatt line simplification).

Usage: quad_coherence.py [ntrial]
"""

import sys

import numpy as np
from quad_check import area4
from quad_check import diagonal_scan
from quad_check import exhaustive
from quad_check import hill_climb
from quad_check import random_polygon

FIXED_DIR = np.array([0.6, 0.8, 0.0])


def plane_normal(p):
  return sum(np.cross(p[i].astype(np.float64), p[(i + 1) % len(p)].astype(np.float64)) for i in range(len(p)))


def greedy(p):
  """Anchored greedy as in Gregorius (GDC 2015): support point along a fixed direction, the point
  farthest from it, the point making the largest triangle with them, then the point that adds the
  most area outside the triangle."""
  normal = plane_normal(p)
  a = int(np.argmax(p @ FIXED_DIR))
  b = int(np.argmax(np.sum((p - p[a]) ** 2, axis=1)))
  c = int(np.argmax(np.abs(np.cross(p[b] - p[a], p - p[a]) @ normal)))
  tri = (a, b, c) if np.cross(p[b] - p[a], p[c] - p[a]) @ normal > 0 else (a, c, b)
  added = np.zeros(len(p))
  for i, j in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
    added = np.maximum(added, -(np.cross(p[j] - p[i], p - p[i]) @ normal))
  return tuple(sorted({a, b, c, int(np.argmax(added))}))


def ear_decimation(p):
  """Removes the vertex whose triangle with its two neighbors has the smallest area, until 4 remain."""
  idx = list(range(len(p)))
  normal = plane_normal(p)

  def ear(k):
    prev, cur, nxt = p[idx[k - 1]], p[idx[k]], p[idx[(k + 1) % len(idx)]]
    return np.cross(cur - prev, nxt - cur) @ normal

  while len(idx) > 4:
    del idx[int(np.argmin([ear(k) for k in range(len(idx))]))]
  return tuple(idx)


def points(p, quad):
  return p[list(quad)].astype(np.float64)


def moved(q1, q2, tol=1e-4):
  """True if some selected point has no counterpart within tol in the other selection."""
  if len(q1) != len(q2):
    return True
  dist = np.linalg.norm(q1[:, None] - q2[None], axis=2)
  return bool((dist.min(axis=1) > tol).any() or (dist.min(axis=0) > tol).any())


def main():
  ntrial = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
  rng = np.random.default_rng(0)
  methods = (
    ("hill climb", hill_climb),
    ("diagonal scan", diagonal_scan),
    ("greedy", greedy),
    ("ear decimation", ear_decimation),
  )
  stats = {name: {"shortfall": [], "dup": 0, "moved": 0} for name, _ in methods}
  for _ in range(ntrial):
    p = random_polygon(rng)
    q = p + rng.normal(scale=1e-8, size=p.shape).astype(np.float32) * np.array([1, 1, 0], np.float32)
    best = area4(p, *exhaustive(p))
    for name, method in methods:
      s1, s2 = method(p), method(q)
      st = stats[name]
      st["dup"] += len(set(s1)) < len(s1) or len(set(s1)) < 4
      if len(set(s1)) == 4:
        st["shortfall"].append((best - area4(p, *s1)) / best)
      else:
        st["shortfall"].append(1.0)
      st["moved"] += moved(points(p, sorted(set(s1))), points(q, sorted(set(s2))))
  print(f"{'selection':14s} {'mean short':>10s} {'p99 short':>10s} {'max short':>10s} {'<4 pts':>7s} {'moved':>6s}")
  for name, st in stats.items():
    s = np.array(st["shortfall"])
    print(
      f"{name:14s} {s.mean():10.2e} {np.percentile(s, 99):10.2e} {s.max():10.2e} "
      f"{st['dup'] / ntrial * 100:6.1f}% {st['moved'] / ntrial * 100:5.1f}%"
    )


if __name__ == "__main__":
  main()
