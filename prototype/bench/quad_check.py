"""Check the pointer-based maximum-area quadrilateral against exhaustive search, on the CPU.

NumPy ports of three selections are compared with all 4-subsets on random convex polygons in
float32, with extra vertices on edges, near-duplicate vertices, and the 16-gon counterexample of
Keikha et al. to the Dobkin-Snyder algorithm: the diagonal scan (_polygon_quad: for each diagonal
ac, the farthest b and d), the same with forward-only pointers, and the original hill climbing.
Reports the worst relative area shortfall of each.

Usage: quad_check.py [ntrial]
"""

import itertools
import sys

import numpy as np

KEIKHA_16 = [
  (26096, 6750), (26130, 9933), (25940, 10728), (23090, 22189), (18106, 23681), (13484, 24407),
  (13174, 24343), (3090, 22189), (0, 17308), (80, 14350), (323, 13331), (3090, 2189), (8459, 385),
  (12837, 0), (13392, 114), (23090, 2189),
]  # fmt: skip


def area4(p, a, b, c, d):
  return 0.5 * np.linalg.norm(np.cross(p[a] - p[c], p[b] - p[d]))


def exhaustive(p):
  return max(itertools.combinations(range(len(p)), 4), key=lambda q: area4(p, *q))


def farthest_scan(p, start, end, origin, direction):
  dist2 = [np.sum(np.cross(p[i] - origin, direction) ** 2) for i in range(start, end)]
  return start + int(np.argmax(dist2)), np.sqrt(max(dist2))


def diagonal_scan(p):
  n, best, res = len(p), -1.0, (0, 1, 2, 3)
  for a in range(n - 3):
    for c in range(a + 2, n - 1):
      ac = p[c] - p[a]
      b, abc = farthest_scan(p, a + 1, c, p[a], ac)
      d, acd = farthest_scan(p, c + 1, n, p[a], ac)
      if abc + acd > best:
        best, res = abc + acd, (a, b, c, d)
  return res


def farthest(p, idx, end, origin, direction):
  dist2 = np.dot(np.cross(p[idx] - origin, direction), np.cross(p[idx] - origin, direction))
  while idx + 1 < end:
    v = np.cross(p[idx + 1] - origin, direction)
    if v @ v < dist2:
      break
    idx += 1
    dist2 = v @ v
  return idx, np.sqrt(dist2)


def pointers(p):
  n, best, res = len(p), -1.0, (0, 1, 2, 3)
  for a in range(n - 3):
    b, d = a + 1, a + 3
    for c in range(a + 2, n - 1):
      ac = p[c] - p[a]
      b, abc = farthest(p, b, c, p[a], ac)
      d, acd = farthest(p, max(d, c + 1), n, p[a], ac)
      if abc + acd > best:
        best, res = abc + acd, (a, b, c, d)
  return res


def hill_climb(p):
  n = len(p)
  b, c, d = 1, 2, 3
  res, m = (0, b, c, d), area4(p, 0, b, c, d)
  for a in range(n):
    while True:
      m_next = area4(p, a, b, c, (d + 1) % n)
      if m_next <= m:
        break
      m, d, res = m_next, (d + 1) % n, (a, b, c, (d + 1) % n)
      while True:
        m_next = area4(p, a, b, (c + 1) % n, d)
        if m_next <= m:
          break
        m, c, res = m_next, (c + 1) % n, (a, b, (c + 1) % n, d)
      while True:
        m_next = area4(p, a, (b + 1) % n, c, d)
        if m_next <= m:
          break
        m, b, res = m_next, (b + 1) % n, (a, (b + 1) % n, c, d)
    if b == a:
      b = (b + 1) % n
      if c == b:
        c = (c + 1) % n
        if d == c:
          d = (d + 1) % n
  return res


def random_polygon(rng):
  n = rng.integers(5, 17)
  angle = np.sort(rng.uniform(0, 2 * np.pi, n))
  p = np.stack([np.cos(angle) * rng.uniform(0.05, 0.3), np.sin(angle) * rng.uniform(0.05, 0.3), np.zeros(n)], 1)
  extra = []
  for i in range(n):
    extra.append(p[i])
    if rng.random() < 0.3:  # a vertex on the edge, as from clipping two tilted faces
      extra.append(p[i] + rng.uniform(0.05, 0.95) * (p[(i + 1) % n] - p[i]))
    if rng.random() < 0.2:  # a near-duplicate of the next vertex
      extra.append(p[(i + 1) % n] + rng.normal(scale=1e-6, size=3) * [1, 1, 0])
  return np.array(extra[:32], dtype=np.float32) + np.float32(0.4)


def shortfall(p, quad, best):
  if len(set(quad)) < 4:
    return 1.0
  return float((best - area4(p, *quad)) / best)


def main():
  ntrial = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
  rng = np.random.default_rng(0)
  cases = [np.array([(x, y, 0) for x, y in KEIKHA_16], dtype=np.float32)]
  cases += [random_polygon(rng) for _ in range(ntrial)]
  methods = (("diagonal scan", diagonal_scan), ("pointers", pointers), ("hill climb", hill_climb))
  worst = {name: 0.0 for name, _ in methods}
  fails = {name: 0 for name, _ in methods}
  for i, p in enumerate(cases):
    best = area4(p, *exhaustive(p))
    for name, method in methods:
      s = shortfall(p, method(p), best)
      if i == 0:
        print(f"Keikha 16-gon, {name}: shortfall {s:.2e}")
      worst[name] = max(worst[name], s)
      fails[name] += s > 1e-5
  for name in worst:
    print(f"{name}: worst shortfall {worst[name]:.2e}, cases > 1e-5: {fails[name]} / {len(cases)}")


if __name__ == "__main__":
  main()
