"""Print which CCD exit each state in find_red_states.txt takes, using an instrumented package.

Run with PYTHONPATH pointing at a package copy patched by instrument.py.
Usage: label_exits.py find_red_states.txt
"""

import re
import sys

import warp as wp
from scenes import SCENES
from scenes import static_xml

from mujoco_warp import test_data
from mujoco_warp._src.collision_gjk_test import _geom_dist


def main():
  case = None
  for line in open(sys.argv[1]):
    if line.startswith("CASE"):
      case = line.split()[1].rstrip(":")
      continue
    pos = re.search(r'pos="([^"]+)"', line).group(1)
    depth = re.search(r"depth=(\S+)", line).group(1)
    kind, bottom, top = SCENES[case.split("-")[1]]
    _, _, m, d = test_data.fixture(xml=static_xml(kind, bottom, top, pos))
    print(f'STATE {case} depth={depth} pos="{pos}"', flush=True)
    _geom_dist(m, d, 0, 1)
    wp.synchronize()


if __name__ == "__main__":
  main()
