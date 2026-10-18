"""Run aloha_pot and aloha_clutter as benchmarks/run.py configures them, with this venv's testspeed.

benchmarks/run.py runs testspeed through `uv run` in the target checkout; this reuses its benchmark
discovery, asset assembly and flags, but calls the given testspeed so the worktree needs no venv.

Usage: aloha.py <repo> <testspeed> <assets_root>
"""

import argparse
import subprocess
import sys
from pathlib import Path

NAMES = ("aloha_pot", "aloha_clutter")
KEYS = ("steps_per_second", "converged_worlds", "solver_niter_mean", "collision", "convex_narrowphase", "solve")


def main():
  repo, testspeed, assets_root = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
  sys.path.insert(0, str(repo / "benchmarks"))
  import run

  run._ARGS = argparse.Namespace(assets_root=assets_root, clear_warp_cache=False, filter=f"({'|'.join(NAMES)})$")
  for bm in run._discover_benchmarks(repo):
    run._assemble_benchmark(bm)
    root = Path(assets_root) / bm["name"]
    cmd = [testspeed, (root / bm["mjcf"]).as_posix(), "--format=short", "--event_trace=true", "--measure_solver=true"]
    stdout = subprocess.run(cmd + run._bm_flags(bm, root), capture_output=True, text=True, check=True).stdout
    for line in stdout.splitlines():
      if line.split() and line.split()[0].rstrip(":").split(".")[-1].split("[")[0] in KEYS:
        print(f"{bm['name']}.{line}", flush=True)


if __name__ == "__main__":
  main()
