#!/usr/bin/env bash
S=${SCRATCH:?directory with the stack XMLs and checkouts of 2310f42 at $SCRATCH/wt-hill and 1e5956f at $SCRATCH/wt-diag}
B=/home/sam/Repos/mujoco_warp/.venv/bin
declare -A ROOT=([hill]=$S/wt-hill [diag]=$S/wt-diag [greedy]=$(cd "$(dirname "$0")/../.." && pwd))
for round in 1 2; do
  for scene in stack_box_aligned stack_mesh_aligned stack_cylinder; do
    for v in hill diag greedy; do
      out=$(PYTHONPATH=${ROOT[$v]} $B/mjwarp-testspeed $S/$scene.xml --nworld=4096 --nstep=500 --event_trace --measure_solver --overflow_behavior=continue 2>&1)
      echo "round $round $scene $v sps=$(echo "$out" | grep 'steps per second' | awk '{print $NF}') convex_narrowphase=$(echo "$out" | grep 'convex_narrowphase:' | awk '{print $2}')"
    done
  done
  for v in hill diag greedy; do
    cd /home/sam/Repos/mujoco_warp-d/rfc-bench
    PYTHONPATH=${ROOT[$v]}:. $B/python aloha.py ${ROOT[$v]} $B/mjwarp-testspeed /home/sam/Repos/mujoco_warp-d/rfc-bench/.assets 2>/dev/null | grep -E "steps_per_second|convex_narrowphase" | sed "s/^/round $round $v /"
  done
done
