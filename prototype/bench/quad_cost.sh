#!/usr/bin/env bash
# Narrowphase cost of the three _polygon_quad versions, interleaved, two rounds.
S=${SCRATCH:?directory with the three stack XMLs and a checkout of 2310f42 at $SCRATCH/wt-hill}
B=/home/sam/Repos/mujoco_warp/.venv/bin
declare -A ROOT=([hill]=$S/wt-hill [brute]=/home/sam/Repos/mujoco_warp-d2 [diag]=/home/sam/Repos/mujoco_warp-d)
for round in 1 2; do
  for scene in stack_box_aligned stack_mesh_aligned stack_cylinder; do
    for v in hill brute diag; do
      out=$(PYTHONPATH=${ROOT[$v]} $B/mjwarp-testspeed $S/$scene.xml --nworld=4096 --nstep=500 --event_trace --measure_solver --overflow_behavior=continue 2>&1)
      sps=$(echo "$out" | grep "steps per second" | awk '{print $NF}')
      narrow=$(echo "$out" | grep "convex_narrowphase:" | awk '{print $2}')
      echo "round $round $scene $v sps=$sps convex_narrowphase=$narrow"
    done
  done
done
