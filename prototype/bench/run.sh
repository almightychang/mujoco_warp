#!/usr/bin/env bash
# Collect the RFC's red tests, drop rates and performance numbers for the worktree's current code.
# Usage: run.sh <label>   (results land in rfc-bench/results/<label>/)
set -uo pipefail

label=${1:?usage: run.sh <label>}
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
bin=${VENV_BIN:-/home/sam/Repos/mujoco_warp/.venv/bin}
ptxas=${PTXAS:-$(dirname "$bin")/lib/python3.14/site-packages/nvidia/cuda_nvcc/bin/ptxas}
out=$here/results/$label
mkdir -p "$out"
export PYTHONPATH=$repo:$here
cd "$here"

echo "== tests" >&2
"$bin/python" -m pytest "$repo/mujoco_warp/_src/collision_gjk_test.py" -q -p no:cacheprovider -k "aligned or settling" >| "$out/tests_red.txt" 2>&1
"$bin/python" -m pytest "$repo/mujoco_warp/_src/collision_gjk_test.py" "$repo/mujoco_warp/_src/collision_driver_test.py" \
  -q -p no:cacheprovider -n 8 >| "$out/tests_collision.txt" 2>&1

echo "== drop rate" >&2
"$bin/python" drop_rate.py >| "$out/drop_rate.txt" 2>/dev/null

echo "== stacks" >&2
"$bin/python" make_stacks.py
for stack in stack_*.xml; do
  "$bin/mjwarp-testspeed" "$stack" --nworld=4096 --nstep=1000 --measure_solver --event_trace --overflow_behavior=continue \
    2>&1 | sed -n '/^Summary/,$p' >| "$out/${stack%.xml}.txt"
done

echo "== aloha" >&2
"$bin/python" aloha.py "$repo" "$bin/mjwarp-testspeed" "$here/.assets" 2>/dev/null >| "$out/aloha.txt"

echo "== ccd kernel resources" >&2
WARP_CACHE_PATH=$out/warp_cache "$bin/python" -c "
import mujoco, mujoco_warp as mjw
for f in ('stack_mesh_aligned.xml', 'stack_box_aligned.xml'):
  mjm = mujoco.MjModel.from_xml_path(f)
  m = mjw.put_model(mjm); d = mjw.put_data(mjm, mujoco.MjData(mjm))
  mjw.forward(m, d)
" >/dev/null 2>&1
for ptx in "$out"/warp_cache/*/wp_ccd_kernel_builder*/*.ptx; do
  "$ptxas" -arch=sm_120 -v "$ptx" -o /dev/null 2>&1 | grep -E "Compiling|registers|spill" | sed 's/ptxas info *: //'
done >| "$out/ccd_resources.txt"
rm -rf "$out/warp_cache"

echo "results in $out" >&2
