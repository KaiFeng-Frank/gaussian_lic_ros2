#!/usr/bin/env bash
set +u +t  # +u: ROS2 setup.bash has unbound vars; +t: parent exports onecmd
# GL2 step-1: replay track A's mapper_contract bag -> CUDA Gaussian mapper ->
# dump renders/ + gt/ via SaveMap -> compute PSNR. NO fabricated metrics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG=$WS/run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_mapper_contract
PARAMS=$WS/run_lio/config/cbd_mapper.yaml
OUTDIR=$WS/run_lio/data/cbd_gs_map
MAPBIN=$WS/build/gaussian_lic_mapping/mapping_node
LOG=$WS/run_lio/data/gl2_step1_mapper.log

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export LD_LIBRARY_PATH=/home/frank/Software/libtorch/lib:${LD_LIBRARY_PATH:-}
export PYTORCH_ALLOC_CONF=expandable_segments

cleanup() {
  [ -n "${MAP_PID:-}" ] && kill -9 "$MAP_PID" 2>/dev/null
  pkill -9 -f "gaussian_lic_mapping/mapping_node" 2>/dev/null
}
trap cleanup EXIT

rm -rf "$OUTDIR"; mkdir -p "$OUTDIR"

echo "[1/5] launch mapping_node (CUDA)..."
"$MAPBIN" --ros-args --params-file "$PARAMS" \
    -p save_map_render_evaluation:=true > "$LOG" 2>&1 &
MAP_PID=$!

# wait until the node advertises the save service (= fully constructed)
echo "[2/5] wait for /gaussian_lic/save_map service..."
for i in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q "/gaussian_lic/save_map"; then echo "  ready ($i s)"; break; fi
  if ! kill -0 "$MAP_PID" 2>/dev/null; then echo "  MAPPER DIED — log tail:"; tail -25 "$LOG"; exit 1; fi
  sleep 1
done

echo "[3/5] play mapper_contract bag..."
ros2 bag play "$BAG" --rate 0.5 2>&1 | tail -2

echo "[4/5] drain: poll status until mapping frames stop rising..."
prev=-1; stable=0
for i in $(seq 1 90); do
  n=$(timeout 4 ros2 topic echo /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus --once 2>/dev/null \
        | grep -E "^num_mapping_frames:" | awk '{print $2}')
  g=$(timeout 4 ros2 topic echo /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus --once 2>/dev/null \
        | grep -E "^gaussian_optimization_count:" | awk '{print $2}')
  echo "  t=${i}s mapping_frames=${n:-?} opt_count=${g:-?}"
  if [ -n "$n" ] && [ "$n" = "$prev" ]; then stable=$((stable+1)); else stable=0; fi
  [ "$stable" -ge 4 ] && { echo "  drained."; break; }
  prev="$n"; sleep 2
done

echo "[5/5] SaveMap -> renders/gt + PSNR..."
ros2 service call /gaussian_lic/save_map gaussian_lic_msgs/srv/SaveMap \
    "{path: '$OUTDIR/map.ply', include_skybox: false}" 2>&1 | tail -4

echo "=== render eval dirs ==="
ls -la "$OUTDIR" 2>/dev/null
echo "=== PSNR ==="
if [ -d "$OUTDIR/renders" ] && [ -d "$OUTDIR/gt" ]; then
  echo '{}' > "$OUTDIR/metrics.json"
  python3 "$WS/scripts/eval_render_quality.py" \
      --result-dir "$OUTDIR" \
      --render-dir "$OUTDIR/renders" \
      --reference-dir "$OUTDIR/gt" 2>&1 | tail -25
  echo "=== metrics.json ==="; cat "$OUTDIR/metrics.json" 2>/dev/null
else
  echo "NO renders/gt produced — see $LOG"; tail -30 "$LOG"
fi
