#!/usr/bin/env bash
set +e +u +t +o pipefail 2>/dev/null  # ROS2 setup.bash + inherited SHELLOPTS quirks
# GL2 ros2jazzy reproduction — one-key baseline / coupled / degraded / psnr.
# All metrics MEASURED vs the CBD reference. See docs/GL2_RESULTS.md.
#
# Usage: bash scripts/gl2_reproduce.sh {baseline|coupled|degraded-baseline|degraded-coupled|psnr}

MODE="${1:-baseline}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
export LD_LIBRARY_PATH=/home/frank/Software/libtorch/lib:${LD_LIBRARY_PATH:-}
export PYTORCH_ALLOC_CONF=expandable_segments

REF=$WS/baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum
LICO=$WS/run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LICO.txt
NODE=$WS/install/cocolic/lib/cocolic/odometry_node
MAPBIN=$WS/build/gaussian_lic_mapping/mapping_node
CMP="python3 $WS/scripts/trajectory_compare.py --max-association-dt 0.1 --min-coverage 0.2 --min-matches 30"

# NOTE: never `pkill -f mapping_node` inside this script — it matches the script's own
# cmdline and self-kills. Clean strays from an interactive shell before running.

ate() {  # ate <traj.tum> <out.json>
  $CMP --baseline "$REF" --current "$1" --output "$2" >/dev/null 2>&1
  python3 -c "import json;t=json.load(open('$2'))['translation'];c=json.load(open('$2'));print(f\"ATE rmse={t['rmse_m']*100:.2f}cm mean={t['mean_m']*100:.2f}cm cov={c['coverage']:.2f}\")" 2>/dev/null
}

run_trackA() {  # run_trackA <config>
  rm -f "$LICO"
  "$NODE" "$1" > /tmp/gl2_trackA.log 2>&1
  echo "  track A exit=$? (134=benign teardown terminate after save); dev: $(grep -iE 'Start-to-end deviation' /tmp/gl2_trackA.log | tail -1 | sed 's/^ *//')"
  [ -f "$LICO" ] && grep -qiE 'deviation' /tmp/gl2_trackA.log
}

start_mapper() {  # sets MAP_PID (uses coupled reliable-QoS config)
  "$MAPBIN" --ros-args --params-file "$WS/run_lio/config/cbd_mapper_coupled.yaml" \
      ${1:+-p save_map_render_evaluation:=true} > /tmp/gl2_mapper.log 2>&1 &
  MAP_PID=$!
  for i in $(seq 1 90); do
    ros2 service list 2>/dev/null | grep -q /gaussian_lic/save_map && { echo "  mapper ready (${i}s)"; return 0; }
    kill -0 $MAP_PID 2>/dev/null || { echo "  MAPPER DIED"; return 1; }
    sleep 1
  done
}

case "$MODE" in
  baseline)
    echo "[baseline] track A standalone (coupling OFF)"
    run_trackA "$WS/run_lio/config/ct_odometry_lico_full_baseline.yaml" && { cp "$LICO" /tmp/gl2_baseline.tum; ate /tmp/gl2_baseline.tum /tmp/gl2_baseline_ate.json; } ;;
  coupled)
    echo "[coupled] track A + concurrent mapper + render-photometric"
    start_mapper && { run_trackA "$WS/run_lio/config/ct_odometry_lico_gs_live_full.yaml" && { cp "$LICO" /tmp/gl2_coupled.tum; ate /tmp/gl2_coupled.tum /tmp/gl2_coupled_ate.json; }; kill -9 $MAP_PID 2>/dev/null; } ;;
  degraded-baseline)
    echo "[degraded-baseline] lidar_weight 30, coupling OFF"
    run_trackA "$WS/run_lio/config/ct_odometry_lico_degraded_baseline.yaml" && { cp "$LICO" /tmp/gl2_degbase.tum; ate /tmp/gl2_degbase.tum /tmp/gl2_degbase_ate.json; } ;;
  degraded-coupled)
    echo "[degraded-coupled] lidar_weight 30 + render-photometric w=2.0"
    start_mapper && { run_trackA "$WS/run_lio/config/ct_odometry_lico_degraded_coupled.yaml" && { cp "$LICO" /tmp/gl2_degcoup.tum; ate /tmp/gl2_degcoup.tum /tmp/gl2_degcoup_ate.json; }; kill -9 $MAP_PID 2>/dev/null; } ;;
  psnr)
    echo "[psnr] coupled map -> SaveMap -> renders/gt -> PSNR"
    OUT=$WS/run_lio/data/gl2_psnr_map; rm -rf "$OUT"; mkdir -p "$OUT"
    if start_mapper save; then
      run_trackA "$WS/run_lio/config/ct_odometry_lico_gs_live_full.yaml"
      for i in $(seq 1 20); do n=$(ros2 topic echo /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus --once 2>/dev/null | grep -E "^num_mapping_frames:" | awk '{print $2}'); sleep 2; [ "$i" -ge 4 ] && break; done
      ros2 service call /gaussian_lic/save_map gaussian_lic_msgs/srv/SaveMap "{path: '$OUT/map.ply', include_skybox: false}" 2>&1 | grep -E "success" | head -1
      kill -9 $MAP_PID 2>/dev/null
      echo '{}' > "$OUT/metrics.json"
      python3 "$WS/scripts/eval_render_quality.py" --result-dir "$OUT" --render-dir "$OUT/renders" --reference-dir "$OUT/gt" 2>&1 | grep -E "novel_psnr|train_psnr|matched_pairs" | head
    fi ;;
  *) echo "usage: gl2_reproduce.sh {baseline|coupled|degraded-baseline|degraded-coupled|psnr}"; exit 1 ;;
esac
echo "=== gl2_reproduce [$MODE] done ==="
