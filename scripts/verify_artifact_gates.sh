#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_DIR="${GAUSSIAN_LIC_ARTIFACT_DIR:-/tmp/gaussian_lic_artifact_gates}"
cd "${ROOT_DIR}"
rm -rf "${ARTIFACT_DIR}"
mkdir -p "${ARTIFACT_DIR}"

echo "[artifact] trajectory comparison"
cat >/tmp/gaussian_lic_baseline.tum <<'EOF'
0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.100000000 1.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.200000000 2.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
EOF
cat >/tmp/gaussian_lic_current.tum <<'EOF'
0.000000000 0.002000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.100000000 1.003000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.200000000 2.004000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
EOF
./scripts/trajectory_compare.py \
  --baseline /tmp/gaussian_lic_baseline.tum \
  --current /tmp/gaussian_lic_current.tum \
  --output "${ARTIFACT_DIR}/trajectory_compare.json" \
  --max-rmse-m 0.01 \
  --max-mean-m 0.01 \
  --max-error-m 0.01
grep -q '"ok": true' "${ARTIFACT_DIR}/trajectory_compare.json"
grep -q '"current_to_baseline_ratio"' "${ARTIFACT_DIR}/trajectory_compare.json"
grep -q '"signed_relative_drift"' "${ARTIFACT_DIR}/trajectory_compare.json"
./scripts/trajectory_compare.py \
  --baseline /tmp/gaussian_lic_baseline.tum \
  --current /tmp/gaussian_lic_current.tum \
  --output "${ARTIFACT_DIR}/trajectory_compare_offset_sweep.json" \
  --max-rmse-m 0.01 \
  --max-mean-m 0.01 \
  --max-error-m 0.01 \
  --time-offset-sweep-min -0.1 \
  --time-offset-sweep-max 0.1 \
  --time-offset-sweep-step 0.1
grep -q '"best_offset_sec"' "${ARTIFACT_DIR}/trajectory_compare_offset_sweep.json"
grep -q '"best_rmse_m"' "${ARTIFACT_DIR}/trajectory_compare_offset_sweep.json"
grep -q '"candidate_count": 3' "${ARTIFACT_DIR}/trajectory_compare_offset_sweep.json"

set +e
./scripts/trajectory_compare.py \
  --baseline /tmp/gaussian_lic_baseline.tum \
  --current /tmp/gaussian_lic_current.tum \
  --output "${ARTIFACT_DIR}/trajectory_compare_path_ratio_expected_fail.json" \
  --max-rmse-m 10 \
  --max-mean-m 10 \
  --max-error-m 10 \
  --max-path-drift 1 \
  --min-current-path-ratio 1.01 \
  >"${ARTIFACT_DIR}/trajectory_compare_path_ratio_expected_fail.log" 2>&1
path_ratio_status=$?
set -e
if [ "${path_ratio_status}" -eq 0 ]; then
  echo "trajectory_compare path-ratio gate should fail for compressed/short current trajectory" >&2
  exit 1
fi
grep -q 'current/reference path ratio' \
  "${ARTIFACT_DIR}/trajectory_compare_path_ratio_expected_fail.json"

cat >/tmp/gaussian_lic_long_baseline.tum <<'EOF'
0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.100000000 1.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.200000000 2.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.300000000 3.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.400000000 4.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
EOF
cat >/tmp/gaussian_lic_window_current.tum <<'EOF'
0.200000000 2.002000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.300000000 3.003000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
0.400000000 4.004000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
EOF
set +e
./scripts/trajectory_compare.py \
  --baseline /tmp/gaussian_lic_long_baseline.tum \
  --current /tmp/gaussian_lic_window_current.tum \
  --output "${ARTIFACT_DIR}/trajectory_compare_all_coverage.json" \
  --min-coverage 0.8 \
  --max-rmse-m 0.01 \
  --max-mean-m 0.01 \
  --max-error-m 0.01 \
  >"${ARTIFACT_DIR}/trajectory_compare_all_coverage_expected_fail.log" 2>&1
coverage_all_status=$?
set -e
if [ "${coverage_all_status}" -eq 0 ]; then
  echo "trajectory_compare all-coverage gate should fail for clipped current trajectory" >&2
  exit 1
fi
grep -q '"coverage_mode": "all"' "${ARTIFACT_DIR}/trajectory_compare_all_coverage.json"
grep -q '"coverage_denominator_poses": 5' "${ARTIFACT_DIR}/trajectory_compare_all_coverage.json"
./scripts/trajectory_compare.py \
  --baseline /tmp/gaussian_lic_long_baseline.tum \
  --current /tmp/gaussian_lic_window_current.tum \
  --output "${ARTIFACT_DIR}/trajectory_compare_overlap_coverage.json" \
  --coverage-mode overlap \
  --min-coverage 0.99 \
  --max-rmse-m 0.01 \
  --max-mean-m 0.01 \
  --max-error-m 0.01
grep -q '"ok": true' "${ARTIFACT_DIR}/trajectory_compare_overlap_coverage.json"
grep -q '"coverage_denominator_poses": 3' "${ARTIFACT_DIR}/trajectory_compare_overlap_coverage.json"

echo "[artifact] point cloud comparison"
cat >/tmp/gaussian_lic_baseline.ply <<'EOF'
ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
0.000000000 0.000000000 0.000000000 255 32 16
1.000000000 0.000000000 0.000000000 255 32 16
2.000000000 0.000000000 0.000000000 255 32 16
EOF
cat >/tmp/gaussian_lic_current.ply <<'EOF'
ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
0.001000000 0.000000000 0.000000000 255 32 16
1.002000000 0.000000000 0.000000000 255 32 16
2.003000000 0.000000000 0.000000000 255 32 16
EOF
./scripts/pointcloud_compare.py \
  --baseline /tmp/gaussian_lic_baseline.ply \
  --current /tmp/gaussian_lic_current.ply \
  --output "${ARTIFACT_DIR}/pointcloud_compare.json" \
  --voxel-size 0 \
  --max-nearest-m 0.01 \
  --max-centroid-drift-m 0.01 \
  --max-chamfer-rmse-m 0.01 \
  --max-chamfer-mean-m 0.01 \
  --max-chamfer-max-m 0.01
grep -q '"ok": true' "${ARTIFACT_DIR}/pointcloud_compare.json"

echo "[artifact] baseline manifest"
rm -rf /tmp/gaussian_lic_baseline_manifest
mkdir -p /tmp/gaussian_lic_baseline_manifest/renders
cp /tmp/gaussian_lic_baseline.tum /tmp/gaussian_lic_baseline_manifest/trajectory.tum
cp /tmp/gaussian_lic_baseline.ply /tmp/gaussian_lic_baseline_manifest/point_cloud.ply
printf '{"tracking_hz": 10.0, "mapping_hz": 10.0, "mean_iteration_ms": 1.0}\n' \
  >/tmp/gaussian_lic_baseline_manifest/metrics.json
printf 'synthetic baseline run\n' >/tmp/gaussian_lic_baseline_manifest/run.log
printf 'synthetic render placeholder\n' >/tmp/gaussian_lic_baseline_manifest/renders/frame_000001.txt
./scripts/baseline_manifest.py \
  --baseline /tmp/gaussian_lic_baseline_manifest \
  --sequence synthetic_verify \
  --write \
  --json \
  >"${ARTIFACT_DIR}/baseline_manifest_stdout.json"
grep -q '"ok": true' /tmp/gaussian_lic_baseline_manifest/baseline_manifest.json
grep -q '"trajectory_poses": 3' /tmp/gaussian_lic_baseline_manifest/baseline_manifest.json
grep -q '"point_cloud_vertices": 3' /tmp/gaussian_lic_baseline_manifest/baseline_manifest.json
cp /tmp/gaussian_lic_baseline_manifest/baseline_manifest.json \
  "${ARTIFACT_DIR}/baseline_manifest.json"

echo "[artifact] baseline readiness"
rm -rf /tmp/gaussian_lic_dataset_root /tmp/gaussian_lic_current_readiness
mkdir -p /tmp/gaussian_lic_dataset_root /tmp/gaussian_lic_current_readiness
printf 'synthetic bag placeholder\n' >/tmp/gaussian_lic_dataset_root/synthetic_verify.bag
cp /tmp/gaussian_lic_current.tum /tmp/gaussian_lic_current_readiness/trajectory.tum
cp /tmp/gaussian_lic_current.ply /tmp/gaussian_lic_current_readiness/point_cloud.ply
printf '{"tracking_hz": 9.8, "mapping_hz": 9.7, "mean_iteration_ms": 1.05}\n' \
  >/tmp/gaussian_lic_current_readiness/metrics.json
./scripts/baseline_readiness.py \
  --dataset-root /tmp/gaussian_lic_dataset_root \
  --baseline-dir /tmp/gaussian_lic_baseline_manifest \
  --current-results-dir /tmp/gaussian_lic_current_readiness \
  --sequence synthetic_verify \
  --output "${ARTIFACT_DIR}/baseline_readiness.json" \
  --markdown "${ARTIFACT_DIR}/baseline_readiness.md"
grep -q '"ok": true' "${ARTIFACT_DIR}/baseline_readiness.json"
grep -q '| FAST-LIVO2 data | PASS |' "${ARTIFACT_DIR}/baseline_readiness.md"

echo "[artifact] reproduction report"
rm -rf /tmp/gaussian_lic_current_report
mkdir -p /tmp/gaussian_lic_current_report
cp /tmp/gaussian_lic_current.tum /tmp/gaussian_lic_current_report/trajectory.tum
cp /tmp/gaussian_lic_current.ply /tmp/gaussian_lic_current_report/point_cloud.ply
printf '{"tracking_hz": 9.8, "mapping_hz": 9.7, "mean_iteration_ms": 1.05}\n' \
  >/tmp/gaussian_lic_current_report/metrics.json
./scripts/reproduction_report.py \
  --baseline-dir /tmp/gaussian_lic_baseline_manifest \
  --current-dir /tmp/gaussian_lic_current_report \
  --sequence synthetic_verify \
  --output "${ARTIFACT_DIR}/reproduction_report.json" \
  --markdown "${ARTIFACT_DIR}/reproduction_report.md" \
  --max-trajectory-rmse-m 0.01 \
  --max-trajectory-mean-m 0.01 \
  --max-trajectory-error-m 0.01 \
  --pointcloud-voxel-size 0 \
  --max-nearest-m 0.01 \
  --max-centroid-drift-m 0.01 \
  --max-chamfer-rmse-m 0.01 \
  --max-chamfer-mean-m 0.01 \
  --max-chamfer-max-m 0.01
grep -q '"ok": true' "${ARTIFACT_DIR}/reproduction_report.json"
grep -q '| metrics | PASS |' "${ARTIFACT_DIR}/reproduction_report.md"

echo "[artifact] rosbag2 timing audit"
rm -rf /tmp/gaussian_lic_timing_bag /tmp/gaussian_lic_timing_bad
mkdir -p /tmp/gaussian_lic_timing_bag /tmp/gaussian_lic_timing_bad
python3 - <<'PY'
import sqlite3
from pathlib import Path

for root, stamps in [
    (Path("/tmp/gaussian_lic_timing_bag"), [100, 200, 300]),
    (Path("/tmp/gaussian_lic_timing_bad"), [100, 300, 200]),
]:
    connection = sqlite3.connect(root / "data_0.db3")
    connection.execute(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, "
        "serialization_format TEXT, offered_qos_profiles TEXT)"
    )
    connection.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    connection.execute("INSERT INTO topics VALUES(1, '/image_for_gs', 'sensor_msgs/msg/Image', 'cdr', '')")
    for index, stamp in enumerate(stamps, start=1):
        connection.execute("INSERT INTO messages VALUES(?, 1, ?, ?)", (index, stamp, b""))
    connection.commit()
    connection.close()
    (root / "metadata.yaml").write_text(
        """rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  relative_file_paths:
    - data_0.db3
  duration:
    nanoseconds: 200
  starting_time:
    nanoseconds_since_epoch: 100
  message_count: 3
  topics_with_message_count:
    - topic_metadata:
        name: /image_for_gs
        type: sensor_msgs/msg/Image
        serialization_format: cdr
        offered_qos_profiles: ''
      message_count: 3
""",
        encoding="utf-8",
    )
PY
./scripts/rosbag2_timing_audit.py \
  --bag /tmp/gaussian_lic_timing_bag \
  --required-topic /image_for_gs \
  --strict-storage \
  --output "${ARTIFACT_DIR}/rosbag2_timing_audit.json"
grep -q '"ok": true' "${ARTIFACT_DIR}/rosbag2_timing_audit.json"
if ./scripts/rosbag2_timing_audit.py \
  --bag /tmp/gaussian_lic_timing_bad \
  --required-topic /image_for_gs \
  --strict-storage >"${ARTIFACT_DIR}/rosbag2_timing_bad.txt" 2>&1; then
  echo "timestamp regression unexpectedly passed" >&2
  exit 1
fi
grep -q 'timestamps regressed' "${ARTIFACT_DIR}/rosbag2_timing_bad.txt"

cat >"${ARTIFACT_DIR}/README.md" <<EOF
# Gaussian-LIC Artifact Gate Reports

Synthetic reports generated by scripts/verify_artifact_gates.sh.

- trajectory_compare.json
- pointcloud_compare.json
- baseline_manifest.json
- baseline_readiness.json
- baseline_readiness.md
- reproduction_report.json
- reproduction_report.md
- rosbag2_timing_audit.json
EOF

echo "[artifact] reports: ${ARTIFACT_DIR}"
echo "[artifact] passed"
