# Release Milestones

## v0.1.0 - Baseline & Infrastructure

Goal: make strict reproduction measurable before the native algorithm port grows.

- Run and archive upstream Gaussian-LIC/Gaussian-LIC2 baseline artifacts for the selected FAST-LIVO2 sequence.
- Add rosbag conversion tooling for ROS1 `.bag` to rosbag2 `.mcap`.
- Keep Jazzy as the primary target and keep GitHub Actions on a Jazzy-only matrix.
- Keep `gaussian_lic_offline` available for non-launch rosbag2 artifact extraction.
- Freeze `/gaussian_lic/status` and `metrics.json` schemas for later CI comparison.
- Add performance regression gates for tracking FPS, mapping FPS, and iteration time.
- Add a trajectory regression gate for timestamp-associated TUM trajectory drift.
- Add a point-cloud regression gate for ASCII PLY map drift.
- Add a baseline manifest generator for archived ROS1 artifact fingerprinting.
- Add a FAST-LIVO2 data fetch/readiness gate for `CBD_Building_01`.
- Add a combined reproduction report that aggregates manifest, metrics, trajectory, and map gates.

Release artifacts:

```text
baseline/fastlivo2/<sequence>/trajectory.tum
baseline/fastlivo2/<sequence>/point_cloud.ply
baseline/fastlivo2/<sequence>/renders/
baseline/fastlivo2/<sequence>/metrics.json
baseline/fastlivo2/<sequence>/run.log
baseline/fastlivo2/<sequence>/baseline_manifest.json
```

Raw FAST-LIVO2 data is checked with:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01
```

## v0.2.0 - Gaussian-LIC2 ROS2 Native Frontend

Goal: the Gaussian-LIC2 frontend/tracking path runs natively in ROS2 without depending on a ROS1 runtime.

- Refactor timestamp math to `int64_t` nanoseconds before ROS2 API replacement.
- Keep `gaussian_lic_frontend/lic2_contract_adapter` as the raw-topic to mapper-contract boundary while the full odometry frontend is being ported.
- Port PointCloud2/Livox LiDAR input, IMU input, camera input, continuous-time trajectory, and odometry output against the released Gaussian-LIC2 upstream surface; if upstream still delegates odometry to legacy code, keep that boundary explicit and native in ROS2.
- Publish `/gaussian_lic/frontend/odometry`, `/gaussian_lic/frontend/path`, TF, and the mapper input topics:
  `/image_for_gs`, `/depth_for_gs`, `/pose_for_gs`, `/points_for_gs`.
- Compare trajectory against the ROS1 baseline with `scripts/trajectory_compare.py` and fail CI when drift exceeds the accepted threshold.
- Compare saved PLY maps against the ROS1 baseline with `scripts/pointcloud_compare.py` once the native map backend is active.

## v0.3.0 - Gaussian Mapping Full Backend

Goal: replace the debug mapping slice with the real Gaussian-LIC backend.

- Port CUDA rasterizer, simple-knn, fused-ssim, optimizer, densification, pruning, and upstream-compatible PLY persistence.
- Change the default render path to `render_mode:=rasterizer` after replacing the interim CPU Gaussian splat preview with the upstream CUDA rasterizer.
- Keep `render_mode:=debug_cpu` only as a temporary diagnostic path and remove it after v0.3 stabilization.
- Keep TensorRT/depth completion optional; `depth_completion:=false` must run without TensorRT installed, and SPNet engine generation must stay outside git.

## v0.4.0 - Strict FAST-LIVO2 Reproduction

Goal: one-command paper-level reproduction on the selected FAST-LIVO2 sequence.

- Run full Gaussian-LIC2 frontend plus Gaussian mapping end to end from rosbag2.
- Generate `metrics.json`, trajectory, Gaussian PLY, rendered images, and comparison report.
- Compare ROS2 outputs against archived ROS1 baseline artifacts with `scripts/reproduction_report.py`.
- Publish release artifacts and expected screenshots/video for README and GitHub release notes.

Status as of 2026-05-05: the mapper-contract/CUDA strict `CBD_Building_01`
chain is executable against the archived ROS1 upstream baseline, and the latest archived strict `CBD_Building_01` report is green. The current artifact is
`results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe`;
trajectory, novel PSNR/SSIM/LPIPS, GT-associated render-pair, and point-cloud
gates pass. Novel-view PSNR regresses 0.39%, point-cloud centroid drift is
0.147855 m, and the bidirectional nearest-neighbor mean distance is 0.099418 m
against the 0.100000 m strict threshold. SPNet TensorRT engine generation is
validated locally with TensorRT 10.9 on `sm_120`.
The native tracking path now has a launch smoke gate and analytic geometric
Jacobians for point-to-point, point-to-plane, and visual-alignment residuals,
plus runtime-gated bias observability and 2-DoF photometric translation
linearization status, dense-prior rank/singular-value marginalization health,
plus analytic SE3 photometric normal-equation and runtime sliding-window factor
coverage from rendered/current/depth images, with robust sample-quality status
gates. Full current IMU preintegration factors, pose-prior, state-prior,
retained dense-prior, IMU bias-continuity, and SE3 photometric window rows now
have analytic Jacobian coverage; full native Coco-LIC2 frontend parity remains
follow-up work.
The release gate for full-dataset parity is now executable via
`scripts/check_strict_parity_matrix.py` and intentionally reports incomplete
status until required native reference trajectory evidence and full-sequence
FAST-LIVO/M2DGR/MCD/R3LIVE strict artifacts are archived. Current local matrix
status is `required=4/9`, with FAST-LIVO2 mapper-contract/CUDA strict parity,
FAST-LIVO hku1/hku2 mapper-contract/CUDA strict parity, and the 120s CBD native
visual/SE3 BA health report passing.
The matching data-side gate is now executable via
`scripts/audit_strict_data_inputs.py`; `docs/strict_data_status.md` records the
current blocker explicitly: raw/frontend inputs are local for every required
profile, disk headroom is back above the 100 GiB audit threshold, and the
remaining blockers are archived ROS1 baseline artifacts plus trusted native
reference trajectories for the profiles still marked pending in the matrix.
R3LIVE `hku_park_00` now also has a frontend-raw conversion plus 60s native
sensor-only tracking health report, but it remains optional runtime coverage
until ROS1-vs-ROS2 strict parity artifacts are archived.
FAST-LIVO2 `Retail_Street` is also fetched, converted to frontend-raw, and
runtime-gated through a 60s native scan-order deskew tracking report; it remains
optional coverage until a matching ROS1 baseline/reference trajectory and strict
current artifact are archived.
MCD acquisition has started through `scripts/fetch_google_drive_file.py`; the
local `ntu_day_01` directory has Livox `mid70` and `vn100` components, and the
data audit keeps MCD failing until the `d435i` camera component and downstream
frontend/baseline/current/reference artifacts are present.
The native tracking launch now defaults the sliding-window BA, visual-alignment
window factors, and SE3 photometric window factors to enabled so ordinary
tracking launch paths exercise the joint optimizer when the corresponding
sensor/render/depth inputs are present.
Optimized velocity and bias now feed back into odometry and safe IMU
propagation re-anchoring, and cubic B-spline pose/velocity trajectory queries
are available as the first deskew lookup source before IMU-history fallback.
`TrackingStatus` and the native smoke gate now expose optimized IMU re-anchor
counts and B-spline trajectory-control counts so the production BA feedback
path is runtime-observable.
