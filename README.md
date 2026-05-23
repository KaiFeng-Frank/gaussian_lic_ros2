# Gaussian-LIC ROS2

Native ROS2 engineering port for [Gaussian-LIC/Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC).

The long-term target is a paper-level ROS2 reproduction of LiDAR-Inertial-Camera Gaussian Splatting SLAM: native ROS2 tracking, native Gaussian mapping, rosbag2 replay, offline reproducibility tooling, and strict comparison against the ROS1 upstream baseline.

This repository is **not a ROS1 bridge wrapper**. It is a clean ROS2 workspace that keeps algorithm code isolated from middleware glue so the tracking and mapping backends can be ported incrementally without locking the project into ROS1-era APIs.

## Current Release State

This repository is now an executable ROS2 porting checkpoint for the public Gaussian-LIC/Gaussian-LIC2 code path. It has native ROS2 message, launch, adapter, mapper, CUDA/Torch Gaussian backend plumbing, native tracking factors, artifact extraction, strict replay/readiness tooling, an official FAST-LIVO2 Bright substitute proof chain, local strict FAST-LIVO2 `CBD_Building_01`, MCD `ntu_day_01`, and R3LIVE `hku_park_00` reproduction chains against archived ROS1 upstream baselines, plus trusted upstream/native reference trajectory gates for the FAST-LIVO2 `CBD_Building_01` frontend and the new continuous-time tracker producer chain. The archived strict mapper-contract/CUDA reports are green for trajectory, PSNR/SSIM/LPIPS, GT-associated render-pair, and point-cloud gates; the full local strict evidence matrix now reports `required=17/17`.

It is **not a claim of universal super-paper performance on every possible sequence**: the five continuous-time matrix entries are currently real-bag liveness/producer-chain gates, not RMSE-gated paper parity. The current release claim is narrower and executable: the required local strict parity matrix passes, including mapper-contract/CUDA parity, the FAST-LIVO2 `CBD_Building_01` native frontend comparison against an upstream Coco-LIC-generated reference, and continuous-time tracker evidence on FAST-LIVO2, M2DGR, MCD, and R3LIVE bags.

Available now:

- Native ROS2 packages for messages, launch/config, mapping scaffold, and test tools.
- `gaussian_lic_frontend/lic2_contract_adapter` for routing raw ROS2 camera/LiDAR/IMU/pose topics into the mapper contract.
- Optional Livox `CustomMsg` to `PointCloud2` bridge for raw Livox driver packets.
- ROS2 synchronization/conversion for the Gaussian-LIC mapper input contract:
  `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/imu_for_gs`.
- Odometry, path, TF, debug map cloud, rendered debug preview, status, GaussianArray, and SaveMap interfaces.
- Optional libtorch/CUDA path for keyframe-gated Gaussian tensor initialization, skybox seeding, incremental foreground insertion, CUDA rasterizer preview/loss, fused SSIM, SparseGaussianAdam, gradient-aware densification, multi-criteria pruning, opacity reset, and optional TensorRT/SPNet depth-completion wrapper.
- Native tracking package with signed-nanosecond trajectory and IMU primitives, invalid ROS2 builtin-stamp nanosecond rejection at mapper/frontend boundaries, non-monotonic tracking stream stamp rejection with status counters, non-finite IMU measurement rejection before propagation/preintegration, LiDAR malformed-frame/invalid-point/invalid-point-time/out-of-range point-time status counters, CameraInfo/image/depth/rendered invalid-frame counters, mapper per-input QoS overrides, frontend-adapter per-stream input/output QoS controls, and native tracking per-stream raw-input/mapper-output QoS controls for image/point cloud/pose/CameraInfo/depth/IMU/odometry streams, finite CameraInfo intrinsic gating before mapper/tracker state updates, finite LiDAR-to-IMU/camera-to-IMU extrinsic, IMU gravity, LiDAR keyframe-threshold, tracking QoS depth, visual/SE3 photometric gate, LiDAR-factor, cache-size, and sliding-window BA-bound validation before tracker startup, signed-nanosecond image/LiDAR/IMU status gates, bounded best-effort tracking sensor QoS exposed through launch/status gates, executor callback serialization for ROS1-style estimator mutation order, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, signed-nanosecond visual-factor freshness gating with bounded nearest-stamp rendered/depth frame caches, IMU history interpolation, raw-sample IMU preintegration/reintegration with finite-input validation, SO(3) log-map rotation residuals, AutoDiff start-bias reintegration sensitivity, optional full 9x9 IMU residual sqrt-information whitening, per-block rotation/velocity/position residual weights, optimized pose/velocity/bias feedback into odometry, the continuous-time trajectory-control cache, safe IMU re-anchoring, and finite optimized-state/control-pose rejection before odometry/IMU/B-spline feedback, IMU bias continuity and marginalization-prior anchoring inside the optional sliding window, bias observability status/probes, reusable normal-equation linearization, factor-agnostic BA solve triggering when non-IMU LiDAR/visual factors are available, bounded LM state increments, Schur-complement retained-window dense priors with rank/singular-value health, timestamp-safe cubic B-spline position/velocity plus SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state continuous-time trajectory smoothness factors plus pose/state/dense/SE3 priors with SO(3) log-map residuals and closed-form left-Jacobian inverse rotation blocks, per-point LiDAR deskew with invalid-pose and scan-time-bound rejection, bounded 6-DoF LiDAR residual correction with finite-sample/pose validation and Huber-weighted Kabsch centroids/covariance, corrected-pose LiDAR/Gaussian correspondence construction before BA ingestion, spatial-indexed robust direct LiDAR/Gaussian-map point-to-point window factors with `(0, 1]` correspondence-weight validation, LiDAR point-to-plane window factors with residual and local-planarity confidence weighting, LiDAR correspondence confidence and spatial-index health status, sliding-window BA optimization timing and numeric-Jacobian fallback status, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric Jacobians for point-to-point, point-to-plane, visual-alignment, and full current IMU preintegration factors, visual residual/direct subpixel image-alignment comparison against mapper renders, runtime-gated 2-DoF photometric translation linearization, analytic SE3 camera photometric pixel Jacobians, multi-sample normal equations, sparse LiDAR-projected depth dilation plus valid-depth SE3 sampling for mapper-rendered feedback, optional visual-alignment window factors, chunk-complete GaussianMap snapshot caching, odometry/path/status/optional TF publication, and deterministic probes.
- Dataset profile YAMLs derived from upstream Gaussian-LIC: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- `gaussian_lic_offline` CLI for rosbag2 artifact extraction without launching ROS nodes.
- Profile-aware ROS1-to-ROS2 raw frontend conversion for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE; ROS1 mapper-contract conversion, upstream ROS1 baseline runner, strict rosbag2 replay wrapper, baseline manifest/readiness gates, and combined reproduction reports.
- Current executable Bright substitute report with `metrics`, `trajectory`, `point_cloud`, and dedicated Torch Gaussian `gaussian_color` gates passing.
- Strict FAST-LIVO2 `CBD_Building_01` artifact/readiness pipeline with trajectory, PSNR/SSIM/LPIPS, render-pair, and point-cloud gates passing for the mapper-contract/CUDA path.
- `scripts/run_strict_parity_queue.sh` turns the remaining data/evidence backlog into a resumable full-profile queue: it reuses or creates frontend-raw bags, emits ROS1 mapper-contract bags, runs the upstream baseline with the matching profile config, collects ROS2 CUDA current artifacts, and writes strict readiness/reproduction reports for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE targets.
- `scripts/check_strict_parity_matrix.py` for the final full-dataset release gate. It now reports `required=17/17`: FAST-LIVO2 mapper strict parity, FAST-LIVO hku1/hku2/Visual_Challenge/LiDAR_Degenerate mapper strict parity, M2DGR room_01/room_02/room_03 mapper strict parity, MCD `ntu_day_01` mapper strict parity, R3LIVE `hku_park_00` mapper strict parity, CBD native BA runtime health, CBD native reference trajectory parity, and five continuous-time native tracker producer-chain gates are all archived and passing.
- `scripts/audit_strict_data_inputs.py` for the data-side gate. The current local audit is archived at `docs/strict_data_status.md` / `docs/strict_data_status.json`; it now separates raw/frontend data, ROS1 baseline artifacts, ROS2 current artifacts, and native reference trajectories. As of 2026-05-12 all required profiles pass those input checks and the strict evidence matrix is `17/17`.
- R3LIVE `hku_park_00` can be converted to ROS2 frontend-raw, passes the native sensor-only tracking health gate, and now has a passing full-sequence mapper-contract/CUDA strict parity report.
- FAST-LIVO2 `Retail_Street` is now fetched from the official Google Drive index, converted to ROS2 frontend-raw, has an archived ROS1 upstream mapper baseline with 1,354 poses, 704,826 PLY vertices, and 1,354 renders, and passes a 60s native scan-order deskew tracking health gate; this remains additional runtime/data coverage, not an RMSE-gated native tracker parity claim.
- Generic Google Drive file fetching is available through `scripts/fetch_google_drive_file.py`; MCD `ntu_day_01` now has local `d435i`, `mid70`, `vn100`, ground-truth TUM, frontend-raw conversion, archived ROS1 baseline artifacts, ROS2 current artifacts, and a passing strict mapper-contract/CUDA report.
- SPNet TensorRT engine generation for the local `sm_120` GPU via TensorRT 10.9, with the generated FP16 engine kept outside git.
- Native tracking probes are registered with CTest, so `colcon test --packages-select gaussian_lic_tracking` runs trajectory, IMU, LiDAR, sliding-window, bias observability, geometric Jacobian, Gaussian snapshot, trajectory smoothness, SE3 photometric Jacobian/factor, and visual checks automatically. `scripts/tracking_smoke_test.sh` also verifies the launch path through `/gaussian_lic/frontend/status`, including signed-nanosecond time status, tracking QoS, executor callback serialization, auto-start IMU preintegration re-integration semantics, cumulative IMU-factor/preintegration and visual/SE3 factor evidence that survives window marginalization, dense-prior health with stamp/reference validation, Schur/fallback marginalization health, same-stamp prior plus same-source IMU/LiDAR/visual/SE3/smoothness replacement counters, orphan-factor health, active-window state-cadence health, bias observability, accepted BA feedback stamp/delta/limit health, optimized IMU re-anchors, last-consumed IMU preintegration sample/dt/span health with time-gap skip gating, B-spline trajectory control poses, LiDAR correspondence confidence and spatial-index health, sliding-window optimization timing, zero global numeric-Jacobian fallback, nonzero trajectory smoothness factors, accepted/rejected/limited BA step health, linearization/linear-solve failure counters, factor skip counters, visual alignment, bounded photometric sample weights, photometric linearization status, rendered/depth cache diagnostics, and SE3 photometric sample quality/rejection counters.
- CI semantic checks keep the GitHub Actions build matrix Jazzy-only; adding ROS2 Humble to `.github/workflows/ci.yaml` is treated as a contract violation.

Still pending:

- RMSE-gated continuous-time native tracker parity on the full long-window datasets; the data and reference-trajectory archive gate itself is now complete.
- Latest 2026-05-14 native Coco-LIC2 tracker work: LiDAR residuals carry Coco-LIC-style feature scale, persistent/Gaussian map anchors enter the online window, rendered-image QoS is explicit, and the Gaussian feedback path now has a machine-checked production preset in `docs/native_production_preset.json`. `run_native_tracking_bag_report.sh --enable-gaussian-map-feedback --require-gaussian-snapshot` defaults strict feedback replay to `--rate 0.25`, `tracking_max_pose_step_m=0.0265`, pre/post-BA direction agreement release at `0.045` with cosine `0.95` and delta `0.02`, visual/SE3 weights `0.25/0.5`, LiDAR plane weight `4.0`, gyro/accel feedback clamps `0.02/0.12`, rasterizer feedback, and sensor-frame Gaussian point clouds. The native report now also exposes per-time-bin gates for TrackingStatus samples plus visual/SE3 factor deltas, and `trajectory_compare.py` exposes per-error-bin RMSE/mean/bias gates, so full-window runs can fail automatically when map/photometric support disappears or late-window trajectory shape drifts even if aggregate RMSE still looks tolerable. The accepted full122 evidence is `results/fastlivo2/CBD_Building_01_gaussian_feedback_rate0p25_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`: 1174 matches, 95.37% coverage, 1.346 m RMSE, 1.219 m mean error, 2.829 m max error, 5.00% path drift, 1015 visual/SE3 factors, and dense prior rank/cols 180/180. Confidence release, confidence warmups, stronger visual/SE3 weights, state-sync, QoS, bias-model, and window-size ablations are archived as rejected evidence; this is a measurable production-preset advance, not a 100% paper/super-paper parity claim.
- Latest 2026-05-23 structural tracker work: visual-alignment and SE3 photometric sliding-window factors can now attach to the two bracketing continuous-time states through default-off `enable_visual_factor_time_interpolation` instead of snapping every delayed observation to one discrete state. The optimizer now carries analytic continuous-time support Jacobians for those interpolated visual/SE3 factors, so the diagnostic path no longer relies on runtime numeric finite differences. The legacy production preset still keeps interpolation off until continuity matches the accepted full122 gate. The analytic 12 s proof at `results/fastlivo2/CBD_Building_01_visual_time_interpolation_analytic_12s_probe/native_tracking_report.json` passes with 119 poses, 84 visual factors, 84 SE3 factors, 42 interpolated visual factors, 42 interpolated SE3 factors, dense prior rank/cols 180/180, zero orphan factors, and zero numeric-Jacobian blocks. The analytic full122 proof at `results/fastlivo2/CBD_Building_01_visual_time_interpolation_analytic_full122_probe/native_tracking_report.json` improves aggregate RMSE to `1.334 m` versus the accepted preset's `1.346 m`, but is not promoted because coverage drops to `89.20%`, visual/SE3 factors drop to `870/1015`, and path drift rises to `5.21%` versus `5.00%`. This is a structural continuous-time semantics improvement, not a parameter-tuning result, and full production promotion still requires preserving factor continuity and path-drift parity.
- Latest 2026-05-23 visual-cache reconciliation work: the tracker now has a default-off causal cache reconciliation diagnostic for observed/rendered image pairs that were missed by ROS2 callback ordering. The first bulk backfill is explicitly rejected (`results/fastlivo2/CBD_Building_01_visual_cache_reconcile_full122_drain1015_probe/native_tracking_report.json`): it reaches `1085/1015` visual/SE3 factors but creates `43` visual-alignment pending stale drops and regresses RMSE to `2.484 m`. The causal rewrite processes at most one backfilled pair per callback and skips observations already outside the active sliding-window reference horizon. Its 12 s proof at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_causal_12s_probe/native_tracking_report.json` passes with `99` reconciled pairs, `177/177` visual/SE3 factors, zero pending stale drops, dense prior `180/180`, and zero numeric-Jacobian blocks. The full122 causal replay at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_causal_full122_drain1015_probe/native_tracking_report.json` is also not promoted: it reaches `1039/1015` visual/SE3 factors with zero stale drops, but RMSE regresses to `1.895 m` and path drift to `5.35%` versus the accepted `1.346 m / 5.00%` preset. A follow-up provenance guard now handles saturated 2D visual-alignment factors only for reconciled pairs: direct skip improves path drift to `2.53%` but starves visual alignment at `761/1015`, local photometric-step fallback restores `1083/1015` visual/SE3 factors and lowers drift to `3.81%` but still regresses RMSE to `1.723 m`, and same-direction agreement gating overfeeds the window at `1252/1015` visual factors with RMSE `2.284 m` despite `2.20%` drift (`results/fastlivo2/CBD_Building_01_visual_cache_reconcile_photostep_agreement_full122_drain1015_probe/native_tracking_report.json`). The result closes a ROS2 execution-order bug class while proving that simply adding more delayed visual constraints, deleting saturated 2D shifts, locally replacing them with 2D photometric steps, or using same-direction agreement is not the remaining paper-parity fix.
- Latest 2026-05-23 observed-frame deduplication work: reconciled visual-cache mode now treats one observed camera stamp as a single BA observation even if several rendered frames become eligible later, preventing ROS2 callback-order replay from consuming the same image multiple times with different render stamps. The 12 s proof at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_observed_dedup_12s_probe/native_tracking_report.json` passes with `74` reconciled pairs, `97/97` visual/SE3 factors, `79` rejected duplicate visual pairs, zero stale drops, dense prior `180/180`, and zero numeric-Jacobian blocks. The full122 replay at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_observed_dedup_full122_drain1015_probe/native_tracking_report.json` is rejected for production: it preserves high trajectory coverage (`1174` matches, `95.37%`) and records `636` rejected duplicates, but underfeeds the drain at `959/1015` visual and `960/1015` SE3 factors and regresses RMSE/path drift to `1.727 m / 6.36%`. This proves duplicate observed-frame consumption was real, but dedup alone starves late visual support rather than closing parity.
- Latest 2026-05-23 monotonic unique visual pairing work: `visual_cache_reconciliation_monotonic_unique` is now a default-off diagnostic that requires each observed and rendered image stamp to be consumed at most once during reconciled visual-cache pairing. The 12 s proof at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_monotonic_unique_12s_probe/native_tracking_report.json` passes with `110` processed pairs, `0` duplicate pairs, `107/110` visual/SE3 factors, zero stale drops, dense prior `180/180`, and zero numeric-Jacobian blocks. The full122 replay at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_monotonic_unique_full122_drain1015_probe/native_tracking_report.json` is rejected as a production preset: it reaches all `1186` poses and keeps `1174` trajectory matches at `95.37%` coverage with `0` duplicate pairs, but drains only `966/1015` visual and `972/1015` SE3 factors, regresses to `1.403 m` RMSE and `5.36%` path drift, and leaves bin 7 at only `68/68` visual/SE3 factors. This narrows the remaining frontend gap to late-window observation support/global coupling rather than repeated image-pair consumption.
- Latest 2026-05-23 deferred reconciliation scheduling work: `visual_cache_reconciliation_defer_to_pointcloud` is now a default-off executor diagnostic that runs cache reconciliation from point-cloud callbacks instead of image/render callbacks. The 12 s proof at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_deferred_monotonic_12s_probe/native_tracking_report.json` passes with `106` raw images, `103` rendered images, `106` processed pairs, `0` duplicate pairs, `105/106` visual/SE3 factors, zero stale drops, dense prior `180/180`, and zero numeric-Jacobian blocks. The full122 replay at `results/fastlivo2/CBD_Building_01_visual_cache_reconcile_deferred_monotonic_full122_drain1015_probe/native_tracking_report.json` is rejected for production despite improving aggregate RMSE to `1.327 m`: it misses the drain at `983/1015` visual and `986/1015` SE3 factors, raises path drift to `6.01%`, and collapses bin 7 to `51/51` visual/SE3 factors. This shows callback scheduling can improve ATE but does not solve long-window visual continuity, so it stays diagnostic.
- Latest 2026-05-23 full visual-pair callback deferral work: `visual_pair_processing_defer_to_pointcloud` is now a default-off executor diagnostic that caches direct observed/rendered image callbacks and processes visual pairs from point-cloud callbacks. The 12 s proof at `results/fastlivo2/CBD_Building_01_visual_pair_processing_deferred_monotonic_12s_probe/native_tracking_report.json` passes with `106` raw images, `96` rendered images, `106` processed pairs, `0` duplicate pairs, `105/106` visual/SE3 factors, zero stale drops, dense prior `180/180`, and zero numeric-Jacobian blocks. The full122 replay at `results/fastlivo2/CBD_Building_01_visual_pair_processing_deferred_monotonic_full122_drain1015_probe/native_tracking_report.json` is rejected: it reaches `1174` matches and `95.37%` coverage with `0` duplicate pairs, but times out the drain at `912/1015` visual and `919/1015` SE3 factors, regresses to `1.838 m` RMSE and `6.75%` path drift, and leaves bin 7 at only `57/59` visual/SE3 factors. A rendered-image depth-64 replay at `results/fastlivo2/CBD_Building_01_visual_pair_processing_deferred_depth64_full122_drain1015_probe/native_tracking_report.json` restores delivery to `943` received rendered images from `946` mapper renders, but still regresses to `1.721 m` RMSE and `8.20%` drift with `790/794` visual/SE3 factors and `179/179` pending stale drops. A pre-ingest scheduling fix now reconciles visual caches before the current point-cloud update consumes pending visual factors; its 12 s proof at `results/fastlivo2/CBD_Building_01_visual_pair_processing_preingest_12s_rerun_probe/native_tracking_report.json` passes with `98/98` visual/SE3 factors, `0` duplicates, and `0/0` stale drops, while the full122 depth-64 replay at `results/fastlivo2/CBD_Building_01_visual_pair_processing_preingest_depth64_full122_drain1015_probe/native_tracking_report.json` is still rejected at `946/952` visual/SE3 factors, `53/53` stale drops, `1.663 m` RMSE, and `4.94%` path drift. This closes the one-cycle executor lag but proves the remaining gap is rendered/mapper latency plus late-window history coupling, not simply more queue depth or callback deferral.
- Latest 2026-05-23 mapper nearest-sync hypothesis: a structural attempt to select the nearest pose/image/depth sample within the existing sync tolerance was tested and then reverted because it did not beat the accepted production preset. The 12 s proof at `results/fastlivo2/CBD_Building_01_mapper_nearest_sync_12s_probe/native_tracking_report.json` passed the health gate with `100` aligned/rendered mapper frames, but still recorded `15` pointcloud-anchor image-too-new drops. The full122 replay at `results/fastlivo2/CBD_Building_01_mapper_nearest_sync_full122_drain1015_probe/native_tracking_report.json` lowered path drift to `4.13%`, but regressed aggregate RMSE to `1.537 m`, missed the visual/SE3 drain at `928/1015`, and recorded one invalid LiDAR frame. This rules out queue-front nearest-sample selection as the remaining parity fix; the production code remains on the accepted strict FIFO synchronizer plus typed diagnostics.
- Latest 2026-05-23 estimator observability reporting: `run_native_tracking_bag_report.sh` now emits an `estimator_observability_continuity` section that bins normal-equation min/max singular values and condition number, dense-prior singular values, gyro/accel bias norms/components, bias-observability scalars, and IMU/visual/SE3/LiDAR/smoothness factor deltas. `scripts/analyze_estimator_observability.py` applies the same summary to older `metrics.json` artifacts without rerunning bags. The 12 s proof at `results/fastlivo2/CBD_Building_01_observability_report_12s_rerun_probe/native_tracking_report.json` passes with 8 finalized bins, max normal-equation condition `5.535e8`, max gyro/accel bias norms `0.01345/0.20548`, and worst-information/worst-bias bin summaries. Running the analyzer on the accepted full122 metrics shows late-window bias concentration in bins 7/5/6 with max gyro/accel bias norms `0.05085/0.55527`, so the remaining diagnosis stays on marginalization/bias information retention instead of synchronization or parameter sweeps. This is a diagnostic/gate upgrade for the remaining long-window frontend gap, not a tuning change.
- The estimator observability path now also exposes dense-marginalization-prior singular values restricted to gyro-bias and accel-bias columns, so a full dense prior can no longer hide a weak bias sub-block. The 12 s proof at `results/fastlivo2/CBD_Building_01_dense_prior_bias_observability_12s_probe/native_tracking_report.json` passes with last gyro/accel bias prior min singular values `6.636/6.636` and max values `26.45/26.45`, proving the short-window prior carries bias information. The full122 diagnostic at `results/fastlivo2/CBD_Building_01_dense_prior_bias_observability_full122_drain1015_probe/native_tracking_report.json` is rejected for production because the visual/SE3 drain stalls at `926/1015`, but it keeps `1174` matches, `95.37%` coverage, `1.357 m` RMSE, `4.05%` path drift, and last gyro/accel bias prior min singular values `6.691/6.691`. This rules out a collapsed dense-prior bias sub-block as the main remaining gap; the next structural target is late-window rendered/observed visual support and global trajectory coupling, especially bins 6/7 where visual/SE3 deltas fall to `96/90`.
- Latest 2026-05-23 visual nearest-frame latency reporting: `TrackingStatus` and `run_native_tracking_bag_report.sh` now record nearest rendered/observed cache deltas even when a pair is stale and rejected. The 12 s proof at `results/fastlivo2/CBD_Building_01_visual_nearest_delta_12s_probe/native_tracking_report.json` passes the native health gate with `108/108` visual/SE3 factors; it shows image-callback rendered candidates can lag by `1.82 s` while rendered-callback observed candidates stay within `23.5 ms`. This turns a hidden ROS2 callback/render-latency failure mode into a measurable report field for the remaining late-window visual-support work.
- Latest 2026-05-23 rendered delivery continuity reporting: `native_tracking_recorder` now bins `num_raw_images` and `num_rendered_images`, and the native report adds `rendered_delivery_continuity` to compare mapper-rendered previews, tracker-received rendered images, and visual-pair consumption in each time bin. The 12 s proof at `results/fastlivo2/CBD_Building_01_rendered_delivery_continuity_12s_probe/native_tracking_report.json` records `90` mapper rendered previews, `85` tracker rendered-image callbacks, and `97` visual pairs; that makes delivery gaps and one-rendered-to-many-observed pairing visible before changing the production visual-pairing policy. The full122 diagnostic at `results/fastlivo2/CBD_Building_01_rendered_delivery_continuity_full122_drain1015_probe/native_tracking_report.json` is rejected for production: trajectory remains close (`1174` matches, `95.37%` coverage, `1.468 m` RMSE, `3.68%` path drift), but post-play drain stops at `1185/1186` poses and `895/1015` visual/SE3 factors. Mapper output produced `1075` rendered previews while the tracker received only `880` rendered-image callbacks and consumed `895` visual pairs; bin 7 alone loses `127` produced-vs-received rendered frames. This narrows the remaining non-tuning blocker to rendered-delivery/backlog and visual-pair ownership semantics, not dense-prior collapse or another scalar parameter sweep.
- Latest 2026-05-23 direct visual-pair one-to-one semantics: `visual_pair_monotonic_unique` is now a default-off diagnostic that applies one-observed/one-rendered stamp ownership to direct image/render callbacks as well as reconciled cache pairs. The corrected 12 s proof at `results/fastlivo2/CBD_Building_01_visual_pair_monotonic_unique_12s_rerun_probe/native_tracking_report.json` passes with `93/93` visual/SE3 factors, `0` duplicate pairs, `0` rendered-size mismatches, and received-vs-consumed rendered frames exactly matched (`93/93`). The full122 proof at `results/fastlivo2/CBD_Building_01_visual_pair_monotonic_unique_full122_rerun_drain1015_probe/native_tracking_report.json` is rejected: it keeps `1174` matches and `95.37%` coverage, but drains only `890/1015` visual/SE3 factors, records `1.399 m` RMSE, `5.82%` path drift, and one invalid LiDAR frame. It proves repeated rendered/observed reuse is not the production blocker; the remaining gap is upstream of pairing, in rendered delivery/backlog and late-window support continuity.
- Latest 2026-05-15 bin-7 drift work: the sliding-window smoothness factor now supports measured local-shape targets for rotation-rate, position-rate, velocity-acceleration, and bias-rate deltas. The default remains the legacy zero-curvature target, but `sliding_window_smoothness_use_motion_targets:=true` and `run_native_tracking_bag_report.sh --sliding-window-smoothness-use-motion-targets` can now test whether pre-BA local motion curvature reduces late-window trajectory-shape drift without hiding regressions in aggregate RMSE, visual/SE3 factor continuity, or per-error-bin gates. The first unbounded full122 hypothesis at `results/fastlivo2/CBD_Building_01_gaussian_feedback_motion_targets_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json` improved aggregate RMSE to `1.305 m` but regressed path drift to `6.36%` and collapsed bin-7 visual/SE3 continuity to `60`, so it is rejected as a production preset. The first support-gated/clamped full122 hypothesis preserves factor continuity and improves bin-7 RMSE to `1.124 m` with `2.15%` path drift, but regresses aggregate RMSE to `2.302 m`, so it is also rejected. The motion-target path now has support gates, a late-window `start_after_s` trigger, norm clamps, and TrackingStatus counters/norm telemetry so the next run can target late-window drift without accepting low-observation or early-window trajectory shaping.
- Bias feedback is now also instrumented with a default-off visual-support gate, exposed as `sliding_window_min_bias_feedback_visual_factors` in the node, launch file, status message, and native report script. The full122 gate=1 hypothesis run is rejected as a production setting: `results/fastlivo2/CBD_Building_01_gaussian_feedback_bias_visual_gate1_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json` records 1010 matches, 82.05% coverage, 3.266 m RMSE, 3.008 m mean error, 5.982 m max error, and 6.60% path drift after 238 held bias-feedback updates. Its diagnostics show post-70 s visual/SE3 factors dropping to zero, so freezing bias when visual support is missing breaks the feedback chain instead of closing long-window parity.
- The sliding-window size hypothesis is rejected for the production Gaussian-feedback path. `max_states=14` reaches only 717 matched poses, 58.25% coverage, 3.384 m RMSE, and 20.87% path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_maxstates14_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`; its post-70 s visual/SE3 factors drop to zero. `max_states=16` keeps a full-rank dense prior and more post-70 s visual/SE3 factors, but throughput collapses to 523 matched poses, 42.49% coverage, 3.624 m RMSE, and 36.17% path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_maxstates16_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`. The remaining full122 gap is therefore not solved by simply retaining more knots; the next work item is selective BA-step trust from geometric/visual agreement and stronger visual/Gaussian factor continuity.
- Synchronizing limited/post-guard BA feedback back into the internal sliding-window state was also tested and rejected. The full122 run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_state_sync_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json` records 1051 matches, 85.38% coverage, 1.819 m RMSE, 1.693 m mean error, 3.133 m max error, and 4.56% path drift, regressing from the accepted production preset above. The code path was not kept in production.
- Rendered-image feedback QoS is now explicit on both mapper publisher and tracker subscriber through `rendered_image_qos_reliability`, `rendered_image_qos_durability`, and `rendered_image_qos_depth`; defaults preserve the previous `reliable/transient_local/depth=1` behavior, and the native report script archives the selected QoS. The full122 `best_effort/volatile/depth=4` hypothesis run is rejected as a production preset: `results/fastlivo2/CBD_Building_01_rendered_qos_best_effort_volatile_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json` records 1083 matches, 87.98% coverage, 1.835 m RMSE, 1.723 m mean error, 3.175 m max error, and 3.53% path drift, with only 117 visual/SE3 factors. A wider 500 ms rendered/observed pairing window is also rejected (`results/fastlivo2/CBD_Building_01_gaussian_feedback_visualdt500ms_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`: 2.617 m RMSE, 0.25% path drift, 113 visual/SE3 factors). These probes confirm the 60-100 s visual-factor gap is not fixed by transport QoS or stale-pair tolerance alone.
- Latest 2026-05-16 relative-history diagnostics: mapper feedback now has an explicit `sync_anchor_stream` status/launch/report contract, the tracker reports rendered/observed-frame cache freshness counters, guarded-pose priors are default-off reportable controls, and relative-motion history can be sourced from `pre_ba`, `published`, or `published_after_warmup`. A default-off delayed published-to-published multi-hop relative factor now carries a separate `source_id` so it cannot replace the production pre-BA multi-hop factor; `sliding_window_delayed_published_multihop_max_factors` caps its per-update load. The source-id max6 full122 run is rejected (`results/fastlivo2/CBD_Building_01_delayed_published_multihop_sourceid_after70_full122_drain1015_probe/native_tracking_report.json`: 1.547 m RMSE, 4.71% drift, 954 visual/SE3 factors). The corrected source-id max1 rerun is also rejected (`results/fastlivo2/CBD_Building_01_delayed_published_multihop_sourceid_max1_after70_full122_drain1015_probe_rerun/native_tracking_report.json`: 1.528 m RMSE, 4.47% drift, 952 visual/SE3 factors, bin-7 2.343 m versus the accepted 2.219 m). This confirms delayed published endpoint consistency is useful evidence, not the production fix for final-window trajectory shape.
- Visual-alignment edge saturation is now explicit instead of hidden inside the 2D image-shift factor. `TrackingStatus` reports `visual_alignment_saturated`, `visual_alignment_saturated_count`, and `visual_alignment_effective_weight`, while `tracking.launch.py` and `run_native_tracking_bag_report.sh` expose default-off saturation downweighting. The full122 `scale=0.1` hypothesis is rejected as a production preset: `results/fastlivo2/CBD_Building_01_gaussian_feedback_saturated_visual_weight010_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` records 1174 matches, 95.37% coverage, 1.346 m RMSE, 1.244 m mean error, 2.669 m max error, 5.16% path drift, 1005 visual/SE3 factors, and 442 saturated alignments. It proves the search-boundary diagnostic is live, but it misses the factor-continuity drain target and slightly worsens the accepted path-drift gate, so defaults preserve the previous factor weight.
- The visual search-window hypothesis is also rejected. `run_native_tracking_bag_report.sh --visual-alignment-max-shift-px` now archives the 2D search radius in `gate_config`; the full122 `max_shift=12` run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_maxshift12_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` times out the visual/SE3 drain at 807/1015, regresses to 1.590 m RMSE and 6.26% path drift, and still records 253 saturated alignments. The boundary hits are therefore not explained by the old 8 px search radius alone; the next tracker work stays on visual matching quality/global Gaussian coupling rather than simply widening the search window.
- A default-off ZNCC visual-alignment objective is now implemented and covered by `visual_factor_probe`, including an affine-brightness translation case. The full122 ZNCC replay is rejected as a production preset: `results/fastlivo2/CBD_Building_01_gaussian_feedback_zncc_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` lowers path drift to 3.16% but misses the visual/SE3 drain at 845/1015 and regresses aggregate RMSE to 1.496 m. Brightness-normalized 2D matching alone is therefore not enough; the remaining gap is in global Gaussian/trajectory coupling and observation selection.
- The default-off `visual_alignment_factor_source` diagnostic can now feed the sliding-window 2D visual factor from the local photometric Gauss-Newton step instead of the exhaustive search result. The conservative `saturated_photometric_step` run is rejected: `results/fastlivo2/CBD_Building_01_gaussian_feedback_saturated_photostep_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` misses the drain at 858/1015 and regresses to 1.619 m RMSE despite 4.69% path drift. Replacing only saturated search measurements with local photometric steps does not close long-window parity.
- Native reports now include `visual_factor_continuity` and `mapper_feedback_continuity` blocks derived from finalized tracking and mapping status bins. They record per-bin visual/SE3 factor deltas, rendered/observed/depth miss and stale deltas, visual pair processed/duplicate deltas, SE3 valid/rejected batch deltas, mapper rendered-preview deltas, per-stream drop deltas, and pending queue deltas, so future full-window runs can distinguish mapper throughput/pairing loss from duplicate-pair suppression or visual quality rejection before another production-preset candidate is considered.
- Mapper synchronization diagnostics now publish signed pointcloud-to-pose/image deltas, max absolute alignment deltas, and per-anchor too-new drop counters through `MappingStatus`, recorder summaries, and `mapper_feedback_continuity`. The 12 s proof at `results/fastlivo2/CBD_Building_01_mapper_sync_diagnostics_12s_rerun_probe/native_tracking_report.json` passes with `99` rendered previews, `99` aligned/converted frames, `14` pointcloud drops, and zero pose-delta drift; all `14` pointcloud drops are explained by `pointcloud_anchor_image_too_new_drops`, with max absolute pointcloud-image alignment delta `23.385 ms`. This turns the late-render evidence into a typed ROS2 synchronization signal instead of an inferred symptom.
- Gaussian-feedback strict replay now exposes mapper image QoS controls through `--mapper-feedback-image-qos-reliability` and `--mapper-feedback-image-qos-depth`, while keeping the default at `best_effort`. The option is wired to both the tracking-side `/image_for_gs` publisher and the mapper-side subscriber; using it on only one side is rejected because a reliable mapper subscriber cannot receive from the default best-effort internal publisher.
- The dual-endpoint mapper image QoS path has a 12 s compatibility smoke at `results/fastlivo2/CBD_Building_01_gaussian_feedback_reliable_internal_image_qos_12s_smoke/native_tracking_report.json`. With both `/image_for_gs` endpoints set to `reliable/depth=512`, the native health gate passes with `107` poses, `108` point frames, `99` mapper image messages, `89` rendered previews, zero image drops, zero render errors, and `92/92` visual/SE3 factors. The trajectory compare inside that smoke is not production evidence because the short run was launched with a full-sequence `min_matches=1000` reference gate and only matches `98` poses; full122 evidence is still required before changing the production preset.
- The matching full122 reliable-internal-image replay is rejected as a production preset: `results/fastlivo2/CBD_Building_01_gaussian_feedback_reliable_internal_image_qos_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` records `1174` matches, `95.37%` coverage, `1.494 m` RMSE, `1.339 m` mean error, `3.060 m` max error, and `4.79%` path drift, with only `718/1015` visual/SE3 factors. The mapper side is healthy enough to narrow the fault: `1021` image messages, `1019` rendered previews, only `2` image drops, and zero render errors. The remaining blocker is tracker rendered/observed-frame pairing staleness and late-bin visual support, not ROS2 QoS compatibility alone.
- Increasing `/gaussian_lic/rendered_image` QoS depth is useful but not sufficient. The full122 `rendered_image_qos_depth=64` run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_reliable_internal_image_rendered_depth64_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` raises visual/SE3 support to `1008/1015`, but drops `50` queued visual/SE3 factors and regresses to `1.741 m` RMSE and `6.73%` path drift. Increasing `visual_pending_factor_queue_size` to `256` is also rejected (`results/fastlivo2/CBD_Building_01_gaussian_feedback_reliable_internal_image_rendered_depth64_pending256_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json`: `881/1015` visual/SE3, `150` pending drops, `1.568 m` RMSE, `6.14%` drift). Default-off quality-ranked pending factor selection and quality weighting are now available for diagnosis, but full122 max-per-reference sweeps are rejected: max=2 misses drain at `956/1015` and regresses to `1.505 m` RMSE, while max=3 reaches `1018/1015` but still starves bin 7 and regresses to `1.712 m`. Late-only quality selection after 100 s is also rejected (`994/1015`, `1.409 m` RMSE, `7.13%` drift); disabling the reference cap improves path drift to `4.41%` but still misses drain at `982/1015`, and delaying that no-cap trim to 108 s gets closer (`1.371 m` RMSE, `5.13%` drift) while still starving bins 6/7 at `961/1015`. Quality weighting with a 0.25 floor lowers path drift to `3.31%` but regresses RMSE to `1.519 m` and reaches only `934/1015`, so it also stays default-off. A current-default revalidation with diagnostics disabled reaches `1.393 m` RMSE and `4.86%` drift but only `912/1015` visual/SE3 factors. The next fix has to preserve late-bin visual/Gaussian support without underfeeding or over-weakening BA observations.
- Sliding-window factor `source_id` is now 64-bit internally, but visual rendered/observed source hashing defaults to the accepted `legacy_8bit` mode after full-window evidence. The `full_64bit` diagnostic run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_sourceid64_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/native_tracking_report.json` lowered path drift to 4.02% but missed the 1015 visual/SE3 drain target at 946, failed bin 7 continuity at 96, and regressed RMSE to 1.490 m, so it remains default-off.
- Bias random-walk and insertion-prior sweeps do not beat the current best. `sliding_window_bias_random_walk_reference_dt_s=1.0` over-constrains bias continuity and regresses to 3.504 m RMSE with accel-bias norm 8.258 at `results/fastlivo2/CBD_Building_01_gaussian_feedback_biasrw1p0_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`. Small insertion bias priors (`gyro=0.02`, `accel=0.02`) reduce final accel-bias norm to 0.665 but still regress to 2.239 m RMSE and 8.43% path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_biasprior002_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`. The gap is therefore not solved by simply making bias smoother or smaller.
- Latest 2026-05-23 trajectory-shape hypothesis: the sliding-window BA now has a default-off relative-distance factor that constrains only motion length between adjacent or multi-hop states, leaving direction to the active LiDAR/visual/IMU factors. This targets the path-compression failure mode without freezing stale pre-BA directions. It is exposed through `tracking.launch.py`, `run_native_tracking_bag_report.sh`, `TrackingStatus`, and a deterministic optimizer probe. The full122 `relative_distance=0.05` / `multihop_distance=0.02` run is rejected as a production preset: `results/fastlivo2/CBD_Building_01_relative_distance_w005_mh002_full122_probe/native_tracking_report.json` records 1174 matches, 95.37% coverage, 2.087 m RMSE, 1.667 m mean error, 5.022 m max error, 5.37% path drift, and only 930/1015 visual/SE3 factors. This confirms the factor path is live but shifts error into bins 5/6 instead of closing full-window parity.
- Latest 2026-05-23 late-window guard release hypothesis: post-BA step guard now exposes default-off `post_ba_step_guard_pre_ba_agreement_late_start_marginalizations` and `post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m`, and `TrackingStatus` reports agreement-release counts plus the active late cap. The CBD 12 s smoke at `results/fastlivo2/CBD_Building_01_late_agreement_start10_max005_12s_probe/native_tracking_report.json` passes and proves the late release path is live (`5` late releases, `91/91` visual/SE3 factors). The full122 start=700/max=0.05 run is rejected: `results/fastlivo2/CBD_Building_01_late_agreement_start700_max005_full122_probe/native_tracking_report.json` lowers path drift to `4.73%` and bin 7 RMSE to `1.206 m`, but misses visual/SE3 drain at `794/1015` and regresses aggregate RMSE to `2.214 m`, with bins 5/6 rising to `3.954/4.113 m`. This rejects staged step-release as the production fix and points back to observation support and mid-late window coupling rather than looser post-BA motion admission.
- Continuous-time tracker quality hardening: the producer chain is now cross-profile and matrix-gated, persistent world-frame plane constraints plus online solve-step rejection are in tree, but the newest continuous-time entries are still liveness-gated while RMSE/paper-grade parity remains the next numerical target.
- Continued production hardening of the native tracker under faster-than-strict replay; strict reference parity still uses controlled rosbag2 replay to preserve ROS1-style sequential timing semantics.

## Progress Ledger

| Scope | Current status |
| --- | --- |
| ROS2 workspace, messages, launch, composable node, profiles, bag checks, artifact gates | Complete for the current porting surface |
| FAST-LIVO2 Bright substitute evidence chain | Complete and executable with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` passing |
| Strict `CBD_Building_01` paper-data gate | Official bag is local; ROS1 baseline is archived; latest ROS2 mapper-contract/CUDA strict report passes `reproduction_report.py --strict` |
| Paper-level Gaussian-LIC/Gaussian-LIC2 algorithm migration | Mapper CUDA/Torch backend, executable strict mapper-contract chain, local SPNet TensorRT engine generation, continuous-time spline factors, persistent world-frame LiDAR plane map constraints, and solve-step rejection are in place; full RMSE-gated native Coco-LIC2 tracking BA remains |
| Full-dataset strict parity matrix | PASS: `scripts/check_strict_parity_matrix.py` reports `required=17/17` with FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE covered by required mapper/native/continuous-time evidence |
| Full raw/frontend/baseline/reference data inputs | PASS: `scripts/audit_strict_data_inputs.py` reports raw, frontend, ROS1 baseline, ROS2 current, native-reference, and disk-headroom checks passing for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE |
| R3LIVE coverage | `hku_park_00` frontend-raw conversion, 60s sensor-only native tracking health gate, and full-sequence mapper-contract/CUDA strict parity report pass |

The Bright substitute report is a regression/evidence chain for current ROS2 plumbing and Torch Gaussian tensor boundaries. It is not a claim that the full paper algorithm has been ported.

## Current Executable Proof Chain

The current Bright proof chain uses the official FAST-LIVO2 `Bright_Screen_Wall` bag as a substitute for fast regression coverage. It compares ROS2 current artifacts against an upstream ROS1 Gaussian-LIC/Gaussian-LIC2 baseline and adds a Torch Gaussian color gate in the same report.

Validated artifacts:

```text
/home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag
baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s/
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply
```

Refresh the combined report from existing artifacts:

```bash
./scripts/run_curated_fastlivo2_report.sh \
  --fastlivo2-camera-lidar-transform \
  --colorize-pointcloud \
  --sequence Bright_Screen_Wall_fastlivo2_color_8s \
  --mapper-bag /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag \
  --baseline-dir baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s \
  --current-dir results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq \
  --skip-convert \
  --skip-current \
  --skip-baseline \
  --skip-build \
  --gaussian-color-current-point-cloud results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply
```

Expected output:

```text
reproduction report OK: sequence=Bright_Screen_Wall_fastlivo2_color_8s metrics=PASS trajectory=PASS point_cloud=PASS gaussian_color=PASS
```

The latest local report has Torch Gaussian mean RGB drift `11.657 < 40.0`.

## Algorithmic Gaps

The mapper backend now has the major CUDA/Torch surfaces in tree, but the full paper system is not finished.

- The native tracker has timestamp-safe trajectory/IMU primitives, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, IMU history interpolation, raw-sample IMU preintegration/reintegration, optimized pose/velocity/bias feedback into odometry, continuous-time trajectory controls, safe IMU re-anchoring, SO(3) log-map IMU rotation residuals with AutoDiff start-bias reintegration sensitivity, optional full 9x9 IMU residual sqrt-information whitening, IMU bias continuity residuals, per-block rotation/velocity/position residual weights, dense marginalization-prior anchoring, bias observability metrics/probes, reusable normal-equation linearization, Ceres-based cumulative B-spline trajectory estimation, continuous-time IMU/LiDAR/photometric residual surfaces, timestamped position-prior and SO(3) orientation-prior residuals for optional external/frontend odometry fusion, voxel-plane extraction, delayed point-cloud factor queuing until a same-stamp B-spline pose exists, persistent world-frame LiDAR plane map constraints, online Ceres solve-step rejection for non-finite or implausibly large knot updates, TUM prior publisher tooling, and real-bag continuous-time smoke/parity reports across every required profile.
- Full joint BA remains to be numerically hardened beyond current liveness-gated continuous-time evidence: the next paper-grade step is RMSE-gated continuous-time parity with stable IMU/gravity/bias initialization, persistent world-frame LiDAR map constraints, and Gaussian photometric feedback coupled into the spline window.
- Latest CBD continuous-time hardening, 2026-05-12: FAST-LIVO2 normalized-g IMU scaling (`9.80665`), one online Ceres iteration, lower IMU information weights, and rejected-solve position-extrapolation damping keep the frontend finite. The damped 60 s prior-seeded smoke drops the previous RMSE `59.498 m` blow-up to `1.803 m` at `results/fastlivo2/CBD_Building_01_continuous_time_damped0_prior_smoke_60s/trajectory_compare_report.json`, but that is a zero-path safety fallback. The parity script now records `runtime_diagnostics` from the continuous-time node; a 4 s diagnostic run at `results/fastlivo2/CBD_Building_01_continuous_time_diagnostics_probe_4s/native_tracking_report.json` shows 916 IMU messages, 45 pointcloud frames, 2,124 pointcloud correspondences, and 41 rejected solve steps, confirming the data enters the window while rotation updates remain the bottleneck. Real position-prior and SO(3) orientation-prior residuals were added for optional external/frontend odometry fusion; the 12 s position-prior diagnostic with weight `0.2` reaches RMSE `0.139 m`, but the 60 s diagnostic at `results/fastlivo2/CBD_Building_01_continuous_time_position_prior_w0p2_apply_diag_60s/trajectory_compare_report.json` still has RMSE `1.802 m`, 48.33% coverage, path length `39.18 m` vs `10.73 m`, and 381 rotation-limited steps. A 12 s pose-prior diagnostic with position weight `0.2` and orientation weight `0.5` reaches only RMSE `0.307 m` with `7.26x` path drift, so the pose-prior interface is not a paper-parity substitute. Delayed point-cloud queuing now holds plane/point-map LiDAR factors until the corresponding continuous-time pose can be queried; on the 12 s CBD diagnostic it releases 118/119 queued clouds, raises plane matches/updates to 2,459/3,554, and records zero queue drops. The 60 s queue diagnostic still reports RMSE `1.802 m` with 592 rejected Ceres steps, so the current blocker is inconsistent long-window rotation/LIO/VIO coupling, not missing point-cloud data. Per-factor Ceres cost diagnostics are now in the native report; the 4 s cost probe shows the last step is IMU-dominated (`405046 -> 227.6`), LiDAR cost increases (`0.0033 -> 0.116`), and all 41 rejections are rotation-update rejections with `0.685 rad` proposed, pointing at rotation coupling rather than data starvation. Ceres trust-region radii are exposed for online-window tuning: a `0.05` radius suppresses 4 s rotation rejections, but the 12 s CBD probe remains near the zero-path fallback (`0.145 m` RMSE, `0.016 m` path vs `0.917 m` baseline), so this is a diagnostic control rather than the parity fix. LiDAR plane-normal, independently weighted LiDAR scan-to-map SE(3) pose priors, bias-hold sweep controls, and default-off second-difference position smoothness factors are now available for diagnostics; local 12 s sweeps show they can reduce rejections, add 140k+ scan-map matches, or tune path scale. Limited-rotation updates now optionally scale position by the same ratio, fixing the earlier 17m path blow-up and giving a finite `0.957/1.002 m` 12 s path-scale probe, but ATE still does not close. The update gate also exposes a default-off edge-knot margin for guard-knot diagnostics; 12 s CBD margin-2 probes did not improve ATE (`0.203 m` without limited rotation, `0.205 m` with limited rotation), so it remains a tuning hook rather than a parity default. Default-off SO(3) second-difference smoothness and LiDAR scan-to-map velocity priors now give Ceres real rotation-shape and motion-scale residuals. The best 12 s CBD probe so far, `results/fastlivo2/CBD_Building_01_continuous_time_rot_smooth_0p01_lidar_pose_pos5_vel1_12s/trajectory_compare_report.json`, reaches RMSE `0.198 m`, zero rejected steps, and path `0.990/0.988 m`. The matching 60 s run keeps path scale finite at `10.089/10.603 m` with zero rejected steps, but RMSE remains `1.806 m`; a dense scan-to-map sweep worsens path scale, so full-sequence RMSE-gated parity now points at long-window trajectory shape/visual-global coupling rather than map capacity. A default-off image-to-image visual rotation-prior diagnostic now subscribes to raw camera frames in the continuous-time parity path with bounded image subsampling and frame stride. The 12 s stride-3 probe produces 36-40 visual priors with zero point-cloud drops, but worsens path scale (`4.58-10.12 m` vs `~0.98 m` baseline depending on sign/scale), so it is archived as evidence that naive monocular rotation coupling is insufficient. The standalone continuous-time node now also has a default-off sparse-depth visual SE3 prior that projects LiDAR depth into camera frames, applies SE3 photometric normal equations, transforms camera deltas through the camera-to-IMU extrinsic, and reports depth misses, valid batches, coverage tiles, mean residual, Hessian rank/condition, and prior counts. The best short 12 s CBD sweep so far (`visual_se3_max_dt_ns=250000000`, position/orientation weights `0.02/0.02`) reaches RMSE `0.19695 m`, slightly better than the no-visual `0.19797 m` checkpoint, but the matching 60 s run records RMSE `1.811 m` despite 581 SE3 batches and 298 valid priors, so it remains an experimental diagnostic rather than a paper-parity default. These are hardening checkpoints, not full-sequence paper parity.
- Latest CBD continuous-time hardening, 2026-05-13: LiDAR residual Huber gates now preserve physical units when residuals are preweighted, and native LiDAR correspondences carry Coco-LIC-style `use_lidar_scale` feature confidence. The parity script archives `pointcloud_factor_weight`, `pointcloud_use_lidar_scale`, and `pointcloud_min_lidar_scale`. The 12 s stable-weight probe at `results/fastlivo2/CBD_Building_01_lidar_scale_huber_weighted_default_w0p1_12s/trajectory_compare_report.json` records RMSE `0.116 m` and path `0.994 m` vs `0.811 m`; the 60 s run at `results/fastlivo2/CBD_Building_01_lidar_scale_huber_weighted_default_w0p1_60s/trajectory_compare_report.json` remains at RMSE `1.798 m`. Upstream-style `lidar_weight=500` no longer rejects every solve but overdrives the online window (`2.35-2.38 m` short-window path), so high LiDAR weight is not a parity default yet.
- The experimental persistent point-map path behind `enable_persistent_point_map` now feeds a coupled 3D continuous-time point-to-point residual instead of three independent axis-plane residuals, batches scan-local map insertions until after all associations are built, and only allows mature cross-stamp map entries to become association targets. The parity script archives `persistent_point_map_merge_distance_m`, `persistent_point_map_min_match_age_s`, and `persistent_point_map_min_observations_for_match` so point-anchor sweeps are reproducible. This fixes the residual/Jacobian/update semantics needed by Gaussian-map anchors, but it remains disabled as a parity default: CBD 12 s probes with mature 3D anchors still sit around RMSE `0.144-0.145 m` and shorten path length to `0.23-0.48 m` against a `0.92 m` reference.
- Sparse-depth visual SE3 velocity priors are also wired as a default-off diagnostic. Local 12 s CBD sweeps with velocity-only and mixed position/velocity/orientation weights show the factor is observable in Ceres, but it currently shortens path scale and does not improve ATE, so it stays off by default.
- LiDAR scan-to-map angular-velocity priors are wired on the continuous-time SO(3) derivative. The 10 Hz output trajectory stats exposed excessive long-window angular speed, and a 60 s CBD sweep with `lidar_pose_prior_angular_velocity_weight=0.1` reduces path drift from `264.75%` to `184.79%` while lowering p95 speed and angular speed. Full-data CBD is the stricter gate, though: `results/fastlivo2/CBD_Building_01_ct_stamp_driven_full_angvel01_output10hz/native_tracking_report.json` keeps RMSE at `3.664 m` and worsens path drift to `130.86%`, so angular velocity remains opt-in instrumentation rather than a default parity setting.
- Two-stamp scan-to-scan relative pose priors are now implemented in the continuous-time Ceres backend as default-off relative translation and SO(3) residuals between B-spline query times. Batch and streaming-window probes verify that the factors constrain relative motion without requiring a global anchor. Real CBD 12 s ablations prove the path is live but not yet a parity setting: relative position+orientation priors at weight `0.1` record RMSE `0.144711 m` with path `0.0355/0.9159 m`, and orientation-only records RMSE `0.144454 m` with path `0.0548/0.9159 m`. The remaining blocker is still long-window trajectory-shape and visual/Gaussian-map coupling, not missing relative-pose plumbing.
- Scan-to-scan target-quality gates now record raw target/prediction ratios and can either use the predicted relative translation or skip translation priors when ICP collapses the target motion. CBD 12 s probes show the gate is useful instrumentation but not the parity fix: ratio `0.5` prediction fallback fires 39 times and raises cumulative target path to `7.92 m`, yet the optimized path remains `0.045/0.914 m`; stronger weights pull path to `0.338 m` and then overdrive it to `2.41 m` without improving RMSE. This keeps the new knobs default-off and points the next sprint at global visual/Gaussian-map constraints rather than scan-to-scan scalar tuning.
- Gaussian/visual feedback replay now defaults the mapper feedback sync tolerance to `0.05 s` instead of `0.01 s`. A full120 CBD attempt with the old tolerance received points/poses/images but aligned zero mapper frames, so no Gaussian snapshot or visual/SE3 factors entered BA. The mapper feedback script now exposes `--mapper-feedback-render-mode`, and `--enable-gaussian-map-feedback` promotes it to real `rasterizer` feedback unless explicitly overridden. Torch/rasterizer preview fallback is map-only instead of observed-image-backed, which fixes the earlier fake-zero photometric residual path. Native reports now also record structured mapper Gaussian backend counters from `/gaussian_lic/status`; those counters exposed that alpha-hole filtering kept the feedback map stuck near 16k Gaussians. The production preset now disables that filter for feedback replay, enables uniform pruning, and caps foreground Gaussians at 400k. The tuned 60 s production candidate at `/tmp/gaussian_feedback_capped_growth_60s_probe/native_tracking_report.json` has 525 matched poses, 0.585 m RMSE, 7.30% path drift, a complete 400k Gaussian snapshot, and 284 valid visual/SE3 photometric BA factors. The full120 capped real-rasterizer checkpoint is archived at `results/fastlivo2/CBD_Building_01_gaussian_rasterizer_feedback_full120_capped400k/native_tracking_report.json`; it has 1143 matched poses, 92.85% coverage, 2.488 m RMSE, 23.65% path drift, nonzero visual/SE3 photometric residuals, 400k mapped Gaussians, 749 mapper extend calls, and 725 cap-prune calls. This is real production BA evidence, not strict paper parity yet.
- 2026-05-13 online CUDA optimization hardening: native mapper feedback now distinguishes world-frame mapper-contract point clouds from native tracking `sensor/body` point clouds and applies the camera-to-IMU extrinsic before creating `TorchCamera` records. This fixes the runtime failure where `enable_torch_gaussian_optimization=true` still reported `gaussian_optimization_count=0`; the diagnostic probe showed `depth_valid>400k` but `in_bounds=0` because IMU/body poses were being used as camera poses. `run_native_tracking_bag_report.sh --enable-gaussian-map-feedback --mapper-feedback-torch-optimization-steps 1` now passes a 12 s CBD gate with `gaussian_optimization_count=87`, `gaussian_optimization_steps=87`, supervised Gaussians around `287k`, and a complete 400k-Gaussian snapshot at `/tmp/gaussian_feedback_opt_camera_extrinsic_12s_probe/native_tracking_report.json`. The current best 60 s online-optimization preset runs optimization every 16 keyframes with conservative learning rates; `/tmp/gaussian_feedback_opt_camera_extrinsic_60s_every16_probe/native_tracking_report.json` records `23` real CUDA optimization steps, `244,777` supervised Gaussians, `43.22%` reference coverage, RMSE `0.745 m`, and `8.30%` path drift. The same corrected external-camera no-opt control is still slightly better at RMSE `0.690 m`, so low-frequency online optimization is live and gated but remains a tuning surface, not a strict paper-parity claim.
- Mapper feedback now applies the configured `max_depth` before pending points enter the Torch Gaussian map, not only during later projection/optimization, and Gaussian-LIC mapper profiles require successful projected image color before inserting non-RGB LiDAR points. This keeps native `sensor` point-cloud semantics while preventing far-range or out-of-camera-FOV intensity-only points from polluting the photometric map; the Gaussian feedback replay preset now inherits the mapper-profile `20 m` depth gate instead of the earlier diagnostic `200 m` setting. An opt-in `--mapper-feedback-zbuffer-projected-points` ablation keeps only the nearest projected non-RGB LiDAR point per image pixel, but it remains default-off after the 60 s CBD probe worsened RMSE. MappingStatus reports nonpositive-depth, max-depth, unprojected, and occluded skipped point counters so replay artifacts can prove the input-boundary gates are active.
- The native feedback report script now exposes and archives `--mapper-feedback-select-every-k-frame`, `--mapper-feedback-allow-unprojected-color-fallback`, `gaussian_snapshot_lidar_factor_weight`, and the projected-color gate so mapper/tracker ablations are reproducible. Fresh 60 s CBD ablations did not beat the current `sensor + max_depth=20 + projected-color + every-frame + 400k` preset (`0.715 m` RMSE, `43.2%` coverage): visual weight `0.001` regressed to `0.935 m`, Gaussian snapshot LiDAR weight `0.5` to `0.912 m`, mapper select-every-2 to `1.215 m`, unprojected grayscale fallback to `0.981 m`, world-coordinate projected points to `0.747 m` with worse drift, world+fallback to `1.029 m`, and `300k` foreground cap to `1.090 m`. The next strict-parity target is therefore the BA factor/trajectory-shape coupling, not another replay preset flip.
- Gaussian snapshot BA diagnostics now also expose and archive a tracking-side nearest-distance gate plus residual-preweight toggle. These controls are live but not parity defaults: disabling residual preweight regressed the 60 s CBD run to `1.231 m` RMSE, tightening Gaussian nearest-neighbor matching to `0.35 m` regressed to `0.921 m`, and enabling Gaussian point-to-plane anchors at default weight regressed to `0.946 m`. The current best measured default therefore still keeps residual preweight on, inherits the LiDAR nearest-distance gate, and leaves plane anchors opt-in while the remaining work focuses on trajectory-shape/global coupling.
- SE3 photometric BA weight is now tuned to `0.5` in the Gaussian feedback preset. The 60 s CBD sweep improved from the projected-color default `0.715 m` RMSE at weight `0.25` to `0.649 m` at weight `0.5`; higher weights over-pull the window (`0.75 -> 0.720 m`, `1.0 -> 0.856 m`). This is a real default improvement, but strict parity still needs the remaining late-window drift removed and the full120 gate rerun.
- Native tracking reference comparisons now archive 8 time-ordered error bins by default through `run_native_tracking_bag_report.sh --reference-error-bin-count` and run a default `[-0.5, 0.5] s` timestamp-offset sweep through `--reference-time-offset-sweep`, so each report shows where late-window drift starts and whether it is just a replay-time offset. The latest 60 s CBD factor-weight probes make native LiDAR plane weight the best new preset lever: plane weight `2.0` improves to `0.792 m` RMSE, `4.0` records `0.750 m` then `0.723 m` on repeat, and the no-explicit-flag default confirmation records `0.825 m`; `8.0` records the best single run (`0.705 m`) but does not reproduce as a stable default (`1.006 m` on confirmation); `16.0` regresses to `0.799 m`, point weight `2.0` gives `0.822 m`, and point+plane `2.0` regresses to `1.201 m`. The Gaussian feedback preset therefore promotes `lidar_window_plane_factor_weight=4.0`. The adjacent-state relative-translation BA factor is now wired from raw IMU/pre-LIO motion into the same sliding window, archived by the native report, and promoted by the Gaussian feedback preset at `weight=0.1` because it improves full120 CBD from the plane4 `2.345 m` RMSE checkpoint to `1.895 m` with `95.13%` coverage and `20.73%` path drift. Shorter 60 s repeats still show variance (`0.702 m` then `0.833 m`), and bins 5/6 remain the long-window failure (`2.80/3.10 m` RMSE, roughly `-2.63 m` Y bias), so this is not strict paper parity yet. The default-off multi-hop relative-translation diagnostic now also exposes `sliding_window_multihop_relative_translation_max_factors`, allowing several longer-baseline pre-BA motion factors per frame while preserving the old default of one. New 2026-05-14 full120 probes show this is a real long-window lever when combined with the fixed step guard: `tracking_max_pose_step_m=0.0265`, adjacent relative weight `0.10`, and multihop `max3/w0.05` records `1066` matches, `86.60%` coverage, RMSE `2.492 m`, and `16.86%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_step00265_multihop005_full120_probe/native_tracking_report.json`; the more aggressive `0.028` guard lowers drift to `11.83%` but worsens RMSE to `2.609 m`, and `0.0260` regresses to `21.39%` drift. Gaussian snapshot plane anchors, low-frequency CUDA optimization, and position-velocity consistency remain non-default after 60 s probes (`0.863 m`, `0.789 m`, and `1.068 m`).
- Sliding-window BA sweep controls now expose and archive `sliding_window_max_states` and `sliding_window_max_iterations`. Directly enlarging the online BA window is not yet production-viable: 24 states processed only `10.8 s` of a 60 s CBD replay, and 16 states required slow replay to cover 60 s but regressed to `1.139 m` RMSE. Iteration/cadence sweeps also did not beat the default 12-state, 3-iteration, optimize-every-4 preset: 5 iterations reached `1.110 m`, 1 iteration `1.161 m`, optimize-every-2 failed coverage with `0.818 m` over the partial window, optimize-every-8 reached `1.104 m`, and post-BA reject-over-`0.04 m` reached `1.098 m` with worse drift. The remaining blocker is factor quality/global trajectory-shape coupling, not a hidden window-size or iteration-count knob.
- The sliding-window Schur marginalization prior weight is now exposed through launch and `run_native_tracking_bag_report.sh --sliding-window-marginalization-prior-weight`, then archived in `native_tracking_report.json::gate_config`. A 12 s CBD smoke proves the value is live (`0.5`) with Gaussian feedback enabled and the relative-translation preset active. The 60 s archived sweep records `0.5` as the best tested value: `results/fastlivo2/CBD_Building_01_gaussian_feedback_marginalization_w05_60s_probe/native_tracking_report.json` reaches RMSE `0.667 m`, coverage `40.86%`, and path drift `12.30%`; `0.25` regresses to RMSE `0.771 m`, and `0.75` records RMSE `0.670 m` but misses 40% coverage with `20.09%` path drift. Full120 rejects this candidate: `results/fastlivo2/CBD_Building_01_gaussian_feedback_marginalization_w05_full120_probe/native_tracking_report.json` reaches only RMSE `2.472 m`, coverage `89.93%`, and path drift `24.07%`, worse than the current `relative_w010` full120 checkpoint. This stays a diagnostic sweep surface, not a default parity setting.
- Post-BA step guarding now has a default-off pre-BA agreement allowance: `--post-ba-step-guard-pre-ba-agreement-max-pose-step-m` only raises the post-BA clamp when the optimized step agrees in direction and magnitude with the pre-BA LiDAR/IMU pose. The 12 s CBD smoke proves the parameters are live and archived, but RMSE regressed to `0.184 m`, so the production preset still keeps the fixed `0.025 m` guard while full-window path-length recovery moves to stronger global visual/Gaussian constraints.
- The Gaussian feedback preset now enables tracking-side visual and SE3 photometric BA factors by default. A 12 s regression with only `--enable-gaussian-map-feedback --mapper-feedback-torch-optimization-steps 1` records `75` visual factors, `75` SE3 photometric factors, `24` accepted feedback updates, `5` CUDA optimization steps, and a complete 400k-Gaussian snapshot at `/tmp/gaussian_feedback_visual_default_12s_regression/native_tracking_report.json`. Step-guard ablations remain conservative: 60 s `tracking_max_pose_step_m=0.05` improves partial-window RMSE to `0.664 m` but overdrives path drift to `108%` and drops coverage to `37.6%`, while `0.035` worsens RMSE to `0.976 m`; the production preset therefore keeps the proven `0.025` guard until a confidence-gated speed model is validated.
- Gaussian snapshot point-to-plane anchors are now available as a default-off production BA experiment through `--enable-gaussian-snapshot-lidar-plane-factor`. The tracker consumes each Gaussian's rotation and anisotropic scale, selects the thinnest local axis as a surface normal, filters by `1 - min(scale) / max(scale)`, and injects robust LiDAR-to-Gaussian plane factors into the same sliding window as visual/SE3 feedback. The C++ probe validates normal extraction and factor construction; a 12 s CBD feedback run at `/tmp/gaussian_feedback_plane_anchor_12s_probe/native_tracking_report.json` records `24,756` total plane correspondences, proving the path is live. The 60 s ablations are not yet a parity default: weight `0.25` gives RMSE `1.019 m`, while weight `0.05` improves to RMSE `0.765 m` but misses the `40%` coverage gate at `38.67%`. This keeps plane anchors as reproducible opt-in instrumentation until they beat the current conservative Gaussian/visual preset on long windows.
- LiDAR point-to-plane pose correction iterations are now exposed through `lidar_pose_factor_iterations` and the report script's `--lidar-pose-factor-iterations` switch. The unit probe already shows multi-iteration correction reduces synthetic pose error, but the CBD 60 s production-feedback ablation with `4` iterations reaches only RMSE `1.096 m` and `38.75%` coverage at `/tmp/gaussian_feedback_lidar_iter4_60s_probe/native_tracking_report.json`, so the default remains `1` while long-window trajectory-shape tuning continues.
- Full120 Gaussian/visual feedback now has an archived yaw-gated health pass at `results/fastlivo2/CBD_Building_01_gaussian_visual_feedback_full120_yaw_gate_fixed/native_tracking_report.json`. The run records `1143` odometry poses, `1144` mapper point frames, `92.20%` reference coverage, `1135` yaw-aligned matches, RMSE `2.513 m`, mean `2.338 m`, max `4.591 m`, and `24.48%` path drift, with a complete `34,827`-point Gaussian snapshot, `814` visual factors, `814` SE3 photometric factors, `285` feedback updates, non-degenerate BA, and `sliding_window_orphan_factors=0`. This proves the production Gaussian/visual BA chain can run full-length, but it is still a meter-level tracker-quality checkpoint, not strict paper parity.
- Tracking step-guard diagnostics now report total clamp counts, pre-LIO clamp counts, post-BA clamp counts, the last confidence score, and the last raw/allowed step, dt, and reference speed. Adaptive, stage-specific, and confidence-gated post-BA step-guard knobs are exposed through `tracking.launch.py` and `run_native_tracking_bag_report.sh`, but remain opt-in: a 30 s CBD probe with velocity/acceleration scaling proved the parameters are live by raising current step p95 to `0.073 m`, but it overdrives path length to `10.30 m` vs `2.82 m` reference and worsens RMSE to `0.674 m`. The 60 s stage ablations show why the fixed production guard remains in place: disabling only the pre-LIO guard gives RMSE `0.913 m`, `38.18%` coverage, and `38.39%` path drift at `/tmp/gaussian_feedback_no_pre_guard_60s_probe/native_tracking_report.json`; disabling only the post-BA guard gives RMSE `0.695 m`, `37.12%` coverage, and `53.02%` path drift at `/tmp/gaussian_feedback_no_post_guard_60s_probe/native_tracking_report.json`; setting only the post-BA limit to `0.035 m` gives RMSE `0.721 m`, `38.91%` coverage, and `45.08%` path drift at `/tmp/gaussian_feedback_post0035_guard_60s_probe/native_tracking_report.json`; the confidence-gated `0.035 m` post-BA allowance with visual residual gate `0.2` passes the report but still records RMSE `0.750 m` and `35.23%` path drift at `/tmp/gaussian_feedback_conf_guard_60s_probe/native_tracking_report.json`. The Gaussian feedback preset therefore keeps the measured fixed `0.025 m` guard for rasterizer feedback until a global trajectory-shape constraint is proven on full windows.
- Stage-specific velocity step-guard scales are now exposed as default-off controls: `pre_lio_tracking_step_guard_velocity_scale` and `post_ba_tracking_step_guard_velocity_scale` can release only one guard stage while preserving the other. CBD Gaussian-feedback reports prove the new pre-LIO-only path is live (`pre_lio=1.0`, `post_ba=0.0`, hard velocity cap `0.6 m/s`): the 60 s probe at `results/fastlivo2/CBD_Building_01_gaussian_feedback_pre_lio_velocity_scale_60s_probe/native_tracking_report.json` reaches RMSE `0.607 m` with `40.13%` coverage, but path drift rises to `18.97%`; the full120 probe at `results/fastlivo2/CBD_Building_01_gaussian_feedback_pre_lio_velocity_scale_full120_probe/native_tracking_report.json` regresses to RMSE `2.844 m` and `89.28%` coverage. It remains a diagnostic surface rather than a production preset.
- Post-BA clamp direction blending is also exposed as `post_ba_step_guard_pre_ba_blend_on_clamp`, default-off. It blends an over-limit post-BA step direction toward the same-stamp pre-BA LiDAR/IMU direction before applying the step clamp. The CBD 12 s Gaussian-feedback smoke at `results/fastlivo2/CBD_Building_01_gaussian_feedback_post_ba_preblend_12s_smoke/native_tracking_report.json` proves the report path is live at blend `0.5`, but RMSE regresses to `0.173 m`, so the knob is archived as a failed diagnostic rather than run full120.
- Visual/SE3 pairing-window widening is archived as a non-default diagnostic. Increasing `visual_factor_max_dt_ns` from `300 ms` to `500 ms` lowers the 60 s RMSE to `0.539 m` at `results/fastlivo2/CBD_Building_01_gaussian_feedback_visual_dt500ms_60s_probe/native_tracking_report.json`, but the full120 run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_visual_dt500ms_full120_probe/native_tracking_report.json` regresses to RMSE `2.410 m`, coverage `88.95%`, and path drift `23.49%`. The default stays at the stricter timing window because the long-window gate rejects this looser timestamp pairing.
- `trajectory_compare.py` now has `--error-bin-count` for time-ordered drift-shape diagnostics. On the current 60 s default, the last eighth of the trajectory is the worst bin (`1.144 m` RMSE, bias `[+0.60, -0.71, +0.40] m`) while current step p95 is capped at `0.025 m` versus reference step p95 `0.0558 m`. A controlled velocity-guard retry with hard cap `0.6 m/s` removes some clamps but over-expands path drift to `105.9%` and RMSE `0.977 m`; a stricter post-BA-only visual confidence gate at `0.06 m` still regresses to RMSE `0.848 m` and `42.1%` drift. The next fix must strengthen global visual/Gaussian factors rather than simply releasing the step guard.
- The analytic relative-motion checkpoint confirms the same guard diagnosis on 2026-05-14: its best full122 accepted run is path-short (`24.67 m` current vs `29.20 m` reference), but direct velocity-based releases are unsafe. The max6/light-rotation 60 s probes with `tracking_step_guard_velocity_scale=1.0` and feedback velocity caps `0.8 m/s` and `0.45 m/s` both over-expand the path (`189.75%` and `87.25%` path drift), so the next release-quality guard needs visual/Gaussian/geometric agreement rather than speed alone.
- Strict native replay now exposes `--playback-start-offset` and `--clock-topics-all`, and records both in `gate_config`, so ROS1-style rosbag timing ablations are auditable instead of implicit. A CBD 60 s diagnostic with `--clock-topics-all --playback-start-offset 0.5` removes the startup IMU-history point-cloud drop, but regresses to `1.238 m` RMSE and `15.13%` path drift, so start-offset replay is a diagnostic rather than a parity default.
- Native tracking now exposes `lidar_window_point_factor_weight` and `lidar_window_plane_factor_weight` through the launch/report path so BA weight sweeps can balance LiDAR map anchors against Gaussian photometric factors without code edits. The first CBD 60 s down-weight probe (`lidar_window_point_factor_weight=0.25`) regressed to `1.068 m` RMSE, so the Gaussian feedback preset keeps the default `1.0` LiDAR point/plane factors.
- A default-off sliding-window position-velocity consistency residual now ties the finite-difference position rate to the optimized state velocity, giving the BA a real trajectory-shape residual instead of only second-difference smoothness. Local CBD sweeps confirm the residual is active but not yet a production default: weight `0.05` reduces path drift to `15.17%` but worsens RMSE to `1.081 m`; weight `0.01` reaches RMSE `0.635 m` on the 60 s window but misses coverage at `38.91%`, and the matching 70 s coverage retry regresses to RMSE `1.333 m` with `24.15%` path drift; weight `0.005` regresses to RMSE `1.059 m`. The analytic Jacobian probe now covers the residual's position-velocity rows, but the fixed `0.025 m` guard remains the documented Gaussian feedback preset until a trajectory-shape residual beats the current checkpoint and holds on full120.
- Native tracking evidence now records online `first`/`last`/`min`/`max`/`mean`/`delta` summaries for every numeric `TrackingStatus` and mapper `MappingStatus` field in `metrics.json`, and retains compact time-binned MappingStatus summaries at final shutdown. This turns each bag report into a time-series health artifact for step-guard clamps, visual/SE3 quality, Gaussian snapshot growth, mapper optimization counters, mapper feedback throughput, and BA conditioning instead of relying on only the final status sample.
- Native tracking evidence also retains compact time-binned status summaries at `metrics.tracking_status.binned_summary`. The recorder now writes those bins only during final shutdown flush so the 1 Hz periodic metrics writer does not construct large JSON payloads during strict replay. New full120 artifacts can align the 8 trajectory error bins with LiDAR confidence, visual/SE3 factor quality, BA cost, step-guard clamps, Gaussian snapshot health, and IMU/deskew counters instead of relying on one global aggregate when late-window drift appears.
- Native tracking evidence now treats callback drain as a multi-signal gate instead of only a pose-count gate. The recorder handles SIGINT/SIGTERM by forcing the final binned-summary flush, `run_native_tracking_bag_report.sh` can wait for target trajectory poses, visual factors, and SE3 photometric factors before shutdown, and `MappingStatus` now publishes per-stream input counts, drop counts, pending queue sizes, rendered-preview count, and render errors. The CBD 12 s smoke at `results/fastlivo2/CBD_Building_01_evidence_drain_visual_se3_12s_probe/native_tracking_report.json` passes with drain targets `80/50/50` and reaches `119` poses, `96` visual factors, `96` SE3 factors, `86` rendered previews, zero render errors, and zero pending mapper queues. This prevents a late-window trajectory tail with incomplete mapper feedback from being mistaken for a valid full-window parity run.
- Measured-motion smoothness targets remain rejected as a production path. The no-recent-support full130/drain1180 run reached `1174` matches, `95.37%` coverage, `1.418 m` RMSE, `3.72%` path drift, and bin7 `2.195 m`, but regressed aggregate RMSE and failed visual/SE3 bin continuity. The recent-support-gated sibling reduced motion-target applications to `8`, yet regressed to `1.623 m` RMSE and bin7 `2.503 m`. A post-BA guarded-state sync hypothesis is also default-off after `results/fastlivo2/CBD_Building_01_gaussian_feedback_guarded_state_sync_full122_drain1015_probe/native_tracking_report.json`: it recorded `1174` matches but only `728` visual/SE3 factors, RMSE `1.404 m`, and `6.48%` path drift, so it did not beat the accepted `1.346 m / 5.00%` preset. The next numerical work stays on late-window visual/Gaussian factor continuity and estimator coupling, not more motion-target tuning.
- Native tracking drift diagnostics can now additionally write per-sample compact status history with `run_native_tracking_bag_report.sh --write-status-history` and post-process it through `scripts/diagnose_native_tracking_drift.py`. The diagnostic emits bias trajectory, factor-count, Hessian-health, and reference-motion-regime CSV/PNG artifacts. The 2026-05-14 CBD full120 diagnosis found that widening the active window to 30 states is not a production fix because it only processed `98` poses before replay shutdown; the production fix instead delays window marginalization until the current frame's state priors, IMU, LiDAR, visual, SE3, and relative factors are all inserted. The regression probe verifies that the retained dense prior remains full-rank (`45/45`) when a new current-frame prior would previously have been missed. A complete 12-state status-history run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_step00265_relrot002_mhrot001_status_full120_probe/diagnostics/diagnostic_summary.json` records `1047` status samples, stable Hessian health (`lambda_min` `0.05670 -> 0.05669` from 70 s to end), continuing factor counts, and a post-70 s accel-bias norm increase (`0.486`), pointing at long-window trajectory-shape/measurement coupling rather than a pure marginalization rank collapse. IMU bias continuity now exposes separate gyro/accel multipliers for ablations while keeping the old default behavior, and `TrackingStatus` now publishes effective gyro/accel bias random-walk sqrt-information mean/max so over-tight bias priors are visible in every report. Simple bias locking is rejected by evidence: `gyro/accel_bias_prior=0.5` with stronger bias continuity regresses to `2.592 m` RMSE, stronger bias smoothness without insertion priors regresses to `2.888 m`, accel-only tightening (`accel_bias_weight=4.0`) regresses to `3.300 m` with coverage failure, and accel-only relaxation (`accel_bias_weight=0.25`) regresses to `3.044 m` with coverage failure. A 2026-05-17 full122 replay with raw upstream Coco-LIC random-walk sigmas (`2.617993833e-07`, `1.96e-06`) is also rejected at `results/fastlivo2/CBD_Building_01_gaussian_feedback_cocolic_bias_sigma_raw_full122_probe/native_tracking_report.json`: coverage remains `95.37%`, but RMSE regresses to `3.023 m` and path drift to `44.66%` because the current path is compressed to `18.46 m` vs the `33.35 m` reference.
- Relative-motion BA now carries an optional rotation residual in the same adjacent/multi-hop factor used for pre-BA translation shape, exposed through `sliding_window_relative_rotation_weight` and `sliding_window_multihop_relative_rotation_weight` while preserving the old default of zero. CBD full120 evidence shows this is a real long-window improvement: the previous drift-parity candidate (`step=0.0265`, adjacent relative `0.10`, multihop `max3/w0.05`) was `2.492 m` RMSE and `16.86%` path drift, while `relative_rotation=0.02` plus `multihop_relative_rotation=0.01` reaches `1047` matches, `85.05%` coverage, `2.264 m` RMSE, mean `2.141 m`, max `4.153 m`, and `15.50%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_step00265_relrot002_mhrot001_full120_probe/native_tracking_report.json`. The relative-motion translation/rotation blocks now have closed-form Jacobians for both world-frame and local-frame modes instead of runtime numeric fallback; the 2026-05-14 CBD 20 s analytic smoke at `results/fastlivo2/CBD_Building_01_gaussian_feedback_step00265_relrot002_mhrot001_analytic_relrot_20s_probe/native_tracking_report.json` records `115` reference matches, `0.162 m` RMSE, and `sliding_window_numeric_jacobian_blocks/columns` max `0`. With analytic Jacobians, the best accepted RMSE checkpoint so far is the lighter-rotation max6 run at `results/fastlivo2/CBD_Building_01_gaussian_feedback_step00265_relrot001_mhrot0005_mhtrans003_max6_analytic_full122_probe/native_tracking_report.json`: `1038` matches, `84.32%` coverage, `2.210 m` RMSE, mean `2.037 m`, max `4.232 m`, and `15.51%` path drift, all with numeric Jacobian max `0`. The stronger max6 full122 sibling reaches the same RMSE class (`2.210 m`) but higher drift (`16.41%`), and full125 regresses to RMSE `2.416 m`; the controls remain opt-in until cross-dataset strict parity is proven.
- Direction-agreement step-guard release is the current best native CBD trajectory-shape checkpoint and is promoted into the `--enable-gaussian-map-feedback` preset. Under the slow feedback replay preset, max6/light relative rotation plus `post_ba_step_guard_pre_ba_agreement_max_pose_step_m=0.045`, `min_cosine=0.95`, and `max_delta=0.02` reaches `1174` matches, `95.37%` coverage, `1.346 m` RMSE, mean `1.219 m`, max `2.829 m`, and `5.00%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_rate0p25_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json`. The matching `0.040` run records `1.510 m` RMSE and `6.82%` drift, `0.0475` regresses to `1.451 m` RMSE with `4.31%` drift, and `0.050` lowers drift to `3.52%` but regresses RMSE to `1.426 m`; the preset therefore chooses the ATE-best `0.045` rather than the lower-drift but less accurate `0.050`. Confidence-gated release to `0.065` nearly matches reference path length (`0.44%` drift, speed p95 `0.640 m/s` vs reference `0.616 m/s`) but regresses RMSE to `1.692 m`; the new default-off `post_ba_step_guard_confidence_warmup_marginalizations` gate keeps that release disabled until a chosen Schur-marginalization count, but warmup `700` and `1000` still regress to `1.507 m` and `1.475 m` RMSE, respectively. A stronger visual/SE3 weight run (`0.5`/`1.0`) regresses to `1.526 m` RMSE with `4.67%` drift, so these stay diagnostics. This is a substantial improvement over the previous `1.895 m / 20.73%` best, but not yet a 100% paper/super-paper parity claim.
- Gaussian-feedback strict replay now defaults to `--rate 0.25` when feedback is enabled and no explicit `--rate` is passed. This is a replay-throughput guard, not a live-runtime shortcut: the full122 diagnostic at `results/fastlivo2/CBD_Building_01_gaussian_feedback_rate0p25_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/native_tracking_report.json` restores visual/SE3 factor continuity from the previous intermittent `242` total factors to `1005`, records `1173` reference matches (`95.29%` coverage), and improves RMSE to `1.510 m` with `1.378 m` mean error and `3.020 m` max error. Path drift rises to `6.82%`, so this is accepted as the new data-correct replay preset while the next optimization target is trajectory length/shape under continuous visual feedback.
- SE3 photometric mean-residual gating is live but not a default parity setting. `--se3-photometric-max-mean-abs-residual 0.2` improves the 60 s Gaussian-feedback probe to RMSE `0.598 m` and `12.18%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_se3_residual_gate02_60s_probe/native_tracking_report.json`, but the matching full120 run reaches only RMSE `2.300 m`, coverage `89.93%`, and path drift `22.61%` at `results/fastlivo2/CBD_Building_01_gaussian_feedback_se3_residual_gate02_full120_probe/native_tracking_report.json`. The full120 binned diagnostic with final-only recorder flush is archived at `results/fastlivo2/CBD_Building_01_gaussian_feedback_relative_w010_binned_final_full120_probe/native_tracking_report.json` and remains worse than the best checkpoint (`2.739 m` RMSE), so these are triage artifacts rather than a new production preset.
- LiDAR/Gaussian correspondence confidence shaping is now exposed as default-off `lidar_window_confidence_power`, forwarded by `tracking.launch.py` and `run_native_tracking_bag_report.sh`, and archived in `gate_config`. The CBD 12 s smoke proves the launch/report path is live at `2.0`, but the longer probes reject it as a production preset: power `2.0` records 60 s RMSE `1.020 m`, power `1.5` improves the 60 s RMSE to `0.530 m` but regresses full120 to RMSE `2.603 m`, coverage `90.01%`, and path drift `24.51%`, and power `1.5` plus IMU velocity prior `0.1` records 60 s RMSE `0.603 m` with `15.38%` path drift. IMU velocity priors also stay opt-in: weight `1.0` improves 60 s path drift to `6.10%` but worsens RMSE to `1.477 m`, while weight `0.1` records RMSE `0.720 m` and `9.19%` drift. A small fixed step-guard release to `tracking_max_pose_step_m=0.028` improves full120 path drift to `14.60%` but worsens RMSE to `2.036 m`, so the strict preset remains the `relative_w010` checkpoint while the remaining blocker stays long-window trajectory-shape/global visual-map coupling.
- Gaussian snapshot scan-to-map pose correction is now exposed as a default-off diagnostic through `enable_gaussian_snapshot_lidar_pose_correction`, with bounded gain, translation, and rotation limits archived in `gate_config`. The 12 s CBD smoke verifies the full launch/report/runtime path. A conservative 60 s probe with `gaussian_snapshot_lidar_nearest_distance_m=0.35`, gain `0.1`, max translation `0.015 m`, and max rotation `0.005 rad` reduces path drift to `4.97%` but still records RMSE `0.734 m`; the full120 replay rejects it as a preset with RMSE `2.741 m` and path drift `22.65%` at `results/fastlivo2/CBD_Building_01_gaussian_snapshot_pose_correction_g01_t015_nn035_full120_probe/native_tracking_report.json`. The follow-up `min_match_ratio=0.2` and `max_mean_residual=0.15 m` quality gate is live but not sufficient, recording 60 s RMSE `0.769 m` and `9.11%` path drift. A 4x4/min6 spatial coverage gate records RMSE `0.855 m` and `7.34%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_snapshot_pose_correction_coverage_gate_60s_probe/native_tracking_report.json`. A mutual-nearest bidirectional gate improves 60 s RMSE to `0.706 m` with `7.58%` drift, but full120 still rejects it with RMSE `2.546 m` and `23.25%` drift at `results/fastlivo2/CBD_Building_01_gaussian_snapshot_pose_correction_bidirectional_full120_probe/native_tracking_report.json`. This confirms the global-map correction route is useful instrumentation, but strict parity needs deeper geometry consistency or direct global visual/Gaussian residuals before applying scan-to-Gaussian pose updates.
- SE3 photometric gradients can now be switched to the rendered/map image with default-off `se3_photometric_use_rendered_gradient`, and the report archives the choice. The CBD 12 s smoke proves the path is live, but the 60 s evidence rejects it as the next preset: rendered gradients alone reach RMSE `0.631 m` and `15.09%` path drift at `results/fastlivo2/CBD_Building_01_gaussian_feedback_rendered_gradient_60s_probe/native_tracking_report.json`, while rendered gradients plus the `0.2` residual gate regress to RMSE `0.731 m` despite lower `8.04%` path drift. This rules out a simple observed-vs-rendered gradient flip as the missing full120 visual/Gaussian constraint.
- SE3 photometric sample-count sweeps are now reproducible through `run_native_tracking_bag_report.sh --se3-photometric-max-samples` and archived in `gate_config`. The current `2000`-sample default is not the bottleneck: `4000` samples regresses to RMSE `0.633 m` with `22.30%` path drift, and `1000` samples regresses to RMSE `0.702 m` with `13.88%` path drift on the CBD 60 s Gaussian-feedback gate. This points the next visual/Gaussian work at sample quality/global factor shape, not sample count.
- SE3 photometric sample quality is now testable with default-off `se3_photometric_rank_samples_by_gradient`, which keeps the existing coverage-tile quotas but ranks each tile's sparse-depth candidates by image-gradient magnitude before selecting samples. The 60 s CBD gate shows the tradeoff: ranking alone records RMSE `0.664 m` with `9.70%` path drift, and ranking plus residual gate `0.2` improves short-window RMSE to `0.552 m` but raises drift to `22.45%`. The matching full120 run regresses to RMSE `2.542 m`, coverage `90.25%`, and path drift `23.56%`, so gradient ranking stays diagnostic and the remaining blocker is still cross-time trajectory-shape/global map consistency.
- The post-BA reject-to-pre-BA gate is also rejected as a production default: `post_ba_step_guard_reject_to_pre_ba_over_m=0.05` records RMSE `0.867 m` and `20.73%` path drift on the 60 s Gaussian-feedback probe at `results/fastlivo2/CBD_Building_01_gaussian_feedback_reject_preba_over005_60s_probe/native_tracking_report.json`.
- The sliding-window optimizer now keeps the analytic state-prior Jacobian on the correct 15-row state residual and exposes a default-off `sliding_window_imu_velocity_prior_weight` knob. This adds a production-safe velocity-shape constraint toward the propagated IMU state for long-window drift sweeps without changing the documented default preset until real bag probes prove an RMSE gain.
- A default-off post-BA rejection gate can now send over-large BA candidates back to the pre-BA LiDAR/IMU pose through `post_ba_step_guard_reject_to_pre_ba_over_m`, with `tracking_step_guard_post_ba_rejections` published in `TrackingStatus` and archived by native reports. This targets the long-window failure mode where clamped post-BA directions repeatedly drag the trajectory while preserving the existing default behavior.
- Native tracking bag reports now merge `trajectory_compare` into `native_tracking_report.json` even when a required reference gate fails. That preserves failing RMSE/max-error/path-drift diagnostics in the main artifact instead of exiting after printing the comparison.
- The continuous-time point-cloud wait queue now covers every pose-dependent factor path, including scan-to-scan priors, Gaussian snapshot anchors, plane-normal factors, voxel-plane extraction, and persistent map constraints. That prevents ROS2 replay/callback timing from silently degrading those constraints into same-scan or stale-pose factors before the B-spline can answer the scan stamp. A 12 s pose-ready probe released all delayed point clouds with zero drops. Scan-to-scan velocity/angular-velocity priors are implemented and reproducibly exposed, but they stay opt-in because their best short-window probe (`0.096 m` RMSE) regressed the 60 s default run to `2.062 m` RMSE and `1361%` path drift. The default parity preset therefore keeps LiDAR scan-to-map pose/velocity priors plus SO(3) smoothness active while leaving scan-to-scan as a diagnostic until long-window trajectory shape is stable.
- Continuous-time parity replay now has a stamp-driven optimization mode. The ROS2 node still keeps its wall timer path for normal runtime, but the parity script enables `use_stamp_driven_steps` so Ceres solve cadence follows message stamps instead of wall-clock replay speed. A 4 s CBD check at playback rates `0.5` and `1.0` now produces the same 12 estimator steps instead of doubling the solve count under slow replay. The script also waits for `/continuous_time_node` before playing the bag, uses a fixed rosbag2 playback duration, records the capture wall duration, and defaults this parity path to playback rate `1.0` unless explicitly overridden. `PLAYBACK_DURATION=full` now resolves the bag duration from `metadata.yaml`, and parity output defaults to 10 Hz via `pose_output_period_seconds=0.1` so reports do not pass on sparse 5 Hz coverage; the native report also archives path drift, max translation error, best time-offset sweep diagnostics, and matched trajectory dt/step/speed/angular-speed statistics.
- Full-data CBD continuous-time evidence is now explicit: `results/fastlivo2/CBD_Building_01_ct_stamp_driven_full_default_stats_output10hz/native_tracking_report.json` replays the local 118.709 s frontend-raw bag with `PLAYBACK_DURATION=full`, captures 1184 odometry poses, matches 1173 reference poses (`95.29%` coverage), and records RMSE `3.665 m`, mean `3.303 m`, max `8.128 m`, path drift `64.78%`, current speed p95 `1.225 m/s` vs reference `0.613 m/s`, and current angular-speed p95 `4.141 rad/s`. This resolves the "data first" blocker for this gate and confirms the remaining strict blocker is long-window BA trajectory shape, not missing CBD input data.
- TensorRT/SPNet depth completion has a native optional wrapper and a local TensorRT 10.9 FP16 engine benchmark; the generated `.engine` is hardware/runtime-specific and intentionally not checked in.
- Strict paper reproduction now has the local `CBD_Building_01` bag, ROS1 upstream baseline artifacts, strict CUDA current collection, final-map render-pair extraction, a green mapper-contract/CUDA report, a green CBD native reference trajectory report, and continuous-time tracker producer-chain evidence across every required profile; the latest archived strict `CBD_Building_01` report is green. RMSE-gated continuous-time Coco-LIC2 tracking parity is still the next numerical target.

## Platform

Primary tested platform:

```text
Ubuntu 24.04
ROS2 Jazzy
CUDA 12.8
libtorch 2.7.0
```

Jazzy is the first-class development and CI target. Runtime helper scripts default to `ROS_DISTRO=jazzy`; other installed distros can be tried locally by exporting `ROS_DISTRO`, but Humble is not part of the required GitHub Actions matrix for this port.

On this machine, use the ASCII workspace path:

```bash
cd /home/frank/gaussian_lic_ros2
```

`/home/frank/桌面/gaussian_lic_ros2` is only a symlink. Some ROS2 interface-generation tools are fragile under non-ASCII parent paths.

## Quick Start

Build and run the default synthetic smoke test:

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build_ros2.sh
source install/setup.bash
./scripts/smoke_test.sh --tf
```

Native C++ packages default to `RelWithDebInfo` when `CMAKE_BUILD_TYPE` is not
set, so local smoke and BA timing runs exercise optimized production code while
still preserving debug symbols. Explicit `colcon build --cmake-args
-DCMAKE_BUILD_TYPE=Debug` remains honored.
Native tracking smoke status and log artifacts are isolated by ROS domain and
process id, so concurrent local/CI runs cannot race on the same `/tmp` status
file; the latest successful status is still mirrored to
`/tmp/gaussian_lic_tracking_smoke_status.txt` for inspection.
`scripts/verify_workspace.sh` also runs a strict rosbag2 timing audit on the
frontend visual MCAP bag; when ROS2 or `rosbags` readers are available it checks
per-topic message header stamp ordering, not only rosbag metadata.

Run the native tracking probe suite:

```bash
colcon test --packages-select gaussian_lic_tracking --event-handlers console_direct+
colcon test-result --verbose
```

Expected local result is `35 tests, 0 errors, 0 failures, 0 skipped`.

Run a real frontend-raw native tracking evidence pass:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_12s
```

This records native tracking odometry/status and mapper-contract outputs, then
gates signed-time, IMU, LiDAR, sliding-window, and numeric-Jacobian health in
`native_tracking_report.json`.

Add reference trajectory gating when the frontend-raw bag carries a trusted
odometry stream:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag bags/synthetic_frontend_raw_visual_demo \
  --output /tmp/gaussian_lic_native_tracking_reference_prior_probe \
  --playback-duration 4 \
  --enable-external-odometry-prior \
  --require-reference-trajectory \
  --min-reference-poses 5
```

The default native path stays sensor-only; `--enable-external-odometry-prior`
must be set explicitly before the `/gaussian_lic/frontend/input_odometry`
stream is admitted as a sliding-window pose prior. The report then records both
`trajectory.tum` and `reference_trajectory.tum`, runs
`scripts/trajectory_compare.py`, and stores the result in
`native_tracking_trajectory_compare.json`.

For pure sensor-only native tracking, require the last BA solve to be
non-degenerate and tune the LiDAR factor budget explicitly:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_nondegenerate_12s \
  --playback-duration 12 \
  --timeout 45 \
  --min-poses 30 \
  --min-point-frames 20 \
  --lidar-max-frame-points 1200 \
  --lidar-max-map-points 16000 \
  --sliding-window-max-iterations 2 \
  --sliding-window-max-state-gap-s 1.5 \
  --require-nondegenerate-ba
```

For the real-bag visual/SE3 photometric feedback path, launch mapper feedback
beside native tracking:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_12s \
  --playback-duration 12 \
  --timeout 45 \
  --min-poses 30 \
  --min-point-frames 20 \
  --lidar-max-frame-points 1200 \
  --lidar-max-map-points 16000 \
  --sliding-window-max-iterations 2 \
  --sliding-window-max-state-gap-s 1.5 \
  --require-nondegenerate-ba \
  --enable-mapper-feedback
```

For a longer full-sequence CBD production-BA health run, use the frontend-raw
bag and loosen mapper feedback frame synchronization enough for asynchronous
rosbag2 replay:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/CBD_Building_01_frontend_raw \
  --output results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s \
  --playback-duration 120 \
  --timeout 240 \
  --min-poses 100 \
  --min-point-frames 100 \
  --enable-scan-order-deskew \
  --require-deskew \
  --enable-mapper-feedback \
  --mapper-feedback-sync-tolerance-sec 0.05 \
  --require-nondegenerate-ba
```

Latest local real-bag check:

```text
results/fastlivo2/Bright_Screen_Wall_native_tracking_8s/native_tracking_report.json
ok=true, poses=20, /points_for_gs=21, status_samples=20, imu_factors=3,
normal_equation_rows=87779, numeric_jacobian_blocks=0

results/fastlivo2/CBD_Building_01_native_tracking_8s/native_tracking_report.json
ok=true, poses=22, /points_for_gs=23, status_samples=22, imu_factors=3,
normal_equation_rows=65611, numeric_jacobian_blocks=0

/tmp/gaussian_lic_native_tracking_reference_prior_probe/native_tracking_report.json
ok=true, poses=32, reference_poses=32, external_prior_matches=25,
trajectory_rmse=0.101496 m, coverage=100.00%

results/fastlivo2/Bright_Screen_Wall_native_tracking_truncated_imu_gate_12s/native_tracking_report.json
ok=true, poses=38, /points_for_gs=38, status_samples=38, imu_factors=36,
state=tracking_with_sliding_window, normal_equation_degenerate=false,
state_gap_degenerate=false, accepted_steps=2, feedback_updates=36,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
pointcloud_imu_wait_deferred=7, pointcloud_imu_wait_released=7,
pointcloud_imu_wait_dropped=0, normal_equation_rows=30465,
condition=5.773e5, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_12s/native_tracking_report.json
ok=true, poses=36, /points_for_gs=36, status_samples=36, imu_factors=35,
visual_factors=2, se3_photometric_factors=2, se3_samples=68,
visual_depth_cache_size=4, visual_depth_match_delta_ns=95735550,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
normal_equation_rows=33855, condition=8.363e5, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_windowed_visual_30s/native_tracking_report.json
ok=true, poses=122, /points_for_gs=122, status_samples=122, imu_factors=121,
visual_factors=64, se3_photometric_factors=64, se3_samples=85,
visual_depth_cache_size=8, visual_depth_match_delta_ns=6003618,
visual_rendered_cache_size=64, visual_rendered_match_delta_ns=6003618,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
invalid_optimized_states=0, feedback_updates=121,
normal_equation_rows=22588, condition=6.532e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_scan_order_deskew_hits_12s/native_tracking_report.json
ok=true, poses=36, /points_for_gs=36, status_samples=36, imu_factors=35,
scan_order_deskew=true, max_abs_point_time_offset_s=0.05,
trajectory_deskew_queries=864000, trajectory_deskew_hits=762866,
lidar_invalid_point_times=0, lidar_out_of_range_point_times=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, normal_equation_rows=31137,
condition=1.035e6, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_gate_30s/native_tracking_report.json
ok=true, poses=112, /points_for_gs=112, status_samples=112, imu_factors=111,
visual_factors=59, se3_photometric_factors=9,
trajectory_deskew_queries=2688000, trajectory_deskew_hits=2616215,
max_abs_point_time_offset_s=0.05, lidar_invalid_point_times=0,
lidar_out_of_range_point_times=0, imu_factor_skip_count=0,
imu_time_gap_skip_count=0, invalid_optimized_states=0,
normal_equation_rows=17908, condition=5.044e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_queue_imu_order_30s/native_tracking_report.json
ok=true, poses=109, /points_for_gs=109, status_samples=109, imu_factors=107,
visual_factors=64, se3_photometric_factors=16,
trajectory_deskew_queries=2616000, trajectory_deskew_hits=2552484,
max_abs_point_time_offset_s=0.05, lidar_invalid_point_times=0,
lidar_out_of_range_point_times=0, imu_factor_skip_count=0,
imu_time_gap_skip_count=0, invalid_optimized_states=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
normal_equation_rows=14730, condition=8.467e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_30s/native_tracking_report.json
ok=true, poses=110, /points_for_gs=110, status_samples=110, imu_factors=108,
visual_factors=56, se3_photometric_factors=11,
visual_factor_replacements=0, se3_photometric_replacements=0,
visual_pending_queue_size=0, se3_pending_queue_size=0,
trajectory_deskew_queries=2640000, trajectory_deskew_hits=2562222,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, normal_equation_rows=18064,
condition=9.500e6, rank_ratio=1.0, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_queue_gate_60s/native_tracking_report.json
ok=true, poses=272, /points_for_gs=272, status_samples=270, imu_factors=270,
visual_factors=155, se3_photometric_factors=10,
visual_factor_replacements=0, se3_photometric_replacements=0,
visual_pending_queue_size=0, se3_pending_queue_size=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
trajectory_deskew_queries=6528000, trajectory_deskew_hits=6470281,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, feedback_updates=271,
normal_equation_rows=7679, condition=1.709e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_valid_depth_sampling_default_60s/native_tracking_report.json
ok=true, poses=195, /points_for_gs=195, status_samples=195, imu_factors=194,
visual_factors=99, se3_photometric_factors=12,
se3_total_batches=99, se3_valid_batches=12, se3_total_samples=21627,
visual_depth_dilation_px=5, visual_factor_max_dt_ns=300000000,
visual_pending_queue_size=0, se3_pending_queue_size=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
trajectory_deskew_queries=4680000, trajectory_deskew_hits=4668986,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, feedback_updates=194,
normal_equation_rows=20489, condition=1.391e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_depth_window_split_default_30s/native_tracking_report.json
ok=true, visual_factor_max_dt_ns=300000000, visual_depth_max_dt_ns=0,
visual_factors=45, se3_photometric_factors=10,
se3_total_batches=45, se3_valid_batches=10, se3_total_samples=19176,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
normal_equation_rows=44808, condition=7.019e6,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_hessian_gate_30s/native_tracking_report.json
ok=true, poses=87, /points_for_gs=88, status_samples=87, imu_factors=85,
visual_factors=46, se3_photometric_factors=10,
se3_total_batches=46, se3_valid_batches=10, se3_degenerate_batches=0,
se3_total_samples=19176, se3_min_hessian_rank_gate=3,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=7.240e3,
trajectory_deskew_queries=2088000, trajectory_deskew_hits=2031012,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=85,
normal_equation_rows=40555, condition=9.543e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_sample_quality_gate_30s/native_tracking_report.json
ok=true, poses=82, /points_for_gs=83, status_samples=82, imu_factors=80,
visual_factors=73, se3_photometric_factors=22,
se3_total_batches=73, se3_valid_batches=22, se3_degenerate_batches=0,
se3_quality_rejected_batches=0, se3_total_samples=39321,
se3_min_sample_inlier_ratio_gate=0.25,
last_accepted_se3_sampled_depth=446, last_accepted_se3_samples=422,
last_accepted_se3_sample_inlier_ratio=0.946,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=2.439e7,
trajectory_deskew_queries=1968000, trajectory_deskew_hits=1906016,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=80,
normal_equation_rows=40801, condition=1.877e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_coverage_gate_30s/native_tracking_report.json
ok=true, poses=82, /points_for_gs=82, status_samples=82, imu_factors=81,
visual_factors=43, se3_photometric_factors=12,
se3_total_batches=43, se3_valid_batches=12, se3_degenerate_batches=0,
se3_quality_rejected_batches=1, se3_total_samples=24210,
se3_coverage_grid=4x4, se3_min_coverage_tiles_gate=4,
last_accepted_se3_sampled_depth=1962, last_accepted_se3_samples=1837,
last_accepted_se3_sample_inlier_ratio=0.936,
last_accepted_se3_coverage_tiles=4/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=9.787e3,
trajectory_deskew_queries=1968000, trajectory_deskew_hits=1924727,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=81,
normal_equation_rows=39122, condition=4.754e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_balanced_sampling_30s/native_tracking_report.json
ok=true, poses=83, /points_for_gs=83, status_samples=83, imu_factors=82,
visual_factors=56, se3_photometric_factors=15,
se3_total_batches=56, se3_valid_batches=15, se3_degenerate_batches=0,
se3_quality_rejected_batches=4, se3_total_samples=35004,
se3_coverage_grid=4x4, se3_min_coverage_tiles_gate=4,
last_accepted_se3_sampled_depth=2000, last_accepted_se3_samples=1899,
last_accepted_se3_sample_inlier_ratio=0.950,
last_accepted_se3_coverage_tiles=5/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=7.467e3,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=82,
normal_equation_rows=49171, condition=1.143e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s/native_tracking_report.json
ok=true, poses=478, /points_for_gs=478, status_samples=478, imu_factors=477,
visual_factors=238, se3_photometric_factors=27,
se3_total_batches=238, se3_valid_batches=27, se3_degenerate_batches=0,
se3_quality_rejected_batches=21, se3_total_samples=76250,
visual_rendered_miss_count=2, visual_rendered_cache_size=64,
visual_depth_cache_size=64, mapper_feedback_sync_tolerance_sec=0.05,
last_accepted_se3_sample_inlier_ratio=0.935,
last_accepted_se3_coverage_tiles=6/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=6.187e3,
trajectory_deskew_queries=11472000, trajectory_deskew_hits=11430279,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=477,
normal_equation_rows=6258, condition=5.818e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/r3live/hku_park_00_native_tracking_scan_order_60s/native_tracking_report.json
ok=true, poses=554, /points_for_gs=554, status_samples=554, imu_factors=553,
total_window_point_correspondences=18004,
total_window_plane_correspondences=7474, smoothness_factors=10,
trajectory_deskew_queries=13296000, trajectory_deskew_hits=8542971,
lidar_map_points=16000, pointcloud_imu_wait_deferred=395,
pointcloud_imu_wait_released=395, pointcloud_imu_wait_dropped=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=553,
normal_equation_rows=567, condition=3.317e4, rank_ratio=1.0,
numeric_jacobian_blocks=0
```

Run the full local verification wrapper:

```bash
./scripts/verify_workspace.sh --full-profiles
```

The build wrapper pins ROS2 interface generation to `/usr/bin/python3` so conda Python does not get selected by CMake. `scripts/build_jazzy.sh` remains as a compatibility wrapper around `scripts/build_ros2.sh`.

## Synthetic Rosbag2 Demo

Generate a small synthetic bag:

```bash
./scripts/create_synthetic_bag.sh --output bags/synthetic_gs_demo
```

Generate a synthetic raw frontend bag for the LIC2 adapter boundary:

```bash
./scripts/create_synthetic_bag.sh \
  --frontend-raw \
  --output bags/synthetic_frontend_raw_demo
```

Use `--frontend-raw-odometry` to record `/gaussian_lic/frontend/input_odometry` instead of `/gaussian_lic/frontend/pose`.

Replay it through the native mapper:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

Run the same generated-bag replay smoke used by the Jazzy CI leg:

```bash
./scripts/ci_replay_smoke.sh --bag bags/ci_synthetic_gs_demo --duration 4 --timeout 20
```

Add `--frontend-bag bags/ci_synthetic_frontend_raw_demo` to choose the raw frontend bag path used by the LIC2 adapter replay leg.
Add `--artifact-dir /path/to/reports` to keep the generated bag contract reports, offline `metrics.json`, `trajectory.tum`, and `point_cloud_debug.ply`. GitHub Actions uploads those files as `jazzy-replay-artifacts`.
The script also writes `replay_summary.md`, which CI appends to the job summary.

Run the same bag with a dataset profile:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

The synthetic demo image is `1x1`; real dataset intrinsics can project the synthetic point outside the image, so profile smoke tests skip the red-pixel assertion.

Exercise the native LIC2 frontend contract adapter with synthetic raw sensor topics:

```bash
./scripts/smoke_test.sh --frontend-adapter --tf
```

This publishes `/camera/image`, `/camera/camera_info`, `/camera/depth`, `/livox/lidar`, `/imu`, and `/gaussian_lic/frontend/pose`, then lets `lic2_contract_adapter` forward them into `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/points_for_gs`, `/imu_for_gs`, and `/pose_for_gs`. The adapter also publishes `/gaussian_lic/frontend/odometry` and `/gaussian_lic/frontend/path` from incoming pose/odometry. It is a ROS2 boundary adapter, not the finished LIC2 odometry algorithm.
Add `--frontend-odometry-input` to publish synthetic `/gaussian_lic/frontend/input_odometry` instead of PoseStamped.

## Offline Mode

Extract reproducibility artifacts from a rosbag2 directory without starting a ROS launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag bags/synthetic_gs_demo \
  --output /tmp/gaussian_lic_offline_demo
```

Current outputs:

```text
trajectory.tum
point_cloud_debug.ply
metrics.json
```

This is the seed for the future `gaussian_lic_offline` reproduction binary. The current implementation reads mapper contract topics, preserves PointCloud2 `rgb`/`rgba` or `r/g/b` colors in the debug PLY when available, and writes metrics for topic rates, trajectory path length, point-cloud bounds, and color coverage. It does not yet run the full Gaussian-LIC2 algorithm offline.

The same artifact extractor can read ROS1 `.bag` inputs directly when the optional `rosbags` package is installed in that Python environment:

```bash
/usr/bin/python3 -m pip install --user rosbags
PYTHONPATH=src/gaussian_lic_tools \
  python3 -m gaussian_lic_tools.offline \
  --bag /path/to/input.bag \
  --bag-format ros1 \
  --output /tmp/gaussian_lic_ros1_offline_demo
```

Direct ROS1 extraction writes the same `trajectory.tum`, `point_cloud_debug.ply`, and `metrics.json` files, including `bag_format` and `storage_identifier` fields, so converted and original bags can be compared with the same scripts.

Validate that a rosbag2 directory has the mapper input contract before replaying it:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag bags/synthetic_gs_demo \
  --json
```

For intermediate replay bags that only contain the mapper's synchronized core topics, use:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/intermediate_bag \
  --contract mapper_minimal \
  --json
```

`mapper_minimal` requires `/points_for_gs`, `/pose_for_gs`, and `/image_for_gs`. It still reports `/camera_info_for_gs`, `/depth_for_gs`, and `/imu_for_gs` when present, but missing optional topics do not fail the check. Use it with `require_depth_topic:=false` and profile intrinsics when replaying bags that do not carry depth or CameraInfo.

Validate a raw sensor bag before feeding it through `lic2_contract_adapter`:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag bags/synthetic_frontend_raw_demo \
  --contract frontend_raw \
  --json
```

`frontend_raw` requires raw image, CameraInfo, PointCloud2 LiDAR, and IMU topics, requires at least one pose source from `/gaussian_lic/frontend/pose` or `/gaussian_lic/frontend/input_odometry`, and treats `/camera/depth` as optional.

For true LIC2 frontend input bags that do not have pose yet, use:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/frontend_sensor_raw_bag \
  --contract frontend_sensor_raw \
  --json
```

Convert an official ROS1 dataset bag into that ROS2 raw-sensor contract. The
profile selects source-topic fallbacks, camera intrinsics, and the LiDAR frame;
`fastlivo2_ros1_to_frontend_raw.py` remains as a compatibility entrypoint, but
new runs should use `ros1_to_frontend_raw.py`:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/ros1_to_frontend_raw.py \
  --profile fastlivo2 \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall.bag \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --overwrite
```

Valid profiles are `fastlivo`, `fastlivo2`, `m2dgr`, `mcd`, and `r3live`.
For rotating-LiDAR datasets that already publish `sensor_msgs/PointCloud2`, the
converter preserves the original fields and only normalizes the ROS2 bag
contract topics. For split datasets such as MCD, pass multiple ROS1 bags as a
comma-separated `--input` list so the converter merges camera, LiDAR, and IMU
streams into one timestamp-sorted ROS2 `frontend_raw` bag.

Replay a minimal rosbag2 mapper bag with:

```bash
./scripts/smoke_test.sh \
  --bag /path/to/intermediate_ros2_bag \
  --minimal-inputs \
  --config src/gaussian_lic_bringup/config/r3live.yaml
```

The same checker can inspect ROS1 `.bag` metadata before conversion when the optional `rosbags` package is installed:

```bash
/usr/bin/python3 -m pip install --user rosbags
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/input.bag \
  --bag-format ros1 \
  --contract mapper_minimal \
  --json
```

## ROS1 Bag Conversion

Convert an archived ROS1 `.bag` into a ROS2 bag directory for replay:

```bash
/usr/bin/python3 -m pip install --user rosbags
./scripts/convert_ros1_bag_to_rosbag2.py \
  --input /path/to/input.bag \
  --output bags/input_ros2 \
  --storage mcap \
  --dst-typestore ros2_jazzy
```

Use repeated `--topic` or `--include-topic` arguments to convert only the mapper contract topics when preparing focused reproduction bags.

## Torch Backend Probe

Build the optional libtorch/CUDA path:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_ros2.sh --packages-select gaussian_lic_mapping
source install/setup.bash
ros2 run gaussian_lic_mapping torch_backend_probe
```

Expected shape output includes:

```text
image_sizes=[3, 1, 1]
depth_sizes=[1, 1]
gaussian_xyz_sizes=[2, 3]
gaussian_features_dc_sizes=[2, 1, 3]
gaussian_features_rest_sizes=[2, 15, 3]
gaussian_count_after_init=1
appended_count=1
gaussian_count=2
```

On the tested machine, CUDA is available. TensorRT is optional for the ROS2 porting surface; the local upstream ROS1 baseline Docker path has also been validated with TensorRT 8.6.1.6.

For SPNet depth-completion engine generation on the local RTX 5070 Ti
(`sm_120`) machine, TensorRT 8.6.1.6 is too old for engine building. Use the
local TensorRT 10.9 CUDA 12.8 SDK layout generated by
`scripts/install_local_tensorrt_10_9.sh` instead.

Current local SPNet runtime status:

```text
/home/frank/Software/TensorRT-10.9.0.34-cuda12.8
/home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine
```

The latest `trtexec` benchmark reports mean latency `26.4492 ms` at
`512x640`, below the 30 ms/frame target. See
`docs/SPNET_RUNTIME_STATUS.md` for hashes and the TensorRT 8.6 incompatibility
note.

Rebuild the local engine with:

```bash
./scripts/install_local_tensorrt_10_9.sh
SPNET_PYTHON=/home/frank/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python \
TENSORRT_ROOT=/home/frank/Software/TensorRT-10.9.0.34-cuda12.8 \
  ./scripts/build_spnet_engine.sh --output-dir /home/frank/Software/TensorRT-engines
```

## ROS2 Interfaces

Default logical sensor contract:

```text
/camera/image
/camera/camera_info
/livox/lidar
/imu
/tf
/tf_static
```

Current mapper contract:

```text
/points_for_gs       sensor_msgs/msg/PointCloud2
/pose_for_gs         geometry_msgs/msg/PoseStamped
/image_for_gs        sensor_msgs/msg/Image
/camera_info_for_gs  sensor_msgs/msg/CameraInfo
/depth_for_gs        sensor_msgs/msg/Image
/imu_for_gs          sensor_msgs/msg/Imu
```

`gaussian_lic_frontend/lic2_contract_adapter` provides the first native frontend boundary. Its default raw inputs are:

```text
/camera/image
/camera/camera_info
/camera/depth
/livox/lidar
/imu
/gaussian_lic/frontend/pose
/gaussian_lic/frontend/input_odometry
```

It republishes raw sensor and pose/odometry inputs into the mapper contract above. It also publishes:

```text
/gaussian_lic/frontend/odometry
/gaussian_lic/frontend/path
```

If only odometry is available, set `raw_odometry_topic` and it converts `nav_msgs/msg/Odometry` into `/pose_for_gs` while keeping the stable frontend output topics separate from the raw odometry input.

When `/points_for_gs` has no `rgb`, `rgba`, or `r/g/b` fields, the mapper projects each valid camera-frame point into `/image_for_gs` using the active `CameraInfo` intrinsics and samples RGB from the image. Points outside the image keep the white fallback color.

Use `./scripts/smoke_test.sh --image-color-fallback-check` to verify that fallback path with a synthetic uncolored cloud.

Set `require_depth_topic:=false` to run the mapper without synchronized `/depth_for_gs`; it will create a sparse metric depth image by projecting valid map points into the current image. The default profile keeps depth required, while upstream-derived dataset profiles allow missing depth so replay is not blocked when TensorRT/SPNet depth completion is unavailable.

Current outputs:

```text
/gaussian_lic/odometry
/gaussian_lic/path
/gaussian_lic/map_points
/gaussian_lic/rendered_image
/gaussian_lic/gaussian_map
/gaussian_lic/status
/gaussian_lic/save_map
```

`/gaussian_lic/rendered_image` currently defaults to the CUDA rasterizer path:

```text
render_mode:=rasterizer
torch_gaussian_device:=cuda
```

The packaged dataset profiles and launch defaults are biased toward the paper
gate path now: Torch camera conversion, Gaussian initialization, CUDA device,
photometric optimization, pruning, append-only upstream-style extension, and
rasterizer preview are on by default. `scripts/collect_current_results.sh`
still preserves its explicit `--torch` contract: when the script is called
without `--torch`, it passes launch overrides that disable Torch even though the
general launch defaults are CUDA-first. ROS2 gradient split/clone densification
is available as an explicit diagnostic switch, but it is off by default because
the released Gaussian-LIC2 path uses append-only `extend()`.
`rendered_image_mode` is kept only as a deprecated compatibility alias.

See [docs/STATUS_SCHEMA.md](docs/STATUS_SCHEMA.md) for the status topic schema.

## Dataset Profiles

Packaged profiles:

```text
src/gaussian_lic_bringup/config/fastlivo.yaml
src/gaussian_lic_bringup/config/fastlivo2.yaml
src/gaussian_lic_bringup/config/m2dgr.yaml
src/gaussian_lic_bringup/config/mcd.yaml
src/gaussian_lic_bringup/config/r3live.yaml
```

Validate profile schema:

```bash
./scripts/check_profiles.py
```

These profiles currently cover the native mapper surface: topic names, global and per-input QoS, synchronization, upstream camera intrinsics, image size, depth-completion toggles, Gaussian initialization/map-growth parameters, upstream optimizer/loss/exposure parameter names, output topics, and active profile labels. The contract adapter and native tracking launch also expose per-stream raw-input and mapper-output QoS so selected streams can be promoted without making every camera/LiDAR/IMU/depth/pose topic reliable. The native tracking launch keeps high-rate tracking inputs at bounded `best_effort` by default, while `scripts/run_native_tracking_bag_report.sh` promotes the strict-replay IMU and point-cloud inputs to reliable QoS with deep queues; image topics remain best-effort. Tracking now auto-calibrates initial IMU gravity from the startup accelerometer window, preserves a deeper IMU history for delayed point-cloud callbacks, and queries the propagated IMU state at each point-cloud stamp instead of using a future latest state. The launch exposes per-point LiDAR deskew controls for common `offset_time`, `time`, `timestamp`, and `t` fields while preserving signed-nanosecond estimator math, rejects invalid builtin stamp nanosecond fields, non-monotonic tracking stream stamps, non-finite IMU measurements, non-finite CameraInfo intrinsics, non-finite extrinsic parameters, non-finite IMU gravity, invalid LiDAR keyframe thresholds, invalid cache sizes, invalid visual/SE3 photometric gates, invalid LiDAR thresholds, and invalid sliding-window BA bounds before mapper/frontend arithmetic, rejects invalid deskew poses before they can create NaN points, publishes latest signed-nanosecond image/LiDAR/IMU stamps, stamp-regression counters, and invalid-IMU counters for smoke gating, and serializes tracking callbacks by default so a future multi-threaded executor cannot concurrently mutate IMU/LiDAR/image estimator state. The sliding-window optimizer is now default-enabled for production BA runtime coverage, with reusable normal-equation linearization, rank/condition health diagnostics and solve guards, factor-agnostic solve triggering when LiDAR/visual/smoothness factors arrive without a valid IMU factor, bounded LM state increments plus accepted/rejected step status, dense prior anchoring, Schur-complement retained-window priors with rank/singular-value health, optimized pose/velocity/bias feedback into odometry, trajectory controls, and safe IMU re-anchoring, timestamp-safe cubic B-spline position/velocity and SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state trajectory smoothness factors with closed-form SO(3) rotation blocks, bias observability status, Huber-weighted LiDAR 6-DoF pose correction, corrected-pose LiDAR/Gaussian correspondence construction, spatial-indexed per-correspondence robust LiDAR/Gaussian-map weights bounded to `(0, 1]`, LiDAR point-to-plane factors with residual/local-planarity confidence weighting, LiDAR confidence/spatial-index status, sliding-window optimization timing plus numeric-Jacobian fallback status, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric and full current IMU preintegration Jacobians, robust analytic SE3 camera photometric normal equations/window factors from rendered/current/depth images selected through bounded nearest-stamp render/depth caches with body-frame sqrt-information, runtime status for 2-DoF and SE3 photometric linearization quality, and subpixel visual alignment in place. The local CBD native reference checkpoint now passes under controlled strict replay; cross-dataset long-window RMSE-gated production BA remains the next tracker quality target.

## Performance Regression

Compare current metrics against a baseline:

```bash
./scripts/perf_regression.py \
  --metrics results/fastlivo2/current/metrics.json \
  --baseline baseline/fastlivo2/CBD_Building_01/metrics.json
```

Default gates check:

```text
tracking_hz
mapping_hz
mean_iteration_ms
```

A regression greater than 15% fails by default.

Compare a current TUM trajectory against an archived ROS1 baseline:

```bash
./scripts/trajectory_compare.py \
  --baseline baseline/fastlivo2/CBD_Building_01/trajectory.tum \
  --current results/fastlivo2/current/trajectory.tum \
  --output results/fastlivo2/current/trajectory_compare.json \
  --max-rmse-m 0.05 \
  --max-path-drift 0.05
```

The trajectory gate associates poses by timestamp, reports translation RMSE/mean/max plus path-length drift, and exits non-zero when any threshold fails. Use `--align first` only for datasets whose baseline and current trajectories differ by a known initial translation offset. Use `--align yaw` for local-world frontend trajectories, such as M2DGR native tracking, where the estimator origin/yaw is arbitrary but scale is fixed.

M2DGR `room_01` native tracking now has a 60s slow-replay reference pass at `results/m2dgr/room_01_tracking_sweep_yaw_guarded008_60s/all_frames_default_60s/native_tracking_report.json`: 106 odometry poses, 105 yaw-aligned GT matches, RMSE 0.4479 m, mean 0.4342 m, max 0.7597 m, and zero report errors. This is a frontend trajectory parity checkpoint, not a replacement for the remaining full-dataset ROS1 mapper/render strict artifacts.

Run the remaining full-profile strict artifact backlog from one queue:

```bash
./scripts/run_strict_parity_queue.sh --continue-on-error
```

Use `--dry-run` first to see the exact per-profile commands. The queue uses the existing raw/frontend data under `/home/frank/data`, writes baselines under `baseline/<profile>/<sequence>`, writes current artifacts under `results/<profile>/<sequence>_strict_current`, and finishes by refreshing `docs/strict_data_status.*` plus `scripts/check_strict_parity_matrix.py --allow-incomplete`.

Compare a current PLY map against an archived ROS1 baseline:

```bash
./scripts/pointcloud_compare.py \
  --baseline baseline/fastlivo2/CBD_Building_01/point_cloud.ply \
  --current results/fastlivo2/current/point_cloud.ply \
  --output results/fastlivo2/current/pointcloud_compare.json \
  --voxel-size 0.05 \
  --max-chamfer-rmse-m 0.15
```

The point-cloud gate supports ASCII and binary little-endian PLY files with standard `x/y/z` vertex properties. It reports point-count ratio, centroid drift, bidirectional nearest-neighbor RMSE/mean/max, unmatched ratio, and RGB mean drift when both PLY files include `red/green/blue`. Add `--derive-gaussian-rgb` when the PLY stores Gaussian color in upstream `f_dc_0..2` coefficients instead of explicit RGB fields.

Generate one combined reproduction report:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-dir results/fastlivo2/current \
  --output results/fastlivo2/current/reproduction_report.json \
  --markdown results/fastlivo2/current/reproduction_report.md
```

The combined report validates the baseline manifest and runs metrics, trajectory, and PLY map drift gates in one command. It exits non-zero when any gate fails and is intended as the future CI comparison artifact.

For the current Bright substitute proof chain, add a dedicated Torch Gaussian color gate:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s \
  --current-dir results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq \
  --sequence Bright_Screen_Wall_fastlivo2_color_8s \
  --metric-key debug_points \
  --trajectory-align first \
  --min-trajectory-coverage 0.4 \
  --max-association-dt 0.2 \
  --pointcloud-align centroid \
  --max-pointcloud-points 50000 \
  --max-nearest-m 0.5 \
  --max-chamfer-rmse-m 0.5 \
  --max-chamfer-mean-m 0.5 \
  --max-chamfer-max-m 0.5 \
  --max-unmatched-ratio 0.25 \
  --min-point-count-ratio 0.5 \
  --max-point-count-ratio 5.0 \
  --max-centroid-drift-m 0.5 \
  --gaussian-color-current-point-cloud results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply \
  --output results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json \
  --markdown results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.md
```

That report includes five gates: `baseline_manifest`, `metrics`, `trajectory`, `point_cloud`, and `gaussian_color`.

Run the same Python-only artifact gates used by GitHub Actions locally:

```bash
./scripts/verify_artifact_gates.sh
```

Set `GAUSSIAN_LIC_ARTIFACT_DIR=/path/to/reports` to choose where the synthetic JSON/Markdown reports are written. In GitHub Actions those files are uploaded as the `artifact-gate-reports` workflow artifact.

Check real FAST-LIVO2 baseline readiness:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --strict \
  --markdown baseline_readiness.md
```

If `--current-results-dir` is omitted and `results/fastlivo2/current` does not
exist, the readiness tool now auto-selects a sequence-specific archived current
artifact with the required `trajectory.tum`, `point_cloud.ply`, and
`metrics.json` files. On the local strict archive it resolves to
`results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/` and
recomputes the strict report instead of trusting stale JSON.

Fetch the official FAST-LIVO2 quick-start bag only when the data gate reports missing raw data:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Run or resume the strict chain from the local `CBD_Building_01` bag:

```bash
./scripts/run_strict_cbd_pipeline.sh \
  --overwrite \
  --current-torch-max-foreground 1500000
```

The strict script defaults to CUDA rasterizer mode, `torch_gaussian_device=cuda`,
`--current-torch-optimization-steps 100`,
`--current-torch-optimization-sampling upstream_random`, a fixed
`--current-torch-optimization-seed 20260505` for reproducible strict reports, a
slowed current playback rate of `0.25`, `--current-record-sec 0` so collection
runs until rosbag2 playback plus the 60 second post-playback settle window
finish, a 1.5M foreground Gaussian cap, uniform foreground count-cap policy with
`torch_gaussian_prune_min_opacity=0.0`,
upstream-style alpha-hole filtering before Gaussian extension, disabled ROS2
gradient split/clone densification, disabled opacity resets, and disabled
high-rate `GaussianArray` publication. In this path the step count means up to
100 accumulated train-frame optimizer samples per keyframe, and the selected
frames are sampled and shuffled in the same style as upstream Gaussian-LIC
`optimize()` instead of being processed as a sorted, evenly spaced list that
always forces the newest frame. Set the seed to `0` to use `std::random_device`
like upstream's non-deterministic run. Gradient split/clone densification is off
by default because Gaussian-LIC2's released mapping path grows the map through
append-only `extend()`; enabling the ROS2-only split/clone path produced large
blurred splats and a much lower strict PSNR/SSIM run locally. Opacity reset is
also off by default because Gaussian-LIC2's released optimizer does not run a
periodic reset; leaving the ROS2 recovery heuristic on can reset the whole
foreground to opacity 0.01 immediately before final-map evaluation. The slowed
playback is deliberate: the strict CUDA path is heavier than the live preview
path, so 1x rosbag2 replay can underfeed the final-map metric gate by dropping
unprocessed frames. The settle window lets the mapper consume queued frames
before `SaveMap` writes final-map render pairs. Disabling the visualization
Gaussian map topic keeps the strict recorder from writing tens of GiB of
high-frequency chunked Gaussian messages that are not used by the metric gate.
CUDA current collection also defaults
`PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` unless the caller has already
set it, which reduces late-run allocator fragmentation around million Gaussian
maps.
Strict quality extraction defaults LPIPS to `cuda` on the target workstation; set
`--quality-lpips-device cpu` or `QUALITY_LPIPS_DEVICE=cpu` only for CPU-only
report refreshes. On this workstation the CUDA report environment is
`${HOME}/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python` with
Torch `2.7.0+cu128`; if a requested CUDA LPIPS device is unavailable, the
evaluator records the requested device and falls back to CPU instead of leaving
LPIPS null.

That command converts the official ROS1 bag to a sqlite-backed ROS2
`frontend_raw` bag, audits replay timing, writes the ROS1 mapper-contract bag,
runs the upstream ROS1 baseline container, collects ROS2 current artifacts, and
emits strict readiness and reproduction reports. During report generation it also
fills `metrics.json::quality`: the baseline uses upstream GT, while the current
run uses its saved `gt/` frames so dropped/reordered replay frames do not get
scored against the wrong source image. Render-pair comparison uses decoded GT
hashes to associate ROS1 and ROS2 frame identities before sampling pairs. The
strict report keeps every per-pair outlier in JSON, but gates the supplemental
baseline-vs-current render-pair check by bounded outlier ratio plus mean
PSNR/SSIM; the primary paper-quality gate remains the full matched-frame
PSNR/SSIM/LPIPS regression against GT. Set `QUALITY_PYTHON` to a Python
environment with `torch`, `torchvision`, `numpy`, and `Pillow` when refreshing
those metrics outside the default CUDA venv.

Latest archived local strict run, 2026-05-05:

- ROS1 baseline visual dump: 1186 render/GT pairs, 237 train and 949 novel frames.
- ROS2 current strict path: CUDA rasterizer, final-map evaluation, IMU-fallback
  world-frame point-cloud rotation enabled, ROS1-compatible projected-depth
  nearest-pixel rounding, high-rate GaussianArray publication disabled, 1.5M
  foreground cap with no opacity pruning and uniform count-cap policy,
  alpha-hole extension filtering
  enabled at threshold `0.99`, ROS2 gradient split/clone densification disabled,
  opacity reset disabled, deterministic upstream-random optimization-frame
  sampling enabled with seed `20260505`,
  `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`, and
  `results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/` as
  the current archived strict artifact. The trajectory gate passes with 1063
  matched poses, 89.63% coverage, zero translation RMSE, and zero path drift.
- Current vs ROS1 quality passes the archived strict metric gate: ROS2 novel
  PSNR is 12.6542 dB vs ROS1 12.7036 dB (0.39% regression), ROS2 novel SSIM is
  0.368961 vs ROS1 0.3644, and ROS2 novel LPIPS is 0.741661 vs ROS1 0.75136.
  Future quality extraction uses Gaussian-LIC's 11x11 Gaussian-window SSIM and
  keeps the previous global SSIM as a diagnostic field.
- The GT-associated ROS1-vs-ROS2 render-pair smoke check passes: 1063 pairs
  match by decoded GT hash, 64 pairs are evaluated, mean PSNR is 21.600 dB,
  mean SSIM is 0.785417, min PSNR is 17.448 dB, and min SSIM is 0.477513.
- Chamfer/point-cloud parity passes: count ratio 0.978, centroid drift
  0.147855 m, bidirectional nearest RMSE 0.145491 m, and bidirectional nearest
  mean 0.099418 m against the 0.100000 m strict threshold.

Visual review artifacts from the same strict run:

```text
docs/assets/strict_cbd_montage.jpg
docs/assets/strict_cbd_render_demo.gif
```

So the strict chain now passes the local `CBD_Building_01` reproduction gate for
the mapper-contract/CUDA current path. The remaining paper-port work is full
native Coco-LIC2 tracking parity and dataset-validated production joint BA beyond
the mapper-contract path.

The final full-dataset parity gate is explicit and now passes for the required
local evidence matrix:

```bash
./scripts/check_strict_parity_matrix.py
```

To write the same report while allowing future optional items to remain incomplete:

```bash
./scripts/check_strict_parity_matrix.py \
  --allow-incomplete \
  --output results/strict_parity_matrix.json \
  --markdown results/strict_parity_matrix.md
```

Current local matrix status: `PASS`, `required=17/17`, `covered_profiles=fastlivo,fastlivo2,m2dgr,mcd,r3live`.
Passing required items are FAST-LIVO2 `CBD_Building_01` mapper-contract/CUDA
strict parity, FAST-LIVO hku1/hku2/Visual_Challenge/LiDAR_Degenerate
mapper-contract/CUDA strict parity, M2DGR room_01/room_02/room_03 mapper-contract/CUDA strict
parity, MCD `ntu_day_01` mapper-contract/CUDA strict parity, R3LIVE
`hku_park_00` mapper-contract/CUDA strict parity, and the 120s CBD native
visual/SE3 BA health report. The final required item, CBD native reference
trajectory parity, now passes against an upstream Coco-LIC-generated trusted TUM
reference decimated to the ROS2 native output cadence. The five added
continuous-time native tracker entries cover FAST-LIVO2 CBD, FAST-LIVO2 Retail,
M2DGR room_01, MCD ntu_day_01, and R3LIVE hku_park_00 as liveness/producer-chain
gates.

The full-profile strict queue uses `--current-record-sec 0` and a slower default
`0.15x` replay rate than the focused CBD script so CUDA final-render runs do not
silently truncate or drop synchronized point/pose frames on longer FAST-LIVO
sequences.
For offline strict replay it also overrides every mapper and adapter stream QoS
to reliable with keep-last depth `50`; live profiles keep the YAML defaults
(`best_effort`, depth `5`) unless the launch caller overrides them.

The matching data audit is:

```bash
./scripts/audit_strict_data_inputs.py \
  --output docs/strict_data_status.json \
  --markdown docs/strict_data_status.md \
  --cleanup-candidates 12 \
  --min-free-gb 100
```

As of 2026-05-12 it reports `109.39 GiB` free on `/home/frank/data`, above the
100 GiB release threshold. Raw bags, converted frontend-raw bags, ROS1 baseline
artifacts, ROS2 current artifacts, and native reference trajectories are local
for every required profile: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
FAST-LIVO2 `Retail_Street` now also has an archived ROS1 upstream mapper
baseline and native-reference TUM. The script also lists the largest non-matrix
`results/` directories that can be reviewed before reclaiming space; it
deliberately does not list archived baseline directories as cleanup candidates.

## Release Roadmap

The public plan is release-driven:

```text
v0.1.0  Baseline & Infrastructure
v0.2.0  Gaussian-LIC2 ROS2 Native Frontend
v0.3.0  Gaussian Mapping Full Backend
v0.4.0  Strict FAST-LIVO2 Reproduction
```

Strict paper-data action: run `scripts/run_strict_cbd_pipeline.sh` from the local
`CBD_Building_01` bag, keep the ROS1 baseline archive fixed, and confirm the
ROS2 current report stays inside the 5% gate.

If the local raw bag is missing, the data fetch is executable:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Then run the original Gaussian-LIC/Gaussian-LIC2 upstream on FAST-LIVO2 and archive artifacts under:

```text
baseline/fastlivo2/<sequence>/
```

Validate and fingerprint the archived baseline before comparing ROS2 results:

```bash
./scripts/baseline_manifest.py \
  --baseline baseline/fastlivo2/CBD_Building_01 \
  --sequence CBD_Building_01 \
  --write
```

The baseline manifest checks `trajectory.tum`, `point_cloud.ply`, `metrics.json`, `run.log`, and `renders/`, then writes artifact sizes, SHA-256 hashes, pose count, PLY vertex count, and render count to `baseline_manifest.json`.

Current readiness gate:

```text
./scripts/baseline_readiness.py --dataset-root /home/frank/data/fast_livo --sequence CBD_Building_01
```

Latest native-tracker status (2026-05-16): data and reference trajectories are
available, and the remaining blocker is long-window continuous-time BA quality.
The best current CBD full122 Gaussian-feedback diagnostic is
`results/fastlivo2/CBD_Building_01_gaussian_feedback_rate0p25_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_probe/`:
1174 reference matches, 95.37% coverage, `1.346 m` RMSE, `1.219 m` mean error,
`2.829 m` max error, and `5.00%` path drift. Slowing strict replay to `--rate
0.25` restores rendered-image feedback continuity and the `0.045`
direction-agreement release improves both RMSE and drift over the `0.040`
feedback baseline (`1.510 m` / `6.82%`). The `0.050` release lowers drift to
`3.52%` but regresses RMSE to `1.426 m`, so the production preset chooses
ATE-first `0.045`. The latest visual-saturation downweight probe is archived but
not promoted because it keeps RMSE flat while missing the visual/SE3 continuity
drain target and raising path drift to `5.16%`. The latest source-id/pair-count
diagnostic at
`results/fastlivo2/CBD_Building_01_gaussian_feedback_legacy_sourceid_pair_counts_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/`
records `876` processed visual pairs, `876` visual/SE3 factors, and `0`
duplicate pairs, while mapper feedback emits only `1040` rendered previews with
`144` dropped pointcloud/pose messages and zero render errors. That rejects
source-id collisions and duplicate-pair suppression as the blocker; the next
strict native Coco-LIC2 gap is mapper-feedback throughput/pairing continuity plus
trajectory length/shape under continuous visual feedback, not missing CBD data.
The follow-up mapper-continuity run at
`results/fastlivo2/CBD_Building_01_gaussian_feedback_legacy_sourceid_mapper_continuity_agree0045_step00265_relrot001_mhrot0005_mhtrans003_max6_full122_drain1015_probe/`
records only `900/1187` mapper image messages and `899` renders under best-effort
mapper image QoS. The first direct reliable image-QoS trial is rejected because
it was applied only on the mapper subscriber and therefore received zero mapper
images from the best-effort `/image_for_gs` internal publisher. The QoS control
is now wired to both sides, but remains explicit rather than a default reliable
image subscription.

See [docs/BASELINE_DATA.md](docs/BASELINE_DATA.md), [docs/RELEASE_MILESTONES.md](docs/RELEASE_MILESTONES.md), and [docs/ROADMAP.md](docs/ROADMAP.md).

## Docker

Non-torch Jazzy smoke-test container:

```bash
docker compose -f docker/compose.yaml run --rm smoke
```

See [docs/DOCKER.md](docs/DOCKER.md).

## Upstream Sources

Fetch upstream sources for porting work:

```bash
./scripts/fetch_upstreams.sh
```

This clones:

```text
external/Gaussian-LIC
```

`external/Gaussian-LIC` is the primary upstream and now carries the Gaussian-LIC2 release path. To also fetch the historical Coco-LIC ROS1 reference, run:

```bash
./scripts/fetch_upstreams.sh --with-legacy-cocolic
```

That adds:

```text
external/Coco-LIC
```

Those directories are ignored by git. The upstream Gaussian-LIC/Gaussian-LIC2 project is GPL-3.0; Coco-LIC is treated as an optional legacy GPL-3.0 reference.

## License

This repository is GPL-3.0-or-later compatible because Gaussian-LIC/Gaussian-LIC2 is a GPL-3.0 upstream project. Do not copy upstream source into this repository under a different license.
