# Roadmap

The public release plan is milestone-based:

- `v0.1.0` Baseline & Infrastructure
- `v0.2.0` Gaussian-LIC2 ROS2 Native Frontend
- `v0.3.0` Gaussian Mapping Full Backend
- `v0.4.0` Strict FAST-LIVO2 Reproduction

See `docs/RELEASE_MILESTONES.md` for release criteria and artifacts.

## Immediate Next Step

Blocker:

```text
FAST-LIVO2 bag data is required at /home/frank/data/fast_livo/
```

First concrete action:

```bash
docker pull ros:noetic-ros-base-focal
```

Then bring up the upstream baseline environment, run Gaussian-LIC/Gaussian-LIC2 on FAST-LIVO2, and archive outputs under:

```text
baseline/fastlivo2/<sequence>/
```

Required baseline artifacts:

```text
trajectory.tum
point_cloud.ply
renders/
metrics.json
run.log
baseline_manifest.json
```

## Milestone 0: ROS2 Foundation

- [x] Create a clean ROS2 workspace layout.
- [x] Add public Gaussian/status messages.
- [x] Add a launch entrypoint.
- [x] Add topic probe and status stub nodes.
- [x] Add native mapping-node scaffold.
- [x] Add upstream fetch helper.
- [x] Build successfully on local ROS2 Jazzy.

## Milestone 1: Input Contract

- [x] Add image, camera info, point cloud, IMU, and TF topic surfaces for local smoke tests.
- [x] Add rosbag2 smoke-test instructions.
- [x] Add calibration file schema.
- [x] Add dataset-specific ROS2 mapping profiles for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- [x] Add a native ROS2 frontend contract adapter from raw sensor/pose topics to mapper topics.
- [ ] Add full real-dataset topic remaps/adapters once the native Gaussian-LIC2 frontend/tracking port is in place.

## Milestone 2: Gaussian-LIC Mapping Port

- [x] Inventory ROS1 APIs used by upstream Gaussian-LIC.
- [x] Replace the four-topic ROS1 input synchronization surface with native ROS2.
- [x] Convert aligned ROS2 frames into mapper-ready image/depth/pose/point data.
- [x] Consume CameraInfo intrinsics with parameter fallback.
- [x] Accumulate converted frames into a non-torch `MapperDataset`.
- [x] Publish ROS2 odometry, path, and accumulated map-point topics from the native mapper.
- [x] Add optional `map -> camera` TF broadcasting.
- [x] Port upstream Gaussian-LIC backend parameter names into ROS2 profiles.
- [ ] Replace remaining ROS1 node handles, publishers, and launch files.
- [ ] Isolate CUDA/Gaussian map code from ROS middleware code.
- [x] Add optional libtorch/CUDA build path.
- [x] Add live `MapperDataset -> TorchCamera` conversion behind a build flag.
- [x] Add foreground `MapperDataset -> TorchGaussianMap` initialization behind a build flag.
- [x] Add keyframe-gated skybox seeding and incremental foreground Gaussian insertion.
- [x] Publish initialized and extended Gaussian map as chunked `GaussianArray`.
- [x] Add save-map service for initialized Gaussian PLY export.
- [x] Add save-map fallback for accumulated debug XYZRGB PLY export.
- [x] Publish CPU projected-map preview on `/gaussian_lic/rendered_image`.
- [x] Register `mapping_node` as an `rclcpp_components` composable node.
- [x] Add configurable input sensor QoS for ROS2 driver/rosbag2 compatibility.
- [x] Fill `MappingStatus` tracking/mapping rate fields from live frame counters.
- [x] Make TensorRT depth completion optional at the ROS2 mapper input boundary.
- [ ] Port Gaussian rasterization, optimization, densification, and pruning.
- [x] Publish mapper status.
- [ ] Replace rendered-image preview with Gaussian rasterizer output.

## Milestone 3: Gaussian-LIC2 Frontend/Tracking Port

- [ ] Inventory the released Gaussian-LIC2 surface in `external/Gaussian-LIC`, including depth completion, rasterizer/optimizer changes, and any frontend/tracking code available in upstream commits.
- [x] Add `gaussian_lic_frontend/lic2_contract_adapter` as the ROS2 boundary for raw camera/LiDAR/IMU/pose inputs.
- [ ] Port Livox/custom point handling to ROS2.
- [ ] Replace dynamic reconfigure/global parameters with ROS2 parameters.
- [ ] Publish odometry, path, TF, and Gaussian mapper input topics natively.
- [ ] Keep any ROS1 bridge mode clearly marked as temporary.

## Milestone 4: Usability

- [x] Docker image with NVIDIA runtime support for the non-torch ROS2 smoke-test path.
- [x] Add RViz2 launch option and default visualization config.
- [x] One-command synthetic smoke-test script.
- [x] One-command synthetic rosbag2 generation and replay smoke test.
- [ ] Screenshots and short demo video.
- [x] Common errors page.
- [x] Status schema page.
- [x] Offline rosbag2 artifact extraction CLI skeleton.
- [x] Performance regression comparison script.
- [x] Jazzy/Humble build-only CI skeleton.
- [x] Python-only CI artifact gates for baseline manifests and reproduction reports.
- [x] Jazzy CI smoke tests with a generated mini rosbag2 sequence.
- [ ] ROS1 upstream baseline archive for FAST-LIVO2.
- [x] ROS1 `.bag` to rosbag2 `.mcap` converter backend.
- [ ] PR comparison report on real or curated mini-sequence artifacts.
