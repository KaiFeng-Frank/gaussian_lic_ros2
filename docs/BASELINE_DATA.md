# Baseline Data

This repository treats the FAST-LIVO2 baseline as an executable gate, not a manual note.

The default strict sequence is `CBD_Building_01`, matching the Gaussian-LIC/Gaussian-LIC2 quick start. The expected local raw-data path is:

```bash
/home/frank/data/fast_livo/CBD_Building_01.bag
```

Discover and fetch that sequence from the official FAST-LIVO2 Google Drive mirror:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Check whether the raw data, archived ROS1 baseline, and ROS2 current artifacts are ready:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --probe-sources \
  --markdown baseline_readiness.md
```

Required ROS1 baseline archive:

```text
baseline/fastlivo2/CBD_Building_01/
├── trajectory.tum
├── point_cloud.ply
├── metrics.json
├── run.log
├── renders/
└── baseline_manifest.json
```

After the ROS2 result directory has the matching `trajectory.tum`, `point_cloud.ply`, and `metrics.json`, run:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --output results/fastlivo2/current/reproduction_report.json \
  --markdown results/fastlivo2/current/reproduction_report.md
```

Official source anchors:

- Gaussian-LIC/Gaussian-LIC2 upstream: <https://github.com/APRIL-ZJU/Gaussian-LIC>
- Gaussian-LIC2 project page: <https://xingxingzuo.github.io/gaussian_lic2/>
- FAST-LIVO2 upstream: <https://github.com/hku-mars/FAST-LIVO2>
- FAST-LIVO2 Google Drive mirror: <https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link>
