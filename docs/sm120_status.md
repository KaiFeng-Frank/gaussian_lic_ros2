# sm_120 CUDA Status

Last verified: 2026-05-05 on RTX 5070 Ti Laptop GPU, CUDA 12.8, ROS2 Jazzy.

`scripts/sm120_compat_probe.sh` passed with native CUDA architecture detection:

- Device: `NVIDIA GeForce RTX 5070 Ti Laptop GPU`
- Compute capability: `12.0`
- NVCC target: `compute_120,sm_120`
- libtorch: `2.7.0`, CUDA header/toolkit `12.8`
- Probe result: `max_error=0`

The strict Gaussian-LIC ROS2 build also configured `gaussian_lic_mapping` with
CUDA architecture `12.0` and built the CUDA/Torch mapper targets successfully.
