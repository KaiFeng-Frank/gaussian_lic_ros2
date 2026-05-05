# SPNet TensorRT Runtime Status

Status date: 2026-05-05

## Local Result

The SPNet depth-completion runtime artifact has been generated locally for the
RTX 5070 Ti Laptop GPU (`sm_120`) path:

```text
TensorRT SDK: /home/frank/Software/TensorRT-10.9.0.34-cuda12.8
Checkpoint:   external/Gaussian-LIC/ckpt/Large_300.pth
ONNX:         /home/frank/Software/TensorRT-engines/spnet_512_640.onnx
Engine:       /home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine
```

Artifact hashes:

```text
Large_300.pth:
  e4a3dbaae1aff425c26db6083cb6ea8e1d6f13d719cc567054b2522b3bf84559
spnet_512_640.onnx:
  d977ba4ddf843d224778170d43453d4be4c532a72945a4949b97d1874654c24d
spnet_512_640_fp16.engine:
  67b0f9f8360dd771b4451ef2a06d17b57a238c808396719a5a725c6b64433158
```

The engine is intentionally not checked into git. It is a 460 MB
hardware/runtime-specific TensorRT plan.

## Benchmark

Command:

```bash
./scripts/install_local_tensorrt_10_9.sh
SPNET_PYTHON=/home/frank/.cache/gaussian_lic_ros2/quality-venv/bin/python \
TENSORRT_ROOT=/home/frank/Software/TensorRT-10.9.0.34-cuda12.8 \
  scripts/build_spnet_engine.sh \
  --output-dir /home/frank/Software/TensorRT-engines
```

Observed `trtexec` summary:

```text
TensorRT version: 10.9.0
Compute Capability: 12.0
Engine generation completed in 121.335 seconds.
Created engine with size: 459.087 MiB
Throughput: 38.4531 qps
Latency mean: 26.4492 ms
GPU compute mean: 25.8102 ms
```

This satisfies the depth-completion runtime target of <= 30 ms/frame on the
tested machine.

## Compatibility Note

TensorRT 8.6.1.6 can parse the exported ONNX, but engine creation fails on this
GPU with:

```text
Error Code 2: Internal Error (Assertion major >= 0 && major < 10 failed.)
```

That failure is a TensorRT 8.x `sm_120` support gap, not an SPNet model export
failure. The build helper now prefers the local TensorRT 10.9 CUDA 12.8 SDK and
emits an explicit error if a caller tries TensorRT 8.x on the local `sm_120`
GPU.
