# Status Schema

`/gaussian_lic/status` uses `gaussian_lic_msgs/msg/MappingStatus`.

## State Fields

```text
state            Aggregate node state.
tracking_state  Gaussian-LIC2 frontend state, or STATE_UNCONFIGURED when absent.
mapping_state   Gaussian-LIC backend state.
render_mode     One of RENDER_MODE_OFF, DEBUG_CPU, RASTERIZER, DEBUG_INPUT.
status_text     Human-readable diagnostic summary.
active_profile  Dataset/config profile name.
```

Valid component states:

```text
STATE_UNCONFIGURED
STATE_INACTIVE
STATE_ACTIVE
STATE_OPTIMIZING
STATE_ERROR
```

## Counters And Rates

```text
num_tracking_frames   Frames aligned or accepted by the tracking/frontend path.
num_mapping_frames    Frames converted or consumed by the mapping/backend path.
num_keyframes         Current mapper keyframe count.
num_gaussians         Current Gaussian count, zero when the Gaussian backend is not initialized.
num_dropped_messages  Dropped input messages caused by queues or sync trimming.
num_errors            Conversion, render, or backend initialization errors.
tracking_hz           Recent tracking/alignment rate.
mapping_hz            Recent mapping/conversion rate.
```

## Performance Fields

```text
tracking_latency_ms
mapping_latency_ms
mean_iteration_ms
gpu_memory_mb
```

The current mapping slice fills rates, counters, mapping latency, and mean iteration time. Full tracking/mapping ports must fill tracking latency and GPU memory fields before v0.4 strict reproduction.

## Render Mode Policy

`render_mode` is the canonical parameter.

```text
debug_cpu    Current default; CPU projected map preview.
debug_input  Input image passthrough diagnostic.
rasterizer   Future default once the CUDA rasterizer is ported.
off          Do not publish rendered preview images.
```

`rendered_image_mode` is a deprecated compatibility alias. CPU debug rendering is planned to be removed after the v0.3 rasterizer release stabilizes.
