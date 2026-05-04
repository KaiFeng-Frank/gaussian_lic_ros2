# SPDX-License-Identifier: GPL-3.0-or-later

uint8 STATE_UNCONFIGURED=0
uint8 STATE_INACTIVE=1
uint8 STATE_ACTIVE=2
uint8 STATE_OPTIMIZING=3
uint8 STATE_ERROR=255

uint8 RENDER_MODE_OFF=0
uint8 RENDER_MODE_DEBUG_CPU=1
uint8 RENDER_MODE_RASTERIZER=2
uint8 RENDER_MODE_DEBUG_INPUT=3

std_msgs/Header header

# Overall node state kept for backward compatibility with the initial ROS2 port.
uint8 state
string status_text

# Component-level states. Tracking belongs to the Gaussian-LIC2 frontend; mapping
# belongs to the Gaussian backend. Use STATE_UNCONFIGURED when a component is
# not present in the current process.
uint8 tracking_state
uint8 mapping_state
uint8 render_mode

uint64 num_gaussians
uint64 num_keyframes
uint64 num_tracking_frames
uint64 num_mapping_frames
uint64 num_dropped_messages
uint64 num_errors

float32 tracking_hz
float32 mapping_hz
float32 tracking_latency_ms
float32 mapping_latency_ms
float32 mean_iteration_ms
float32 gpu_memory_mb

string active_profile
