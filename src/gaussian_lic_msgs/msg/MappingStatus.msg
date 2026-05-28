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
uint64 skipped_nonpositive_depth_points
uint64 skipped_max_depth_points
uint64 skipped_unprojected_points
uint64 skipped_occluded_points

# Torch/CUDA Gaussian backend diagnostics. These make native tracking reports
# prove whether the mapper only initialized a fixed map or kept extending,
# optimizing, densifying, pruning, and resetting opacity during replay.
uint64 gaussian_init_count
uint64 gaussian_extend_count
uint64 gaussian_last_inserted
uint64 gaussian_optimization_count
uint64 gaussian_optimization_steps
uint64 gaussian_optimization_supervised
float32 gaussian_optimization_loss
uint64 gaussian_optimization_errors
uint64 gaussian_densify_count
uint64 gaussian_densified_total
uint64 gaussian_last_densified
uint64 gaussian_densify_errors
uint64 gaussian_prune_count
uint64 gaussian_pruned_total
uint64 gaussian_last_pruned
uint64 gaussian_prune_errors
uint64 gaussian_opacity_reset_count
uint64 gaussian_last_opacity_reset
uint64 gaussian_opacity_reset_errors

float32 tracking_hz
float32 mapping_hz
float32 tracking_latency_ms
float32 mapping_latency_ms
float32 mean_iteration_ms
float32 gpu_memory_mb

string active_profile
string sync_anchor_stream
string rendered_feedback_source_stream

# Input synchronization diagnostics. `num_dropped_messages` keeps the legacy
# aggregate, while these fields identify whether late feedback gaps come from
# point/pose/image/depth alignment, queue pressure, or render throughput.
uint64 pointcloud_messages
uint64 pose_messages
uint64 image_messages
uint64 depth_messages
uint64 aligned_frames
uint64 converted_frames
uint64 dropped_pointcloud_messages
uint64 dropped_pose_messages
uint64 dropped_image_messages
uint64 dropped_depth_messages
uint64 pending_pointcloud_messages
uint64 pending_pose_messages
uint64 pending_image_messages
uint64 pending_depth_messages
uint64 rendered_preview_count
uint64 render_error_count

# Image+pose rendered-feedback diagnostics. These are nonzero only when the
# mapper publishes typed feedback from camera frames instead of aligned
# pointcloud frames, so visual BA support can be audited separately from point
# cloud/map synchronization.
uint64 image_pose_feedback_candidates
uint64 image_pose_feedback_published
uint64 image_pose_feedback_pose_waits
uint64 image_pose_feedback_pose_window_drops
uint64 image_pose_feedback_dropped_images
uint64 image_pose_feedback_image_queue_drops
uint64 image_pose_feedback_pose_queue_drops
uint64 pending_image_pose_feedback_images
uint64 pending_image_pose_feedback_poses
uint64 image_pose_feedback_errors

# Signed synchronization deltas for the last aligned frame, measured as
# stream_stamp - pointcloud_stamp. These expose ROS2 replay/synchronizer drift
# without changing the mapper contract.
int64 last_aligned_pointcloud_pose_delta_ns
int64 last_aligned_pointcloud_image_delta_ns
uint64 max_abs_aligned_pointcloud_pose_delta_ns
uint64 max_abs_aligned_pointcloud_image_delta_ns

# Anchor-side drops caused by another required stream being newer than the
# current anchor by more than sync_tolerance_sec.
uint64 pointcloud_anchor_pose_too_new_drops
uint64 pointcloud_anchor_image_too_new_drops
uint64 pointcloud_anchor_depth_too_new_drops
uint64 image_anchor_pointcloud_too_new_drops
uint64 image_anchor_pose_too_new_drops
uint64 image_anchor_depth_too_new_drops
