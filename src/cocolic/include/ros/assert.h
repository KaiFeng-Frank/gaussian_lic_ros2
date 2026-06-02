// ROS1->ROS2 port compat shim: upstream Coco-LIC includes <ros/assert.h> for
// ROS_ASSERT in otherwise ROS-agnostic estimator/manager code. ROS2 has no
// ros/assert.h, so provide the macros mapped to <cassert>. This keeps the
// ported files verbatim (no per-file edits) while removing the ROS1 dependency.
#pragma once
#include <cassert>
#include <cstdio>

#ifndef ROS_ASSERT
#define ROS_ASSERT(cond) assert(cond)
#endif
#ifndef ROS_ASSERT_MSG
#define ROS_ASSERT_MSG(cond, ...) assert(cond)
#endif
