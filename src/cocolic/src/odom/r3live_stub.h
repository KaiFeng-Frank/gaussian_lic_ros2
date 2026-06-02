/*
 * Coco-LIC ROS2 Jazzy port — R3LIVE camera front-end STUB (LIO-first milestone).
 *
 * The real camera/r3live.hpp is the R3LIVE visual-inertial front-end (~17K LOC,
 * deep ROS1 coupling: ros/ros.h, common_lib.h, so3_math.h, ikd-Tree, optical
 * flow, rgb_map). Porting it is S4 and is REQUIRED for cm-fidelity, because the
 * cocolic_livo reference is upstream's LICO (camera-on) output.
 *
 * For the LIO-first runnable milestone we replace it with this no-op stub so
 * odometry_manager compiles and runs in LiDAR-Inertial mode (odometry_mode=LIO):
 * every camera code path in odometry_manager is gated by `if (process_image)`
 * (false without image ingest) and is additionally #if 0'd where it touches
 * camera member types; only the three method calls below survive, as no-ops.
 *
 * To enable full LICO (and reach cm), port the real module and swap the include
 * in odometry_manager.h (r3live_stub.h → camera/r3live.hpp).
 */
#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include <utility>

#include <utils/parameter_struct.h>  // cocolic::ExtrinsicParam

namespace cocolic
{

  // Minimal no-op stand-in for the R3LIVE visual front-end (LIO-first build).
  class R3LIVE
  {
  public:
    typedef std::shared_ptr<R3LIVE> Ptr;

    R3LIVE(const YAML::Node & /*cam_node*/, const ExtrinsicParam & /*EP_CtoI*/) {}

    // The only camera calls reachable in the LIO build (the rest are #if 0'd in
    // odometry_manager). Variadic ⇒ accept any args, only instantiated if called.
    template <class... A> void UpdateVisualSubMap(A &&...) {}
    template <class... A> void UpdateVisualGlobalMap(A &&...) {}
    template <class... A> void AssociateNewPointsToCurrentImg(A &&...) {}
  };

} // namespace cocolic
