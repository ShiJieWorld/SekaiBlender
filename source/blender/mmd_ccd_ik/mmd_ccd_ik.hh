/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * MMD CCD IK solver.
 * E1: per-bone native IK toggle (mmd_native_ik_enabled).
 * E2: CCD core solver + angle limits.
 */

#pragma once

#include <vector>

namespace blender {
struct bPoseChannel;
struct Bone;
struct Depsgraph;
struct Object;

namespace mmd {

/** One link in a CCD IK chain, resolved from PMX definition. */
struct CCDIKChainLink {
  bPoseChannel *pchan = nullptr;
  bool limit_angle = false;
  bool physics_owned = false;
  bool limit_axis[3] = {true, true, true};
  /** Angle limits in Blender Z-up coordinates (converted from MMD Y-up). */
  float limit_min[3] = {};
  float limit_max[3] = {};
};

struct CCDIKStats {
  int iterations = 0;
  bool converged = false;
  float final_error = 0.0f;
};

/* -------------------------------------------------------------------- */
/** E1: Native IK toggle */
/* -------------------------------------------------------------------- */

/** Set the native IK toggle on a Bone's system_properties. */
void mmd_native_ik_set_enabled(Bone &bone, bool enabled);

/** Query the native IK toggle from a Bone's system_properties.
 *  Returns true if the toggle is not present (default: IK enabled). */
bool mmd_native_ik_is_enabled(const Bone &bone);

/** Legacy native/iTaSC behavior is available as an emergency rollback. */
bool mmd_ccd_use_legacy_solver();

/** Native V8 solver is enabled by default; set MMD_CCD_V8=0 for V2 fallback. */
bool mmd_ccd_use_v8_solver();

/* -------------------------------------------------------------------- */
/** E2: CCD solver */
/* -------------------------------------------------------------------- */

/**
 * Solve a single CCD IK chain.
 *
 * \param armature_obj The owning armature Object (for world-space conversion).
 * \param chain       The IK chain links, from tip (index 0) to root.
 * \param target_world Target position in world space.
 * \param loop_count  Max iterations (from PMX definition).
 * \param angle_limit Max single-step rotation per iteration (radians).
 * \param r_stats     Optional output statistics.
 * \return true if converged within threshold.
 */
bool mmd_ccd_solve_chain(Object &armature_obj,
                         const std::vector<CCDIKChainLink> &chain,
                         const float target_world[3],
                         int loop_count,
                         float angle_limit,
                          CCDIKStats *r_stats = nullptr);

/** Stateless PMX CCD solver. The effector is the PMX IK target bone, while
 * target_pose is the IK control bone position in armature pose space. */
bool mmd_ccd_v2_solve_chain(Object &armature_obj,
                            const std::vector<CCDIKChainLink> &chain,
                            bPoseChannel &effector,
                            const float target_pose[3],
                            int loop_count,
                            float angle_limit,
                            CCDIKStats *r_stats = nullptr,
                             std::vector<bPoseChannel *> *r_modified_channels = nullptr);

/** Publish pose_mat changes made by native CCD to Blender's derived caches. */
void mmd_ccd_ik_sync_pose_channel(Object &armature_obj,
                                  bPoseChannel &pchan,
                                  bool publish_to_original = true);

/** Finalize the global rest-to-pose matrices used by Armature modifiers. */
void mmd_ccd_ik_finalize_pose_deform(Object &armature_obj,
                                     bool publish_to_original = true);

}  // namespace mmd
}  // namespace blender
