/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * E3: Depsgraph integration for MMD CCD IK solver.
 * Evaluates native CCD IK on every depsgraph pose evaluation,
 * called from BKE_pose_eval_done().
 */

#pragma once

struct Depsgraph;
struct Object;

namespace blender::mmd {

/**
 * Evaluate MMD native CCD IK for an armature.
 *
 * Called from BKE_pose_eval_done() in the depsgraph evaluation pipeline,
 * after FCurves and constraints have been applied. Only runs when at
 * least one IK bone has mmd_native_ik_enabled=true.
 *
 * Step-by-step:
 * 1. Read mmd_pmx_bone_ik_definition from armature_obj->id.system_properties
 * 2. Check if any IK bone has native CCD enabled → early out if none
 * 3. Run native V8 by default, or the V2/legacy fallback when explicitly selected
 * 4. For the fallback path, resolve target position and build CCDIKChainLink data
 *    from the persisted PMX definition before solving each enabled chain
 * 5. Restore any temporarily muted MMD_IK_Approx constraint values
 */
void mmd_ccd_ik_evaluate(Depsgraph *depsgraph, Object *armature_obj);

}  // namespace blender::mmd
