/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "intern/pmx_types.h"

#include "BLI_vector.hh"

#include <string>

struct Collection;
struct ID;
struct Object;
struct ReportList;
struct bPoseChannel;

namespace blender::io::pmx {

struct PMXImportContext;

/**
 * One PMX "axis / deform" definition (mirrors the FIXED_AXIS / LOCAL_AXIS /
 * deform-layer fields of PMXBone).
 *
 * - `has_fixed_axis` + `fixed_axis[3]`: bone can only rotate around this axis.
 * - `has_local_axis` + `local_x[3]` / `local_z[3]`: bone rotation is expressed
 *   in a custom local coordinate frame.
 * - `deform_after_physics`: PHYSICS_AFTER_DEF flag (deform layer, E/D4 input).
 * - `transform_order`: deformation hierarchy index (D3 records it; D3 itself
 *   does not reorder — see red line D3-c).
 */
struct PMXBoneAxisDefinition {
  std::string bone_name;
  bool has_fixed_axis = false;
  float fixed_axis[3] = {0.0f, 0.0f, 0.0f};
  bool has_local_axis = false;
  float local_x[3] = {0.0f, 0.0f, 0.0f};
  float local_z[3] = {0.0f, 0.0f, 0.0f};
  bool deform_after_physics = false;
  int transform_order = 0;
};

/** Schema-1 container persisted to the model collection / armature object. */
struct PMXBoneAxisDefinitionSet {
  int schema_version = 1;
  std::vector<PMXBoneAxisDefinition> bones;
};

/**
 * D3 data base: persist the PMX axis / deform definitions to the model
 * collection (model-level data base, mirroring the physics/IK/append
 * definitions) and to the armature object (read point for the editing-time
 * approximate operators and the future E-phase native solver).
 *
 * This function MUST NOT create any Blender constraint or modify any bone
 * matrix (red lines D3-a / D3-b). It only records the semantic definition.
 * Editing-time approximations are provided separately by the editor operators
 * `wm.pmx_apply_fixed_axis` and `wm.pmx_apply_local_axis`.
 */
void persist_bone_axis_definition(PMXImportContext &ctx, const PMXModel &model);

/**
 * Read back a persisted axis definition set from an owning ID (armature object
 * or model collection). Returns false when no definition is present.
 */
bool read_bone_axis_definition(const ID &owner, PMXBoneAxisDefinitionSet &r_def);

/**
 * Returns 0/1/2 if `axis` is coincident with ±X/±Y/±Z (a unit vector with a
 * single non-zero component), else -1 (arbitrary axis). Used to decide whether
 * FIXED_AXIS can be applied as a native `protectflag` lock (exact) or must fall
 * back to a Limit Rotation constraint approximation.
 */
int pmx_fixed_axis_principal_index(const float axis[3]);

/**
 * Apply the FIXED_AXIS semantic to a pose channel (editing-time, opt-in):
 * - principal axis  -> native `protectflag` lock on the two other Euler axes
 *   (exact, no constraint);
 * - arbitrary axis  -> a Limit Rotation constraint (`MMD_Fixed_Axis_Approx`,
 *   tagged `mmd_approximate`) as an approximation.
 * Returns true if something was applied (false when already applied / nothing
 * to do). Idempotent via the `mmd_fixed_axis_applied` flag on the pose bone.
 */
bool pmx_apply_fixed_axis_to_pchan(Object *ob,
                                   bPoseChannel *pchan,
                                   const PMXBoneAxisDefinition &def,
                                   ReportList *reports);

/**
 * Apply the LOCAL_AXIS semantic to a pose channel (editing-time, opt-in): a
 * Transformation constraint (`MMD_Local_Axis_Approx`, tagged `mmd_approximate`)
 * approximating the custom local coordinate frame. Returns true if applied.
 * Idempotent via the `mmd_local_axis_applied` flag. Never modifies the bone
 * matrix (red line D3-b).
 */
bool pmx_apply_local_axis_to_pchan(Object *ob,
                                   bPoseChannel *pchan,
                                   const PMXBoneAxisDefinition &def,
                                   ReportList *reports);

}  // namespace blender::io::pmx
