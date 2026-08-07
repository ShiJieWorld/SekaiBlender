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

namespace blender::io::pmx {

struct PMXImportContext;

/**
 * One PMX "append transform" (追加変換) definition (mirrors the inherit fields
 * of PMXBone). `bone_name` is the bone that inherits; `parent_name` is the
 * resolved bone it inherits from (`inherit_parent_index`); `ratio` is
 * `inherit_parent_ratio` and MAY be negative ("cancel rotation/translation").
 *
 * `mode` encoding: 1 = rotation only, 2 = translation only, 3 = both.
 */
struct PMXBoneAppendDefinition {
  std::string bone_name;
  int mode = 0;
  std::string parent_name;
  float ratio = 0.0f;
};

/** Schema-1 container persisted to the model collection / armature object. */
struct PMXBoneAppendDefinitionSet {
  int schema_version = 1;
  std::vector<PMXBoneAppendDefinition> append_bones;
};

/**
 * D2 data base: persist the PMX append-transform definitions to the model
 * collection (model-level data base, mirroring the physics/IK definitions) and
 * to the armature object (read point for the editing-time approximate operator
 * and the future E-phase native append solver).
 *
 * This function MUST NOT create any Blender Transformation constraint or shadow
 * bone (red line D2-a). It only records the semantic definition. Editing-time
 * approximate append transform is provided separately by the editor operator
 * `wm.pmx_apply_append_transform`.
 */
void persist_bone_append_definition(PMXImportContext &ctx, const PMXModel &model);

/**
 * Read back a persisted append-transform definition set from an owning ID
 * (armature object or model collection). Returns false when no definition is
 * present.
 */
bool read_bone_append_definition(const ID &owner, PMXBoneAppendDefinitionSet &r_def);

}  // namespace blender::io::pmx
