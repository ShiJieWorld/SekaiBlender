/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "intern/pmx_types.h"

#include "BLI_vector.hh"

#include <array>
#include <string>

struct Collection;
struct ID;
struct Object;
struct ReportList;

namespace blender::io::pmx {

struct PMXImportContext;

/**
 * One link in a PMX IK chain (mirrors PMXIKLink).
 * bone_name is the resolved Blender bone name (= PMX name_local).
 */
struct PMXBoneIKLink {
  std::string bone_name;
  bool limit_angle = false;
  bool physics_owned = false;
  std::array<float, 3> limit_min{};
  std::array<float, 3> limit_max{};
};

/**
 * One PMX IK bone definition (mirrors the IK fields of PMXBone).
 * bone_name is the IK bone; target_name is the IK target bone.
 */
struct PMXBoneIKDefinition {
  std::string bone_name;
  std::string target_name;
  int loop_count = 0;
  float angle_limit = 0.0f;
  std::vector<PMXBoneIKLink> links;
};

/** Schema-2 container persisted to the model collection / armature object. */
struct PMXBoneIKDefinitionSet {
  int schema_version = 2;
  std::vector<PMXBoneIKDefinition> ik_bones;
};

/**
 * D1 data base: persist the PMX IK definitions to the model collection
 * (model-level data base, mirroring the physics definition) and to the
 * armature object (read point for the VMD importer and the future E-phase
 * MMD CCD solver).
 *
 * This function MUST NOT create any Blender IK constraint (red line 145).
 * It only records the semantic definition so a real MMD CCD solver can be
 * rebuilt later. Editing-time approximate IK is provided separately by the
 * editor operator `wm.pmx_apply_ik`.
 */
void persist_bone_ik_definition(PMXImportContext &ctx, const PMXModel &model);

/**
 * Read back a persisted IK definition set from an owning ID (armature object
 * or model collection). Returns false when no definition is present.
 */
bool read_bone_ik_definition(const ID &owner, PMXBoneIKDefinitionSet &r_def);

}  // namespace blender::io::pmx
