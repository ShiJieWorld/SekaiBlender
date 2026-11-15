/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "BLI_vector.hh"

#include <string>

struct Depsgraph;
struct ID;
struct Object;
struct ReportList;

namespace blender::io::pmx {

/**
 * One bone's captured final Pose (armature object space).
 * `pose_mat` is the authoritative transform; `loc`/`quat` are the final channels.
 * `has_ik` / `has_append` are derived read-only from the D1/D2 persisted
 * definitions (informational for the E-phase solver; not used to modify anything).
 */
struct PMXPoseBoneSnapshot {
  std::string name;
  float pose_mat[4][4] = {{0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f},
                           {0.0f, 0.0f, 0.0f, 0.0f}};
  float loc[3] = {0.0f, 0.0f, 0.0f};
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  bool has_ik = false;
  bool has_append = false;
};

/** Schema-1 container of a captured final Pose, consumed by the E-phase solver. */
struct PMXPoseSnapshot {
  int schema_version = 1;
  int bone_count = 0;
  int captured_at_frame = 0;
  std::vector<PMXPoseBoneSnapshot> bones;
};

/**
 * D4 POSE_DONE: sample the armature's currently-solved final Pose and write it
 * to `armature->system_properties["mmd_pose_snapshot"]`, then set
 * `mmd_pose_done = true`.
 *
 * Pure sampling: reads `pose_mat` / `loc` / `quat` only, never modifies a bone
 * or triggers solving (red line D4-b). The caller (operator / importer) is
 * responsible for ensuring the pose exists and is up to date before calling.
 *
 * `has_ik` / `has_append` are looked up read-only from the D1/D2 persisted
 * definitions on the same armature; they do not affect capture correctness.
 */
void mmd_capture_pose_snapshot(Object *armature, ReportList *reports);

/**
 * Read a persisted `mmd_pose_snapshot` from an owning ID into `r_snapshot`.
 * Returns false when no snapshot is present. Does NOT check `mmd_pose_done`.
 */
bool read_pose_snapshot(const ID &owner, PMXPoseSnapshot &r_snapshot);

}  // namespace blender::io::pmx
