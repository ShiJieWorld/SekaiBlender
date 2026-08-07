/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 *
 * PMX export.
 *
 * A complete `PMXModel` is rebuilt from four sources, each of which owns a
 * disjoint part of the format:
 *
 *   1. `mmd_pmx_source_data` on the model Collection -- every section that has
 *      no Blender representation, or that survives import only as derived data
 *      (material flags, bone `transform_order`, display frames, non-Vertex
 *      Morph offsets). Restored verbatim.
 *   2. The live Mesh objects -- vertex positions, normals, UV, bone weights and
 *      the face/material grouping. Every one of these needs a coordinate
 *      inversion applied on the way out.
 *   3. The Armature -- bone rest positions, which retention deliberately does
 *      not store because the EditBone head is an exact image of them.
 *   4. `mmd_physics_definition` -- rigid bodies and joints in Blender import
 *      space. Position, size, rotation, limits, and springs are transformed
 *      back to PMX space on export.
 *
 * The inversions in (2), (3), and (4) all fail *silently* when wrong: the file still
 * parses and the model still renders, just mirrored, inside out, or at 1/12.5
 * scale. `PMXExportReport` therefore counts every place where the live scene
 * could not be represented exactly, and `pmx_model_diff` is what turns a wrong
 * inversion into a named field difference.
 *
 * Scope: this exports models that came from PMX import. A model authored from
 * scratch in Blender has no retained source data, so bone semantics, material
 * flags and display frames cannot be recovered -- export refuses rather than
 * inventing them.
 */

#pragma once

#include "BLI_path_utils.hh"

#include <string>
#include <vector>

struct Collection;
struct Main;
struct Object;
struct ReportList;

namespace blender::io::pmx {

struct PMXExportOptions {
  char filepath[FILE_MAX] = "";
};

/**
 * Outcome of one export.
 *
 * The `*_count` fields describe what was written. The fields below them count
 * places where the live scene could not be represented exactly in PMX; they are
 * all zero for a model that was imported and not edited.
 */
struct PMXExportReport {
  bool success = false;

  int vertex_count = 0;
  int face_count = 0;
  int texture_count = 0;
  int material_count = 0;
  int bone_count = 0;
  int morph_count = 0;
  int display_frame_count = 0;
  int rigid_body_count = 0;
  int joint_count = 0;
  int mesh_object_count = 0;

  /** Mesh vertices that mapped onto an already-written source vertex. */
  int duplicate_vertices = 0;
  /** Duplicates whose position/normal/UV disagreed; the first one written wins. */
  int divergent_duplicates = 0;
  /** Source vertices with no mesh coverage left, written as zeroed vertices. */
  int missing_source_vertices = 0;
  /** Mesh vertices whose `pmx_vertex_index` was absent or out of range. */
  int invalid_vertex_indices = 0;
  /** Vertices whose influence count exceeded the PMX weight type's capacity. */
  int truncated_weights = 0;
  /** Vertex groups that matched no PMX bone, so their weight was dropped. */
  int unmapped_vertex_groups = 0;
  /** Materials whose recomputed face count disagreed with the retained one. */
  int material_face_count_changes = 0;
  /** Existing source texture files copied next to the exported PMX. */
  int copied_texture_files = 0;
  /** Retained texture paths whose source file no longer exists. */
  int missing_texture_files = 0;

  /** Vertex Morph offsets recovered from Shape Keys and written. */
  int vertex_morph_offset_count = 0;
  /** Vertex Morphs whose Shape Key was on no mesh, so their offsets were lost. */
  int unresolved_shape_keys = 0;
  /** Shape Keys whose element count no longer matched the mesh vertex count. */
  int shape_key_size_mismatches = 0;
  /** Impulse Morph offsets restored from retained source data and written. */
  int impulse_morph_offset_count = 0;

  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

/**
 * Find the model-root Collection carrying `mmd_pmx_source_data`.
 *
 * When several imported models are present, `active_object` disambiguates: the
 * Collection that contains it recursively wins. `r_ambiguous` is set when that
 * is still not enough to pick one.
 *
 * Reads `id.system_properties`, so a plain `IDP_GetProperties` lookup would
 * miss it -- same as `mmd_physics_definition`.
 */
Collection *find_pmx_model_collection(Main *bmain, Object *active_object, bool &r_ambiguous);

/**
 * Rebuild a `PMXModel` from `model_root` and write it to `options.filepath`.
 *
 * Takes no `Main`: everything needed comes from the Collection itself, the
 * retained source data on it, and the Meshes it owns. Locating the Collection
 * is the only step that needs `Main`, and that is
 * `find_pmx_model_collection` above.
 */
bool export_pmx_model(Collection &model_root,
                      const PMXExportOptions &options,
                      ReportList *reports,
                      PMXExportReport &r_report);

}  // namespace blender::io::pmx
