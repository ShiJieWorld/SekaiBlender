/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 *
 * PMX source-data retention.
 *
 * The PMX importer converts a `PMXModel` into native Blender data. That
 * conversion is lossy in one direction only: some PMX sections have no Blender
 * representation at all (display frames, additional UV, non-vertex Morph
 * offsets), and others survive as *derived* data that cannot be inverted
 * reliably (bone roll is computed from `local_x`/`local_z`, material flags
 * collapse into a shader graph).
 *
 * This module persists exactly that non-recoverable remainder, so a later PMX
 * export can rebuild a complete `PMXModel` by combining:
 *
 *   1. retained source data (this module),
 *   2. live Blender data (vertex positions, weights, Shape Keys),
 *   3. the persisted `mmd_physics_definition` (rigid bodies and joints).
 *
 * Deliberately NOT retained, because Blender is the authority for it:
 *   - vertex positions / normals / UV       -> Mesh
 *   - face indices and material grouping    -> Mesh `material_index`
 *   - vertex Morph offsets                  -> Shape Keys
 *   - rigid bodies and joints               -> `mmd_physics_definition`
 *   - IK / append / axis bone semantics     -> `mmd_pmx_bone_*_definition`
 *
 * Keeping bulk geometry out of the IDProperty tree is what makes this cheap:
 * the retained payload stays proportional to the model's *metadata*, not to its
 * vertex count.
 *
 * Storage: `mmd_pmx_source_data` on the model Collection's `system_properties`.
 */

#pragma once

#include "IO_pmx.hh"
#include "intern/pmx_types.h"

#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <array>
#include <string>
#include <vector>

struct Collection;
struct Mesh;
struct Object;
struct ReportList;

namespace blender::io::pmx {

struct PMXImportContext;

/** Current `mmd_pmx_source_data` schema version. */
inline constexpr int kPMXSourceDataSchemaVersion = 1;

/**
 * Per-vertex PMX vertex index, on the Point domain.
 *
 * This is the vertex *identity* map that PMX export needs: it survives into the
 * .blend file, unlike the import-time `SubMeshInfo::new_to_old` mapping. In
 * split-by-material mode every sub-mesh carries the indices of the original
 * model vertices it was built from.
 *
 * Export must validate this attribute rather than trust it: user edits can make
 * it incomplete (new vertices), duplicated (subdivision) or out of range.
 */
inline constexpr char kPMXVertexIndexAttribute[] = "pmx_vertex_index";

/**
 * Additional UV attribute base names, on the Point domain.
 *
 * PMX stores up to four additional UV sets of four floats each. Blender has no
 * float4 attribute type in general use, so each set is split into two float2
 * attributes: `pmx_add_uv<N>_xy` holds components 0-1 and `pmx_add_uv<N>_zw`
 * holds components 2-3.
 */
inline constexpr char kPMXAdditionalUVPrefix[] = "pmx_add_uv";

/** One PMX texture table entry, kept in PMX index order. */
struct PMXSourceTexture {
  /** Path exactly as stored in the PMX file (usually relative, `\` separated). */
  std::string path;
};

/**
 * One PMX material.
 *
 * Retained in full rather than diffed against the created Blender Material:
 * the import path collapses most of these fields into a node graph, so there is
 * no unambiguous way to tell an untouched value from an edited one. Export
 * treats this as the authority for everything except the toon-edge properties,
 * which the MMD Render panel exposes for editing.
 */
struct PMXSourceMaterial {
  int pmx_index = -1;
  std::string name_local;
  std::string name_universal;
  std::array<float, 4> diffuse{};
  std::array<float, 3> specular{};
  float specular_power = 0.0f;
  std::array<float, 3> ambient{};
  int flag = 0;
  std::array<float, 4> edge_color{};
  float edge_size = 0.0f;
  int texture_index = -1;
  int sphere_texture_index = -1;
  int sphere_mode = 0;
  int toon_flag = 0;
  int toon_texture_index = -1;
  int toon_internal_value = 0;
  std::string memo;
  /**
   * PMX face-vertex count for this material.
   *
   * Export recomputes the grouping from the Mesh `material_index` attribute;
   * this value is the cross-check that detects mesh edits which changed the
   * material distribution.
   */
  int face_vertex_count = 0;
  /** Created Blender Material name, or empty when none was created. */
  std::string blender_material_name;
};

/** One PMX IK link, nested inside `PMXSourceBone`. */
struct PMXSourceIKLink {
  int bone_index = -1;
  bool limit_angle = false;
  std::array<float, 3> limit_min{};
  std::array<float, 3> limit_max{};
};

/**
 * One PMX bone.
 *
 * `pos` is intentionally absent: the EditBone head is an exact, invertible
 * image of it. Everything here is either absent from Blender entirely
 * (`transform_order`, the full `flag` bitfield, external parent) or derived in a
 * way that cannot be inverted (`local_x`/`local_z` become a single bone roll).
 *
 * IK and append-transform semantics are already persisted by
 * `mmd_pmx_bone_ik_definition` / `mmd_pmx_bone_append_definition`. They are
 * repeated here so PMX export has one self-contained source for the bone
 * section and does not have to reconcile three schemas that were written for
 * the solver rather than for export.
 */
struct PMXSourceBone {
  int pmx_index = -1;
  std::string name_local;
  std::string name_universal;
  int parent_index = -1;
  int transform_order = 0;
  int flag = 0;
  /** `>= 0` bone index, or `-2` when the tail is a position offset. */
  int tail_pos_bone = -1;
  std::array<float, 3> tail_pos_offset{};
  int inherit_parent_index = -1;
  float inherit_parent_ratio = 0.0f;
  std::array<float, 3> fixed_axis{};
  std::array<float, 3> local_x{};
  std::array<float, 3> local_z{};
  int ik_target_index = -1;
  int ik_loop_count = 0;
  float ik_angle_limit = 0.0f;
  std::vector<PMXSourceIKLink> ik_links;
  int external_parent_index = -1;
  /** Created Blender bone name, or empty when the bone was not created. */
  std::string blender_bone_name;
};

struct PMXSourceBoneMorphOffset {
  int bone_index = -1;
  std::array<float, 3> pos{};
  std::array<float, 4> rot{};
};

struct PMXSourceUVMorphOffset {
  int vertex_index = -1;
  std::array<float, 4> offset{};
};

struct PMXSourceMaterialMorphOffset {
  int material_index = -1;
  int calc_mode = 0;
  std::array<float, 4> diffuse{};
  std::array<float, 3> specular{};
  float specular_power = 0.0f;
  std::array<float, 3> ambient{};
  std::array<float, 4> edge_color{};
  float edge_size = 0.0f;
  std::array<float, 4> texture_factor{};
  std::array<float, 4> sphere_texture_factor{};
  std::array<float, 4> toon_texture_factor{};
};

struct PMXSourceGroupMorphOffset {
  int morph_index = -1;
  float influence = 0.0f;
};

struct PMXSourceImpulseMorphOffset {
  int rigid_index = -1;
  int local_flag = 0;
  std::array<float, 3> velocity{};
  std::array<float, 3> torque{};
};

/**
 * One PMX morph.
 *
 * Vertex Morph offsets are NOT retained; they are the single largest section in
 * a typical PMX file and Blender owns them as Shape Keys. `vertex_offset_count`
 * and `blender_shape_key_name` are kept so export can find the Shape Key and
 * report when its size no longer matches the source.
 *
 * Every other offset kind is retained verbatim, because import produces no
 * Blender data for them at all.
 *
 * Note: a Shape Key stores an offset for every vertex, so exporting a Vertex
 * Morph from it cannot distinguish "offset was exactly zero" from "vertex was
 * not part of the morph". Export therefore emits only non-zero offsets, which
 * is semantically equivalent but need not reproduce the original offset count.
 */
struct PMXSourceMorph {
  int pmx_index = -1;
  std::string name_local;
  std::string name_universal;
  int panel = 0;
  int type = 0;
  int vertex_offset_count = 0;
  std::string blender_shape_key_name;
  std::vector<PMXSourceGroupMorphOffset> group_offsets;
  std::vector<PMXSourceBoneMorphOffset> bone_offsets;
  std::vector<PMXSourceUVMorphOffset> uv_offsets;
  std::vector<PMXSourceMaterialMorphOffset> material_offsets;
  std::vector<PMXSourceImpulseMorphOffset> impulse_offsets;
};

/** One PMX display frame entry (表示枠). Absent from Blender entirely. */
struct PMXSourceDisplayFrame {
  std::string name_local;
  std::string name_universal;
  int flag = 0;
  struct Item {
    int type = 0;
    int index = -1;
  };
  std::vector<Item> items;
};

/** The complete retained remainder of one imported PMX file. */
struct PMXSourceData {
  int schema_version = kPMXSourceDataSchemaVersion;

  /* --- Header --- */
  float pmx_version = 2.0f;
  int encoding = 0;
  int additional_uv_count = 0;

  /* --- Model info --- */
  std::string name_local;
  std::string name_universal;
  std::string comment_local;
  std::string comment_universal;

  /* --- Import provenance --- */
  std::string source_filepath;
  /** Import scale; export divides Blender coordinates by it. */
  float global_scale = 0.08f;
  bool split_by_material = true;

  /* --- Section counts, for validating the live scene against the source --- */
  int vertex_count = 0;
  int face_index_count = 0;
  int rigid_body_count = 0;
  int joint_count = 0;

  /* --- Retained sections --- */
  std::vector<PMXSourceTexture> textures;
  std::vector<PMXSourceMaterial> materials;
  std::vector<PMXSourceBone> bones;
  std::vector<PMXSourceMorph> morphs;
  std::vector<PMXSourceDisplayFrame> display_frames;
};

/**
 * Build the retained source data from a parsed model plus the Blender names the
 * import produced.
 *
 * `bone_names` is indexed by PMX bone index. `morph_indices` and `morph_names`
 * are parallel arrays mapping PMX morph indices to Shape Key names.
 */
PMXSourceData build_pmx_source_data(const PMXModel &model,
                                    const PMXImportParams &params,
                                    Span<std::string> bone_names,
                                    Span<int> morph_indices,
                                    Span<std::string> morph_names,
                                    const PMXImportContext &ctx);

/** Persist `data` to `model_root`'s `system_properties`, replacing any previous value. */
bool serialize_pmx_source_data(Collection &model_root,
                               const PMXSourceData &data,
                               ReportList *reports);

/**
 * Read back previously persisted source data.
 *
 * Returns false when the property is missing, is not schema
 * `kPMXSourceDataSchemaVersion`, or fails internal consistency checks.
 */
bool deserialize_pmx_source_data(const Collection &model_root,
                                 PMXSourceData &data,
                                 ReportList *reports);

/**
 * Write `kPMXVertexIndexAttribute` on one mesh Object.
 *
 * `new_to_old` maps mesh vertex index to model vertex index in
 * split-by-material mode; pass nullptr in single-mesh mode, where the mapping
 * is the identity.
 */
bool write_pmx_vertex_index_attribute(Object *obj,
                                      const PMXModel &model,
                                      const Vector<int> *new_to_old = nullptr);

/**
 * Write the `pmx_add_uv<N>_xy` / `pmx_add_uv<N>_zw` attributes on one mesh
 * Object. No-op when the model has no additional UV sets.
 */
bool write_pmx_additional_uv_attributes(Object *obj,
                                        const PMXModel &model,
                                        const Vector<int> *new_to_old = nullptr);

/**
 * Persist source data plus every per-mesh attribute for one finished import.
 *
 * Call after mesh, armature and Morph import have completed, so the Blender
 * names recorded in the source data are final.
 */
void persist_pmx_source_data(PMXImportContext &ctx,
                             const PMXModel &model,
                             Span<std::string> bone_names);

}  // namespace blender::io::pmx
