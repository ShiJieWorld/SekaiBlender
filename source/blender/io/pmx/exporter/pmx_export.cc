/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "pmx_export.hh"

#include "DNA_armature_types.h"
#include "DNA_collection_types.h"
#include "DNA_key_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"

#include "BKE_armature.hh"
#include "BKE_attribute.hh"
#include "BKE_collection.hh"
#include "BKE_deform.hh"
#include "BKE_idprop.hh"
#include "BKE_key.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"

#include "BLI_array.hh"
#include "BLI_fileops.hh"
#include "BLI_index_range.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "mmd_physics_definition.hh"
#include "importer/pmx_import_material.hh"
#include "importer/pmx_source_data.hh"
#include "intern/pmx_types.h"
#include "intern/pmx_writer.h"

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace blender::io::pmx {
namespace {

/* --- The inversions -------------------------------------------------------- */
/*
 * Every function here is the exact inverse of a named import step. Each is
 * written against the import site it inverts so the pair can be checked by
 * reading them side by side; getting one wrong produces a file that opens and
 * renders but is silently mirrored, inside out, or mis-scaled.
 */

/**
 * Inverse of `transform_coord` (pmx_import_mesh.cc):
 *   blender[0] = pmx[0] * scale
 *   blender[1] = pmx[2] * scale
 *   blender[2] = pmx[1] * scale
 *
 * The Y/Z swap is its own inverse, so only the scale direction flips.
 */
void invert_position(float r_pmx[3], const float3 &blender, const float scale)
{
  r_pmx[0] = blender[0] / scale;
  r_pmx[1] = blender[2] / scale;
  r_pmx[2] = blender[1] / scale;
}

/** Inverse of `mmd_physics::transform_position` in mmd_physics_definition.cc. */
void invert_position(float r_pmx[3], const std::array<float, 3> &blender, const float scale)
{
  r_pmx[0] = blender[0] / scale;
  r_pmx[1] = blender[2] / scale;
  r_pmx[2] = blender[1] / scale;
}

/**
 * Inverse of `mmd_physics::transform_rotation`:
 *   blender = (-pmx.x, -pmx.z, -pmx.y)
 *
 * This is self-inverse. The persisted field is already the Euler triplet used
 * by the Blender physics runtime; no matrix/Euler decomposition is involved.
 */
void invert_rotation(float r_pmx[3], const std::array<float, 3> &blender)
{
  r_pmx[0] = -blender[0];
  r_pmx[1] = -blender[2];
  r_pmx[2] = -blender[1];
}

/** Inverse of the Y/Z-only spring-vector transform. */
void invert_yz_swap(float r_pmx[3], const std::array<float, 3> &blender)
{
  r_pmx[0] = blender[0];
  r_pmx[1] = blender[2];
  r_pmx[2] = blender[1];
}

/** Inverse of the shape-size axis mapping used by the physics importer. */
void invert_shape_size(float r_pmx[3],
                       const std::array<float, 3> &blender,
                       const uint8_t shape_type,
                       const float scale)
{
  r_pmx[0] = blender[0] / scale;
  if (shape_type == 1) {
    r_pmx[1] = blender[2] / scale;
    r_pmx[2] = blender[1] / scale;
  }
  else {
    r_pmx[1] = blender[1] / scale;
    r_pmx[2] = blender[2] / scale;
  }
}

/**
 * Inverse of the corner-normal mapping (pmx_import_mesh.cc):
 *   corner_normals[c] = float3(n[0], n[2], n[1])
 *
 * Same Y/Z swap as position, but normals are never scaled.
 */
void invert_normal(float r_pmx[3], const float3 &blender)
{
  r_pmx[0] = blender[0];
  r_pmx[1] = blender[2];
  r_pmx[2] = blender[1];
}

/**
 * Inverse of the UV V-flip (pmx_import_mesh.cc):
 *   uv_map[c][1] = 1.0f - pmx_uv[1]
 *
 * Self-inverse.
 */
void invert_uv(float r_pmx[2], const float2 &blender)
{
  r_pmx[0] = blender[0];
  r_pmx[1] = 1.0f - blender[1];
}

/**
 * Inverse of the winding reversal (pmx_import_mesh.cc):
 *   corner_verts[face * 3 + c] = face_indices[face * 3 + (2 - c)]
 *
 * Self-inverse: reading corner `2 - c` puts the original PMX order back.
 */
int inverted_corner(const int corner)
{
  return 2 - corner;
}

/* --- Small helpers --------------------------------------------------------- */

void add_warning(PMXExportReport &report, ReportList *reports, const std::string &message)
{
  report.warnings.push_back(message);
  if (reports) {
    BKE_report(reports, RPT_WARNING, message.c_str());
  }
}

void add_error(PMXExportReport &report, ReportList *reports, const std::string &message)
{
  report.errors.push_back(message);
  if (reports) {
    BKE_report(reports, RPT_ERROR, message.c_str());
  }
}

bool path_is_empty_or_whitespace(const std::string &path)
{
  return path.empty() || std::all_of(path.begin(), path.end(), [](const unsigned char c) {
           return std::isspace(c);
         });
}

/**
 * Build the texture table and package every available source texture beside
 * the exported PMX.
 *
 * PMX texture paths are resolved relative to the PMX file, not embedded in the
 * file. Retaining the raw string while writing the PMX to a different directory
 * therefore creates a structurally perfect file that imports as a white model.
 * Safe relative paths keep their exact string (preserving the round-trip
 * contract) and are copied to that same relative destination. Absolute paths or
 * paths that escape through `..` are rewritten into a safe `textures` folder.
 */
bool build_texture_table(const PMXSourceData &source_data,
                         const char *output_filepath,
                         PMXModel &model,
                         PMXExportReport &report,
                         ReportList *reports)
{
  char source_dir[FILE_MAX];
  char output_dir[FILE_MAX];
  BLI_path_split_dir_part(source_data.source_filepath.c_str(), source_dir, sizeof(source_dir));
  BLI_path_split_dir_part(output_filepath, output_dir, sizeof(output_dir));
  BLI_path_normalize_native(source_dir);
  BLI_path_normalize_native(output_dir);

  if (!source_data.textures.empty() && (source_dir[0] == '\0' || output_dir[0] == '\0')) {
    add_error(report, reports, "PMX export: source or destination directory for textures is invalid");
    return false;
  }

  model.textures.reserve(source_data.textures.size());
  for (const int texture_index : IndexRange(source_data.textures.size())) {
    const std::string &raw_path = source_data.textures[texture_index].path;
    PMXTexture texture{raw_path};
    if (path_is_empty_or_whitespace(raw_path)) {
      model.textures.push_back(std::move(texture));
      continue;
    }
    if (raw_path.size() >= FILE_MAX) {
      report.missing_texture_files++;
      add_warning(report,
                  reports,
                  "PMX export: texture path is too long to resolve and will be retained verbatim: '" +
                      raw_path + "'");
      model.textures.push_back(std::move(texture));
      continue;
    }

    char source_path[FILE_MAX];
    BLI_strncpy(source_path, raw_path.c_str(), sizeof(source_path));
    BLI_path_slash_native(source_path);
    const bool raw_is_absolute = BLI_path_is_abs_from_cwd(source_path);
    if (!raw_is_absolute) {
      char joined[FILE_MAX];
      BLI_path_join(joined, sizeof(joined), source_dir, source_path);
      BLI_strncpy(source_path, joined, sizeof(source_path));
    }
    BLI_path_normalize_native(source_path);

    if (!BLI_exists(source_path)) {
      report.missing_texture_files++;
      add_warning(report,
                  reports,
                  "PMX export: retained texture is missing and cannot be packaged: '" + raw_path +
                      "' (resolved: '" + std::string(source_path) + "')");
      model.textures.push_back(std::move(texture));
      continue;
    }

    char destination_path[FILE_MAX];
    bool preserve_relative_path = !raw_is_absolute;
    if (preserve_relative_path) {
      char relative_native[FILE_MAX];
      BLI_strncpy(relative_native, raw_path.c_str(), sizeof(relative_native));
      BLI_path_slash_native(relative_native);
      BLI_path_join(destination_path, sizeof(destination_path), output_dir, relative_native);
      BLI_path_normalize_native(destination_path);
      preserve_relative_path = BLI_path_contains(output_dir, destination_path);
    }

    if (!preserve_relative_path) {
      const char *basename = BLI_path_basename(source_path);
      const std::string safe_name = std::to_string(texture_index) + "_" +
                                    (basename[0] != '\0' ? basename : "texture.bin");
      texture.path = std::string("textures\\") + safe_name;
      BLI_path_join(destination_path, sizeof(destination_path), output_dir, "textures", safe_name.c_str());
      BLI_path_normalize_native(destination_path);
    }

    if (BLI_path_cmp_normalized(source_path, destination_path) != 0) {
      if (!BLI_file_ensure_parent_dir_exists(destination_path) ||
          BLI_copy(source_path, destination_path) != 0)
      {
        add_error(report,
                  reports,
                  "PMX export: failed to copy texture '" + std::string(source_path) + "' to '" +
                      std::string(destination_path) + "'");
        return false;
      }
      report.copied_texture_files++;
    }
    model.textures.push_back(std::move(texture));
  }
  report.texture_count = int(model.textures.size());
  return true;
}

bool mesh_has_vertex_index(const Mesh &mesh)
{
  return mesh.attributes().contains(kPMXVertexIndexAttribute);
}

/** Depth-first walk of `collection`, collecting meshes that carry the identity map. */
void collect_pmx_meshes(Collection &collection, Vector<Object *> &r_objects)
{
  for (CollectionObject &collection_object : collection.gobject) {
    Object *object = collection_object.ob;
    if (object == nullptr || object->type != OB_MESH) {
      continue;
    }
    const Mesh *mesh = reinterpret_cast<const Mesh *>(object->data);
    if (mesh == nullptr || mesh->verts_num == 0 || !mesh_has_vertex_index(*mesh)) {
      continue;
    }
    r_objects.append_non_duplicates(object);
  }
  for (CollectionChild &child : collection.children) {
    if (child.collection != nullptr) {
      collect_pmx_meshes(*child.collection, r_objects);
    }
  }
}

Object *find_armature(Collection &collection)
{
  for (CollectionObject &collection_object : collection.gobject) {
    Object *object = collection_object.ob;
    if (object != nullptr && object->type == OB_ARMATURE) {
      return object;
    }
  }
  for (CollectionChild &child : collection.children) {
    if (child.collection != nullptr) {
      if (Object *found = find_armature(*child.collection)) {
        return found;
      }
    }
  }
  return nullptr;
}

/** How many bone slots the PMX weight type encodes. */
int weight_arity(const BoneWeightType type)
{
  switch (type) {
    case BoneWeightType::BDEF1:
      return 1;
    case BoneWeightType::BDEF2:
    case BoneWeightType::SDEF:
      return 2;
    case BoneWeightType::BDEF4:
    case BoneWeightType::QDEF:
      return 4;
  }
  return 1;
}

struct Influence {
  int bone_index = -1;
  float weight = 0.0f;
};

/**
 * Write `influences` into `vertex` in the exact shape `PMXReader` produces, so a
 * round-trip compares equal.
 *
 * The reader normalizes each weight type: BDEF1 always yields one index with
 * weight 1, BDEF2 and SDEF always yield two indices with `{w, 1 - w}`, and
 * BDEF4/QDEF always yield four of each. Import dropped zero-weight influences,
 * so shorter inputs are padded here; the padded bone index is unrecoverable and
 * `pmx_model_diff` ignores indices whose weight is zero on both sides.
 */
void apply_influences(PMXVertex &vertex, Span<Influence> influences)
{
  const int arity = weight_arity(vertex.weight_type);
  vertex.bone_indices.clear();
  vertex.bone_weights.clear();

  for (const int i : IndexRange(arity)) {
    const bool present = i < influences.size();
    vertex.bone_indices.push_back(present ? influences[i].bone_index : 0);
    vertex.bone_weights.push_back(present ? influences[i].weight : 0.0f);
  }

  switch (vertex.weight_type) {
    case BoneWeightType::BDEF1:
      vertex.bone_weights[0] = 1.0f;
      break;
    case BoneWeightType::BDEF2:
    case BoneWeightType::SDEF:
      /* The reader derives the second weight, so make it consistent here. */
      vertex.bone_weights[1] = 1.0f - vertex.bone_weights[0];
      break;
    case BoneWeightType::BDEF4:
    case BoneWeightType::QDEF:
      break;
  }
}

/**
 * Reduce `influences` to what the weight type can hold.
 *
 * Keeps the largest weights and renormalizes, matching mmd_tools. Returns true
 * when anything was dropped, which only happens on an edited mesh: import never
 * produces more influences than the type allows.
 */
bool truncate_influences(Vector<Influence> &influences, const BoneWeightType type)
{
  const int arity = weight_arity(type);
  if (influences.size() <= arity) {
    return false;
  }
  std::stable_sort(influences.begin(), influences.end(), [](const Influence &a, const Influence &b) {
    return a.weight > b.weight;
  });
  influences.resize(arity);

  float sum = 0.0f;
  for (const Influence &influence : influences) {
    sum += influence.weight;
  }
  if (sum > 0.0f) {
    for (Influence &influence : influences) {
      influence.weight /= sum;
    }
  }
  return true;
}

/* --- Per-object vertex/face gathering -------------------------------------- */

/** One source-model vertex slot, filled by whichever mesh reaches it first. */
struct VertexSlot {
  PMXVertex vertex;
  bool filled = false;
};

/**
 * Map each material slot of `object` to a PMX material index.
 *
 * In split-by-material mode every object holds exactly one material, so the
 * slot index carries no PMX meaning on its own; the mapping has to go through
 * the Material name recorded in retention.
 */
Vector<int> build_material_slot_map(const Object &object,
                                    const Mesh &mesh,
                                    const PMXSourceData &source_data,
                                    PMXExportReport &report,
                                    ReportList *reports)
{
  Vector<int> slot_to_pmx(std::max<int>(mesh.totcol, 0), -1);
  for (const int slot : IndexRange(std::max<int>(mesh.totcol, 0))) {
    const Material *material = mesh.mat ? mesh.mat[slot] : nullptr;
    if (material == nullptr) {
      continue;
    }
    const std::string name = material->id.name + 2;
    for (const PMXSourceMaterial &source : source_data.materials) {
      if (source.blender_material_name == name) {
        slot_to_pmx[slot] = source.pmx_index;
        break;
      }
    }
    if (slot_to_pmx[slot] < 0) {
      add_warning(report,
                  reports,
                  "PMX export: material '" + name + "' on '" +
                      std::string(object.id.name + 2) +
                      "' matches no PMX material; its faces are dropped");
    }
  }
  return slot_to_pmx;
}

/** Resolve this object's vertex groups to PMX bone indices, once per object. */
Vector<int> build_group_to_bone_map(Object &object,
                                    const PMXSourceData &source_data,
                                    PMXExportReport &report,
                                    ReportList *reports)
{
  Map<std::string, int> bone_to_pmx;
  for (const PMXSourceBone &bone : source_data.bones) {
    if (!bone.blender_bone_name.empty()) {
      bone_to_pmx.add_overwrite(bone.blender_bone_name, bone.pmx_index);
    }
  }

  Vector<int> group_to_bone;
  const ListBaseT<bDeformGroup> *groups = BKE_object_defgroup_list(&object);
  if (groups == nullptr) {
    return group_to_bone;
  }
  for (const bDeformGroup &group : *groups) {
    const std::string name = group.name;
    /* `mmd_edge_scale` is PMX edge metadata stored as a vertex group, not a
     * bone weight (pmx_import_weights.cc). It must never become an influence. */
    if (name == kPMXEdgeScaleGroup) {
      group_to_bone.append(-1);
      continue;
    }
    const int bone_index = bone_to_pmx.lookup_default(name, -1);
    if (bone_index < 0) {
      report.unmapped_vertex_groups++;
      add_warning(report,
                  reports,
                  "PMX export: vertex group '" + name + "' on '" +
                      std::string(object.id.name + 2) +
                      "' matches no PMX bone; its weight is dropped");
    }
    group_to_bone.append(bone_index);
  }
  return group_to_bone;
}

void gather_object_vertices(Object &object,
                            const PMXSourceData &source_data,
                            MutableSpan<VertexSlot> slots,
                            PMXExportReport &report,
                            ReportList *reports)
{
  const Mesh &mesh = *reinterpret_cast<const Mesh *>(object.data);
  const bke::AttributeAccessor attributes = mesh.attributes();

  const bke::AttributeReader<int> source_indices = attributes.lookup<int>(
      kPMXVertexIndexAttribute, bke::AttrDomain::Point);
  if (!source_indices) {
    return;
  }
  const bke::AttributeReader<int8_t> weight_types = attributes.lookup<int8_t>(
      "pmx_weight_type", bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_c = attributes.lookup<float3>("pmx_sdef_c",
                                                                       bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_r0 = attributes.lookup<float3>("pmx_sdef_r0",
                                                                        bke::AttrDomain::Point);
  const bke::AttributeReader<float3> sdef_r1 = attributes.lookup<float3>("pmx_sdef_r1",
                                                                        bke::AttrDomain::Point);
  const bke::AttributeReader<int> sdef_bone0 = attributes.lookup<int>("pmx_sdef_bone0",
                                                                     bke::AttrDomain::Point);
  const bke::AttributeReader<int> sdef_bone1 = attributes.lookup<int>("pmx_sdef_bone1",
                                                                     bke::AttrDomain::Point);

  /* PMX stores UV and normals per vertex, import wrote them to every corner of
   * that vertex. Reading the first corner reached per vertex inverts that. A
   * user-made UV seam or split normal makes the corners disagree; the first one
   * wins and the divergence is counted. */
  const StringRef uv_name = mesh.active_uv_map_attribute ? mesh.active_uv_map_attribute : "UVMap";
  const bke::AttributeReader<float2> uv_map = attributes.lookup<float2>(uv_name,
                                                                       bke::AttrDomain::Corner);
  const Span<float3> corner_normals = mesh.corner_normals();
  const Span<int> corner_verts = mesh.corner_verts();

  Array<int> first_corner(mesh.verts_num, -1);
  for (const int corner : IndexRange(corner_verts.size())) {
    const int vertex = corner_verts[corner];
    if (vertex >= 0 && vertex < mesh.verts_num && first_corner[vertex] < 0) {
      first_corner[vertex] = corner;
    }
  }

  const Span<float3> positions = mesh.vert_positions();
  const Span<MDeformVert> dverts = mesh.deform_verts();
  const Vector<int> group_to_bone = build_group_to_bone_map(
      object, source_data, report, reports);
  const int edge_scale_group = BKE_object_defgroup_name_index(&object, kPMXEdgeScaleGroup);

  const int additional_uv_count = std::clamp(source_data.additional_uv_count, 0, 4);
  Vector<bke::AttributeReader<float2>> add_uv_low;
  Vector<bke::AttributeReader<float2>> add_uv_high;
  for (const int set_index : IndexRange(additional_uv_count)) {
    const std::string base = std::string(kPMXAdditionalUVPrefix) + std::to_string(set_index);
    add_uv_low.append(attributes.lookup<float2>(base + "_xy", bke::AttrDomain::Point));
    add_uv_high.append(attributes.lookup<float2>(base + "_zw", bke::AttrDomain::Point));
  }

  for (const int vertex_index : IndexRange(mesh.verts_num)) {
    const int source_index = source_indices.varray[vertex_index];
    if (source_index < 0 || source_index >= slots.size()) {
      report.invalid_vertex_indices++;
      continue;
    }

    PMXVertex vertex{};
    invert_position(vertex.pos, positions[vertex_index], source_data.global_scale);

    const int corner = first_corner[vertex_index];
    if (corner >= 0) {
      if (!corner_normals.is_empty()) {
        invert_normal(vertex.normal, corner_normals[corner]);
      }
      if (uv_map) {
        invert_uv(vertex.uv, uv_map.varray[corner]);
      }
    }

    vertex.additional_uv_count = uint8_t(additional_uv_count);
    for (const int set_index : IndexRange(additional_uv_count)) {
      if (add_uv_low[set_index] && add_uv_high[set_index]) {
        const float2 low = add_uv_low[set_index].varray[vertex_index];
        const float2 high = add_uv_high[set_index].varray[vertex_index];
        vertex.additional_uv[set_index] = {low[0], low[1], high[0], high[1]};
      }
    }

    vertex.weight_type = weight_types ?
                             BoneWeightType(weight_types.varray[vertex_index]) :
                             BoneWeightType::BDEF1;
    if (uint8_t(vertex.weight_type) > uint8_t(BoneWeightType::QDEF)) {
      vertex.weight_type = BoneWeightType::BDEF1;
    }

    if (vertex.weight_type == BoneWeightType::SDEF) {
      if (sdef_c) {
        invert_position(vertex.sdef_c, sdef_c.varray[vertex_index], source_data.global_scale);
      }
      if (sdef_r0) {
        invert_position(vertex.sdef_r0, sdef_r0.varray[vertex_index], source_data.global_scale);
      }
      if (sdef_r1) {
        invert_position(vertex.sdef_r1, sdef_r1.varray[vertex_index], source_data.global_scale);
      }
    }

    Vector<Influence> influences;
    if (!dverts.is_empty()) {
      const MDeformVert &dvert = dverts[vertex_index];
      for (const int i : IndexRange(dvert.totweight)) {
        const MDeformWeight &dw = dvert.dw[i];
        if (dw.def_nr == edge_scale_group) {
          continue;
        }
        if (dw.def_nr < 0 || dw.def_nr >= group_to_bone.size()) {
          continue;
        }
        const int bone_index = group_to_bone[dw.def_nr];
        if (bone_index < 0 || dw.weight <= 0.0f) {
          continue;
        }
        influences.append({bone_index, dw.weight});
      }
    }
    if (truncate_influences(influences, vertex.weight_type)) {
      report.truncated_weights++;
    }
    /* SDEF's two bones have a defined order that vertex groups do not preserve,
     * so use the indices import stored verbatim. */
    if (vertex.weight_type == BoneWeightType::SDEF && sdef_bone0 && sdef_bone1) {
      const int bone0 = sdef_bone0.varray[vertex_index];
      const int bone1 = sdef_bone1.varray[vertex_index];
      if (bone0 >= 0 && bone1 >= 0) {
        float weight0 = influences.is_empty() ? 1.0f : influences[0].weight;
        for (const Influence &influence : influences) {
          if (influence.bone_index == bone0) {
            weight0 = influence.weight;
            break;
          }
        }
        influences.clear();
        influences.append({bone0, weight0});
        influences.append({bone1, 1.0f - weight0});
      }
    }
    apply_influences(vertex, influences);

    if (edge_scale_group >= 0 && !dverts.is_empty()) {
      const MDeformWeight *dw = BKE_defvert_find_index(&dverts[vertex_index], edge_scale_group);
      vertex.edge_factor = dw ? dw->weight : 1.0f;
    }
    else {
      vertex.edge_factor = 1.0f;
    }

    VertexSlot &slot = slots[source_index];
    if (slot.filled) {
      report.duplicate_vertices++;
      const bool same_position = std::fabs(slot.vertex.pos[0] - vertex.pos[0]) < 1.0e-5f &&
                                 std::fabs(slot.vertex.pos[1] - vertex.pos[1]) < 1.0e-5f &&
                                 std::fabs(slot.vertex.pos[2] - vertex.pos[2]) < 1.0e-5f;
      if (!same_position) {
        report.divergent_duplicates++;
      }
      continue;
    }
    slot.vertex = std::move(vertex);
    slot.filled = true;
  }
}

/* --- Vertex Morph offsets from Shape Keys ---------------------------------- */

/**
 * One Morph's offsets, accumulated in *source vertex index* space.
 *
 * A dense array rather than a map, because the emitted order has to be
 * deterministic. PMX imposes no order on a Morph's offsets and a Shape Key
 * cannot preserve the file's, so export picks ascending source index and
 * `pmx_model_diff` compares the offsets order-independently. Reused across
 * Morphs; `reset()` is what makes that safe.
 */
struct MorphOffsetAccumulator {
  /** PMX-space offset per source vertex, valid only where `present` is true. */
  Array<float3> offset;
  Array<bool> present;

  explicit MorphOffsetAccumulator(const int vertex_count)
      : offset(std::max(vertex_count, 0)), present(std::max(vertex_count, 0))
  {
    reset();
  }

  void reset()
  {
    present.as_mutable_span().fill(false);
  }
};

/**
 * Recover one Morph's offsets from `object`'s Shape Key.
 *
 * Inverse of `apply_morph_to_object` (pmx_import_morph.cc), which wrote
 *   `keyblock[v] = basis[v] + transform_coord(offset, scale)`
 * so the offset comes back as `invert_position(keyblock[v] - basis[v])`.
 *
 * Taking the difference first and inverting after is valid because
 * `invert_position` is purely linear -- it permutes axes and divides by the
 * scale, with no translation term -- so it commutes with subtraction.
 *
 * The base is the Key's *reference* block, not the current mesh positions. That
 * is what Blender itself evaluates a relative Shape Key against, so the delta
 * recovered here is the one the user sees, even if the mesh was edited after
 * import.
 *
 * Returns false when this object carries no such Shape Key, which is not an
 * error on its own: only the caller knows whether some other mesh has it.
 */
bool gather_object_morph_offsets(Object &object,
                                 const std::string &shape_key_name,
                                 const float scale,
                                 MorphOffsetAccumulator &accumulator,
                                 PMXExportReport &report,
                                 ReportList *reports)
{
  Mesh *mesh = reinterpret_cast<Mesh *>(object.data);
  if (mesh == nullptr || mesh->key == nullptr || shape_key_name.empty()) {
    return false;
  }
  Key *key = mesh->key;
  KeyBlock *block = BKE_keyblock_find_name(key, shape_key_name.c_str());
  if (block == nullptr || block->data == nullptr) {
    return false;
  }
  /* Import creates "Basis" first, so it is the reference block. A Morph whose
   * own name is "Basis" would resolve to it here and yield an all-zero delta;
   * refusing is what keeps that from looking like a Morph with no offsets. */
  KeyBlock *basis = key->refkey;
  if (basis == nullptr || basis->data == nullptr || basis == block) {
    return false;
  }
  /* A Shape Key is indexed by mesh vertex, so a resize invalidates the whole
   * mapping. Reading it anyway would silently pair up unrelated vertices. */
  if (block->totelem != mesh->verts_num || basis->totelem != mesh->verts_num) {
    report.shape_key_size_mismatches++;
    add_warning(report,
                reports,
                "PMX export: Shape Key '" + shape_key_name + "' on '" +
                    std::string(object.id.name + 2) + "' covers " +
                    std::to_string(block->totelem) + " vertices but the mesh has " +
                    std::to_string(mesh->verts_num) +
                    "; its offsets are dropped. Adding or deleting vertices is not "
                    "representable on round-trip.");
    return false;
  }

  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int> source_indices = attributes.lookup<int>(
      kPMXVertexIndexAttribute, bke::AttrDomain::Point);
  if (!source_indices) {
    return false;
  }

  const float3 *block_data = static_cast<const float3 *>(block->data);
  const float3 *basis_data = static_cast<const float3 *>(basis->data);

  for (const int vertex_index : IndexRange(mesh->verts_num)) {
    const int source_index = source_indices.varray[vertex_index];
    if (source_index < 0 || source_index >= accumulator.present.size()) {
      continue;
    }
    /* Material-seam vertices are duplicated across meshes and import wrote the
     * same delta to each copy, so first-wins matches `gather_object_vertices`
     * and must not accumulate -- summing would double the offset. */
    if (accumulator.present[source_index]) {
      continue;
    }
    const float3 delta = block_data[vertex_index] - basis_data[vertex_index];
    float pmx_offset[3];
    invert_position(pmx_offset, delta, scale);
    accumulator.offset[source_index] = float3(pmx_offset[0], pmx_offset[1], pmx_offset[2]);
    accumulator.present[source_index] = true;
  }
  return true;
}

/**
 * Move the accumulated non-zero offsets into `morph`.
 *
 * An exactly-zero delta is dropped. A Shape Key holds a value for every vertex,
 * so zero means either "this vertex was not in the source Morph" or "an edit
 * cancelled it out"; PMX stores only the offsets that displace something, and
 * writing the zeros would inflate every Morph to the full vertex count.
 */
void emit_morph_offsets(const MorphOffsetAccumulator &accumulator, PMXMorph &morph)
{
  for (const int source_index : IndexRange(accumulator.present.size())) {
    if (!accumulator.present[source_index]) {
      continue;
    }
    const float3 &offset = accumulator.offset[source_index];
    if (offset[0] == 0.0f && offset[1] == 0.0f && offset[2] == 0.0f) {
      continue;
    }
    PMXVertexMorphOffset out{};
    out.vertex_index = source_index;
    out.offset[0] = offset[0];
    out.offset[1] = offset[1];
    out.offset[2] = offset[2];
    morph.vertex_offsets.push_back(out);
  }
}

/**
 * Rebuild PMX physics sections from the persisted Blender-space definition.
 *
 * The forward transforms live in `mmd_physics_definition.cc`; keeping their
 * inverses here next to the mesh inversions makes the coordinate contract
 * reviewable in one file. The definition is the authority for physics because
 * rigid bodies and joints have no equivalent editable Mesh representation.
 */
bool build_physics_model(const PMXSourceData &source_data,
                         const mmd_physics::MMDPhysicsDefinition &definition,
                         PMXModel &model,
                         PMXExportReport &report,
                         ReportList *reports)
{
  if (definition.coordinate_space != "blender_import_space" ||
      definition.rotation_order != "YXZ" ||
      definition.joint_limits_space != "blender_import_space")
  {
    add_error(report,
              reports,
              "PMX export: retained physics definition uses an unsupported coordinate or "
              "rotation convention");
    return false;
  }
  const float scale = definition.coordinate_scale;
  if (!(scale > 0.0f) || !std::isfinite(scale)) {
    add_error(report, reports, "PMX export: retained physics coordinate scale is invalid");
    return false;
  }
  if (std::fabs(scale - source_data.global_scale) > 1.0e-6f) {
    add_error(report,
              reports,
              "PMX export: retained physics and geometry coordinate scales disagree");
    return false;
  }
  if (definition.rigid_bodies.size() != size_t(std::max(source_data.rigid_body_count, 0)) ||
      definition.joints.size() != size_t(std::max(source_data.joint_count, 0)))
  {
    add_error(report,
              reports,
              "PMX export: retained physics counts do not match the retained PMX source data");
    return false;
  }

  const auto finite_vec = [](const std::array<float, 3> &value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
  };

  model.rigid_bodies.reserve(definition.rigid_bodies.size());
  for (const mmd_physics::MMDRigidBodyDefinition &source : definition.rigid_bodies) {
    if (source.pmx_bone_index < -1 || source.pmx_bone_index >= int(model.bones.size()) ||
        source.collision_group > 15 || source.shape_type > 2 || source.physics_type > 2 ||
        !finite_vec(source.shape_size) || !finite_vec(source.position) ||
        !finite_vec(source.rotation) || !std::isfinite(source.mass) ||
        !std::isfinite(source.linear_damping) || !std::isfinite(source.angular_damping) ||
        !std::isfinite(source.restitution) || !std::isfinite(source.friction))
    {
      add_error(report,
                reports,
                "PMX export: retained rigid body definition contains an invalid field");
      return false;
    }

    PMXRigidBody body{};
    body.name_local = source.name_local;
    body.name_universal = source.name_universal;
    body.bone_index = source.pmx_bone_index;
    body.collision_group = source.collision_group;
    body.no_collision_group = source.no_collision_group;
    body.shape_type = source.shape_type;
    invert_shape_size(body.shape_size, source.shape_size, body.shape_type, scale);
    invert_position(body.pos, source.position, scale);
    invert_rotation(body.rot, source.rotation);
    body.mass = source.mass;
    body.linear_damping = source.linear_damping;
    body.angular_damping = source.angular_damping;
    body.restitution = source.restitution;
    body.friction = source.friction;
    body.physics_type = source.physics_type;
    model.rigid_bodies.push_back(std::move(body));
  }

  model.joints.reserve(definition.joints.size());
  for (const mmd_physics::MMDJointDefinition &source : definition.joints) {
    if (source.type != 0 || source.rigid_a_index < 0 ||
        source.rigid_a_index >= int(model.rigid_bodies.size()) || source.rigid_b_index < 0 ||
        source.rigid_b_index >= int(model.rigid_bodies.size()) || !finite_vec(source.position) ||
        !finite_vec(source.rotation) || !finite_vec(source.translation_min) ||
        !finite_vec(source.translation_max) || !finite_vec(source.rotation_min) ||
        !finite_vec(source.rotation_max) || !finite_vec(source.spring_translation) ||
        !finite_vec(source.spring_rotation))
    {
      add_error(report, reports, "PMX export: retained joint definition contains an invalid field");
      return false;
    }

    PMXJoint joint{};
    joint.name_local = source.name_local;
    joint.name_universal = source.name_universal;
    joint.type = source.type;
    joint.rigid_a_index = source.rigid_a_index;
    joint.rigid_b_index = source.rigid_b_index;
    invert_position(joint.pos, source.position, scale);
    invert_rotation(joint.rot, source.rotation);
    /* Import stored PMX max in Blender rotation_min and PMX min in
     * Blender rotation_max, after the Y/Z swap and negation. Undo both the
     * axis transform and that deliberate min/max exchange. */
    invert_rotation(joint.rotation_limit_min, source.rotation_max);
    invert_rotation(joint.rotation_limit_max, source.rotation_min);
    invert_position(joint.translation_limit_min, source.translation_min, scale);
    invert_position(joint.translation_limit_max, source.translation_max, scale);
    invert_yz_swap(joint.spring_translation, source.spring_translation);
    invert_yz_swap(joint.spring_rotation, source.spring_rotation);
    model.joints.push_back(std::move(joint));
  }

  report.rigid_body_count = int(model.rigid_bodies.size());
  report.joint_count = int(model.joints.size());
  return true;
}

}  // namespace

/* --- Model collection lookup ---------------------------------------------- */

Collection *find_pmx_model_collection(Main *bmain, Object *active_object, bool &r_ambiguous)
{
  r_ambiguous = false;
  if (bmain == nullptr) {
    return nullptr;
  }

  Vector<Collection *> candidates;
  for (Collection &collection_ref : bmain->collections) {
    Collection *collection = &collection_ref;
    IDProperty *system = collection->id.system_properties;
    if (system == nullptr ||
        IDP_GetPropertyTypeFromGroup(system, "mmd_pmx_source_data", IDP_GROUP) == nullptr)
    {
      continue;
    }
    candidates.append(collection);
  }

  if (candidates.is_empty()) {
    return nullptr;
  }
  if (candidates.size() == 1) {
    return candidates[0];
  }
  if (active_object != nullptr) {
    Collection *match = nullptr;
    for (Collection *collection : candidates) {
      if (BKE_collection_has_object_recursive(collection, active_object)) {
        if (match != nullptr) {
          r_ambiguous = true;
          return nullptr;
        }
        match = collection;
      }
    }
    if (match != nullptr) {
      return match;
    }
  }
  r_ambiguous = true;
  return nullptr;
}

/* --- Export --------------------------------------------------------------- */

bool export_pmx_model(Collection &model_root,
                      const PMXExportOptions &options,
                      ReportList *reports,
                      PMXExportReport &r_report)
{
  r_report = PMXExportReport{};

  PMXSourceData source_data;
  if (!deserialize_pmx_source_data(model_root, source_data, reports)) {
    add_error(r_report,
              reports,
              "PMX export requires retained source data on the model collection. Only models "
              "imported from a PMX file can be exported.");
    return false;
  }
  if (!(source_data.global_scale > 0.0f) || !std::isfinite(source_data.global_scale)) {
    add_error(r_report, reports, "PMX export: retained import scale is invalid");
    return false;
  }

  mmd_physics::MMDPhysicsDefinition physics_definition;
  if (source_data.rigid_body_count > 0 || source_data.joint_count > 0) {
    if (!mmd_physics::deserialize_physics_definition(model_root, physics_definition, reports)) {
      add_error(r_report,
                reports,
                "PMX export requires a valid retained mmd_physics_definition on the model "
                "collection");
      return false;
    }
  }
  else {
    /* A valid PMX may have no physics at all. Such an import need not have
     * created an Armature or a physics definition, and there is nothing to
     * reconstruct, so use an empty definition with the geometry scale. */
    physics_definition.coordinate_scale = source_data.global_scale;
  }

  Vector<Object *> mesh_objects;
  collect_pmx_meshes(model_root, mesh_objects);
  if (mesh_objects.is_empty()) {
    add_error(r_report,
              reports,
              "PMX export found no mesh carrying the 'pmx_vertex_index' attribute");
    return false;
  }
  r_report.mesh_object_count = int(mesh_objects.size());

  PMXModel model;

  /* --- Header and model info: retained verbatim. --- */
  model.header.version = source_data.pmx_version;
  model.header.encoding = uint8_t(source_data.encoding);
  model.header.add_uv_cnt = uint8_t(std::clamp(source_data.additional_uv_count, 0, 4));
  model.name_local = source_data.name_local;
  model.name_universal = source_data.name_universal;
  model.comment_local = source_data.comment_local;
  model.comment_universal = source_data.comment_universal;

  /* --- Textures: package files while preserving the load-bearing table order. --- */
  if (!build_texture_table(source_data, options.filepath, model, r_report, reports)) {
    return false;
  }

  /* --- Vertices: rebuilt from the live meshes, deduplicated by identity map. --- */
  Array<VertexSlot> slots(source_data.vertex_count);
  for (Object *object : mesh_objects) {
    gather_object_vertices(*object, source_data, slots, r_report, reports);
  }

  model.vertices.reserve(slots.size());
  for (const int i : slots.index_range()) {
    if (!slots[i].filled) {
      r_report.missing_source_vertices++;
      PMXVertex placeholder{};
      placeholder.weight_type = BoneWeightType::BDEF1;
      placeholder.bone_indices = {0};
      placeholder.bone_weights = {1.0f};
      placeholder.edge_factor = 1.0f;
      placeholder.additional_uv_count = model.header.add_uv_cnt;
      model.vertices.push_back(std::move(placeholder));
      continue;
    }
    model.vertices.push_back(slots[i].vertex);
  }
  if (r_report.missing_source_vertices > 0) {
    add_warning(r_report,
                reports,
                "PMX export: " + std::to_string(r_report.missing_source_vertices) +
                    " source vertex/vertices had no mesh coverage and were written as zeroed "
                    "vertices. Deleting vertices is not representable on round-trip.");
  }
  if (r_report.divergent_duplicates > 0) {
    add_warning(r_report,
                reports,
                "PMX export: " + std::to_string(r_report.divergent_duplicates) +
                    " material-seam vertex/vertices disagreed between meshes; the first one "
                    "written wins.");
  }
  if (r_report.truncated_weights > 0) {
    add_warning(r_report,
                reports,
                "PMX export: " + std::to_string(r_report.truncated_weights) +
                    " vertex/vertices had more influences than the PMX weight type holds; the "
                    "largest were kept and renormalized.");
  }
  r_report.vertex_count = int(model.vertices.size());

  /* --- Faces: regrouped by material, winding inverted. --- */
  Array<int> face_counts(source_data.materials.size(), 0);
  Array<Vector<int>> faces_by_material(source_data.materials.size());
  for (Object *object : mesh_objects) {
    const Mesh &mesh = *reinterpret_cast<const Mesh *>(object->data);
    const bke::AttributeAccessor attributes = mesh.attributes();
    const bke::AttributeReader<int> source_indices = attributes.lookup<int>(
        kPMXVertexIndexAttribute, bke::AttrDomain::Point);
    if (!source_indices) {
      continue;
    }
    const bke::AttributeReader<int> material_indices = attributes.lookup<int>(
        "material_index", bke::AttrDomain::Face);
    const Vector<int> slot_to_pmx = build_material_slot_map(
        *object, mesh, source_data, r_report, reports);
    const Span<int> corner_verts = mesh.corner_verts();
    const OffsetIndices faces = mesh.faces();

    for (const int face : faces.index_range()) {
      const IndexRange corners = faces[face];
      if (corners.size() != 3) {
        continue;
      }
      const int slot = material_indices ? material_indices.varray[face] : 0;
      const int pmx_material = slot >= 0 && slot < slot_to_pmx.size() ? slot_to_pmx[slot] : -1;
      if (pmx_material < 0 || pmx_material >= int(faces_by_material.size())) {
        continue;
      }
      for (const int corner : IndexRange(3)) {
        const int mesh_vertex = corner_verts[corners.start() + inverted_corner(corner)];
        const int source_index = mesh_vertex >= 0 && mesh_vertex < mesh.verts_num ?
                                     source_indices.varray[mesh_vertex] :
                                     -1;
        faces_by_material[pmx_material].append(
            source_index >= 0 && source_index < int(model.vertices.size()) ? source_index : 0);
      }
      face_counts[pmx_material] += 3;
    }
  }

  for (const int material_index : faces_by_material.index_range()) {
    for (const int index : faces_by_material[material_index]) {
      model.face_indices.push_back(index);
    }
  }
  r_report.face_count = int(model.face_indices.size() / 3);

  /* --- Materials: retained verbatim, face counts recomputed from the mesh. --- */
  model.materials.reserve(source_data.materials.size());
  for (const PMXSourceMaterial &source : source_data.materials) {
    PMXMaterial material{};
    material.name_local = source.name_local;
    material.name_universal = source.name_universal;
    for (const int i : IndexRange(4)) {
      material.diffuse[i] = source.diffuse[i];
      material.edge_color[i] = source.edge_color[i];
    }
    for (const int i : IndexRange(3)) {
      material.specular[i] = source.specular[i];
      material.ambient[i] = source.ambient[i];
    }
    material.specular_power = source.specular_power;
    material.flag = uint8_t(source.flag);
    material.edge_size = source.edge_size;
    material.texture_idx = source.texture_index;
    material.sphere_texture_idx = source.sphere_texture_index;
    material.sphere_mode = SphereMode(source.sphere_mode);
    material.toon_flag = uint8_t(source.toon_flag);
    material.toon_texture_idx = source.toon_texture_index;
    material.toon_internal_value = uint8_t(source.toon_internal_value);
    material.memo = source.memo;

    const int recomputed = source.pmx_index < int(face_counts.size()) ?
                               face_counts[source.pmx_index] :
                               0;
    if (recomputed != source.face_vertex_count) {
      r_report.material_face_count_changes++;
    }
    material.face_vertex_count = recomputed;
    model.materials.push_back(std::move(material));
  }
  if (r_report.material_face_count_changes > 0) {
    add_warning(r_report,
                reports,
                "PMX export: " + std::to_string(r_report.material_face_count_changes) +
                    " material face count(s) differ from the imported model; the recomputed "
                    "counts are written so the file stays self-consistent.");
  }
  r_report.material_count = int(model.materials.size());

  /* --- Bones: semantics from retention, rest position from the Armature. ---
   *
   * Retention deliberately omits the bone position: the EditBone head is an
   * exact image of it, so storing it would duplicate live data. */
  Object *armature_object = find_armature(model_root);
  const bArmature *armature = armature_object ?
                                  reinterpret_cast<const bArmature *>(armature_object->data) :
                                  nullptr;
  if (armature == nullptr) {
    add_warning(r_report,
                reports,
                "PMX export found no Armature; every bone position is written as the origin.");
  }

  int missing_bones = 0;
  model.bones.reserve(source_data.bones.size());
  for (const PMXSourceBone &source : source_data.bones) {
    PMXBone bone{};
    bone.name_local = source.name_local;
    bone.name_universal = source.name_universal;
    bone.parent_index = source.parent_index;
    bone.transform_order = source.transform_order;
    bone.flag = uint16_t(source.flag);
    bone.tail_pos_bone = source.tail_pos_bone;
    bone.inherit_parent_index = source.inherit_parent_index;
    bone.inherit_parent_ratio = source.inherit_parent_ratio;
    bone.ik_target_index = source.ik_target_index;
    bone.ik_loop_count = source.ik_loop_count;
    bone.ik_angle_limit = source.ik_angle_limit;
    bone.external_parent_index = source.external_parent_index;
    for (const int i : IndexRange(3)) {
      bone.tail_pos_offset[i] = source.tail_pos_offset[i];
      bone.fixed_axis[i] = source.fixed_axis[i];
      bone.local_x[i] = source.local_x[i];
      bone.local_z[i] = source.local_z[i];
    }
    for (const PMXSourceIKLink &source_link : source.ik_links) {
      PMXIKLink link{};
      link.bone_index = source_link.bone_index;
      link.limit_angle = source_link.limit_angle;
      for (const int i : IndexRange(3)) {
        link.limit_min[i] = source_link.limit_min[i];
        link.limit_max[i] = source_link.limit_max[i];
      }
      bone.ik_links.push_back(link);
    }

    const Bone *blender_bone = armature && !source.blender_bone_name.empty() ?
                                   BKE_armature_find_bone_name(
                                       const_cast<bArmature *>(armature),
                                       source.blender_bone_name.c_str()) :
                                   nullptr;
    if (blender_bone != nullptr) {
      invert_position(bone.pos, float3(blender_bone->arm_head), source_data.global_scale);
    }
    else if (armature != nullptr) {
      missing_bones++;
    }
    model.bones.push_back(std::move(bone));
  }
  if (missing_bones > 0) {
    add_warning(r_report,
                reports,
                "PMX export: " + std::to_string(missing_bones) +
                    " bone(s) were not found in the Armature; their positions are written as the "
                    "origin.");
  }
  r_report.bone_count = int(model.bones.size());

  /* --- Morphs: retained metadata, Vertex offsets rebuilt from Shape Keys. ---
   *
   * Every Morph is emitted, including ones whose offsets are left empty. Group
   * and Flip offsets address other Morphs *by index*, so dropping a Morph would
   * silently repoint every later reference. An empty offset list is valid PMX. */
  MorphOffsetAccumulator morph_offsets(source_data.vertex_count);
  model.morphs.reserve(source_data.morphs.size());
  for (const PMXSourceMorph &source : source_data.morphs) {
    PMXMorph morph{};
    morph.name_local = source.name_local;
    morph.name_universal = source.name_universal;
    morph.panel = uint8_t(source.panel);
    morph.type = MorphType(source.type);

    for (const PMXSourceGroupMorphOffset &offset : source.group_offsets) {
      PMXGroupMorphOffset out{};
      out.morph_index = offset.morph_index;
      out.influence = offset.influence;
      morph.group_offsets.push_back(out);
    }
    for (const PMXSourceBoneMorphOffset &offset : source.bone_offsets) {
      PMXBoneMorphOffset out{};
      out.bone_index = offset.bone_index;
      for (const int i : IndexRange(3)) {
        out.pos[i] = offset.pos[i];
      }
      for (const int i : IndexRange(4)) {
        out.rot[i] = offset.rot[i];
      }
      morph.bone_offsets.push_back(out);
    }
    for (const PMXSourceUVMorphOffset &offset : source.uv_offsets) {
      PMXUVMorphOffset out{};
      out.vertex_index = offset.vertex_index;
      for (const int i : IndexRange(4)) {
        out.offset[i] = offset.offset[i];
      }
      morph.uv_offsets.push_back(out);
    }
    for (const PMXSourceMaterialMorphOffset &offset : source.material_offsets) {
      PMXMaterialMorphOffset out{};
      out.material_index = offset.material_index;
      out.calc_mode = uint8_t(offset.calc_mode);
      for (const int i : IndexRange(4)) {
        out.diffuse[i] = offset.diffuse[i];
        out.edge_color[i] = offset.edge_color[i];
        out.texture_factor[i] = offset.texture_factor[i];
        out.sphere_texture_factor[i] = offset.sphere_texture_factor[i];
        out.toon_texture_factor[i] = offset.toon_texture_factor[i];
      }
      for (const int i : IndexRange(3)) {
        out.specular[i] = offset.specular[i];
        out.ambient[i] = offset.ambient[i];
      }
      out.specular_power = offset.specular_power;
      out.edge_size = offset.edge_size;
      morph.material_offsets.push_back(out);
    }

    for (const PMXSourceImpulseMorphOffset &offset : source.impulse_offsets) {
      PMXImpulseMorphOffset out{};
      out.rigid_index = offset.rigid_index;
      out.local_flag = uint8_t(offset.local_flag);
      for (const int i : IndexRange(3)) {
        out.velocity[i] = offset.velocity[i];
        out.torque[i] = offset.torque[i];
      }
      morph.impulse_offsets.push_back(out);
    }

    /* Vertex Morph offsets are recovered from the Shape Keys, which is why every
     * mesh has to be visited per Morph: import wrote one Shape Key of that name
     * on *each* mesh, and in split-by-material mode each holds the deltas for
     * only its own slice of the model. */
    if (morph.type == MorphType::Vertex) {
      morph_offsets.reset();
      bool found_any = false;
      for (Object *object : mesh_objects) {
        found_any |= gather_object_morph_offsets(*object,
                                                source.blender_shape_key_name,
                                                source_data.global_scale,
                                                morph_offsets,
                                                r_report,
                                                reports);
      }
      if (found_any) {
        emit_morph_offsets(morph_offsets, morph);
        r_report.vertex_morph_offset_count += int(morph.vertex_offsets.size());
      }
      else if (source.vertex_offset_count > 0) {
        r_report.unresolved_shape_keys++;
        add_warning(r_report,
                    reports,
                    "PMX export: Morph '" + source.name_local + "' expected Shape Key '" +
                        source.blender_shape_key_name +
                        "' but no mesh carries it; its offsets are dropped.");
      }
    }
    if (morph.type == MorphType::Impulse) {
      r_report.impulse_morph_offset_count += int(morph.impulse_offsets.size());
    }
    model.morphs.push_back(std::move(morph));
  }
  r_report.morph_count = int(model.morphs.size());

  /* --- Display frames: retained verbatim. --- */
  model.display_frames.reserve(source_data.display_frames.size());
  for (const PMXSourceDisplayFrame &source : source_data.display_frames) {
    PMXDisplayFrame frame{};
    frame.name_local = source.name_local;
    frame.name_universal = source.name_universal;
    frame.flag = uint8_t(source.flag);
    for (const PMXSourceDisplayFrame::Item &item : source.items) {
      PMXDisplayFrame::FrameItem out{};
      out.type = uint8_t(item.type);
      out.index = item.index;
      frame.items.push_back(out);
    }
    model.display_frames.push_back(std::move(frame));
  }
  r_report.display_frame_count = int(model.display_frames.size());

  /* --- Physics: inverse of build_physics_definition() ---------------------- */
  if (!build_physics_model(
          source_data, physics_definition, model, r_report, reports))
  {
    return false;
  }

  try {
    PMXWriter::write(model, options.filepath);
  }
  catch (const std::exception &e) {
    add_error(r_report, reports, std::string("PMX export failed to write the file: ") + e.what());
    return false;
  }

  r_report.success = true;
  return true;
}

}  // namespace blender::io::pmx
