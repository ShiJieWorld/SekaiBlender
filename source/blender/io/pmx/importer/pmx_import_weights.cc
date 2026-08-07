/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_attribute.hh"
#include "BKE_deform.hh"
#include "BKE_object.hh"

#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "intern/pmx_types.h"
#include "pmx_import_material.hh"
#include "pmx_import_mesh.hh"
#include "pmx_import_weights.hh"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace blender::io::pmx {

void write_pmx_vertex_skinning_attributes(Mesh &mesh,
                                          const PMXModel &model,
                                          const Vector<int> *new_to_old,
                                          const float global_scale)
{
  if (new_to_old != nullptr) {
    if (new_to_old->size() != mesh.verts_num) {
      return;
    }
    for (const int source_vi : *new_to_old) {
      if (source_vi < 0 || source_vi >= int(model.vertices.size())) {
        return;
      }
    }
  }
  else if (model.vertices.size() != mesh.verts_num) {
    return;
  }

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  bke::SpanAttributeWriter<int8_t> weight_types =
      attributes.lookup_or_add_for_write_only_span<int8_t>("pmx_weight_type",
                                                            bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> sdef_c =
      attributes.lookup_or_add_for_write_only_span<float3>("pmx_sdef_c", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> sdef_r0 =
      attributes.lookup_or_add_for_write_only_span<float3>("pmx_sdef_r0", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> sdef_r1 =
      attributes.lookup_or_add_for_write_only_span<float3>("pmx_sdef_r1", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<int> sdef_bone0 =
      attributes.lookup_or_add_for_write_only_span<int>("pmx_sdef_bone0", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<int> sdef_bone1 =
      attributes.lookup_or_add_for_write_only_span<int>("pmx_sdef_bone1", bke::AttrDomain::Point);

  if (!weight_types || !sdef_c || !sdef_r0 || !sdef_r1 || !sdef_bone0 || !sdef_bone1) {
    return;
  }

  BLI_assert(weight_types.span.size() == mesh.verts_num);
  BLI_assert(sdef_c.span.size() == mesh.verts_num);
  BLI_assert(sdef_r0.span.size() == mesh.verts_num);
  BLI_assert(sdef_r1.span.size() == mesh.verts_num);
  BLI_assert(sdef_bone0.span.size() == mesh.verts_num);
  BLI_assert(sdef_bone1.span.size() == mesh.verts_num);
  BLI_assert(new_to_old == nullptr || new_to_old->size() == mesh.verts_num);

  for (const int vi : weight_types.span.index_range()) {
    const int source_vi = new_to_old ? (*new_to_old)[vi] : vi;
    BLI_assert(source_vi >= 0 && source_vi < int(model.vertices.size()));
    const PMXVertex &vertex = model.vertices[source_vi];
    weight_types.span[vi] = int8_t(static_cast<uint8_t>(vertex.weight_type));

    if (vertex.weight_type == BoneWeightType::SDEF) {
      float converted[3];
      transform_coord(converted, vertex.sdef_c, global_scale);
      sdef_c.span[vi] = float3(converted[0], converted[1], converted[2]);
      transform_coord(converted, vertex.sdef_r0, global_scale);
      sdef_r0.span[vi] = float3(converted[0], converted[1], converted[2]);
      transform_coord(converted, vertex.sdef_r1, global_scale);
      sdef_r1.span[vi] = float3(converted[0], converted[1], converted[2]);
      sdef_bone0.span[vi] = vertex.bone_indices.size() > 0 ? vertex.bone_indices[0] : -1;
      sdef_bone1.span[vi] = vertex.bone_indices.size() > 1 ? vertex.bone_indices[1] : -1;
    }
    else {
      sdef_c.span[vi] = float3(0.0f);
      sdef_r0.span[vi] = float3(0.0f);
      sdef_r1.span[vi] = float3(0.0f);
      sdef_bone0.span[vi] = -1;
      sdef_bone1.span[vi] = -1;
    }
  }

  weight_types.finish();
  sdef_c.finish();
  sdef_r0.finish();
  sdef_r1.finish();
  sdef_bone0.finish();
  sdef_bone1.finish();
}

void assign_vertex_weights_named(Object *obj,
                                  const PMXModel &model,
                                  const Vector<std::string> &bone_names,
                                  const Vector<int> *new_to_old,
                                  const float global_scale)
{
  Mesh *mesh = reinterpret_cast<Mesh *>(obj->data);
  if (!mesh) return;

  write_pmx_vertex_skinning_attributes(*mesh, model, new_to_old, global_scale);

  const int bone_num = int(bone_names.size());
  if (bone_num == 0) return;

  /* Step 1: Create vertex groups from Blender bone names. */
  for (const std::string &name : bone_names) {
    BKE_object_defgroup_new(obj, name.c_str());
  }

  /* Step 2: Get deform verts array. */
  MutableSpan<MDeformVert> dvert_span = mesh->deform_verts_for_write();
  MDeformVert *dverts = dvert_span.data();
  if (!dverts) return;

  /* Step 3: Assign weights. */
  if (new_to_old) {
    /* Split mode: sub-mesh vertices map back to original model vertices. */
    const int sub_vert_num = int(new_to_old->size());
    for (int svi = 0; svi < sub_vert_num; svi++) {
      int old_vi = (*new_to_old)[svi];
      const PMXVertex &v = model.vertices[old_vi];
      const int num_weights = int(v.bone_weights.size());

      for (int wi = 0; wi < num_weights; wi++) {
        int bone_idx = v.bone_indices[wi];
        float weight = v.bone_weights[wi];

        if (bone_idx < 0 || bone_idx >= bone_num) continue;
        if (weight <= 0.0f) continue;

        /* [世界的歌] BDEF4 vertices may reference the same bone index multiple
         * times (e.g. 菲比膝盖 vertices: [左足D,左足D,左足D,左ひざD]).
         * PMX semantics: weights for the same bone must be SUMMED, not
         * overwritten. Using += instead of = correctly accumulates duplicate
         * bone weights. BKE_defvert_ensure_index initializes new entries to
         * 0.0f, so the first occurrence sets the value and subsequent
         * duplicates add to it. This prevents catastrophic weight distortion
         * (e.g. 左足D 0.9899 collapsing to 0.0134) that causes knee mesh
         * tearing during leg bending. */
        MDeformWeight *dw = BKE_defvert_ensure_index(&dverts[svi], bone_idx);
        if (dw) {
          dw->weight += weight;
        }
      }
    }
  }
  else {
    /* Single mode: direct vertex mapping. */
    const int vert_num = int(model.vertices.size());
    for (int vi = 0; vi < vert_num; vi++) {
      const PMXVertex &v = model.vertices[vi];
      const int num_weights = int(v.bone_weights.size());

      for (int wi = 0; wi < num_weights; wi++) {
        int bone_idx = v.bone_indices[wi];
        float weight = v.bone_weights[wi];

        if (bone_idx < 0 || bone_idx >= bone_num) continue;
        if (weight <= 0.0f) continue;

        /* [世界的歌] Accumulate weights for duplicate bone indices (BDEF4).
         * See split-mode comment above for full rationale. */
        MDeformWeight *dw = BKE_defvert_ensure_index(&dverts[vi], bone_idx);
        if (dw) {
          dw->weight += weight;
        }
      }
    }
  }

  /* Step 4: Normalize weights so each vertex's total sums to 1.0.
   * PMX BDEF2/BDEF4 vertices may reference bone indices that are -1 (unused),
   * leaving the assigned weights incomplete. Normalization compensates by
   * scaling up the remaining weights to full influence, preventing partial
   * deformation (stretching/tearing) at joint seams. */
  const int dvert_count = int(dvert_span.size());
  for (int vi = 0; vi < dvert_count; vi++) {
    MDeformVert &dv = dverts[vi];
    if (dv.totweight <= 0) {
      continue;
    }
    float weight_sum = 0.0f;
    for (int wi = 0; wi < dv.totweight; wi++) {
      weight_sum += dv.dw[wi].weight;
    }
    /* Only normalize if weight_sum is significantly below 1.0. */
    if (weight_sum > 1e-6f && weight_sum < 0.999f) {
      const float inv_sum = 1.0f / weight_sum;
      for (int wi = 0; wi < dv.totweight; wi++) {
        dv.dw[wi].weight *= inv_sum;
      }
    }
  }
}

void write_pmx_edge_scale_group(Object *obj, const PMXModel &model, const Vector<int> *new_to_old)
{
  if (obj == nullptr || obj->type != OB_MESH) {
    return;
  }
  Mesh *mesh = reinterpret_cast<Mesh *>(obj->data);
  if (mesh == nullptr || mesh->verts_num == 0) {
    return;
  }
  if (new_to_old != nullptr) {
    if (new_to_old->size() != mesh->verts_num) {
      return;
    }
  }
  else if (int(model.vertices.size()) != mesh->verts_num) {
    return;
  }

  /* Match mmd_tools (core/pmx/importer.py: vg_edge_scale): persist the PMX
   * per-vertex edge scale as its own vertex group so toon-edge width can be
   * rebuilt later without re-reading the PMX file. */
  if (BKE_object_defgroup_find_name(obj, kPMXEdgeScaleGroup) == nullptr) {
    BKE_object_defgroup_new(obj, kPMXEdgeScaleGroup);
  }
  const int group_index = BKE_object_defgroup_name_index(obj, kPMXEdgeScaleGroup);
  if (group_index < 0) {
    return;
  }

  MutableSpan<MDeformVert> dvert_span = mesh->deform_verts_for_write();
  MDeformVert *dverts = dvert_span.data();
  if (dverts == nullptr) {
    return;
  }

  for (const int vi : IndexRange(mesh->verts_num)) {
    const int source_vi = new_to_old ? (*new_to_old)[vi] : vi;
    if (source_vi < 0 || source_vi >= int(model.vertices.size())) {
      continue;
    }
    const float edge_scale = model.vertices[source_vi].edge_factor;
    /* Blender vertex-group weights are limited to [0, 1], matching the clamp
     * applied by mmd_tools through `vertex_groups.add()`. */
    const float weight = std::isfinite(edge_scale) ? std::clamp(edge_scale, 0.0f, 1.0f) : 1.0f;
    if (MDeformWeight *dw = BKE_defvert_ensure_index(&dverts[vi], group_index)) {
      dw->weight = weight;
    }
  }
}

void write_all_pmx_edge_scale_groups(PMXImportContext &ctx, const PMXModel &model)
{
  if (ctx.params->split_by_material) {
    for (const SubMeshInfo &sub : ctx.sub_meshes) {
      write_pmx_edge_scale_group(sub.obj, model, &sub.new_to_old);
    }
  }
  else {
    for (Object *obj : ctx.mesh_objects) {
      write_pmx_edge_scale_group(obj, model, nullptr);
    }
  }
}

void assign_all_vertex_weights(PMXImportContext &ctx,
                                const PMXModel &model,
                                const Vector<std::string> &bone_names)
{
  if (ctx.params->split_by_material) {
    /* Split mode: each sub-mesh needs vertex remapping. */
    for (const SubMeshInfo &sub : ctx.sub_meshes) {
      assign_vertex_weights_named(
          sub.obj, model, bone_names, &sub.new_to_old, ctx.params->global_scale);
    }
  }
  else {
    /* Single mode: direct assignment. */
    for (Object *obj : ctx.mesh_objects) {
      assign_vertex_weights_named(obj, model, bone_names, nullptr, ctx.params->global_scale);
    }
  }
}

}  // namespace blender::io::pmx
