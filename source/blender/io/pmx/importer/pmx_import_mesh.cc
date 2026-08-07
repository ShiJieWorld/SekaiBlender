/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_attribute.hh"
#include "BKE_collection.hh"
#include "BKE_context.hh"
#include "BKE_customdata.hh"
#include "BKE_deform.hh"
#include "BKE_idprop.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "intern/pmx_reader.h"
#include "pmx_import_material.hh"
#include "pmx_import_mesh.hh"

namespace blender::io::pmx {

/* PMX uses a fixed coordinate convention. This conversion is shared by mesh,
 * armature, and morph import so their data remains in the same space. */
void transform_coord(float out[3], const float in[3], float scale)
{
  out[0] = in[0] * scale;
  out[1] = in[2] * scale;
  out[2] = in[1] * scale;
}

static void assign_pmx_material(PMXImportContext &ctx,
                                const PMXModel &model,
                                const int material_index,
                                Object *obj)
{
  Material *material = create_pmx_material(ctx, model, material_index);
  if (material != nullptr) {
    BKE_object_material_assign_single_obdata(ctx.bmain, obj, material, obj->totcol + 1);
  }
}

static void assign_pmx_materials(PMXImportContext &ctx, const PMXModel &model, Object *obj)
{
  for (const int material_index : IndexRange(model.materials.size())) {
    assign_pmx_material(ctx, model, material_index, obj);
  }
}

using PMXTriangleKey = std::array<int64_t, 9>;

static PMXTriangleKey make_triangle_key(const PMXModel &model,
                                        const int face_index,
                                        const float global_scale)
{
  std::array<std::array<int64_t, 3>, 3> vertices;
  for (int corner = 0; corner < 3; corner++) {
    const int vertex_index = model.face_indices[face_index * 3 + corner];
    float converted[3];
    transform_coord(converted, model.vertices[vertex_index].pos, global_scale);
    for (int axis = 0; axis < 3; axis++) {
      vertices[corner][axis] = int64_t(std::llround(double(converted[axis]) * 1'000'000.0));
    }
  }

  std::sort(vertices.begin(), vertices.end());
  PMXTriangleKey key{};
  int key_index = 0;
  for (const auto &vertex : vertices) {
    for (const int64_t coordinate : vertex) {
      key[key_index++] = coordinate;
    }
  }
  return key;
}

static Vector<bool> find_overlapping_materials(const PMXModel &model, const float global_scale)
{
  Vector<bool> result(model.materials.size(), false);
  std::map<PMXTriangleKey, int> first_material_by_triangle;
  int face_index = 0;

  for (int material_index = 0; material_index < int(model.materials.size()); material_index++) {
    const int face_vertex_count = model.materials[material_index].face_vertex_count;
    if (face_vertex_count < 0 || face_vertex_count % 3 != 0) {
      continue;
    }
    const int face_count = face_vertex_count / 3;
    for (int face = 0; face < face_count; face++, face_index++) {
      const PMXTriangleKey key = make_triangle_key(model, face_index, global_scale);
      const auto found = first_material_by_triangle.find(key);
      if (found == first_material_by_triangle.end()) {
        first_material_by_triangle.emplace(key, material_index);
      }
      else if (found->second < material_index) {
        result[material_index] = true;
      }
    }
  }
  return result;
}

static void apply_overlapping_material_blend(PMXImportContext &ctx,
                                             const PMXModel &model,
                                             const float global_scale)
{
  const Vector<bool> overlapping = find_overlapping_materials(model, global_scale);
  for (int material_index = 0; material_index < int(overlapping.size()); material_index++) {
    if (!overlapping[material_index]) {
      continue;
    }
    if (Material **material = ctx.material_cache.lookup_ptr(material_index)) {
      (*material)->blend_method = MA_BM_BLEND;
      (*material)->surface_render_method = MA_SURFACE_METHOD_FORWARD;
      (*material)->blend_flag |= MA_BL_HIDE_BACKFACE;
    }
  }
}

static Mesh *create_mesh_data(const PMXModel &model, const PMXImportParams &params)
{
  const int vert_num = int(model.vertices.size());
  const int face_num = int(model.face_indices.size() / 3);
  const int corner_num = int(model.face_indices.size());

  BLI_assert(model.face_indices.size() % 3 == 0);

  Mesh *mesh = BKE_mesh_new_nomain(vert_num, 0, face_num, corner_num);

  /* Write vertex positions (transformed). */
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  for (int i = 0; i < vert_num; i++) {
    transform_coord(positions[i], model.vertices[i].pos, params.global_scale);
  }

  /* Swapping PMX Y/Z is a reflection, so reverse every triangle to preserve
   * its outward-facing winding in Blender space. */
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
  for (const int face : IndexRange(face_num)) {
    for (const int corner : IndexRange(3)) {
      const int corner_index = face * 3 + corner;
      corner_verts[corner_index] = model.face_indices[face * 3 + (2 - corner)];
    }
  }

  /* Set face offsets (all triangles = groups of 3). */
  offset_indices::fill_constant_group_size(3, 0, mesh->face_offsets_for_write());

  /* Calculate edges. */
  bke::mesh_calc_edges(*mesh, false, false);

  /* Write UV coordinates. */
  if (!model.vertices.empty()) {
    bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
    bke::SpanAttributeWriter<float2> uv_map = attributes.lookup_or_add_for_write_only_span<float2>(
        "UVMap", bke::AttrDomain::Corner);

    for (int face_i = 0; face_i < face_num; face_i++) {
      for (int j = 0; j < 3; j++) {
        int corner_idx = face_i * 3 + j;
        int vert_idx = corner_verts[corner_idx];
        uv_map.span[corner_idx][0] = model.vertices[vert_idx].uv[0];
        uv_map.span[corner_idx][1] = 1.0f - model.vertices[vert_idx].uv[1];
      }
    }
    uv_map.finish();
    mesh->uv_maps_active_set("UVMap");
    mesh->uv_maps_default_set("UVMap");
  }

  /* Assign material indices per face. */
  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  bke::SpanAttributeWriter<int> material_indices =
      attributes.lookup_or_add_for_write_only_span<int>("material_index",
                                                        bke::AttrDomain::Face);

  int face_offset = 0;
  for (int mat_i = 0; mat_i < int(model.materials.size()); mat_i++) {
    const int face_vertex_count = model.materials[mat_i].face_vertex_count;
    if (face_vertex_count < 0 || face_vertex_count % 3 != 0) {
      throw std::runtime_error("PMX material has invalid face vertex count");
    }
    const int face_count = face_vertex_count / 3;
    if (face_count > face_num - face_offset) {
      throw std::runtime_error("PMX material face ranges exceed mesh face count");
    }
    for (int j = 0; j < face_count; j++) {
      material_indices.span[face_offset++] = mat_i;
    }
  }
  if (face_offset != face_num) {
    throw std::runtime_error("PMX material face ranges do not cover mesh face count");
  }
  material_indices.finish();

  /* Smooth shading (MMD models expect smooth shading by default). */
  bke::mesh_smooth_set(*mesh, true);

  /* Preserve PMX per-vertex normals as custom corner normals. mmd_tools does
   * the same before creating its Solidify toon-edge shell; the shell's
   * offset direction depends on these normals at sharp material boundaries. */
  Array<float3> corner_normals(corner_num);
  for (const int corner : IndexRange(corner_num)) {
    const int vertex_index = corner_verts[corner];
    const float *normal = model.vertices[vertex_index].normal;
    corner_normals[corner] = float3(normal[0], normal[2], normal[1]);
  }
  bke::mesh_set_custom_normals(*mesh, corner_normals);

  mesh->tag_positions_changed();
  return mesh;
}

Object *create_model_root(PMXImportContext &ctx, const PMXModel &model)
{
  const char *base_name = model.name_local.empty() ? "PMXModel" : model.name_local.c_str();
  LayerCollection *active_lc = BKE_layer_collection_get_active_editable(ctx.view_layer);
  Collection *parent_collection = active_lc ? active_lc->collection : nullptr;

  ctx.model_collection = BKE_collection_add(ctx.bmain, parent_collection, base_name);
  ctx.geometry_collection = BKE_collection_add(ctx.bmain, ctx.model_collection, "PMX Geometry");
  ctx.controls_collection = BKE_collection_add(ctx.bmain, ctx.model_collection, "PMX Controls");

  /* [世界的歌] Mark implementation collections as collapsed by default in the Outliner. The
   * view-layer tree stores this UI state by ID, so keeping the marker on the
   * collection makes it effective when the tree is built after import and when
   * the .blend file is reopened. This does not hide or disable either collection. */
  IDProperty *geometry_props = IDP_ID_system_properties_ensure(&ctx.geometry_collection->id);
  IDP_AddToGroup(geometry_props, IDP_NewInt(1, "pmx_outliner_collapsed"));
  IDProperty *controls_props = IDP_ID_system_properties_ensure(&ctx.controls_collection->id);
  IDP_AddToGroup(controls_props, IDP_NewInt(1, "pmx_outliner_collapsed"));

  Object *root = BKE_object_add_only_object(ctx.bmain, OB_EMPTY, base_name);
  BKE_collection_object_add(ctx.bmain, ctx.model_collection, root);

  /* Match mmd_tools (core/model.py: empty_display_size = scale / 0.2): the model
   * root Empty display size encodes the import scale. MMD render features read it
   * back instead of storing a separate scale, so toon-edge thickness resolves to
   * exactly the import scale (0.2 * display size == global_scale). */
  if (ctx.params != nullptr && std::isfinite(ctx.params->global_scale) &&
      ctx.params->global_scale > 0.0f)
  {
    root->empty_drawsize = ctx.params->global_scale / 0.2f;
  }

  ctx.root_obj = root;
  return root;
}

static void parent_keep_world_transform(Object *child, Object *parent)
{
  if (!child || !parent) {
    return;
  }
  /* The import root is created with identity transform. Keep child transforms in
   * local space and make the parent relationship explicit. */
  child->parent = parent;
  child->partype = PAROBJECT;
  unit_m4(child->parentinv);
}

void create_mesh_object(PMXImportContext &ctx, PMXModel &model)
{
  Main *bmain = ctx.bmain;
  const PMXImportParams &params = *ctx.params;
  const char *base_name = model.name_local.empty() ? "PMXModel" : model.name_local.c_str();

  if (!params.split_by_material) {
    /* Single mesh (all materials combined). */
    Mesh *mesh = create_mesh_data(model, params);
    Mesh *mesh_in_main = BKE_mesh_add(bmain, base_name);
    BKE_mesh_nomain_to_mesh(mesh, mesh_in_main, nullptr);

    Object *obj = BKE_object_add_only_object(bmain, OB_MESH, base_name);
    obj->data = &mesh_in_main->id;

    BKE_collection_object_add(bmain, ctx.geometry_collection, obj);

    Base *base = BKE_view_layer_base_find(ctx.view_layer, obj);
    if (base) {
      BKE_view_layer_base_select_and_set_active(ctx.view_layer, base);
    }

    ctx.mesh_obj = obj;
    parent_keep_world_transform(obj, ctx.root_obj);
    ctx.mesh_objects.append(obj);
    assign_pmx_materials(ctx, model, obj);
  }
  else {
    /* Split by material: create one mesh object per material group. */
    int vert_num = int(model.vertices.size());
    int face_num = int(model.face_indices.size() / 3);
    BLI_assert(model.face_indices.size() % 3 == 0);

    /* Compute material index for each face. */
    Vector<int> face_mat(face_num);
    int face_offset = 0;
    for (int mat_i = 0; mat_i < int(model.materials.size()); mat_i++) {
      const int face_vertex_count = model.materials[mat_i].face_vertex_count;
      if (face_vertex_count < 0 || face_vertex_count % 3 != 0) {
        throw std::runtime_error("PMX material has invalid face vertex count");
      }
      const int face_count = face_vertex_count / 3;
      if (face_count > face_num - face_offset) {
        throw std::runtime_error("PMX material face ranges exceed mesh face count");
      }
      for (int j = 0; j < face_count; j++) {
        face_mat[face_offset++] = mat_i;
      }
    }
    if (face_offset != face_num) {
      throw std::runtime_error("PMX material face ranges do not cover mesh face count");
    }

    /* Collect all created mesh objects. */
    Vector<Object *> mesh_objects;

    for (int mat_i = 0; mat_i < int(model.materials.size()); mat_i++) {
      /* Collect faces for this material. */
      Vector<int> sub_faces;
      Vector<bool> vert_used(vert_num, false);

      for (int fi = 0; fi < face_num; fi++) {
        if (face_mat[fi] != mat_i) continue;
        sub_faces.append(fi);
        for (int j = 0; j < 3; j++) {
          vert_used[model.face_indices[fi * 3 + j]] = true;
        }
      }

      if (sub_faces.is_empty()) continue;

      /* Build vertex remapping. */
      Vector<int> old_to_new(vert_num, -1);
      Vector<int> new_to_old;
      for (int vi = 0; vi < vert_num; vi++) {
        if (vert_used[vi]) {
          old_to_new[vi] = new_to_old.size();
          new_to_old.append(vi);
        }
      }

      /* Build sub-model data. */
      PMXModel sub_model;
      sub_model.header = model.header;
      sub_model.name_local = model.name_local;
      sub_model.name_universal = model.name_universal;
      sub_model.materials.push_back(model.materials[mat_i]);

      for (int vi : new_to_old) {
        sub_model.vertices.push_back(model.vertices[vi]);
      }

      for (int fi : sub_faces) {
        for (int j = 0; j < 3; j++) {
          int old_idx = model.face_indices[fi * 3 + j];
          sub_model.face_indices.push_back(old_to_new[old_idx]);
        }
      }

      /* Build material name for object naming. */
      std::string mat_name = model.materials[mat_i].name_local;
      if (mat_name.empty()) mat_name = "Material" + std::to_string(mat_i);
      std::string obj_name = std::string(base_name) + "_" + mat_name;

      Mesh *mesh = create_mesh_data(sub_model, params);
      Mesh *mesh_in_main = BKE_mesh_add(bmain, obj_name.c_str());
      BKE_mesh_nomain_to_mesh(mesh, mesh_in_main, nullptr);

      Object *obj = BKE_object_add_only_object(bmain, OB_MESH, obj_name.c_str());
      obj->data = &mesh_in_main->id;

      BKE_collection_object_add(bmain, ctx.geometry_collection, obj);

      /* Assign the PMX material through the common combined/split material path. */
      assign_pmx_material(ctx, model, mat_i, obj);

      /* Save sub-mesh info for later weight assignment. */
      SubMeshInfo sub_info;
      sub_info.obj = obj;
      sub_info.new_to_old = std::move(new_to_old);
      ctx.sub_meshes.append(std::move(sub_info));

      parent_keep_world_transform(obj, ctx.root_obj);
      mesh_objects.append(obj);
      ctx.mesh_objects.append(obj);
    }

    ctx.mesh_obj = ctx.root_obj;
  }

  if (!ctx.mesh_obj) {
    /* Fallback: find first mesh object. */
    Object *ob = static_cast<Object *>(bmain->objects.first);
    while (ob) {
      if (ob->type == OB_MESH) {
        ctx.mesh_obj = ob;
        break;
      }
      ob = static_cast<Object *>(ob->id.next);
    }
  }

  /* Match mmd_tools: only the later material in an overlapping face pair uses forward blending. */
  apply_overlapping_material_blend(ctx, model, params.global_scale);

  /* Notify dependency graph. */
  for (Object *obj : ctx.mesh_objects) {
    DEG_id_tag_update_ex(bmain, &obj->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  }
  DEG_relations_tag_update(bmain);
}

}  // namespace blender::io::pmx
