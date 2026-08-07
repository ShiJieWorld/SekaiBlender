/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "pmx_source_data.hh"

#include "DNA_collection_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_attribute.hh"
#include "BKE_collection.hh"
#include "BKE_idprop.hh"
#include "BKE_mesh.hh"
#include "BKE_report.hh"

#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"

#include "MEM_guardedalloc.h"

#include "pmx_import_mesh.hh"

#include <algorithm>
#include <cmath>
#include <string>

namespace blender::io::pmx {
namespace {

constexpr char kSourceDataProperty[] = "mmd_pmx_source_data";

/* Top-level sections (textures / materials / bones / morphs / display frames).
 * Matches the mmd_physics_definition limit. */
constexpr int kMaxPersistedItems = 100000;
/* Nested per-item offsets (Morph offsets, IK links, display frame items). PMX
 * allows far more, but a model that exceeds this is not something this retention
 * layer should silently truncate: serialization fails and reports instead. */
constexpr int kMaxPersistedNestedItems = 1000000;

void report_warning(ReportList *reports, const std::string &message)
{
  if (reports) {
    BKE_report(reports, RPT_WARNING, message.c_str());
  }
}

/* --- IDProperty write helpers ---------------------------------------------- */

void add_property(IDProperty *group, IDProperty *property)
{
  if (!property || !IDP_AddToGroup(group, property)) {
    if (property) {
      IDP_FreeProperty(property);
    }
  }
}

void add_int(IDProperty *group, const char *name, const int value)
{
  add_property(group, IDP_NewInt(value, name));
}

void add_float(IDProperty *group, const char *name, const float value)
{
  add_property(group, bke::idprop::create(name, value).release());
}

void add_bool(IDProperty *group, const char *name, const bool value)
{
  add_property(group, bke::idprop::create_bool(name, value).release());
}

void add_string(IDProperty *group, const char *name, const std::string &value)
{
  add_property(group, IDP_NewString(value.c_str(), name));
}

template<size_t N>
void add_floats(IDProperty *group, const char *name, const std::array<float, N> &values)
{
  add_property(group, bke::idprop::create(name, Span<float>(values.data(), N)).release());
}

void add_ints(IDProperty *group, const char *name, const Span<int32_t> values)
{
  add_property(group, bke::idprop::create(name, values).release());
}

/* NOTE: in this fork `IDP_AppendArray` copies `item`'s content into the array
 * slot via a shallow memcpy (`IDP_SetIndexArray`) and does NOT take ownership of
 * `item` — the slot shares `item`'s child nodes. Release only the now-redundant
 * source struct here; its children are owned by the slot and freed with the
 * array tree. Freeing the struct alone (not the children) avoids a dangling
 * pointer / double-free. */
void append_group(IDProperty *array, IDProperty *item)
{
  IDP_AppendArray(array, item);
  MEM_delete(item);
}

/* --- IDProperty read helpers ----------------------------------------------- */

IDProperty *get_array(IDProperty *group, const char *name, const int length, const char subtype)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_ARRAY);
  return property && property->len == length && property->subtype == subtype ? property : nullptr;
}

bool read_int(IDProperty *group, const char *name, int &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_INT);
  if (!property) {
    return false;
  }
  value = IDP_int_get(property);
  return true;
}

bool read_float(IDProperty *group, const char *name, float &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_FLOAT);
  if (!property || !std::isfinite(IDP_float_get(property))) {
    return false;
  }
  value = IDP_float_get(property);
  return true;
}

bool read_bool(IDProperty *group, const char *name, bool &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_BOOLEAN);
  if (!property) {
    return false;
  }
  value = IDP_bool_get(property);
  return true;
}

bool read_string(IDProperty *group, const char *name, std::string &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_STRING);
  if (!property) {
    return false;
  }
  value = IDP_string_get(property);
  return true;
}

template<size_t N>
bool read_floats(IDProperty *group, const char *name, std::array<float, N> &value)
{
  IDProperty *property = get_array(group, name, int(N), IDP_FLOAT);
  if (!property) {
    return false;
  }
  const float *values = IDP_array_float_get(property);
  for (size_t i = 0; i < N; i++) {
    if (!std::isfinite(values[i])) {
      return false;
    }
    value[i] = values[i];
  }
  return true;
}

/** Read one IDPArray, validating its length against the recorded count. */
IDProperty *read_item_array(IDProperty *group, const char *name, const int expected_count)
{
  IDProperty *array = IDP_GetPropertyTypeFromGroup(group, name, IDP_IDPARRAY);
  return array && array->len == expected_count ? array : nullptr;
}

IDProperty *array_item(IDProperty *array, const int index)
{
  IDProperty *item = IDP_GetIndexArray(array, index);
  return item && item->type == IDP_GROUP ? item : nullptr;
}

/* --- Mesh attribute helpers ------------------------------------------------ */

std::string additional_uv_attribute_name(const int set_index, const bool high_components)
{
  return std::string(kPMXAdditionalUVPrefix) + std::to_string(set_index) +
         (high_components ? "_zw" : "_xy");
}

/**
 * Resolve the mesh of one Object and verify its vertex count matches the
 * mapping that is about to be written.
 */
Mesh *mesh_for_attribute_write(Object *obj, const PMXModel &model, const Vector<int> *new_to_old)
{
  if (obj == nullptr || obj->type != OB_MESH) {
    return nullptr;
  }
  Mesh *mesh = reinterpret_cast<Mesh *>(obj->data);
  if (mesh == nullptr || mesh->verts_num == 0) {
    return nullptr;
  }
  const int expected = new_to_old ? int(new_to_old->size()) : int(model.vertices.size());
  return expected == mesh->verts_num ? mesh : nullptr;
}

}  // namespace

/* --- Build ----------------------------------------------------------------- */

PMXSourceData build_pmx_source_data(const PMXModel &model,
                                    const PMXImportParams &params,
                                    const Span<std::string> bone_names,
                                    const Span<int> morph_indices,
                                    const Span<std::string> morph_names,
                                    const PMXImportContext &ctx)
{
  PMXSourceData data;
  data.schema_version = kPMXSourceDataSchemaVersion;

  data.pmx_version = model.header.version;
  data.encoding = int(model.header.encoding);
  data.additional_uv_count = int(model.header.add_uv_cnt);

  data.name_local = model.name_local;
  data.name_universal = model.name_universal;
  data.comment_local = model.comment_local;
  data.comment_universal = model.comment_universal;

  data.source_filepath = params.filepath;
  data.global_scale = params.global_scale;
  data.split_by_material = params.split_by_material;

  data.vertex_count = int(model.vertices.size());
  data.face_index_count = int(model.face_indices.size());
  data.rigid_body_count = int(model.rigid_bodies.size());
  data.joint_count = int(model.joints.size());

  data.textures.reserve(model.textures.size());
  for (const PMXTexture &texture : model.textures) {
    PMXSourceTexture entry;
    entry.path = texture.path;
    data.textures.push_back(std::move(entry));
  }

  data.materials.reserve(model.materials.size());
  for (const int material_index : IndexRange(model.materials.size())) {
    const PMXMaterial &source = model.materials[material_index];
    PMXSourceMaterial entry;
    entry.pmx_index = material_index;
    entry.name_local = source.name_local;
    entry.name_universal = source.name_universal;
    entry.diffuse = {source.diffuse[0], source.diffuse[1], source.diffuse[2], source.diffuse[3]};
    entry.specular = {source.specular[0], source.specular[1], source.specular[2]};
    entry.specular_power = source.specular_power;
    entry.ambient = {source.ambient[0], source.ambient[1], source.ambient[2]};
    entry.flag = int(source.flag);
    entry.edge_color = {
        source.edge_color[0], source.edge_color[1], source.edge_color[2], source.edge_color[3]};
    entry.edge_size = source.edge_size;
    entry.texture_index = source.texture_idx;
    entry.sphere_texture_index = source.sphere_texture_idx;
    entry.sphere_mode = int(source.sphere_mode);
    entry.toon_flag = int(source.toon_flag);
    entry.toon_texture_index = source.toon_texture_idx;
    entry.toon_internal_value = int(source.toon_internal_value);
    entry.memo = source.memo;
    entry.face_vertex_count = source.face_vertex_count;
    if (Material *const *material = ctx.material_cache.lookup_ptr(material_index)) {
      if (*material != nullptr) {
        entry.blender_material_name = (*material)->id.name + 2;
      }
    }
    data.materials.push_back(std::move(entry));
  }

  data.bones.reserve(model.bones.size());
  for (const int bone_index : IndexRange(model.bones.size())) {
    const PMXBone &source = model.bones[bone_index];
    PMXSourceBone entry;
    entry.pmx_index = bone_index;
    entry.name_local = source.name_local;
    entry.name_universal = source.name_universal;
    entry.parent_index = source.parent_index;
    entry.transform_order = source.transform_order;
    entry.flag = int(source.flag);
    entry.tail_pos_bone = source.tail_pos_bone;
    entry.tail_pos_offset = {
        source.tail_pos_offset[0], source.tail_pos_offset[1], source.tail_pos_offset[2]};
    entry.inherit_parent_index = source.inherit_parent_index;
    entry.inherit_parent_ratio = source.inherit_parent_ratio;
    entry.fixed_axis = {source.fixed_axis[0], source.fixed_axis[1], source.fixed_axis[2]};
    entry.local_x = {source.local_x[0], source.local_x[1], source.local_x[2]};
    entry.local_z = {source.local_z[0], source.local_z[1], source.local_z[2]};
    entry.ik_target_index = source.ik_target_index;
    entry.ik_loop_count = source.ik_loop_count;
    entry.ik_angle_limit = source.ik_angle_limit;
    entry.ik_links.reserve(source.ik_links.size());
    for (const PMXIKLink &link : source.ik_links) {
      PMXSourceIKLink out_link;
      out_link.bone_index = link.bone_index;
      out_link.limit_angle = link.limit_angle;
      out_link.limit_min = {link.limit_min[0], link.limit_min[1], link.limit_min[2]};
      out_link.limit_max = {link.limit_max[0], link.limit_max[1], link.limit_max[2]};
      entry.ik_links.push_back(out_link);
    }
    entry.external_parent_index = source.external_parent_index;
    if (bone_index < bone_names.size()) {
      entry.blender_bone_name = bone_names[bone_index];
    }
    data.bones.push_back(std::move(entry));
  }

  data.morphs.reserve(model.morphs.size());
  for (const int morph_index : IndexRange(model.morphs.size())) {
    const PMXMorph &source = model.morphs[morph_index];
    PMXSourceMorph entry;
    entry.pmx_index = morph_index;
    entry.name_local = source.name_local;
    entry.name_universal = source.name_universal;
    entry.panel = int(source.panel);
    entry.type = int(source.type);
    entry.vertex_offset_count = int(source.vertex_offsets.size());

    /* Vertex Morph offsets live in Shape Keys; only record where to find them. */
    for (const int i : morph_indices.index_range()) {
      if (morph_indices[i] == morph_index && i < morph_names.size()) {
        entry.blender_shape_key_name = morph_names[i];
        break;
      }
    }

    for (const PMXGroupMorphOffset &offset : source.group_offsets) {
      PMXSourceGroupMorphOffset out;
      out.morph_index = offset.morph_index;
      out.influence = offset.influence;
      entry.group_offsets.push_back(out);
    }
    for (const PMXBoneMorphOffset &offset : source.bone_offsets) {
      PMXSourceBoneMorphOffset out;
      out.bone_index = offset.bone_index;
      out.pos = {offset.pos[0], offset.pos[1], offset.pos[2]};
      out.rot = {offset.rot[0], offset.rot[1], offset.rot[2], offset.rot[3]};
      entry.bone_offsets.push_back(out);
    }
    for (const PMXUVMorphOffset &offset : source.uv_offsets) {
      PMXSourceUVMorphOffset out;
      out.vertex_index = offset.vertex_index;
      out.offset = {offset.offset[0], offset.offset[1], offset.offset[2], offset.offset[3]};
      entry.uv_offsets.push_back(out);
    }
    for (const PMXMaterialMorphOffset &offset : source.material_offsets) {
      PMXSourceMaterialMorphOffset out;
      out.material_index = offset.material_index;
      out.calc_mode = int(offset.calc_mode);
      out.diffuse = {
          offset.diffuse[0], offset.diffuse[1], offset.diffuse[2], offset.diffuse[3]};
      out.specular = {offset.specular[0], offset.specular[1], offset.specular[2]};
      out.specular_power = offset.specular_power;
      out.ambient = {offset.ambient[0], offset.ambient[1], offset.ambient[2]};
      out.edge_color = {offset.edge_color[0],
                        offset.edge_color[1],
                        offset.edge_color[2],
                        offset.edge_color[3]};
      out.edge_size = offset.edge_size;
      out.texture_factor = {offset.texture_factor[0],
                            offset.texture_factor[1],
                            offset.texture_factor[2],
                            offset.texture_factor[3]};
      out.sphere_texture_factor = {offset.sphere_texture_factor[0],
                                   offset.sphere_texture_factor[1],
                                   offset.sphere_texture_factor[2],
                                   offset.sphere_texture_factor[3]};
      out.toon_texture_factor = {offset.toon_texture_factor[0],
                                 offset.toon_texture_factor[1],
                                 offset.toon_texture_factor[2],
                                 offset.toon_texture_factor[3]};
      entry.material_offsets.push_back(out);
    }
    for (const PMXImpulseMorphOffset &offset : source.impulse_offsets) {
      PMXSourceImpulseMorphOffset out;
      out.rigid_index = offset.rigid_index;
      out.local_flag = int(offset.local_flag);
      out.velocity = {offset.velocity[0], offset.velocity[1], offset.velocity[2]};
      out.torque = {offset.torque[0], offset.torque[1], offset.torque[2]};
      entry.impulse_offsets.push_back(out);
    }
    data.morphs.push_back(std::move(entry));
  }

  data.display_frames.reserve(model.display_frames.size());
  for (const PMXDisplayFrame &source : model.display_frames) {
    PMXSourceDisplayFrame entry;
    entry.name_local = source.name_local;
    entry.name_universal = source.name_universal;
    entry.flag = int(source.flag);
    entry.items.reserve(source.items.size());
    for (const PMXDisplayFrame::FrameItem &item : source.items) {
      PMXSourceDisplayFrame::Item out;
      out.type = int(item.type);
      out.index = item.index;
      entry.items.push_back(out);
    }
    data.display_frames.push_back(std::move(entry));
  }

  return data;
}

/* --- Serialize ------------------------------------------------------------- */

bool serialize_pmx_source_data(Collection &model_root,
                               const PMXSourceData &data,
                               ReportList *reports)
{
  if (data.schema_version != kPMXSourceDataSchemaVersion) {
    report_warning(reports, "PMX source data: unsupported schema version");
    return false;
  }
  if (data.textures.size() > kMaxPersistedItems ||
      data.materials.size() > kMaxPersistedItems || data.bones.size() > kMaxPersistedItems ||
      data.morphs.size() > kMaxPersistedItems ||
      data.display_frames.size() > kMaxPersistedItems)
  {
    report_warning(reports, "PMX source data: section item count exceeds persistence limit");
    return false;
  }
  for (const PMXSourceBone &bone : data.bones) {
    if (bone.ik_links.size() > kMaxPersistedNestedItems) {
      report_warning(reports, "PMX source data: IK link count exceeds persistence limit");
      return false;
    }
  }
  for (const PMXSourceMorph &morph : data.morphs) {
    if (morph.group_offsets.size() > kMaxPersistedNestedItems ||
        morph.bone_offsets.size() > kMaxPersistedNestedItems ||
        morph.uv_offsets.size() > kMaxPersistedNestedItems ||
        morph.material_offsets.size() > kMaxPersistedNestedItems ||
        morph.impulse_offsets.size() > kMaxPersistedNestedItems)
    {
      report_warning(reports, "PMX source data: Morph offset count exceeds persistence limit");
      return false;
    }
  }
  for (const PMXSourceDisplayFrame &frame : data.display_frames) {
    if (frame.items.size() > kMaxPersistedNestedItems) {
      report_warning(reports, "PMX source data: display frame item count exceeds limit");
      return false;
    }
  }

  IDProperty *root = bke::idprop::create_group(kSourceDataProperty).release();
  if (!root) {
    report_warning(reports, "PMX source data: failed to allocate root property");
    return false;
  }

  add_int(root, "schema_version", data.schema_version);
  add_float(root, "pmx_version", data.pmx_version);
  add_int(root, "encoding", data.encoding);
  add_int(root, "additional_uv_count", data.additional_uv_count);

  add_string(root, "name_local", data.name_local);
  add_string(root, "name_universal", data.name_universal);
  add_string(root, "comment_local", data.comment_local);
  add_string(root, "comment_universal", data.comment_universal);

  add_string(root, "source_filepath", data.source_filepath);
  add_float(root, "global_scale", data.global_scale);
  add_bool(root, "split_by_material", data.split_by_material);

  add_int(root, "vertex_count", data.vertex_count);
  add_int(root, "face_index_count", data.face_index_count);
  add_int(root, "rigid_body_count", data.rigid_body_count);
  add_int(root, "joint_count", data.joint_count);

  add_int(root, "texture_count", int(data.textures.size()));
  add_int(root, "material_count", int(data.materials.size()));
  add_int(root, "bone_count", int(data.bones.size()));
  add_int(root, "morph_count", int(data.morphs.size()));
  add_int(root, "display_frame_count", int(data.display_frames.size()));

  IDProperty *textures = IDP_NewIDPArray("textures");
  for (const PMXSourceTexture &texture : data.textures) {
    IDProperty *item = bke::idprop::create_group("texture").release();
    add_string(item, "path", texture.path);
    append_group(textures, item);
  }
  add_property(root, textures);

  IDProperty *materials = IDP_NewIDPArray("materials");
  for (const PMXSourceMaterial &material : data.materials) {
    IDProperty *item = bke::idprop::create_group("material").release();
    add_int(item, "pmx_index", material.pmx_index);
    add_string(item, "name_local", material.name_local);
    add_string(item, "name_universal", material.name_universal);
    add_floats(item, "diffuse", material.diffuse);
    add_floats(item, "specular", material.specular);
    add_float(item, "specular_power", material.specular_power);
    add_floats(item, "ambient", material.ambient);
    add_int(item, "flag", material.flag);
    add_floats(item, "edge_color", material.edge_color);
    add_float(item, "edge_size", material.edge_size);
    add_int(item, "texture_index", material.texture_index);
    add_int(item, "sphere_texture_index", material.sphere_texture_index);
    add_int(item, "sphere_mode", material.sphere_mode);
    add_int(item, "toon_flag", material.toon_flag);
    add_int(item, "toon_texture_index", material.toon_texture_index);
    add_int(item, "toon_internal_value", material.toon_internal_value);
    add_string(item, "memo", material.memo);
    add_int(item, "face_vertex_count", material.face_vertex_count);
    add_string(item, "blender_material_name", material.blender_material_name);
    append_group(materials, item);
  }
  add_property(root, materials);

  IDProperty *bones = IDP_NewIDPArray("bones");
  for (const PMXSourceBone &bone : data.bones) {
    IDProperty *item = bke::idprop::create_group("bone").release();
    add_int(item, "pmx_index", bone.pmx_index);
    add_string(item, "name_local", bone.name_local);
    add_string(item, "name_universal", bone.name_universal);
    add_int(item, "parent_index", bone.parent_index);
    add_int(item, "transform_order", bone.transform_order);
    add_int(item, "flag", bone.flag);
    add_int(item, "tail_pos_bone", bone.tail_pos_bone);
    add_floats(item, "tail_pos_offset", bone.tail_pos_offset);
    add_int(item, "inherit_parent_index", bone.inherit_parent_index);
    add_float(item, "inherit_parent_ratio", bone.inherit_parent_ratio);
    add_floats(item, "fixed_axis", bone.fixed_axis);
    add_floats(item, "local_x", bone.local_x);
    add_floats(item, "local_z", bone.local_z);
    add_int(item, "ik_target_index", bone.ik_target_index);
    add_int(item, "ik_loop_count", bone.ik_loop_count);
    add_float(item, "ik_angle_limit", bone.ik_angle_limit);
    add_int(item, "ik_link_count", int(bone.ik_links.size()));
    add_int(item, "external_parent_index", bone.external_parent_index);
    add_string(item, "blender_bone_name", bone.blender_bone_name);

    IDProperty *links = IDP_NewIDPArray("ik_links");
    for (const PMXSourceIKLink &link : bone.ik_links) {
      IDProperty *link_item = bke::idprop::create_group("ik_link").release();
      add_int(link_item, "bone_index", link.bone_index);
      add_bool(link_item, "limit_angle", link.limit_angle);
      add_floats(link_item, "limit_min", link.limit_min);
      add_floats(link_item, "limit_max", link.limit_max);
      append_group(links, link_item);
    }
    add_property(item, links);
    append_group(bones, item);
  }
  add_property(root, bones);

  IDProperty *morphs = IDP_NewIDPArray("morphs");
  for (const PMXSourceMorph &morph : data.morphs) {
    IDProperty *item = bke::idprop::create_group("morph").release();
    add_int(item, "pmx_index", morph.pmx_index);
    add_string(item, "name_local", morph.name_local);
    add_string(item, "name_universal", morph.name_universal);
    add_int(item, "panel", morph.panel);
    add_int(item, "type", morph.type);
    add_int(item, "vertex_offset_count", morph.vertex_offset_count);
    add_string(item, "blender_shape_key_name", morph.blender_shape_key_name);
    add_int(item, "group_offset_count", int(morph.group_offsets.size()));
    add_int(item, "bone_offset_count", int(morph.bone_offsets.size()));
    add_int(item, "uv_offset_count", int(morph.uv_offsets.size()));
    add_int(item, "material_offset_count", int(morph.material_offsets.size()));
    add_int(item, "impulse_offset_count", int(morph.impulse_offsets.size()));

    IDProperty *group_offsets = IDP_NewIDPArray("group_offsets");
    for (const PMXSourceGroupMorphOffset &offset : morph.group_offsets) {
      IDProperty *offset_item = bke::idprop::create_group("group_offset").release();
      add_int(offset_item, "morph_index", offset.morph_index);
      add_float(offset_item, "influence", offset.influence);
      append_group(group_offsets, offset_item);
    }
    add_property(item, group_offsets);

    IDProperty *bone_offsets = IDP_NewIDPArray("bone_offsets");
    for (const PMXSourceBoneMorphOffset &offset : morph.bone_offsets) {
      IDProperty *offset_item = bke::idprop::create_group("bone_offset").release();
      add_int(offset_item, "bone_index", offset.bone_index);
      add_floats(offset_item, "pos", offset.pos);
      add_floats(offset_item, "rot", offset.rot);
      append_group(bone_offsets, offset_item);
    }
    add_property(item, bone_offsets);

    IDProperty *uv_offsets = IDP_NewIDPArray("uv_offsets");
    for (const PMXSourceUVMorphOffset &offset : morph.uv_offsets) {
      IDProperty *offset_item = bke::idprop::create_group("uv_offset").release();
      add_int(offset_item, "vertex_index", offset.vertex_index);
      add_floats(offset_item, "offset", offset.offset);
      append_group(uv_offsets, offset_item);
    }
    add_property(item, uv_offsets);

    IDProperty *material_offsets = IDP_NewIDPArray("material_offsets");
    for (const PMXSourceMaterialMorphOffset &offset : morph.material_offsets) {
      IDProperty *offset_item = bke::idprop::create_group("material_offset").release();
      add_int(offset_item, "material_index", offset.material_index);
      add_int(offset_item, "calc_mode", offset.calc_mode);
      add_floats(offset_item, "diffuse", offset.diffuse);
      add_floats(offset_item, "specular", offset.specular);
      add_float(offset_item, "specular_power", offset.specular_power);
      add_floats(offset_item, "ambient", offset.ambient);
      add_floats(offset_item, "edge_color", offset.edge_color);
      add_float(offset_item, "edge_size", offset.edge_size);
      add_floats(offset_item, "texture_factor", offset.texture_factor);
      add_floats(offset_item, "sphere_texture_factor", offset.sphere_texture_factor);
      add_floats(offset_item, "toon_texture_factor", offset.toon_texture_factor);
      append_group(material_offsets, offset_item);
    }
    add_property(item, material_offsets);

    IDProperty *impulse_offsets = IDP_NewIDPArray("impulse_offsets");
    for (const PMXSourceImpulseMorphOffset &offset : morph.impulse_offsets) {
      IDProperty *offset_item = bke::idprop::create_group("impulse_offset").release();
      add_int(offset_item, "rigid_index", offset.rigid_index);
      add_int(offset_item, "local_flag", offset.local_flag);
      add_floats(offset_item, "velocity", offset.velocity);
      add_floats(offset_item, "torque", offset.torque);
      append_group(impulse_offsets, offset_item);
    }
    add_property(item, impulse_offsets);

    append_group(morphs, item);
  }
  add_property(root, morphs);

  IDProperty *frames = IDP_NewIDPArray("display_frames");
  for (const PMXSourceDisplayFrame &frame : data.display_frames) {
    IDProperty *item = bke::idprop::create_group("display_frame").release();
    add_string(item, "name_local", frame.name_local);
    add_string(item, "name_universal", frame.name_universal);
    add_int(item, "flag", frame.flag);
    add_int(item, "item_count", int(frame.items.size()));

    /* Frame items are two parallel int arrays rather than groups: a display
     * frame can hold every bone and Morph in the model, and the item payload is
     * small enough that per-item groups would dominate its cost. */
    Vector<int32_t> types(frame.items.size());
    Vector<int32_t> indices(frame.items.size());
    for (const int i : IndexRange(frame.items.size())) {
      types[i] = int32_t(frame.items[i].type);
      indices[i] = int32_t(frame.items[i].index);
    }
    add_ints(item, "item_types", types.as_span());
    add_ints(item, "item_indices", indices.as_span());
    append_group(frames, item);
  }
  add_property(root, frames);

  IDProperty *system = IDP_ID_system_properties_ensure(&model_root.id);
  IDP_ReplaceInGroup(system, root);
  return true;
}

/* --- Deserialize ----------------------------------------------------------- */

bool deserialize_pmx_source_data(const Collection &model_root,
                                 PMXSourceData &data,
                                 ReportList *reports)
{
  IDProperty *system = model_root.id.system_properties;
  IDProperty *root = system ? IDP_GetPropertyTypeFromGroup(
                                  system, kSourceDataProperty, IDP_GROUP) :
                              nullptr;
  if (!root) {
    report_warning(reports, "PMX source data: property is missing or not a group");
    return false;
  }

  int schema = 0;
  if (!read_int(root, "schema_version", schema) || schema != kPMXSourceDataSchemaVersion) {
    report_warning(reports, "PMX source data: unsupported schema version");
    return false;
  }

  int texture_count = 0;
  int material_count = 0;
  int bone_count = 0;
  int morph_count = 0;
  int display_frame_count = 0;
  if (!read_int(root, "texture_count", texture_count) ||
      !read_int(root, "material_count", material_count) ||
      !read_int(root, "bone_count", bone_count) || !read_int(root, "morph_count", morph_count) ||
      !read_int(root, "display_frame_count", display_frame_count) || texture_count < 0 ||
      material_count < 0 || bone_count < 0 || morph_count < 0 || display_frame_count < 0 ||
      texture_count > kMaxPersistedItems || material_count > kMaxPersistedItems ||
      bone_count > kMaxPersistedItems || morph_count > kMaxPersistedItems ||
      display_frame_count > kMaxPersistedItems)
  {
    report_warning(reports, "PMX source data: invalid section counts");
    return false;
  }

  IDProperty *textures = read_item_array(root, "textures", texture_count);
  IDProperty *materials = read_item_array(root, "materials", material_count);
  IDProperty *bones = read_item_array(root, "bones", bone_count);
  IDProperty *morphs = read_item_array(root, "morphs", morph_count);
  IDProperty *frames = read_item_array(root, "display_frames", display_frame_count);
  if (!textures || !materials || !bones || !morphs || !frames) {
    report_warning(reports, "PMX source data: array lengths do not match recorded counts");
    return false;
  }

  PMXSourceData out;
  out.schema_version = schema;

  if (!read_float(root, "pmx_version", out.pmx_version) ||
      !read_int(root, "encoding", out.encoding) ||
      !read_int(root, "additional_uv_count", out.additional_uv_count) || out.encoding < 0 ||
      out.encoding > 1 || out.additional_uv_count < 0 || out.additional_uv_count > 4)
  {
    report_warning(reports, "PMX source data: invalid header values");
    return false;
  }
  if (!read_string(root, "name_local", out.name_local) ||
      !read_string(root, "name_universal", out.name_universal) ||
      !read_string(root, "comment_local", out.comment_local) ||
      !read_string(root, "comment_universal", out.comment_universal))
  {
    report_warning(reports, "PMX source data: invalid model info");
    return false;
  }
  if (!read_string(root, "source_filepath", out.source_filepath) ||
      !read_float(root, "global_scale", out.global_scale) ||
      !read_bool(root, "split_by_material", out.split_by_material) || out.global_scale <= 0.0f)
  {
    report_warning(reports, "PMX source data: invalid import provenance");
    return false;
  }
  if (!read_int(root, "vertex_count", out.vertex_count) ||
      !read_int(root, "face_index_count", out.face_index_count) ||
      !read_int(root, "rigid_body_count", out.rigid_body_count) ||
      !read_int(root, "joint_count", out.joint_count) || out.vertex_count < 0 ||
      out.face_index_count < 0 || out.rigid_body_count < 0 || out.joint_count < 0)
  {
    report_warning(reports, "PMX source data: invalid section counts");
    return false;
  }

  out.textures.resize(texture_count);
  for (const int i : IndexRange(texture_count)) {
    IDProperty *item = array_item(textures, i);
    if (!item || !read_string(item, "path", out.textures[i].path)) {
      report_warning(reports, "PMX source data: invalid texture entry");
      return false;
    }
  }

  out.materials.resize(material_count);
  for (const int i : IndexRange(material_count)) {
    IDProperty *item = array_item(materials, i);
    PMXSourceMaterial &material = out.materials[i];
    if (!item || !read_int(item, "pmx_index", material.pmx_index) ||
        material.pmx_index != i || !read_string(item, "name_local", material.name_local) ||
        !read_string(item, "name_universal", material.name_universal) ||
        !read_floats(item, "diffuse", material.diffuse) ||
        !read_floats(item, "specular", material.specular) ||
        !read_float(item, "specular_power", material.specular_power) ||
        !read_floats(item, "ambient", material.ambient) ||
        !read_int(item, "flag", material.flag) ||
        !read_floats(item, "edge_color", material.edge_color) ||
        !read_float(item, "edge_size", material.edge_size) ||
        !read_int(item, "texture_index", material.texture_index) ||
        !read_int(item, "sphere_texture_index", material.sphere_texture_index) ||
        !read_int(item, "sphere_mode", material.sphere_mode) ||
        !read_int(item, "toon_flag", material.toon_flag) ||
        !read_int(item, "toon_texture_index", material.toon_texture_index) ||
        !read_int(item, "toon_internal_value", material.toon_internal_value) ||
        !read_string(item, "memo", material.memo) ||
        !read_int(item, "face_vertex_count", material.face_vertex_count) ||
        !read_string(item, "blender_material_name", material.blender_material_name))
    {
      report_warning(reports, "PMX source data: invalid material entry");
      return false;
    }
  }

  out.bones.resize(bone_count);
  for (const int i : IndexRange(bone_count)) {
    IDProperty *item = array_item(bones, i);
    PMXSourceBone &bone = out.bones[i];
    int ik_link_count = 0;
    if (!item || !read_int(item, "pmx_index", bone.pmx_index) || bone.pmx_index != i ||
        !read_string(item, "name_local", bone.name_local) ||
        !read_string(item, "name_universal", bone.name_universal) ||
        !read_int(item, "parent_index", bone.parent_index) ||
        !read_int(item, "transform_order", bone.transform_order) ||
        !read_int(item, "flag", bone.flag) ||
        !read_int(item, "tail_pos_bone", bone.tail_pos_bone) ||
        !read_floats(item, "tail_pos_offset", bone.tail_pos_offset) ||
        !read_int(item, "inherit_parent_index", bone.inherit_parent_index) ||
        !read_float(item, "inherit_parent_ratio", bone.inherit_parent_ratio) ||
        !read_floats(item, "fixed_axis", bone.fixed_axis) ||
        !read_floats(item, "local_x", bone.local_x) ||
        !read_floats(item, "local_z", bone.local_z) ||
        !read_int(item, "ik_target_index", bone.ik_target_index) ||
        !read_int(item, "ik_loop_count", bone.ik_loop_count) ||
        !read_float(item, "ik_angle_limit", bone.ik_angle_limit) ||
        !read_int(item, "ik_link_count", ik_link_count) ||
        !read_int(item, "external_parent_index", bone.external_parent_index) ||
        !read_string(item, "blender_bone_name", bone.blender_bone_name) || ik_link_count < 0 ||
        ik_link_count > kMaxPersistedNestedItems)
    {
      report_warning(reports, "PMX source data: invalid bone entry");
      return false;
    }
    IDProperty *links = read_item_array(item, "ik_links", ik_link_count);
    if (!links) {
      report_warning(reports, "PMX source data: invalid IK link array");
      return false;
    }
    bone.ik_links.resize(ik_link_count);
    for (const int j : IndexRange(ik_link_count)) {
      IDProperty *link_item = array_item(links, j);
      PMXSourceIKLink &link = bone.ik_links[j];
      if (!link_item || !read_int(link_item, "bone_index", link.bone_index) ||
          !read_bool(link_item, "limit_angle", link.limit_angle) ||
          !read_floats(link_item, "limit_min", link.limit_min) ||
          !read_floats(link_item, "limit_max", link.limit_max))
      {
        report_warning(reports, "PMX source data: invalid IK link entry");
        return false;
      }
    }
  }

  out.morphs.resize(morph_count);
  for (const int i : IndexRange(morph_count)) {
    IDProperty *item = array_item(morphs, i);
    PMXSourceMorph &morph = out.morphs[i];
    int group_count = 0;
    int bone_offset_count = 0;
    int uv_count = 0;
    int material_offset_count = 0;
    int impulse_count = 0;
    if (!item || !read_int(item, "pmx_index", morph.pmx_index) || morph.pmx_index != i ||
        !read_string(item, "name_local", morph.name_local) ||
        !read_string(item, "name_universal", morph.name_universal) ||
        !read_int(item, "panel", morph.panel) || !read_int(item, "type", morph.type) ||
        !read_int(item, "vertex_offset_count", morph.vertex_offset_count) ||
        !read_string(item, "blender_shape_key_name", morph.blender_shape_key_name) ||
        !read_int(item, "group_offset_count", group_count) ||
        !read_int(item, "bone_offset_count", bone_offset_count) ||
        !read_int(item, "uv_offset_count", uv_count) ||
        !read_int(item, "material_offset_count", material_offset_count) ||
        !read_int(item, "impulse_offset_count", impulse_count) ||
        morph.vertex_offset_count < 0 || group_count < 0 || bone_offset_count < 0 ||
        uv_count < 0 || material_offset_count < 0 || impulse_count < 0 ||
        group_count > kMaxPersistedNestedItems ||
        bone_offset_count > kMaxPersistedNestedItems || uv_count > kMaxPersistedNestedItems ||
        material_offset_count > kMaxPersistedNestedItems ||
        impulse_count > kMaxPersistedNestedItems)
    {
      report_warning(reports, "PMX source data: invalid morph entry");
      return false;
    }

    IDProperty *group_offsets = read_item_array(item, "group_offsets", group_count);
    IDProperty *bone_offsets = read_item_array(item, "bone_offsets", bone_offset_count);
    IDProperty *uv_offsets = read_item_array(item, "uv_offsets", uv_count);
    IDProperty *material_offsets = read_item_array(
        item, "material_offsets", material_offset_count);
    IDProperty *impulse_offsets = read_item_array(item, "impulse_offsets", impulse_count);
    if (!group_offsets || !bone_offsets || !uv_offsets || !material_offsets || !impulse_offsets) {
      report_warning(reports, "PMX source data: invalid morph offset arrays");
      return false;
    }

    morph.group_offsets.resize(group_count);
    for (const int j : IndexRange(group_count)) {
      IDProperty *offset_item = array_item(group_offsets, j);
      PMXSourceGroupMorphOffset &offset = morph.group_offsets[j];
      if (!offset_item || !read_int(offset_item, "morph_index", offset.morph_index) ||
          !read_float(offset_item, "influence", offset.influence))
      {
        report_warning(reports, "PMX source data: invalid group morph offset");
        return false;
      }
    }
    morph.bone_offsets.resize(bone_offset_count);
    for (const int j : IndexRange(bone_offset_count)) {
      IDProperty *offset_item = array_item(bone_offsets, j);
      PMXSourceBoneMorphOffset &offset = morph.bone_offsets[j];
      if (!offset_item || !read_int(offset_item, "bone_index", offset.bone_index) ||
          !read_floats(offset_item, "pos", offset.pos) ||
          !read_floats(offset_item, "rot", offset.rot))
      {
        report_warning(reports, "PMX source data: invalid bone morph offset");
        return false;
      }
    }
    morph.uv_offsets.resize(uv_count);
    for (const int j : IndexRange(uv_count)) {
      IDProperty *offset_item = array_item(uv_offsets, j);
      PMXSourceUVMorphOffset &offset = morph.uv_offsets[j];
      if (!offset_item || !read_int(offset_item, "vertex_index", offset.vertex_index) ||
          !read_floats(offset_item, "offset", offset.offset))
      {
        report_warning(reports, "PMX source data: invalid UV morph offset");
        return false;
      }
    }
    morph.material_offsets.resize(material_offset_count);
    for (const int j : IndexRange(material_offset_count)) {
      IDProperty *offset_item = array_item(material_offsets, j);
      PMXSourceMaterialMorphOffset &offset = morph.material_offsets[j];
      if (!offset_item || !read_int(offset_item, "material_index", offset.material_index) ||
          !read_int(offset_item, "calc_mode", offset.calc_mode) ||
          !read_floats(offset_item, "diffuse", offset.diffuse) ||
          !read_floats(offset_item, "specular", offset.specular) ||
          !read_float(offset_item, "specular_power", offset.specular_power) ||
          !read_floats(offset_item, "ambient", offset.ambient) ||
          !read_floats(offset_item, "edge_color", offset.edge_color) ||
          !read_float(offset_item, "edge_size", offset.edge_size) ||
          !read_floats(offset_item, "texture_factor", offset.texture_factor) ||
          !read_floats(offset_item, "sphere_texture_factor", offset.sphere_texture_factor) ||
          !read_floats(offset_item, "toon_texture_factor", offset.toon_texture_factor))
      {
        report_warning(reports, "PMX source data: invalid material morph offset");
        return false;
      }
    }
    morph.impulse_offsets.resize(impulse_count);
    for (const int j : IndexRange(impulse_count)) {
      IDProperty *offset_item = array_item(impulse_offsets, j);
      PMXSourceImpulseMorphOffset &offset = morph.impulse_offsets[j];
      if (!offset_item || !read_int(offset_item, "rigid_index", offset.rigid_index) ||
          !read_int(offset_item, "local_flag", offset.local_flag) ||
          !read_floats(offset_item, "velocity", offset.velocity) ||
          !read_floats(offset_item, "torque", offset.torque))
      {
        report_warning(reports, "PMX source data: invalid impulse morph offset");
        return false;
      }
    }
  }

  out.display_frames.resize(display_frame_count);
  for (const int i : IndexRange(display_frame_count)) {
    IDProperty *item = array_item(frames, i);
    PMXSourceDisplayFrame &frame = out.display_frames[i];
    int item_count = 0;
    if (!item || !read_string(item, "name_local", frame.name_local) ||
        !read_string(item, "name_universal", frame.name_universal) ||
        !read_int(item, "flag", frame.flag) || !read_int(item, "item_count", item_count) ||
        item_count < 0 || item_count > kMaxPersistedNestedItems)
    {
      report_warning(reports, "PMX source data: invalid display frame entry");
      return false;
    }
    IDProperty *types = get_array(item, "item_types", item_count, IDP_INT);
    IDProperty *indices = get_array(item, "item_indices", item_count, IDP_INT);
    if (!types || !indices) {
      report_warning(reports, "PMX source data: invalid display frame item arrays");
      return false;
    }
    const int *type_values = IDP_array_int_get(types);
    const int *index_values = IDP_array_int_get(indices);
    frame.items.resize(item_count);
    for (const int j : IndexRange(item_count)) {
      frame.items[j].type = type_values[j];
      frame.items[j].index = index_values[j];
    }
  }

  data = std::move(out);
  return true;
}

/* --- Mesh attributes ------------------------------------------------------- */

bool write_pmx_vertex_index_attribute(Object *obj,
                                      const PMXModel &model,
                                      const Vector<int> *new_to_old)
{
  Mesh *mesh = mesh_for_attribute_write(obj, model, new_to_old);
  if (mesh == nullptr) {
    return false;
  }

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  bke::SpanAttributeWriter<int> indices =
      attributes.lookup_or_add_for_write_only_span<int>(kPMXVertexIndexAttribute,
                                                       bke::AttrDomain::Point);
  if (!indices) {
    return false;
  }
  for (const int vi : IndexRange(mesh->verts_num)) {
    const int source_vi = new_to_old ? (*new_to_old)[vi] : vi;
    indices.span[vi] = source_vi >= 0 && source_vi < int(model.vertices.size()) ? source_vi : -1;
  }
  indices.finish();
  return true;
}

bool write_pmx_additional_uv_attributes(Object *obj,
                                        const PMXModel &model,
                                        const Vector<int> *new_to_old)
{
  const int set_count = std::min(int(model.header.add_uv_cnt), 4);
  if (set_count <= 0) {
    return true;
  }
  Mesh *mesh = mesh_for_attribute_write(obj, model, new_to_old);
  if (mesh == nullptr) {
    return false;
  }

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  for (const int set_index : IndexRange(set_count)) {
    const std::string low_name = additional_uv_attribute_name(set_index, false);
    const std::string high_name = additional_uv_attribute_name(set_index, true);
    bke::SpanAttributeWriter<float2> low =
        attributes.lookup_or_add_for_write_only_span<float2>(low_name, bke::AttrDomain::Point);
    bke::SpanAttributeWriter<float2> high =
        attributes.lookup_or_add_for_write_only_span<float2>(high_name, bke::AttrDomain::Point);
    if (!low || !high) {
      if (low) {
        low.finish();
      }
      if (high) {
        high.finish();
      }
      return false;
    }
    for (const int vi : IndexRange(mesh->verts_num)) {
      const int source_vi = new_to_old ? (*new_to_old)[vi] : vi;
      if (source_vi < 0 || source_vi >= int(model.vertices.size())) {
        low.span[vi] = float2(0.0f);
        high.span[vi] = float2(0.0f);
        continue;
      }
      const std::array<float, 4> &uv = model.vertices[source_vi].additional_uv[set_index];
      low.span[vi] = float2(uv[0], uv[1]);
      high.span[vi] = float2(uv[2], uv[3]);
    }
    low.finish();
    high.finish();
  }
  return true;
}

/* --- Import entry point ---------------------------------------------------- */

void persist_pmx_source_data(PMXImportContext &ctx,
                             const PMXModel &model,
                             const Span<std::string> bone_names)
{
  if (ctx.params->split_by_material) {
    for (const SubMeshInfo &sub : ctx.sub_meshes) {
      write_pmx_vertex_index_attribute(sub.obj, model, &sub.new_to_old);
      write_pmx_additional_uv_attributes(sub.obj, model, &sub.new_to_old);
    }
  }
  else {
    for (Object *obj : ctx.mesh_objects) {
      write_pmx_vertex_index_attribute(obj, model, nullptr);
      write_pmx_additional_uv_attributes(obj, model, nullptr);
    }
  }

  if (ctx.model_collection == nullptr) {
    report_warning(ctx.reports, "PMX source data: no model collection to persist to");
    return;
  }

  const PMXSourceData data = build_pmx_source_data(
      model, *ctx.params, bone_names, ctx.morph_indices.as_span(), ctx.morph_names.as_span(), ctx);
  if (!serialize_pmx_source_data(*ctx.model_collection, data, ctx.reports)) {
    return;
  }
  if (ctx.reports) {
    BKE_reportf(ctx.reports,
                RPT_INFO,
                "PMX source data persisted: schema %d, %zu textures, %zu materials, %zu bones, "
                "%zu morphs, %zu display frames",
                data.schema_version,
                data.textures.size(),
                data.materials.size(),
                data.bones.size(),
                data.morphs.size(),
                data.display_frames.size());
  }
}

}  // namespace blender::io::pmx
