/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_ID.h"
#include "DNA_object_types.h"
#include "DNA_collection_types.h"
#include "DNA_armature_types.h"

#include "BKE_idprop.hh"
#include "BKE_report.hh"
#include "BKE_armature.hh"

#include "mmd_ccd_ik.hh"

#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include "intern/pmx_types.h"
#include "pmx_import_bone_ik.hh"
#include "pmx_import_mesh.hh"

#include <array>
#include <cmath>
#include <string>

namespace blender::io::pmx {
namespace {

constexpr char kBoneIKDefinitionProperty[] = "mmd_pmx_bone_ik_definition";
constexpr int kBoneIKDefinitionSchemaVersion = 2;
constexpr int kBoneIKDefinitionLegacySchemaVersion = 1;

static bool is_supported_schema_version(const int version)
{
  return version == kBoneIKDefinitionLegacySchemaVersion ||
         version == kBoneIKDefinitionSchemaVersion;
}

static void add_property(IDProperty *group, IDProperty *property)
{
  if (!property || !IDP_AddToGroup(group, property)) {
    if (property) {
      IDP_FreeProperty(property);
    }
  }
}

static void add_int(IDProperty *group, const char *name, const int value)
{
  add_property(group, IDP_NewInt(value, name));
}

static void add_float(IDProperty *group, const char *name, const float value)
{
  add_property(group, blender::bke::idprop::create(name, value).release());
}

static void add_bool(IDProperty *group, const char *name, const bool value)
{
  add_property(group, blender::bke::idprop::create_bool(name, value).release());
}

static void add_string(IDProperty *group, const char *name, const std::string &value)
{
  add_property(group, IDP_NewString(value.c_str(), name));
}

static void add_vec3(IDProperty *group, const char *name, const std::array<float, 3> &values)
{
  add_property(group,
               blender::bke::idprop::create(name, blender::Span<float>(values.data(), 3)).release());
}

static void append_group(IDProperty *array, IDProperty *item)
{
  /* NOTE: in this fork `IDP_AppendArray` copies `item`'s content into the array
   * slot via a shallow memcpy (`IDP_SetIndexArray`) and does NOT take ownership of
   * `item` — the slot shares `item`'s child nodes. Release only the now-redundant
   * source struct here; its children are owned by the slot and freed with the
   * array tree. Freeing the struct alone (not the children) avoids a dangling
   * pointer / double-free. */
  IDP_AppendArray(array, item);
  MEM_delete(item);
}

static std::string bone_name_at(const PMXModel &model, int index)
{
  if (index >= 0 && index < int(model.bones.size())) {
    return model.bones[index].name_local;
  }
  return std::string();
}

/**
 * Build the schema-1 IK definition IDProperty group from the parsed PMX model.
 * Only bones with BONE_FLAG_IK are included.
 */
static IDProperty *build_ik_definition_property(const PMXModel &model, int &r_ik_count)
{
  std::vector<bool> physics_owned_bones(model.bones.size(), false);
  for (const PMXRigidBody &rigid : model.rigid_bodies) {
    if (rigid.physics_type != 0 && rigid.bone_index >= 0 &&
        rigid.bone_index < int(physics_owned_bones.size()))
    {
      physics_owned_bones[rigid.bone_index] = true;
    }
  }

  int ik_count = 0;
  for (const PMXBone &bone : model.bones) {
    if (bone.flag & BONE_FLAG_IK) {
      ik_count++;
    }
  }
  r_ik_count = ik_count;

  IDProperty *definition = blender::bke::idprop::create_group(kBoneIKDefinitionProperty).release();
  add_int(definition, "schema_version", kBoneIKDefinitionSchemaVersion);
  add_int(definition, "ik_bone_count", ik_count);

  IDProperty *ik_bones = IDP_NewIDPArray("ik_bones");
  for (const PMXBone &bone : model.bones) {
    if (!(bone.flag & BONE_FLAG_IK)) {
      continue;
    }
    IDProperty *item = blender::bke::idprop::create_group("ik_bone").release();
    add_string(item, "name", bone.name_local);
    add_string(item, "target", bone_name_at(model, bone.ik_target_index));
    add_int(item, "loop_count", bone.ik_loop_count);
    add_float(item, "angle_limit", bone.ik_angle_limit);

    IDProperty *links = IDP_NewIDPArray("links");
    for (const PMXIKLink &link : bone.ik_links) {
      IDProperty *link_item = blender::bke::idprop::create_group("link").release();
      add_string(link_item, "bone", bone_name_at(model, link.bone_index));
      add_bool(link_item, "limit_angle", link.limit_angle);
      add_bool(link_item,
               "physics_owned",
               link.bone_index >= 0 && link.bone_index < int(physics_owned_bones.size()) &&
                   physics_owned_bones[link.bone_index]);
      std::array<float, 3> min_v, max_v;
      min_v = {link.limit_min[0], link.limit_min[1], link.limit_min[2]};
      max_v = {link.limit_max[0], link.limit_max[1], link.limit_max[2]};
      add_vec3(link_item, "limit_min", min_v);
      add_vec3(link_item, "limit_max", max_v);
      append_group(links, link_item);
    }
    add_property(item, links);
    append_group(ik_bones, item);
  }
  add_property(definition, ik_bones);

  return definition;
}

/** Replace (or insert) the named definition on an ID's system properties. */
static void write_definition_to_id(ID *id, IDProperty *definition)
{
  if (!id || !definition) {
    return;
  }
  IDProperty *properties = IDP_ID_system_properties_ensure(id);
  IDProperty *old = IDP_GetPropertyFromGroup_null(properties, kBoneIKDefinitionProperty);
  if (old) {
    IDP_FreeFromGroup(properties, old);
  }
  IDP_AddToGroup(properties, definition);
}

}  // namespace

void persist_bone_ik_definition(PMXImportContext &ctx, const PMXModel &model)
{
  if (!ctx.model_collection && !ctx.armature_obj) {
    return;
  }

  int ik_count = 0;
  IDProperty *definition = build_ik_definition_property(model, ik_count);

  /* Armature object: read point for the VMD importer / E solver. Copy first so
   * the original `definition` is still valid regardless of which ID consumes it. */
  IDProperty *arm_copy = ctx.armature_obj ? IDP_CopyProperty(definition) : nullptr;

  /* Model collection: model-level data base (mirrors the physics definition). */
  if (ctx.model_collection) {
    write_definition_to_id(&ctx.model_collection->id, definition);
  }
  else {
    IDP_FreeProperty(definition);
  }
  if (arm_copy) {
    write_definition_to_id(&ctx.armature_obj->id, arm_copy);
  }

  /* E1: Write mmd_native_ik_enabled on every IK bone (default: true). */
  if (ctx.armature_obj) {
    bArmature *arm = id_cast<bArmature *>(ctx.armature_obj->data);
    int native_ik_count = 0;
    for (const auto &bone : model.bones) {
      if (!(bone.flag & BONE_FLAG_IK)) { continue; }
      Bone *bl_bone = BKE_armature_find_bone_name(arm, bone.name_local.c_str());
      if (bl_bone) {
        blender::mmd::mmd_native_ik_set_enabled(*bl_bone, true);
        native_ik_count++;
      }
    }
    if (native_ik_count > 0 && ctx.reports) {
      BKE_reportf(ctx.reports,
                  RPT_INFO,
                  "MMD native IK toggle initialized on %d IK bone(s)",
                  native_ik_count);
    }

    /* V8 数据契约：持久化 PMX global_scale 到 armature system_properties。
     * V8 求解器读取 base_pos 时需要还原 PMX 模型空间坐标
     * (base_pos_mmd = arm_head / global_scale + YZ swap)。缺失时回退 0.08。 */
    if (ctx.params != nullptr) {
      IDProperty *arm_props = IDP_ID_system_properties_ensure(&ctx.armature_obj->id);
      IDProperty *old_scale = IDP_GetPropertyFromGroup_null(arm_props, "mmd_pmx_global_scale");
      if (old_scale) {
        IDP_FreeFromGroup(arm_props, old_scale);
      }
      add_float(arm_props, "mmd_pmx_global_scale", ctx.params->global_scale);
    }
  }

  if (ctx.reports) {
    BKE_reportf(ctx.reports,
                RPT_INFO,
                "PMX IK definition persisted (schema %d): %d IK bone(s)",
                kBoneIKDefinitionSchemaVersion,
                ik_count);
  }
}

bool read_bone_ik_definition(const ID &owner, PMXBoneIKDefinitionSet &r_def)
{
  const IDProperty *props = owner.system_properties;
  if (!props) {
    return false;
  }
  const IDProperty *definition = IDP_GetPropertyFromGroup_null(props, kBoneIKDefinitionProperty);
  if (!definition) {
    return false;
  }

  IDProperty *schema_prop = IDP_GetPropertyTypeFromGroup(definition, "schema_version", IDP_INT);
  if (!schema_prop) {
    return false;
  }
  r_def.schema_version = IDP_int_get(schema_prop);

  IDProperty *ik_bones = IDP_GetPropertyTypeFromGroup(definition, "ik_bones", IDP_IDPARRAY);
  if (!ik_bones) {
    return is_supported_schema_version(r_def.schema_version);
  }

  for (int i = 0; i < ik_bones->len; i++) {
    IDProperty *item = IDP_GetIndexArray(ik_bones, i);
    if (!item || item->type != IDP_GROUP) {
      continue;
    }
    PMXBoneIKDefinition def;
    auto get_str = [](const IDProperty *group, const char *name, std::string &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(group, name, IDP_STRING);
      if (p) {
        out = IDP_string_get(p);
      }
    };
    auto get_int = [](const IDProperty *group, const char *name, int &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(group, name, IDP_INT);
      if (p) {
        out = IDP_int_get(p);
      }
    };
    auto get_float = [](const IDProperty *group, const char *name, float &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(group, name, IDP_FLOAT);
      if (p) {
        out = IDP_float_get(p);
      }
    };
    get_str(item, "name", def.bone_name);
    get_str(item, "target", def.target_name);
    get_int(item, "loop_count", def.loop_count);
    get_float(item, "angle_limit", def.angle_limit);

    IDProperty *links = IDP_GetPropertyTypeFromGroup(item, "links", IDP_IDPARRAY);
    if (links) {
      for (int li = 0; li < links->len; li++) {
        IDProperty *link_item = IDP_GetIndexArray(links, li);
        if (!link_item || link_item->type != IDP_GROUP) {
          continue;
        }
        PMXBoneIKLink link;
        get_str(link_item, "bone", link.bone_name);
        IDProperty *limit_prop = IDP_GetPropertyTypeFromGroup(link_item, "limit_angle", IDP_BOOLEAN);
        if (limit_prop) {
          link.limit_angle = IDP_bool_get(limit_prop);
        }
        IDProperty *physics_owned_prop = IDP_GetPropertyTypeFromGroup(
            link_item, "physics_owned", IDP_BOOLEAN);
        if (physics_owned_prop) {
          link.physics_owned = IDP_bool_get(physics_owned_prop);
        }
        auto get_vec3 = [&](const char *name, std::array<float, 3> &out) {
          IDProperty *arr = IDP_GetPropertyTypeFromGroup(link_item, name, IDP_ARRAY);
          if (arr && arr->len == 3 && arr->subtype == IDP_FLOAT) {
            const float *vals = IDP_array_float_get(arr);
            if (vals) {
              out = {vals[0], vals[1], vals[2]};
            }
          }
        };
        get_vec3("limit_min", link.limit_min);
        get_vec3("limit_max", link.limit_max);
        def.links.push_back(std::move(link));
      }
    }
    r_def.ik_bones.push_back(std::move(def));
  }

  return is_supported_schema_version(r_def.schema_version);
}

}  // namespace blender::io::pmx
