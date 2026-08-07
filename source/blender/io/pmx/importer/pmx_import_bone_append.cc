/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_ID.h"
#include "DNA_object_types.h"
#include "DNA_collection_types.h"

#include "BKE_idprop.hh"
#include "BKE_report.hh"

#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include "intern/pmx_types.h"
#include "pmx_import_bone_append.hh"
#include "pmx_import_mesh.hh"

#include <cmath>
#include <string>

namespace blender::io::pmx {
namespace {

constexpr char kBoneAppendDefinitionProperty[] = "mmd_pmx_bone_append_definition";
constexpr int kBoneAppendDefinitionSchemaVersion = 1;

constexpr int kAppendModeRotation = 1;
constexpr int kAppendModeTranslation = 2;

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
 * Build the schema-1 append-transform definition IDProperty group from the
 * parsed PMX model. Only bones with APPEND_ROTATION / APPEND_TRANSLATE are
 * included. `ratio` is stored verbatim (sign preserved, including negatives).
 */
static IDProperty *build_append_definition_property(const PMXModel &model, int &r_append_count)
{
  int append_count = 0;
  for (const PMXBone &bone : model.bones) {
    if (bone.flag & (BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE)) {
      append_count++;
    }
  }
  r_append_count = append_count;

  IDProperty *definition = blender::bke::idprop::create_group(kBoneAppendDefinitionProperty)
                               .release();
  add_int(definition, "schema_version", kBoneAppendDefinitionSchemaVersion);
  add_int(definition, "append_bone_count", append_count);

  IDProperty *append_bones = IDP_NewIDPArray("append_bones");
  for (const PMXBone &bone : model.bones) {
    const bool rot = (bone.flag & BONE_FLAG_APPEND_ROTATION) != 0;
    const bool trans = (bone.flag & BONE_FLAG_APPEND_TRANSLATE) != 0;
    if (!rot && !trans) {
      continue;
    }
    IDProperty *item = blender::bke::idprop::create_group("append_bone").release();
    int mode = 0;
    if (rot) {
      mode |= kAppendModeRotation;
    }
    if (trans) {
      mode |= kAppendModeTranslation;
    }
    add_string(item, "name", bone.name_local);
    add_int(item, "mode", mode);
    add_string(item, "parent", bone_name_at(model, bone.inherit_parent_index));
    /* Store the raw ratio including its sign (negative = "cancel"). */
    add_float(item, "ratio", bone.inherit_parent_ratio);
    append_group(append_bones, item);
  }
  add_property(definition, append_bones);

  return definition;
}

/** Replace (or insert) the named definition on an ID's system properties. */
static void write_definition_to_id(ID *id, IDProperty *definition)
{
  if (!id || !definition) {
    return;
  }
  IDProperty *properties = IDP_ID_system_properties_ensure(id);
  IDProperty *old = IDP_GetPropertyFromGroup_null(properties, kBoneAppendDefinitionProperty);
  if (old) {
    IDP_FreeFromGroup(properties, old);
  }
  IDP_AddToGroup(properties, definition);
}

}  // namespace

void persist_bone_append_definition(PMXImportContext &ctx, const PMXModel &model)
{
  if (!ctx.model_collection && !ctx.armature_obj) {
    return;
  }

  int append_count = 0;
  IDProperty *definition = build_append_definition_property(model, append_count);

  /* Armature object: read point for the editor operator / E solver. Copy first
   * so the original `definition` is still valid regardless of which ID consumes it. */
  IDProperty *arm_copy = ctx.armature_obj ? IDP_CopyProperty(definition) : nullptr;

  if (ctx.model_collection) {
    write_definition_to_id(&ctx.model_collection->id, definition);
  }
  else {
    IDP_FreeProperty(definition);
  }
  if (arm_copy) {
    write_definition_to_id(&ctx.armature_obj->id, arm_copy);
  }

  if (ctx.reports) {
    BKE_reportf(ctx.reports,
                RPT_INFO,
                "PMX append-transform definition persisted (schema %d): %d append bone(s)",
                kBoneAppendDefinitionSchemaVersion,
                append_count);
  }
}

bool read_bone_append_definition(const ID &owner, PMXBoneAppendDefinitionSet &r_def)
{
  const IDProperty *props = owner.system_properties;
  if (!props) {
    return false;
  }
  const IDProperty *definition = IDP_GetPropertyFromGroup_null(props,
                                                                kBoneAppendDefinitionProperty);
  if (!definition) {
    return false;
  }

  IDProperty *schema_prop = IDP_GetPropertyTypeFromGroup(definition, "schema_version", IDP_INT);
  if (!schema_prop) {
    return false;
  }
  r_def.schema_version = IDP_int_get(schema_prop);

  IDProperty *append_bones = IDP_GetPropertyTypeFromGroup(definition, "append_bones", IDP_IDPARRAY);
  if (!append_bones) {
    return r_def.schema_version == kBoneAppendDefinitionSchemaVersion;
  }

  for (int i = 0; i < append_bones->len; i++) {
    IDProperty *item = IDP_GetIndexArray(append_bones, i);
    if (!item || item->type != IDP_GROUP) {
      continue;
    }
    PMXBoneAppendDefinition def;
    auto get_str = [&](const char *name, std::string &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_STRING);
      if (p) {
        out = IDP_string_get(p);
      }
    };
    auto get_int = [&](const char *name, int &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_INT);
      if (p) {
        out = IDP_int_get(p);
      }
    };
    auto get_float = [&](const char *name, float &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_FLOAT);
      if (p) {
        out = IDP_float_get(p);
      }
    };
    get_str("name", def.bone_name);
    get_int("mode", def.mode);
    get_str("parent", def.parent_name);
    get_float("ratio", def.ratio);

    r_def.append_bones.push_back(std::move(def));
  }

  return r_def.schema_version == kBoneAppendDefinitionSchemaVersion;
}

}  // namespace blender::io::pmx
