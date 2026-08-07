/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_ID.h"
#include "DNA_object_types.h"
#include "DNA_action_types.h"
#include "DNA_collection_types.h"
#include "DNA_constraint_types.h"

#include "BKE_constraint.h"
#include "BKE_idprop.hh"
#include "BKE_report.hh"

#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include "intern/pmx_types.h"
#include "pmx_import_bone_axis.hh"
#include "pmx_import_mesh.hh"

#include <cmath>
#include <string>

namespace blender::io::pmx {
namespace {

constexpr char kBoneAxisDefinitionProperty[] = "mmd_pmx_bone_axis_definition";
constexpr int kBoneAxisDefinitionSchemaVersion = 1;

constexpr float kAxisZeroTol = 1e-4f;
constexpr float kAxisUnitTol = 1e-3f;

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

/**
 * Build the schema-1 axis / deform definition IDProperty group from the parsed
 * PMX model. Only bones carrying a D3-relevant flag (FIXED_AXIS / LOCAL_AXIS /
 * PHYSICS_AFTER_DEF) are included. All geometry fields are stored verbatim.
 */
static IDProperty *build_axis_definition_property(const PMXModel &model, int &r_axis_count)
{
  int axis_count = 0;
  for (const PMXBone &bone : model.bones) {
    if (bone.flag & (BONE_FLAG_FIXED_AXIS | BONE_FLAG_LOCAL_AXIS | BONE_FLAG_PHYSICS_AFTER_DEF)) {
      axis_count++;
    }
  }
  r_axis_count = axis_count;

  IDProperty *definition = blender::bke::idprop::create_group(kBoneAxisDefinitionProperty).release();
  add_int(definition, "schema_version", kBoneAxisDefinitionSchemaVersion);
  add_int(definition, "axis_bone_count", axis_count);

  int axis_bone_idx = 0;
  for (const PMXBone &bone : model.bones) {
    const bool fixed = (bone.flag & BONE_FLAG_FIXED_AXIS) != 0;
    const bool local = (bone.flag & BONE_FLAG_LOCAL_AXIS) != 0;
    const bool deform = (bone.flag & BONE_FLAG_PHYSICS_AFTER_DEF) != 0;
    if (!fixed && !local && !deform) {
      continue;
    }

    /* Each child group needs a UNIQUE IDProperty name: IDProperty's children_map
     * de-duplicates by name, so identical "axis_bone" keys would silently drop all
     * but the first item (and free the rest). The bone's real name lives in the
     * `name` sub-field and is read back from there, not from this key. */
    const std::string item_name = "axis_bone_" + std::to_string(axis_bone_idx);
    axis_bone_idx++;
    IDProperty *item = blender::bke::idprop::create_group(item_name.c_str()).release();
    add_string(item, "name", bone.name_local);
    add_bool(item, "has_fixed_axis", fixed);
    add_float(item, "fixed_axis_0", bone.fixed_axis[0]);
    add_float(item, "fixed_axis_1", bone.fixed_axis[1]);
    add_float(item, "fixed_axis_2", bone.fixed_axis[2]);
    add_bool(item, "has_local_axis", local);
    add_float(item, "local_x_0", bone.local_x[0]);
    add_float(item, "local_x_1", bone.local_x[1]);
    add_float(item, "local_x_2", bone.local_x[2]);
    add_float(item, "local_z_0", bone.local_z[0]);
    add_float(item, "local_z_1", bone.local_z[1]);
    add_float(item, "local_z_2", bone.local_z[2]);
    add_bool(item, "deform_after_physics", deform);
    add_int(item, "transform_order", bone.transform_order);
    add_property(definition, item);
  }

  return definition;
}

/** Replace (or insert) the named definition on an ID's system properties. */
static void write_definition_to_id(ID *id, IDProperty *definition)
{
  if (!id || !definition) {
    return;
  }
  IDProperty *properties = IDP_ID_system_properties_ensure(id);
  IDProperty *old = IDP_GetPropertyFromGroup_null(properties, kBoneAxisDefinitionProperty);
  if (old) {
    IDP_FreeFromGroup(properties, old);
  }
  /* Transfer ownership of `definition` into the ID's system properties; it is
   * released when the owning ID is freed (e.g. BKE_main_free in tests). Do NOT
   * copy here — a copy would orphan the original `definition` and leak it, the
   * same bug that made io_pmx fail the test runner's leak check. */
  IDP_AddToGroup(properties, definition);
}

}  // namespace

void persist_bone_axis_definition(PMXImportContext &ctx, const PMXModel &model)
{
  if (!ctx.model_collection && !ctx.armature_obj) {
    return;
  }

  int axis_count = 0;
  IDProperty *definition = build_axis_definition_property(model, axis_count);

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
                "PMX axis/deform definition persisted (schema %d): %d axis bone(s)",
                kBoneAxisDefinitionSchemaVersion,
                axis_count);
    /* Q4: summarize the deform layer (PHYSICS_AFTER_DEF + transform_order) so the
     * operator / E-phase solver has a precise record. No behavior is applied here. */
    for (const PMXBone &bone : model.bones) {
      if (bone.flag & BONE_FLAG_PHYSICS_AFTER_DEF) {
        BKE_reportf(ctx.reports,
                    RPT_INFO,
                    "PMX deform layer: bone '%s' deform_after_physics=true, transform_order=%d",
                    bone.name_local.c_str(),
                    bone.transform_order);
      }
    }
  }
}

bool read_bone_axis_definition(const ID &owner, PMXBoneAxisDefinitionSet &r_def)
{
  const IDProperty *props = owner.system_properties;
  if (!props) {
    return false;
  }
  const IDProperty *definition = IDP_GetPropertyFromGroup_null(props, kBoneAxisDefinitionProperty);
  if (!definition) {
    return false;
  }

  IDProperty *schema_prop = IDP_GetPropertyTypeFromGroup(definition, "schema_version", IDP_INT);
  if (!schema_prop) {
    return false;
  }
  r_def.schema_version = IDP_int_get(schema_prop);

  /* `axis_bone` items are stored as direct children of `definition`.
   * Classic C-style iteration over the ListBase (ListBaseTIterator full definition is
   * not pulled in here; this matches the rest of the pmx importer's iteration style). */
  for (IDProperty *item = static_cast<IDProperty *>(definition->data.group.first); item != nullptr;
       item = item->next) {
    if (item->type != IDP_GROUP) {
      continue;
    }
    PMXBoneAxisDefinition def;
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
    auto get_bool = [&](const char *name, bool &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_BOOLEAN);
      if (p) {
        out = IDP_bool_get(p);
      }
    };
    auto get_f = [&](const char *name, float &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_FLOAT);
      if (p) {
        out = IDP_float_get(p);
      }
    };
    get_str("name", def.bone_name);
    get_bool("has_fixed_axis", def.has_fixed_axis);
    get_f("fixed_axis_0", def.fixed_axis[0]);
    get_f("fixed_axis_1", def.fixed_axis[1]);
    get_f("fixed_axis_2", def.fixed_axis[2]);
    get_bool("has_local_axis", def.has_local_axis);
    get_f("local_x_0", def.local_x[0]);
    get_f("local_x_1", def.local_x[1]);
    get_f("local_x_2", def.local_x[2]);
    get_f("local_z_0", def.local_z[0]);
    get_f("local_z_1", def.local_z[1]);
    get_f("local_z_2", def.local_z[2]);
    get_bool("deform_after_physics", def.deform_after_physics);
    get_int("transform_order", def.transform_order);

    r_def.bones.push_back(std::move(def));
  }

  return r_def.schema_version == kBoneAxisDefinitionSchemaVersion;
}

int pmx_fixed_axis_principal_index(const float axis[3])
{
  int nonzero = 0;
  int idx = -1;
  for (int i = 0; i < 3; i++) {
    if (std::fabs(axis[i]) > kAxisZeroTol) {
      nonzero++;
      idx = i;
      if (std::fabs(std::fabs(axis[i]) - 1.0f) > kAxisUnitTol) {
        /* Non-unit component -> not a principal axis. */
        return -1;
      }
    }
  }
  return nonzero == 1 ? idx : -1;
}

bool pmx_apply_fixed_axis_to_pchan(Object *ob,
                                   bPoseChannel *pchan,
                                   const PMXBoneAxisDefinition &def,
                                   ReportList *reports)
{
  if (!ob || !pchan || !def.has_fixed_axis) {
    return false;
  }
  /* Idempotency: skip bones already flagged as applied. */
  if (pchan->system_properties != nullptr) {
    IDProperty *p = IDP_GetPropertyTypeFromGroup(pchan->system_properties, "mmd_fixed_axis_applied", IDP_BOOLEAN);
    if (p != nullptr && IDP_bool_get(p)) {
      return false;
    }
  }

  const int principal = pmx_fixed_axis_principal_index(def.fixed_axis);
  if (principal >= 0) {
    /* Exact: lock the two non-principal Euler axes via native protectflag. */
    pchan->protectflag |= OB_LOCK_ROTX | OB_LOCK_ROTY | OB_LOCK_ROTZ;
    if (principal == 0) {
      pchan->protectflag &= ~OB_LOCK_ROTX;
    }
    else if (principal == 1) {
      pchan->protectflag &= ~OB_LOCK_ROTY;
    }
    else {
      pchan->protectflag &= ~OB_LOCK_ROTZ;
    }
  }
  else {
    /* Approximate: Limit Rotation constraint in LOCAL space. Lock the two Euler
     * axes least aligned with the fixed axis, free the dominant one. */
    bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, "MMD_Fixed_Axis_Approx", CONSTRAINT_TYPE_ROTLIMIT);
    if (con == nullptr) {
      return false;
    }
    bRotLimitConstraint *lim = static_cast<bRotLimitConstraint *>(con->data);
    const int dominant = (std::fabs(def.fixed_axis[0]) >= std::fabs(def.fixed_axis[1]) &&
                                  std::fabs(def.fixed_axis[0]) >= std::fabs(def.fixed_axis[2])) ?
                             0 :
                             (std::fabs(def.fixed_axis[1]) >= std::fabs(def.fixed_axis[2])) ? 1 : 2;
    lim->flag = eRotLimit_Flags(0);
    for (int i = 0; i < 3; i++) {
      if (i == dominant) {
        if (i == 0) {
          lim->flag = eRotLimit_Flags(int(lim->flag) & ~LIMIT_XROT);
        }
        else if (i == 1) {
          lim->flag = eRotLimit_Flags(int(lim->flag) & ~LIMIT_YROT);
        }
        else {
          lim->flag = eRotLimit_Flags(int(lim->flag) & ~LIMIT_ZROT);
        }
      }
      else {
        if (i == 0) {
          lim->flag = eRotLimit_Flags(int(lim->flag) | LIMIT_XROT);
          lim->xmin = 0.0f;
          lim->xmax = 0.0f;
        }
        else if (i == 1) {
          lim->flag = eRotLimit_Flags(int(lim->flag) | LIMIT_YROT);
          lim->ymin = 0.0f;
          lim->ymax = 0.0f;
        }
        else {
          lim->flag = eRotLimit_Flags(int(lim->flag) | LIMIT_ZROT);
          lim->zmin = 0.0f;
          lim->zmax = 0.0f;
        }
      }
    }
    con->ownspace = CONSTRAINT_SPACE_LOCAL;
    con->enforce = 1.0f;
  }

  /* Mark the pose bone so the approximate nature is queryable (red line D3-a). */
  if (principal < 0) {
    IDProperty *pchan_props = pchan->system_properties;
    if (pchan_props == nullptr) {
      pchan_props = blender::bke::idprop::create_group("mmd_fixed_axis").release();
      pchan->system_properties = pchan_props;
    }
    IDProperty *old = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_approximate");
    if (old != nullptr) {
      IDP_FreeFromGroup(pchan_props, old);
    }
    IDP_AddToGroup(pchan_props, blender::bke::idprop::create_bool("mmd_approximate", true).release());
  }
  IDProperty *pchan_props = pchan->system_properties;
  if (pchan_props == nullptr) {
    pchan_props = blender::bke::idprop::create_group("mmd_fixed_axis").release();
    pchan->system_properties = pchan_props;
  }
  IDProperty *old_applied = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_fixed_axis_applied");
  if (old_applied != nullptr) {
    IDP_FreeFromGroup(pchan_props, old_applied);
  }
  IDP_AddToGroup(pchan_props, blender::bke::idprop::create_bool("mmd_fixed_axis_applied", true).release());

  if (reports) {
    if (principal >= 0) {
      BKE_reportf(reports,
                  RPT_INFO,
                  "PMX Fixed Axis: bone '%s' principal axis -> native lock_rotation (exact)",
                  def.bone_name.c_str());
    }
    else {
      BKE_report(reports,
                 RPT_WARNING,
                 "Applied Approximate Fixed Axis constraint. Rotation near Euler gimbal lock angles "
                 "may show artifacts. Not equivalent to MMD native implementation.");
    }
  }
  return true;
}

bool pmx_apply_local_axis_to_pchan(Object *ob,
                                   bPoseChannel *pchan,
                                   const PMXBoneAxisDefinition &def,
                                   ReportList *reports)
{
  if (!ob || !pchan || !def.has_local_axis) {
    return false;
  }
  /* Idempotency: skip bones already flagged as applied. */
  if (pchan->system_properties != nullptr) {
    IDProperty *p = IDP_GetPropertyTypeFromGroup(pchan->system_properties, "mmd_local_axis_applied", IDP_BOOLEAN);
    if (p != nullptr && IDP_bool_get(p)) {
      return false;
    }
  }

  /* Approximate: Transformation constraint marking the custom local frame.
   * A single Transformation constraint cannot perform true basis conjugation
   * (that would require modifying the bone matrix, forbidden by red line D3-b),
   * so this is an explicit, user-controllable approximation. */
  bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, "MMD_Local_Axis_Approx", CONSTRAINT_TYPE_TRANSFORM);
  if (con == nullptr) {
    return false;
  }
  bTransformConstraint *tcon = static_cast<bTransformConstraint *>(con->data);
  STRNCPY_UTF8(tcon->subtarget, def.bone_name.c_str());
  tcon->from = TRANS_ROTATION;
  tcon->to = TRANS_ROTATION;
  tcon->map[0] = 0;
  tcon->map[1] = 1;
  tcon->map[2] = 2;
  tcon->expo = 0;
  tcon->to_euler_order = CONSTRAINT_EULER_XYZ;
  constexpr float HALF = 3.14159265f;
  for (int i = 0; i < 3; i++) {
    tcon->from_min_rot[i] = -HALF;
    tcon->from_max_rot[i] = HALF;
    tcon->to_min_rot[i] = -HALF;
    tcon->to_max_rot[i] = HALF;
  }
  tcon->mix_mode_rot = TRANS_MIXROT_ADD;
  con->ownspace = CONSTRAINT_SPACE_LOCAL;
  con->tarspace = CONSTRAINT_SPACE_LOCAL;
  con->enforce = 1.0f;

  /* Mark the pose bone so the approximate nature is queryable (red line D3-b). */
  IDProperty *pchan_props = pchan->system_properties;
  if (pchan_props == nullptr) {
    pchan_props = blender::bke::idprop::create_group("mmd_local_axis").release();
    pchan->system_properties = pchan_props;
  }
  IDProperty *old_appr = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_approximate");
  if (old_appr != nullptr) {
    IDP_FreeFromGroup(pchan_props, old_appr);
  }
  IDP_AddToGroup(pchan_props, blender::bke::idprop::create_bool("mmd_approximate", true).release());
  IDProperty *old_applied = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_local_axis_applied");
  if (old_applied != nullptr) {
    IDP_FreeFromGroup(pchan_props, old_applied);
  }
  IDP_AddToGroup(pchan_props, blender::bke::idprop::create_bool("mmd_local_axis_applied", true).release());

  if (reports) {
    BKE_report(reports,
               RPT_WARNING,
               "Local Axis is an approximation. Conflict with baked VMD rotation possible. Verify and "
               "bake animation if needed.");
  }
  return true;
}

}  // namespace blender::io::pmx
