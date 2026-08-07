/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_ID.h"
#include "DNA_object_types.h"
#include "DNA_action_types.h"
#include "DNA_armature_types.h"

#include "BKE_action.hh"
#include "BKE_idprop.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph.hh"

#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include "intern/pmx_types.h"
#include "pmx_import_pose_snapshot.hh"
#include "pmx_import_bone_ik.hh"
#include "pmx_import_bone_append.hh"

#include <set>
#include <string>

namespace blender::io::pmx {
namespace {

constexpr char kPoseSnapshotProperty[] = "mmd_pose_snapshot";
constexpr char kPoseDoneProperty[] = "mmd_pose_done";
constexpr int kPoseSnapshotSchemaVersion = 1;

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
 * Build the schema-1 POSE_DONE snapshot group from the armature's currently
 * solved pose. Pure read of `pose_mat` / `loc` / `quat`; `has_ik` / `has_append`
 * are looked up read-only from the D1/D2 persisted definitions on the same
 * armature. Child bone groups use unique names `pose_bone_<idx>` (IDProperty
 * de-duplicates siblings by name — identical keys would silently drop all but
 * the first, the same bug that broke D3 round-trip).
 */
static IDProperty *build_pose_snapshot_property(const Object *armature, int &r_bone_count)
{
  bPose *pose = armature->pose;
  int bone_count = 0;
  if (pose != nullptr) {
    for (bPoseChannel *pchan = static_cast<bPoseChannel *>(pose->chanbase.first); pchan != nullptr;
         pchan = pchan->next) {
      bone_count++;
    }
  }
  r_bone_count = bone_count;

  IDProperty *snapshot = blender::bke::idprop::create_group(kPoseSnapshotProperty).release();
  add_int(snapshot, "schema_version", kPoseSnapshotSchemaVersion);
  add_int(snapshot, "bone_count", bone_count);
  add_int(snapshot, "captured_at_frame", 0); /* informational; real frame set by E if needed */

  /* has_ik / has_append flags, read-only from D1/D2 persisted definitions. */
  std::set<std::string> ik_bones;
  std::set<std::string> append_bones;
  {
    PMXBoneIKDefinitionSet ik_def;
    if (read_bone_ik_definition(armature->id, ik_def)) {
      for (const PMXBoneIKDefinition &d : ik_def.ik_bones) {
        ik_bones.insert(d.bone_name);
      }
    }
    PMXBoneAppendDefinitionSet ap_def;
    if (read_bone_append_definition(armature->id, ap_def)) {
      for (const PMXBoneAppendDefinition &d : ap_def.append_bones) {
        append_bones.insert(d.bone_name);
      }
    }
  }

  if (pose != nullptr) {
    int idx = 0;
    for (bPoseChannel *pchan = static_cast<bPoseChannel *>(pose->chanbase.first); pchan != nullptr;
         pchan = pchan->next, idx++) {
      const std::string item_name = "pose_bone_" + std::to_string(idx);
      IDProperty *item = blender::bke::idprop::create_group(item_name.c_str()).release();
      add_string(item, "name", pchan->name);
      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
          add_float(item, ("pose_mat_" + std::to_string(r * 4 + c)).c_str(), pchan->pose_mat[r][c]);
        }
      }
      add_float(item, "loc_0", pchan->loc[0]);
      add_float(item, "loc_1", pchan->loc[1]);
      add_float(item, "loc_2", pchan->loc[2]);
      add_float(item, "quat_0", pchan->quat[0]);
      add_float(item, "quat_1", pchan->quat[1]);
      add_float(item, "quat_2", pchan->quat[2]);
      add_float(item, "quat_3", pchan->quat[3]);
      add_bool(item, "has_ik", ik_bones.count(pchan->name) > 0);
      add_bool(item, "has_append", append_bones.count(pchan->name) > 0);
      add_property(snapshot, item);
    }
  }

  return snapshot;
}

/** Replace (or insert) the named snapshot on an ID's system properties. */
static void write_definition_to_id(ID *id, IDProperty *definition)
{
  if (!id || !definition) {
    return;
  }
  IDProperty *properties = IDP_ID_system_properties_ensure(id);
  IDProperty *old = IDP_GetPropertyFromGroup_null(properties, kPoseSnapshotProperty);
  if (old) {
    IDP_FreeFromGroup(properties, old);
  }
  /* Transfer ownership of `definition` into the ID's system properties; it is
   * released when the owning ID is freed (e.g. BKE_main_free in tests). Do NOT
   * copy here — a copy would orphan the original `definition` and leak it (the
   * bug that made io_pmx fail the test runner's leak check during D3). */
  IDP_AddToGroup(properties, definition);
}

}  // namespace

void mmd_capture_pose_snapshot(Object *armature, ReportList *reports)
{
  if (armature == nullptr || armature->type != OB_ARMATURE) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "mmd_capture_pose_snapshot: object is not an armature");
    }
    return;
  }
  if (armature->pose == nullptr) {
    if (reports) {
      BKE_report(reports,
                 RPT_ERROR,
                 "mmd_capture_pose_snapshot: armature has no Pose (ensure pose before capture)");
    }
    return;
  }

  int bone_count = 0;
  IDProperty *snapshot = build_pose_snapshot_property(armature, bone_count);
  write_definition_to_id(&armature->id, snapshot);

  /* Set the POSE_DONE logical gate (red line D4-a: import/playback never sets it). */
  IDProperty *props = IDP_ID_system_properties_ensure(&armature->id);
  IDProperty *old_done = IDP_GetPropertyFromGroup_null(props, kPoseDoneProperty);
  if (old_done) {
    IDP_FreeFromGroup(props, old_done);
  }
  IDP_AddToGroup(props, blender::bke::idprop::create_bool(kPoseDoneProperty, true).release());

  if (reports) {
    BKE_reportf(reports,
                RPT_INFO,
                "PMX POSE_DONE snapshot captured (schema %d): %d bone(s)",
                kPoseSnapshotSchemaVersion,
                bone_count);
  }
}

bool read_pose_snapshot(const ID &owner, PMXPoseSnapshot &r_snapshot)
{
  const IDProperty *props = owner.system_properties;
  if (props == nullptr) {
    return false;
  }
  const IDProperty *snapshot = IDP_GetPropertyFromGroup_null(props, kPoseSnapshotProperty);
  if (snapshot == nullptr) {
    return false;
  }

  IDProperty *schema_prop = IDP_GetPropertyTypeFromGroup(snapshot, "schema_version", IDP_INT);
  if (schema_prop == nullptr) {
    return false;
  }
  r_snapshot.schema_version = IDP_int_get(schema_prop);

  IDProperty *count_prop = IDP_GetPropertyTypeFromGroup(snapshot, "bone_count", IDP_INT);
  if (count_prop != nullptr) {
    r_snapshot.bone_count = IDP_int_get(count_prop);
  }
  IDProperty *frame_prop = IDP_GetPropertyTypeFromGroup(snapshot, "captured_at_frame", IDP_INT);
  if (frame_prop != nullptr) {
    r_snapshot.captured_at_frame = IDP_int_get(frame_prop);
  }

  for (IDProperty *item = static_cast<IDProperty *>(snapshot->data.group.first); item != nullptr;
       item = item->next) {
    if (item->type != IDP_GROUP) {
      continue;
    }
    PMXPoseBoneSnapshot bs;
    auto get_str = [&](const char *name, std::string &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_STRING);
      if (p) {
        out = IDP_string_get(p);
      }
    };
    auto get_f = [&](const char *name, float &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_FLOAT);
      if (p) {
        out = IDP_float_get(p);
      }
    };
    auto get_bool = [&](const char *name, bool &out) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(item, name, IDP_BOOLEAN);
      if (p) {
        out = IDP_bool_get(p);
      }
    };
    get_str("name", bs.name);
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 4; c++) {
        get_f(("pose_mat_" + std::to_string(r * 4 + c)).c_str(), bs.pose_mat[r][c]);
      }
    }
    get_f("loc_0", bs.loc[0]);
    get_f("loc_1", bs.loc[1]);
    get_f("loc_2", bs.loc[2]);
    get_f("quat_0", bs.quat[0]);
    get_f("quat_1", bs.quat[1]);
    get_f("quat_2", bs.quat[2]);
    get_f("quat_3", bs.quat[3]);
    get_bool("has_ik", bs.has_ik);
    get_bool("has_append", bs.has_append);

    r_snapshot.bones.push_back(std::move(bs));
  }

  return r_snapshot.schema_version == kPoseSnapshotSchemaVersion;
}

bool mmd_pose_done_snapshot_readback(Depsgraph * /*depsgraph*/, ID *id, PMXPoseSnapshot *r_snapshot)
{
  if (id == nullptr || r_snapshot == nullptr) {
    return false;
  }
  const IDProperty *props = id->system_properties;
  if (props == nullptr) {
    return false;
  }
  IDProperty *done = IDP_GetPropertyTypeFromGroup(props, kPoseDoneProperty, IDP_BOOLEAN);
  if (done == nullptr || !IDP_bool_get(done)) {
    return false;
  }
  return read_pose_snapshot(*id, *r_snapshot);
}

void mmd_physics_pose_writeback(ID *id, const PMXPoseSnapshot *snapshot)
{
  if (id == nullptr || snapshot == nullptr) {
    return;
  }
  Object *ob = reinterpret_cast<Object *>(id);
  if (ob->type != OB_ARMATURE || ob->pose == nullptr) {
    return;
  }
  for (const PMXPoseBoneSnapshot &bs : snapshot->bones) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, bs.name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    pchan->loc[0] = bs.loc[0];
    pchan->loc[1] = bs.loc[1];
    pchan->loc[2] = bs.loc[2];
    pchan->quat[0] = bs.quat[0];
    pchan->quat[1] = bs.quat[1];
    pchan->quat[2] = bs.quat[2];
    pchan->quat[3] = bs.quat[3];
  }
  /* Mark the armature for recompute; E is responsible for the actual eval. */
  DEG_id_tag_update(id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
}

}  // namespace blender::io::pmx
