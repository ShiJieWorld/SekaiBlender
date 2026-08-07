/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include <initializer_list>

#include "BKE_action.hh"
#include "BKE_armature.hh"
#include "BKE_global.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"
#include "BKE_idprop.hh"

#include "BLI_listbase.hh"
#include "BLI_string.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_object_types.h"

#include "intern/pmx_types.h"
#include "pmx_import_pose_snapshot.hh"

#include "MEM_guardedalloc.h"

#include <cmath>
#include <string>

namespace blender::io::pmx::tests {
namespace {

class PMXPoseSnapshotTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *armature_object = nullptr;
  ReportList reports;

  void SetUp() override
  {
    bmain = BKE_main_new();
    bArmature *armature = BKE_armature_add(bmain, "PoseArmature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "PoseArm");
    armature_object->data = id_cast<ID *>(armature);
    BKE_reports_init(&reports, RPT_STORE);
  }

  void TearDown() override
  {
    BKE_reports_free(&reports);
    BKE_main_free(bmain);
  }

  /** Add a real bone so a pose channel exists after `BKE_pose_ensure`. */
  void add_bone(const char *name)
  {
    bArmature *arm = id_cast<bArmature *>(armature_object->data);
    Bone *b = MEM_new<Bone>("PoseTestBone");
    BLI_strncpy(b->name, name, sizeof(b->name));
    BLI_addtail(&arm->bonebase, b);
  }

  void ensure_pose()
  {
    BKE_pose_ensure(bmain,
                    armature_object,
                    id_cast<bArmature *>(armature_object->data),
                    false);
  }

  /** Overwrite a pose channel's solved values with known data (no depsgraph eval in test). */
  void set_channel(const char *name,
                   const float pose_mat[16],
                   std::initializer_list<float> loc,
                   std::initializer_list<float> quat)
  {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature_object->pose, name);
    ASSERT_NE(pchan, nullptr);
    for (int i = 0; i < 16; i++) {
      pchan->pose_mat[i / 4][i % 4] = pose_mat[i];
    }
    float l[3] = {0.0f, 0.0f, 0.0f};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    int li = 0;
    for (float v : loc) {
      if (li < 3) {
        l[li++] = v;
      }
    }
    int qi = 0;
    for (float v : quat) {
      if (qi < 4) {
        q[qi++] = v;
      }
    }
    pchan->loc[0] = l[0];
    pchan->loc[1] = l[1];
    pchan->loc[2] = l[2];
    pchan->quat[0] = q[0];
    pchan->quat[1] = q[1];
    pchan->quat[2] = q[2];
    pchan->quat[3] = q[3];
  }

  /** Mark a bone as IK in the D1 persisted definition (real schema) so capture records has_ik. */
  void mark_ik_bone(const std::string &name)
  {
    IDProperty *props = IDP_ID_system_properties_ensure(&armature_object->id);
    IDProperty *def = IDP_GetPropertyFromGroup_null(props, "mmd_pmx_bone_ik_definition");
    if (!def) {
      def = blender::bke::idprop::create_group("mmd_pmx_bone_ik_definition").release();
      IDP_AddToGroup(def, IDP_NewInt(1, "schema_version"));
      IDP_AddToGroup(props, def); /* ownership transfer */
    }
    IDProperty *arr = IDP_GetPropertyTypeFromGroup(def, "ik_bones", IDP_IDPARRAY);
    if (!arr) {
      arr = IDP_NewIDPArray("ik_bones");
      IDP_AddToGroup(def, arr);
    }
    IDProperty *item = blender::bke::idprop::create_group("ik_bone_x").release();
    IDP_AddToGroup(item, IDP_NewString(name.c_str(), "name"));
    IDP_AppendArray(arr, item); /* shallow copy into array; item now orphaned */
    MEM_delete(item);
  }

  /** Mark a bone as append-transform in the D2 persisted definition (real schema). */
  void mark_append_bone(const std::string &name)
  {
    IDProperty *props = IDP_ID_system_properties_ensure(&armature_object->id);
    IDProperty *def = IDP_GetPropertyFromGroup_null(props, "mmd_pmx_bone_append_definition");
    if (!def) {
      def = blender::bke::idprop::create_group("mmd_pmx_bone_append_definition").release();
      IDP_AddToGroup(def, IDP_NewInt(1, "schema_version"));
      IDP_AddToGroup(props, def);
    }
    IDProperty *arr = IDP_GetPropertyTypeFromGroup(def, "append_bones", IDP_IDPARRAY);
    if (!arr) {
      arr = IDP_NewIDPArray("append_bones");
      IDP_AddToGroup(def, arr);
    }
    IDProperty *item = blender::bke::idprop::create_group("append_bone_x").release();
    IDP_AddToGroup(item, IDP_NewString(name.c_str(), "name"));
    IDP_AppendArray(arr, item);
    MEM_delete(item);
  }

  static const PMXPoseBoneSnapshot *find(const PMXPoseSnapshot &snap, const std::string &name)
  {
    for (const PMXPoseBoneSnapshot &b : snap.bones) {
      if (b.name == name) {
        return &b;
      }
    }
    return nullptr;
  }
};

TEST_F(PMXPoseSnapshotTest, CaptureAndReadsRoundTrip)
{
  add_bone("Root");
  add_bone("IK_Bone");
  add_bone("Append_Bone");
  add_bone("Normal_Bone");
  ensure_pose();

  const float root_mat[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const float ik_mat[16] = {1, 0, 0, 0, 0, 0.5f, -0.5f, 0, 0, 0.5f, 0.5f, 0, 2, 0, 0, 1};
  const float ap_mat[16] = {0.5f, -0.5f, 0, 0, 0.5f, 0.5f, 0, 0, 0, 0, 1, 0, 0, 3, 0, 1};
  const float nm_mat[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  set_channel("Root", root_mat, {1.0f, 2.0f, 3.0f}, {1.0f, 0.0f, 0.0f, 0.0f});
  set_channel("IK_Bone", ik_mat, {0.1f, 0.2f, 0.3f}, {0.5f, 0.5f, 0.5f, 0.5f});
  set_channel("Append_Bone", ap_mat, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f});
  set_channel("Normal_Bone", nm_mat, {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f});

  mark_ik_bone("IK_Bone");
  mark_append_bone("Append_Bone");

  mmd_capture_pose_snapshot(armature_object, &reports);

  PMXPoseSnapshot snap;
  ASSERT_TRUE(read_pose_snapshot(armature_object->id, snap));
  EXPECT_EQ(snap.schema_version, 1);
  EXPECT_EQ(snap.bone_count, 4);
  ASSERT_EQ(snap.bones.size(), 4);

  const PMXPoseBoneSnapshot *root = find(snap, "Root");
  const PMXPoseBoneSnapshot *ik = find(snap, "IK_Bone");
  const PMXPoseBoneSnapshot *ap = find(snap, "Append_Bone");
  const PMXPoseBoneSnapshot *nm = find(snap, "Normal_Bone");
  ASSERT_NE(root, nullptr);
  ASSERT_NE(ik, nullptr);
  ASSERT_NE(ap, nullptr);
  ASSERT_NE(nm, nullptr);

  for (int i = 0; i < 16; i++) {
    EXPECT_FLOAT_EQ(ik->pose_mat[i / 4][i % 4], ik_mat[i]) << "pose_mat[" << i << "]";
  }
  EXPECT_FLOAT_EQ(ik->loc[0], 0.1f);
  EXPECT_FLOAT_EQ(ik->loc[2], 0.3f);
  EXPECT_FLOAT_EQ(ik->quat[0], 0.5f);
  EXPECT_FLOAT_EQ(ik->quat[3], 0.5f);
  EXPECT_FLOAT_EQ(root->loc[1], 2.0f);

  EXPECT_TRUE(ik->has_ik);
  EXPECT_FALSE(ik->has_append);
  EXPECT_FALSE(ap->has_ik);
  EXPECT_TRUE(ap->has_append);
  EXPECT_FALSE(nm->has_ik);
  EXPECT_FALSE(nm->has_append);
}

TEST_F(PMXPoseSnapshotTest, CapturesEmptyArmature)
{
  /* No bones: pose exists but has zero channels. */
  ensure_pose();

  mmd_capture_pose_snapshot(armature_object, &reports);

  PMXPoseSnapshot snap;
  ASSERT_TRUE(read_pose_snapshot(armature_object->id, snap));
  EXPECT_EQ(snap.bone_count, 0);
  EXPECT_TRUE(snap.bones.empty());
  EXPECT_EQ(snap.schema_version, 1);

  /* mmd_pose_done gate must be set. */
  ASSERT_NE(armature_object->id.system_properties, nullptr);
  IDProperty *done = IDP_GetPropertyTypeFromGroup(
      armature_object->id.system_properties, "mmd_pose_done", IDP_BOOLEAN);
  ASSERT_NE(done, nullptr);
  EXPECT_TRUE(IDP_bool_get(done));
}

TEST_F(PMXPoseSnapshotTest, ReadMissingReturnsFalse)
{
  PMXPoseSnapshot snap;
  EXPECT_FALSE(read_pose_snapshot(armature_object->id, snap));
  /* Readback also refuses when no POSE_DONE gate is set. */
  EXPECT_FALSE(mmd_pose_done_snapshot_readback(nullptr, &armature_object->id, &snap));
}

TEST_F(PMXPoseSnapshotTest, ReadbackFillsStruct)
{
  add_bone("IK_Bone");
  add_bone("Append_Bone");
  ensure_pose();

  const float ik_mat[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  set_channel("IK_Bone", ik_mat, {0.5f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f});
  set_channel("Append_Bone", ik_mat, {0.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f, 0.0f});

  mark_ik_bone("IK_Bone");
  mark_append_bone("Append_Bone");

  mmd_capture_pose_snapshot(armature_object, &reports);

  PMXPoseSnapshot snap;
  ASSERT_TRUE(mmd_pose_done_snapshot_readback(nullptr, &armature_object->id, &snap));
  EXPECT_EQ(snap.bone_count, 2);
  ASSERT_EQ(snap.bones.size(), 2);
  /* captured_at_frame is informational; just verify it round-trips as set (0). */
  EXPECT_EQ(snap.captured_at_frame, 0);

  const PMXPoseBoneSnapshot *ik = find(snap, "IK_Bone");
  const PMXPoseBoneSnapshot *ap = find(snap, "Append_Bone");
  ASSERT_NE(ik, nullptr);
  ASSERT_NE(ap, nullptr);
  EXPECT_TRUE(ik->has_ik);
  EXPECT_FALSE(ik->has_append);
  EXPECT_FALSE(ap->has_ik);
  EXPECT_TRUE(ap->has_append);
  EXPECT_FLOAT_EQ(ik->loc[0], 0.5f);
  EXPECT_FLOAT_EQ(ap->loc[1], 0.5f);
}

TEST_F(PMXPoseSnapshotTest, WritebackSetsChannels)
{
  add_bone("Root");
  ensure_pose();

  const float mat[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  set_channel("Root", mat, {1.0f, 2.0f, 3.0f}, {0.5f, 0.5f, 0.5f, 0.5f});

  mmd_capture_pose_snapshot(armature_object, &reports);

  PMXPoseSnapshot snap;
  ASSERT_TRUE(read_pose_snapshot(armature_object->id, snap));

  /* Mutate the live channel away from the snapshot. */
  bPoseChannel *pchan = BKE_pose_channel_find_name(armature_object->pose, "Root");
  ASSERT_NE(pchan, nullptr);
  pchan->loc[0] = 9.0f;
  pchan->loc[1] = 9.0f;
  pchan->loc[2] = 9.0f;
  pchan->quat[0] = 0.0f;
  pchan->quat[3] = 1.0f;

  /* DEG_id_tag_update uses G.main; set it for the duration of writeback. */
  G_MAIN = bmain;
  mmd_physics_pose_writeback(&armature_object->id, &snap);
  G_MAIN = nullptr;

  EXPECT_FLOAT_EQ(pchan->loc[0], 1.0f);
  EXPECT_FLOAT_EQ(pchan->loc[2], 3.0f);
  EXPECT_FLOAT_EQ(pchan->quat[0], 0.5f);
  EXPECT_FLOAT_EQ(pchan->quat[3], 0.5f);
}

}  // namespace
}  // namespace blender::io::pmx::tests
