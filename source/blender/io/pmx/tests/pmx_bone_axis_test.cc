/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BKE_armature.hh"
#include "BKE_collection.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_action.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"
#include "BKE_idprop.hh"

#include "BLI_listbase.hh"
#include "BLI_string.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_constraint_types.h"
#include "DNA_object_types.h"

#include "intern/pmx_types.h"
#include "pmx_import_bone_axis.hh"
#include "pmx_import_mesh.hh"

#include "MEM_guardedalloc.h"

#include <cmath>
#include <string>

namespace blender::io::pmx::tests {
namespace {

class PMXBoneAxisTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Collection *model_collection = nullptr;
  Object *armature_object = nullptr;
  ReportList reports;

  void SetUp() override
  {
    bmain = BKE_main_new();
    model_collection = BKE_collection_add(bmain, nullptr, "AxisModel");
    bArmature *armature = BKE_armature_add(bmain, "AxisArmature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "AxisArm");
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
    Bone *b = MEM_new<Bone>("AxisTestBone");
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

  PMXImportContext make_ctx()
  {
    PMXImportContext ctx;
    ctx.bmain = bmain;
    ctx.model_collection = model_collection;
    ctx.armature_obj = armature_object;
    ctx.reports = &reports;
    return ctx;
  }

  /** Build a model exercising every D3 semantic. */
  static PMXModel make_axis_model()
  {
    PMXModel model;
    auto add = [&](const char *name) {
      PMXBone bone;
      bone.name_local = name;
      model.bones.push_back(bone);
      return int(model.bones.size()) - 1;
    };

    const int fixed_x = add("FixedX"); /* principal +X */
    model.bones[fixed_x].flag = BONE_FLAG_FIXED_AXIS;
    model.bones[fixed_x].fixed_axis[0] = 1.0f;

    const int fixed_arb = add("FixedArb"); /* arbitrary unit axis */
    model.bones[fixed_arb].flag = BONE_FLAG_FIXED_AXIS;
    model.bones[fixed_arb].fixed_axis[0] = 0.577f;
    model.bones[fixed_arb].fixed_axis[1] = 0.577f;
    model.bones[fixed_arb].fixed_axis[2] = 0.577f;

    const int local = add("LocalBone"); /* custom local frame */
    model.bones[local].flag = BONE_FLAG_LOCAL_AXIS;
    model.bones[local].local_x[0] = 1.0f;
    model.bones[local].local_z[2] = 1.0f;

    const int deform = add("DeformBone"); /* deform after physics */
    model.bones[deform].flag = BONE_FLAG_PHYSICS_AFTER_DEF;
    model.bones[deform].transform_order = 7;

    return model;
  }
};

TEST_F(PMXBoneAxisTest, PersistsAndReadsRoundTrip)
{
  PMXModel model = make_axis_model();
  PMXImportContext ctx = make_ctx();

  persist_bone_axis_definition(ctx, model);

  PMXBoneAxisDefinitionSet from_collection;
  ASSERT_TRUE(read_bone_axis_definition(model_collection->id, from_collection));
  EXPECT_EQ(from_collection.schema_version, 1);
  ASSERT_EQ(from_collection.bones.size(), 4);

  /* FixedX: principal +X. */
  const PMXBoneAxisDefinition *fx = nullptr;
  const PMXBoneAxisDefinition *fa = nullptr;
  const PMXBoneAxisDefinition *lb = nullptr;
  const PMXBoneAxisDefinition *db = nullptr;
  for (const PMXBoneAxisDefinition &d : from_collection.bones) {
    if (d.bone_name == "FixedX") {
      fx = &d;
    }
    else if (d.bone_name == "FixedArb") {
      fa = &d;
    }
    else if (d.bone_name == "LocalBone") {
      lb = &d;
    }
    else if (d.bone_name == "DeformBone") {
      db = &d;
    }
  }
  ASSERT_NE(fx, nullptr);
  EXPECT_TRUE(fx->has_fixed_axis);
  EXPECT_FLOAT_EQ(fx->fixed_axis[0], 1.0f);
  EXPECT_FLOAT_EQ(fx->fixed_axis[1], 0.0f);

  ASSERT_NE(fa, nullptr);
  EXPECT_TRUE(fa->has_fixed_axis);
  EXPECT_FLOAT_EQ(fa->fixed_axis[0], 0.577f);

  ASSERT_NE(lb, nullptr);
  EXPECT_TRUE(lb->has_local_axis);
  EXPECT_FLOAT_EQ(lb->local_x[0], 1.0f);
  EXPECT_FLOAT_EQ(lb->local_z[2], 1.0f);

  ASSERT_NE(db, nullptr);
  EXPECT_TRUE(db->deform_after_physics);
  EXPECT_EQ(db->transform_order, 7);

  /* Same definition must be readable from the armature object (read point for
   * the editor operators / E solver). */
  PMXBoneAxisDefinitionSet from_armature;
  ASSERT_TRUE(read_bone_axis_definition(armature_object->id, from_armature));
  ASSERT_EQ(from_armature.bones.size(), 4);
}

TEST_F(PMXBoneAxisTest, PersistsEmptyModel)
{
  PMXModel model; /* No bones at all. */
  PMXImportContext ctx = make_ctx();

  persist_bone_axis_definition(ctx, model);

  PMXBoneAxisDefinitionSet set;
  ASSERT_TRUE(read_bone_axis_definition(model_collection->id, set));
  EXPECT_TRUE(set.bones.empty());
  EXPECT_EQ(set.schema_version, 1);
}

TEST_F(PMXBoneAxisTest, ReadMissingReturnsFalse)
{
  PMXBoneAxisDefinitionSet set;
  EXPECT_FALSE(read_bone_axis_definition(model_collection->id, set));
}

TEST_F(PMXBoneAxisTest, PrincipalAxisLocksAndArbitraryUsesConstraint)
{
  /* Bones must exist so pose channels can be created. */
  add_bone("FixedX");
  add_bone("FixedArb");
  ensure_pose();

  PMXBoneAxisDefinition def_x;
  def_x.bone_name = "FixedX";
  def_x.has_fixed_axis = true;
  def_x.fixed_axis[0] = 1.0f;

  PMXBoneAxisDefinition def_arb;
  def_arb.bone_name = "FixedArb";
  def_arb.has_fixed_axis = true;
  def_arb.fixed_axis[0] = 0.577f;
  def_arb.fixed_axis[1] = 0.577f;
  def_arb.fixed_axis[2] = 0.577f;

  bPoseChannel *pchan_x = BKE_pose_channel_find_name(armature_object->pose, "FixedX");
  bPoseChannel *pchan_arb = BKE_pose_channel_find_name(armature_object->pose, "FixedArb");
  ASSERT_NE(pchan_x, nullptr);
  ASSERT_NE(pchan_arb, nullptr);

  /* Principal +X -> native lock on Y and Z, X left free. Exact, no constraint. */
  EXPECT_TRUE(pmx_apply_fixed_axis_to_pchan(armature_object, pchan_x, def_x, &reports));
  EXPECT_TRUE(pchan_x->protectflag & OB_LOCK_ROTY);
  EXPECT_TRUE(pchan_x->protectflag & OB_LOCK_ROTZ);
  EXPECT_FALSE(pchan_x->protectflag & OB_LOCK_ROTX);
  bool x_has_con = false;
  for (bConstraint *c = static_cast<bConstraint *>(pchan_x->constraints.first); c != nullptr;
       c = c->next)
  {
    if (strcmp(c->name, "MMD_Fixed_Axis_Approx") == 0) {
      x_has_con = true;
    }
  }
  EXPECT_FALSE(x_has_con);

  /* Arbitrary axis -> Limit Rotation constraint approximation. */
  EXPECT_TRUE(pmx_apply_fixed_axis_to_pchan(armature_object, pchan_arb, def_arb, &reports));
  bool arb_has_con = false;
  bool arb_approx = false;
  for (bConstraint *c = static_cast<bConstraint *>(pchan_arb->constraints.first); c != nullptr;
       c = c->next)
  {
    if (strcmp(c->name, "MMD_Fixed_Axis_Approx") == 0) {
      arb_has_con = true;
      if (c->data != nullptr && c->type == CONSTRAINT_TYPE_ROTLIMIT) {
        bRotLimitConstraint *lim = static_cast<bRotLimitConstraint *>(c->data);
        EXPECT_EQ(c->ownspace, CONSTRAINT_SPACE_LOCAL);
        /* The dominant (largest) component is X -> X must be free, Y/Z limited. */
        EXPECT_FALSE(lim->flag & LIMIT_XROT);
        EXPECT_TRUE(lim->flag & LIMIT_YROT);
        EXPECT_TRUE(lim->flag & LIMIT_ZROT);
      }
    }
  }
  EXPECT_TRUE(arb_has_con);
  if (pchan_arb->system_properties != nullptr) {
    IDProperty *p = IDP_GetPropertyTypeFromGroup(pchan_arb->system_properties, "mmd_approximate", IDP_BOOLEAN);
    if (p != nullptr && IDP_bool_get(p)) {
      arb_approx = true;
    }
  }
  EXPECT_TRUE(arb_approx);

  /* Idempotency: applying again is a no-op. */
  EXPECT_FALSE(pmx_apply_fixed_axis_to_pchan(armature_object, pchan_x, def_x, &reports));
}

TEST_F(PMXBoneAxisTest, DeformLayerRecordedNoConstraint)
{
  PMXModel model = make_axis_model();
  PMXImportContext ctx = make_ctx();

  persist_bone_axis_definition(ctx, model);

  PMXBoneAxisDefinitionSet set;
  ASSERT_TRUE(read_bone_axis_definition(armature_object->id, set));
  const PMXBoneAxisDefinition *db = nullptr;
  for (const PMXBoneAxisDefinition &d : set.bones) {
    if (d.bone_name == "DeformBone") {
      db = &d;
    }
  }
  ASSERT_NE(db, nullptr);
  EXPECT_TRUE(db->deform_after_physics);
  EXPECT_EQ(db->transform_order, 7);
  /* No constraint / bone-matrix change happens at persist time (red line D3-c). */
  EXPECT_FALSE(db->has_fixed_axis);
  EXPECT_FALSE(db->has_local_axis);
}

}  // namespace
}  // namespace blender::io::pmx::tests
