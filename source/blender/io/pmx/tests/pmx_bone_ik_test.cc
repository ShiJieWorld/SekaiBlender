/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BKE_collection.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_armature.hh"
#include "BKE_report.hh"
#include "BKE_idprop.hh"

#include "intern/pmx_types.h"
#include "pmx_import_bone_ik.hh"
#include "pmx_import_mesh.hh"

#include "MEM_guardedalloc.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <string>

namespace blender::io::pmx::tests {
namespace {

class PMXBoneIKTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Collection *model_collection = nullptr;
  Object *armature_object = nullptr;
  ReportList reports;

  void SetUp() override
  {
    bmain = BKE_main_new();
    model_collection = BKE_collection_add(bmain, nullptr, "IKModel");
    bArmature *armature = BKE_armature_add(bmain, "IKArmature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "IKArm");
    armature_object->data = id_cast<ID *>(armature);
    BKE_reports_init(&reports, RPT_STORE);
  }

  void TearDown() override
  {
    /* `BKE_main_free` frees every ID (and its `id->properties` /
     * `id->system_properties`) via `BKE_id_free_ex`, so no manual IDProperty
     * teardown is needed here. The only way to leak is to orphan an IDProperty
     * (e.g. a group passed to `IDP_AppendArray` without releasing it) — that is
     * handled where the property is created, not here. */
    BKE_reports_free(&reports);
    BKE_main_free(bmain);
  }

  /** Build a model with: Parent -> Foot(target) -> Ankle(link); LegIK drives Foot. */
  static PMXModel make_ik_model()
  {
    PMXModel model;
    auto add_bone = [&](const char *name, int parent) {
      PMXBone bone;
      bone.name_local = name;
      bone.parent_index = parent;
      model.bones.push_back(bone);
      return int(model.bones.size()) - 1;
    };
    add_bone("Parent", -1);                              /* 0 */
    const int foot = add_bone("Foot", 0);               /* 1 */
    const int ankle = add_bone("Ankle", foot);          /* 2 */
    add_bone("LegIK", 0);                               /* 3 (IK) */

    PMXBone &ik = model.bones[3];
    ik.flag = BONE_FLAG_IK;
    ik.ik_target_index = foot;
    ik.ik_loop_count = 10;
    ik.ik_angle_limit = 1.0f;
    PMXIKLink link;
    link.bone_index = ankle;
    link.limit_angle = true;
    link.limit_min[0] = -0.5f;
    link.limit_min[1] = -0.25f;
    link.limit_min[2] = -0.75f;
    link.limit_max[0] = 0.5f;
    link.limit_max[1] = 0.25f;
    link.limit_max[2] = 0.75f;
    ik.ik_links.push_back(link);

    PMXRigidBody rigid;
    rigid.bone_index = ankle;
    rigid.physics_type = 2;
    model.rigid_bodies.push_back(rigid);
    return model;
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
};

TEST_F(PMXBoneIKTest, PersistsAndReadsRoundTrip)
{
  PMXModel model = make_ik_model();
  PMXImportContext ctx = make_ctx();

  persist_bone_ik_definition(ctx, model);

  PMXBoneIKDefinitionSet from_collection;
  ASSERT_TRUE(read_bone_ik_definition(model_collection->id, from_collection));
  EXPECT_EQ(from_collection.schema_version, 2);
  ASSERT_EQ(from_collection.ik_bones.size(), 1);

  const PMXBoneIKDefinition &def = from_collection.ik_bones[0];
  EXPECT_EQ(def.bone_name, "LegIK");
  EXPECT_EQ(def.target_name, "Foot");
  EXPECT_EQ(def.loop_count, 10);
  EXPECT_FLOAT_EQ(def.angle_limit, 1.0f);
  ASSERT_EQ(def.links.size(), 1);
  EXPECT_EQ(def.links[0].bone_name, "Ankle");
  EXPECT_TRUE(def.links[0].limit_angle);
  EXPECT_TRUE(def.links[0].physics_owned);
  EXPECT_FLOAT_EQ(def.links[0].limit_min[0], -0.5f);
  EXPECT_FLOAT_EQ(def.links[0].limit_min[1], -0.25f);
  EXPECT_FLOAT_EQ(def.links[0].limit_min[2], -0.75f);
  EXPECT_FLOAT_EQ(def.links[0].limit_max[0], 0.5f);
  EXPECT_FLOAT_EQ(def.links[0].limit_max[1], 0.25f);
  EXPECT_FLOAT_EQ(def.links[0].limit_max[2], 0.75f);

  /* The same definition must be readable from the armature object, which is the
   * read point used by the VMD importer and the future E-phase solver. */
  PMXBoneIKDefinitionSet from_armature;
  ASSERT_TRUE(read_bone_ik_definition(armature_object->id, from_armature));
  ASSERT_EQ(from_armature.ik_bones.size(), 1);
  EXPECT_EQ(from_armature.ik_bones[0].bone_name, "LegIK");
  EXPECT_EQ(from_armature.ik_bones[0].target_name, "Foot");
  EXPECT_EQ(from_armature.ik_bones[0].links.size(), 1);
  EXPECT_EQ(from_armature.ik_bones[0].links[0].bone_name, "Ankle");
}

TEST_F(PMXBoneIKTest, PersistsEmptyModel)
{
  PMXModel model; /* No bones at all. */
  PMXImportContext ctx = make_ctx();

  persist_bone_ik_definition(ctx, model);

  PMXBoneIKDefinitionSet set;
  ASSERT_TRUE(read_bone_ik_definition(model_collection->id, set));
  EXPECT_TRUE(set.ik_bones.empty());
  EXPECT_EQ(set.schema_version, 2);
}

TEST_F(PMXBoneIKTest, ReadMissingReturnsFalse)
{
  /* A freshly created collection has no IK definition. */
  PMXBoneIKDefinitionSet set;
  EXPECT_FALSE(read_bone_ik_definition(model_collection->id, set));
}

}  // namespace
}  // namespace blender::io::pmx::tests
