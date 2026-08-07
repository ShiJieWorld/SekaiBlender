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
#include "pmx_import_bone_append.hh"
#include "pmx_import_mesh.hh"

#include "MEM_guardedalloc.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace blender::io::pmx::tests {
namespace {

class PMXBoneAppendTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Collection *model_collection = nullptr;
  Object *armature_object = nullptr;
  ReportList reports;

  void SetUp() override
  {
    bmain = BKE_main_new();
    model_collection = BKE_collection_add(bmain, nullptr, "AppendModel");
    bArmature *armature = BKE_armature_add(bmain, "AppendArmature");
    armature_object = BKE_object_add_only_object(bmain, OB_ARMATURE, "AppendArm");
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

  /* Parent -> {Rot(rot,0.5), Cancel(rot,-1.0), Trans(trans,0.3), Both(both,0.8)} */
  static PMXModel make_append_model()
  {
    PMXModel model;
    auto add_bone = [&](const char *name, int parent) {
      PMXBone bone;
      bone.name_local = name;
      bone.parent_index = parent;
      model.bones.push_back(bone);
      return int(model.bones.size()) - 1;
    };
    add_bone("Parent", -1);                                  /* 0 */
    const int rot = add_bone("Rot", 0);                      /* 1 */
    const int cancel = add_bone("Cancel", 0);                /* 2 */
    const int trans = add_bone("Trans", 0);                  /* 3 */
    const int both = add_bone("Both", 0);                    /* 4 */

    model.bones[rot].flag = BONE_FLAG_APPEND_ROTATION;
    model.bones[rot].inherit_parent_index = 0;
    model.bones[rot].inherit_parent_ratio = 0.5f;

    model.bones[cancel].flag = BONE_FLAG_APPEND_ROTATION;
    model.bones[cancel].inherit_parent_index = 0;
    model.bones[cancel].inherit_parent_ratio = -1.0f;

    model.bones[trans].flag = BONE_FLAG_APPEND_TRANSLATE;
    model.bones[trans].inherit_parent_index = 0;
    model.bones[trans].inherit_parent_ratio = 0.3f;

    model.bones[both].flag = BONE_FLAG_APPEND_ROTATION | BONE_FLAG_APPEND_TRANSLATE;
    model.bones[both].inherit_parent_index = 0;
    model.bones[both].inherit_parent_ratio = 0.8f;

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

TEST_F(PMXBoneAppendTest, PersistsAndReadsRoundTrip)
{
  PMXModel model = make_append_model();
  PMXImportContext ctx = make_ctx();

  persist_bone_append_definition(ctx, model);

  PMXBoneAppendDefinitionSet from_collection;
  ASSERT_TRUE(read_bone_append_definition(model_collection->id, from_collection));
  EXPECT_EQ(from_collection.schema_version, 1);
  ASSERT_EQ(from_collection.append_bones.size(), 4);

  const PMXBoneAppendDefinition &rot = from_collection.append_bones[0];
  EXPECT_EQ(rot.bone_name, "Rot");
  EXPECT_EQ(rot.mode, 1);
  EXPECT_EQ(rot.parent_name, "Parent");
  EXPECT_FLOAT_EQ(rot.ratio, 0.5f);

  const PMXBoneAppendDefinition &cancel = from_collection.append_bones[1];
  EXPECT_EQ(cancel.bone_name, "Cancel");
  EXPECT_EQ(cancel.mode, 1);
  EXPECT_FLOAT_EQ(cancel.ratio, -1.0f);

  const PMXBoneAppendDefinition &trans = from_collection.append_bones[2];
  EXPECT_EQ(trans.bone_name, "Trans");
  EXPECT_EQ(trans.mode, 2);
  EXPECT_FLOAT_EQ(trans.ratio, 0.3f);

  const PMXBoneAppendDefinition &both = from_collection.append_bones[3];
  EXPECT_EQ(both.bone_name, "Both");
  EXPECT_EQ(both.mode, 3);
  EXPECT_FLOAT_EQ(both.ratio, 0.8f);

  /* Same definition readable from the armature object (read point for the
   * editor operator and the future E-phase native solver). */
  PMXBoneAppendDefinitionSet from_armature;
  ASSERT_TRUE(read_bone_append_definition(armature_object->id, from_armature));
  ASSERT_EQ(from_armature.append_bones.size(), 4);
  EXPECT_EQ(from_armature.append_bones[0].bone_name, "Rot");
  EXPECT_EQ(from_armature.append_bones[3].bone_name, "Both");
  EXPECT_EQ(from_armature.append_bones[3].mode, 3);
}

TEST_F(PMXBoneAppendTest, PersistsEmptyModel)
{
  PMXModel model; /* No bones at all. */
  PMXImportContext ctx = make_ctx();

  persist_bone_append_definition(ctx, model);

  PMXBoneAppendDefinitionSet set;
  ASSERT_TRUE(read_bone_append_definition(model_collection->id, set));
  EXPECT_TRUE(set.append_bones.empty());
  EXPECT_EQ(set.schema_version, 1);
}

TEST_F(PMXBoneAppendTest, ReadMissingReturnsFalse)
{
  PMXBoneAppendDefinitionSet set;
  EXPECT_FALSE(read_bone_append_definition(model_collection->id, set));
}

TEST_F(PMXBoneAppendTest, NegativeRatioMapped)
{
  /* Negative ratio (cancel rotation) must be persisted verbatim, not clamped. */
  PMXModel model = make_append_model();
  PMXImportContext ctx = make_ctx();

  persist_bone_append_definition(ctx, model);

  PMXBoneAppendDefinitionSet set;
  ASSERT_TRUE(read_bone_append_definition(model_collection->id, set));
  bool found_cancel = false;
  for (const PMXBoneAppendDefinition &d : set.append_bones) {
    if (d.bone_name == "Cancel") {
      found_cancel = true;
      EXPECT_FLOAT_EQ(d.ratio, -1.0f);
    }
  }
  EXPECT_TRUE(found_cancel);
}

}  // namespace
}  // namespace blender::io::pmx::tests
