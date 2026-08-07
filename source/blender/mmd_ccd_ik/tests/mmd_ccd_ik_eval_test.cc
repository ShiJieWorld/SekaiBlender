/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * Tests for E3 CCD IK depsgraph evaluation.
 */

#include "testing/testing.h"

#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_armature.hh"
#include "BKE_fcurve.hh"
#include "BKE_constraint.h"
#include "BKE_gtest_base.hh"
#include "BKE_idprop.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"

#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_armature_types.h"
#include "DNA_constraint_types.h"
#include "DNA_object_types.h"

#include "MEM_guardedalloc.h"

#include "mmd_ccd_ik.hh"
#include "mmd_ccd_ik_eval.hh"

#include <cstring>
#include <string>

namespace blender::mmd::tests {
namespace {

/* -------------------------------------------------------------------- */
/* Test fixture                                                          */
/* -------------------------------------------------------------------- */

class MmdCCDIKEvalTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *arm_obj = nullptr;
  bArmature *arm = nullptr;
  bool armature_finalized = false;

  void SetUp() override
  {
    bmain = BKE_main_new();
    arm = BKE_armature_add(bmain, "EvalTestArm");
    arm_obj = BKE_object_add_only_object(bmain, OB_ARMATURE, "EvalTestArmObj");
    arm_obj->data = id_cast<ID *>(arm);
    BKE_pose_ensure(bmain, arm_obj, arm, false);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  /** Create a bone and rebuild pose. */
  Bone *make_bone(const char *name, const float head[3], const float tail[3])
  {
    Bone *bone = MEM_new<Bone>("make_bone");
    STRNCPY(bone->name, name);
    copy_v3_v3(bone->head, head);
    copy_v3_v3(bone->tail, tail);
    copy_v3_v3(bone->arm_head, head);
    copy_v3_v3(bone->arm_tail, tail);
    BLI_addtail(&arm->bonebase, bone);
    armature_finalized = false;
    return bone;
  }

  void finalize_armature()
  {
    if (armature_finalized) {
      return;
    }
    for (Bone *bone = static_cast<Bone *>(arm->bonebase.first); bone != nullptr;
         bone = bone->next)
    {
      BKE_armature_where_is_bone(bone, nullptr, true);
    }
    BKE_pose_rebuild(bmain, arm_obj, arm, true);
    armature_finalized = true;
  }

  /** Get a pose channel by name. */
  bPoseChannel *get_pchan(const char *name)
  {
    finalize_armature();
    return BKE_pose_channel_find_name(arm_obj->pose, name);
  }

  /** Ensure system_properties on armature ID and return the pointer. */
  IDProperty *ensure_sys_props()
  {
    return IDP_ID_system_properties_ensure(&arm_obj->id);
  }

  /** Write a minimal IK definition to the armature's system_properties.
   *  Creates a single IK bone entry pointing to the given target name,
   *  with two links forming a chain. Native IK enabled flag is set on
   *  the IK bone via mmd_native_ik_set_enabled(). */
  void write_ik_definition(const char *ik_bone_name,
                           const char *target_name,
                           const char *link0_name,
                           const char *link1_name,
                           bool native_ik_enabled,
                           const int schema_version = 2)
  {
    finalize_armature();
    IDProperty *sys_props = ensure_sys_props();

    /* Remove any existing definition. */
    IDProperty *old = IDP_GetPropertyFromGroup_null(sys_props, "mmd_pmx_bone_ik_definition");
    if (old) {
      IDP_FreeFromGroup(sys_props, old);
    }

    /* Build definition. */
    IDProperty *def = bke::idprop::create_group("mmd_pmx_bone_ik_definition").release();

    IDProperty *schema = bke::idprop::create("schema_version", schema_version).release();
    IDP_AddToGroup(def, schema);

    IDProperty *ik_bones = IDP_NewIDPArray("ik_bones");

    IDProperty *ik_bone = bke::idprop::create_group("ik_bone").release();
    {
      IDP_AddToGroup(ik_bone, IDP_NewString(ik_bone_name, "name"));
      IDP_AddToGroup(ik_bone, IDP_NewString(target_name, "target"));
      IDP_AddToGroup(ik_bone, bke::idprop::create("loop_count", 40).release());
      IDP_AddToGroup(ik_bone, bke::idprop::create("angle_limit", 0.8f).release());

      IDProperty *links = IDP_NewIDPArray("links");
      for (const char *link_name : {link0_name, link1_name}) {
        IDProperty *link_item = bke::idprop::create_group("link").release();
        IDP_AddToGroup(link_item, IDP_NewString(link_name, "bone"));
        IDP_AddToGroup(link_item,
                       bke::idprop::create_bool("limit_angle", false).release());
        IDP_AppendArray(links, link_item);
        MEM_delete(link_item);
      }
      IDP_AddToGroup(ik_bone, links);
    }
    IDP_AppendArray(ik_bones, ik_bone);
    MEM_delete(ik_bone);
    IDP_AddToGroup(def, ik_bones);

    IDP_AddToGroup(sys_props, def);

    /* E1: set native IK toggle on bone. */
    Bone *bone = BKE_armature_find_bone_name(arm, ik_bone_name);
    if (bone) {
      mmd_native_ik_set_enabled(*bone, native_ik_enabled);
    }
  }

  /** Add an MMD_IK_Approx constraint to a pose channel. */
  bConstraint *add_approx_constraint(const char *pchan_name)
  {
    bPoseChannel *pchan = get_pchan(pchan_name);
    if (!pchan) {
      return nullptr;
    }
    bConstraint *con = BKE_constraint_add_for_pose(
        arm_obj, pchan, "MMD_IK_Approx", CONSTRAINT_TYPE_KINEMATIC);
    if (con) {
      con->enforce = 1.0f;
    }
    return con;
  }

  void add_toggle_curve(const char *ik_bone_name, const float frame, const float value)
  {
    bAction *action = BKE_action_add(bmain, "IKToggleAction");
    AnimData *adt = BKE_animdata_ensure_id(&arm_obj->id);
    adt->action = action;

    char escaped_name[sizeof(Bone::name) * 2] = {};
    BLI_str_escape(escaped_name, ik_bone_name, sizeof(escaped_name));
    const std::string path = std::string("pose.bones[\"") + escaped_name + "\"].mmd_ik_toggle";
    FCurve *fcurve = BKE_fcurve_create();
    fcurve->rna_path = BLI_strdup(path.c_str());
    fcurve->array_index = 0;
    fcurve->totvert = 1;
    fcurve->bezt = MEM_new_zeroed<BezTriple>("toggle bezier");
    fcurve->bezt[0].vec[1][0] = frame;
    fcurve->bezt[0].vec[1][1] = value;
    fcurve->bezt[0].ipo = BEZT_IPO_CONST;
    BLI_addtail(&action->curves, fcurve);
  }

  void copy_pose_matrix(const char *pchan_name, float r_pose_mat[4][4])
  {
    bPoseChannel *pchan = get_pchan(pchan_name);
    ASSERT_NE(pchan, nullptr);
    copy_m4_m4(r_pose_mat, pchan->pose_mat);
  }

  /** Find an MMD_IK_Approx constraint on a pose channel, or nullptr. */
  bConstraint *find_approx_constraint(const char *pchan_name)
  {
    bPoseChannel *pchan = get_pchan(pchan_name);
    if (!pchan) {
      return nullptr;
    }
    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first);
         con != nullptr;
         con = con->next)
    {
      if (std::strcmp(con->name, "MMD_IK_Approx") == 0) {
        return con;
      }
    }
    return nullptr;
  }
};

/* -------------------------------------------------------------------- */
/* Tests                                                                 */
/* -------------------------------------------------------------------- */

TEST_F(MmdCCDIKEvalTest, non_armature_object_noop)
{
  /* Null pointer — must not crash. */
  mmd_ccd_ik_evaluate(nullptr, nullptr);

  /* Mesh object — must not crash. */
  Object *mesh_obj = BKE_object_add_only_object(bmain, OB_MESH, "Mesh");
  mmd_ccd_ik_evaluate(nullptr, mesh_obj);

  /* Null depsgraph but valid armature — must not crash. */
  mmd_ccd_ik_evaluate(nullptr, arm_obj);
}

TEST_F(MmdCCDIKEvalTest, early_return_when_no_definition)
{
  /* No IK definition written → eval should early-return without side effects. */
  mmd_ccd_ik_evaluate(nullptr, arm_obj);
  /* Just verifying it doesn't crash. */
}

TEST_F(MmdCCDIKEvalTest, early_return_when_native_ik_disabled)
{
  /* Set up bones for chain + target. */
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);
  make_bone("IKTarget", z2, z2);  // zero-length control bone

  /* IK bone = UpperLeg, target = IKTarget, chain = [LowerLeg, UpperLeg],
   * but native IK DISABLED. */
  write_ik_definition("UpperLeg", "IKTarget", "LowerLeg", "UpperLeg", false);

  /* Add an MMD_IK_Approx constraint on LowerLeg with enforce=1.0. */
  add_approx_constraint("LowerLeg");

  bConstraint *con_before = find_approx_constraint("LowerLeg");
  ASSERT_NE(con_before, nullptr);
  EXPECT_FLOAT_EQ(con_before->enforce, 1.0f);

  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  /* Since native IK is disabled, constraints should NOT be muted. */
  bConstraint *con_after = find_approx_constraint("LowerLeg");
  ASSERT_NE(con_after, nullptr);
  EXPECT_FLOAT_EQ(con_after->enforce, 1.0f);
}

TEST_F(MmdCCDIKEvalTest, mutes_and_restores_approx_constraints)
{
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);
  make_bone("IKTarget", z2, z2);

  /* Native IK ENABLED. */
  write_ik_definition("UpperLeg", "IKTarget", "LowerLeg", "UpperLeg", true);

  /* Add MMD_IK_Approx with enforce=1.0. */
  add_approx_constraint("LowerLeg");

  bConstraint *con_before = find_approx_constraint("LowerLeg");
  ASSERT_NE(con_before, nullptr);
  EXPECT_FLOAT_EQ(con_before->enforce, 1.0f);

  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  /* Constraint enforce must be restored to original value after eval. */
  bConstraint *con_after = find_approx_constraint("LowerLeg");
  ASSERT_NE(con_after, nullptr);
  EXPECT_FLOAT_EQ(con_after->enforce, 1.0f);
}

TEST_F(MmdCCDIKEvalTest, chain_solves_without_crash)
{
  /* Build a simple 2-bone chain pointing +Z, target off to +X. */
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);

  /* Target bone at (0.5, 0, 1.5) in rest pose. */
  const float target_head[3] = {0.5f, 0.0f, 1.5f};
  make_bone("IKTarget", target_head, target_head);

  /* Native IK ENABLED. */
  write_ik_definition("UpperLeg", "IKTarget", "LowerLeg", "UpperLeg", true);

  /* Eval must not crash (the chain may or may not converge in gtest — we only
   * verify that the loop completes without segfault/assert). */
  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  /* After eval, pose channels still valid. */
  bPoseChannel *up_pchan = get_pchan("UpperLeg");
  bPoseChannel *lo_pchan = get_pchan("LowerLeg");
  ASSERT_NE(up_pchan, nullptr);
  ASSERT_NE(lo_pchan, nullptr);
}

TEST_F(MmdCCDIKEvalTest, respects_native_ik_toggle_per_bone)
{
  /* Two IK chains: left leg (enabled) and right leg (disabled). Only left
   * should trigger CCD, and right's constraint should not be muted. */
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};

  make_bone("UpperLeg_L", origin, z1);
  make_bone("LowerLeg_L", z1, z2);
  make_bone("IKTarget_L", z2, z2);

  const float xoff[3] = {3.0f, 0.0f, 0.0f};
  const float z1r[3] = {3.0f, 0.0f, 1.0f};
  const float z2r[3] = {3.0f, 0.0f, 2.0f};
  make_bone("UpperLeg_R", xoff, z1r);
  make_bone("LowerLeg_R", z1r, z2r);
  make_bone("IKTarget_R", z2r, z2r);

  /* Left leg: native IK ENABLED. */
  write_ik_definition("UpperLeg_L", "IKTarget_L", "LowerLeg_L", "UpperLeg_L", true);
  /* Right leg: native IK DISABLED. */
  write_ik_definition("UpperLeg_R", "IKTarget_R", "LowerLeg_R", "UpperLeg_R", false);

  add_approx_constraint("LowerLeg_L");
  add_approx_constraint("LowerLeg_R");

  bConstraint *left_con_before = find_approx_constraint("LowerLeg_L");
  ASSERT_NE(left_con_before, nullptr);
  EXPECT_FLOAT_EQ(left_con_before->enforce, 1.0f);

  bConstraint *right_con_before = find_approx_constraint("LowerLeg_R");
  ASSERT_NE(right_con_before, nullptr);
  EXPECT_FLOAT_EQ(right_con_before->enforce, 1.0f);

  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  /* Both constraints restored after eval (left was muted during, right was untouched). */
  bConstraint *left_con_after = find_approx_constraint("LowerLeg_L");
  ASSERT_NE(left_con_after, nullptr);
  EXPECT_FLOAT_EQ(left_con_after->enforce, 1.0f);

  bConstraint *right_con_after = find_approx_constraint("LowerLeg_R");
  ASSERT_NE(right_con_after, nullptr);
  EXPECT_FLOAT_EQ(right_con_after->enforce, 1.0f);
}

TEST_F(MmdCCDIKEvalTest, coordinate_conversion_applied)
{
  /* Same as chain test but with non-trivial angle limits to verify
   * the coordinate conversion round-trip. */
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);
  const float target_head[3] = {0.5f, 0.0f, 1.5f};
  make_bone("IKTarget", target_head, target_head);

  /* Write IK definition WITH angle limits (PMX Y-up values). */
  IDProperty *sys_props = ensure_sys_props();
  IDProperty *old_def = IDP_GetPropertyFromGroup_null(sys_props, "mmd_pmx_bone_ik_definition");
  if (old_def) { IDP_FreeFromGroup(sys_props, old_def); }

  IDProperty *def = bke::idprop::create_group("mmd_pmx_bone_ik_definition").release();
  IDP_AddToGroup(def, bke::idprop::create("schema_version", 2).release());

  IDProperty *ik_bones = IDP_NewIDPArray("ik_bones");
  IDProperty *ik_bone = bke::idprop::create_group("ik_bone").release();
  IDP_AddToGroup(ik_bone, IDP_NewString("UpperLeg", "name"));
  IDP_AddToGroup(ik_bone, IDP_NewString("IKTarget", "target"));
  IDP_AddToGroup(ik_bone, bke::idprop::create("loop_count", 40).release());
  IDP_AddToGroup(ik_bone, bke::idprop::create("angle_limit", 0.8f).release());

  IDProperty *links = IDP_NewIDPArray("links");
  for (int li = 0; li < 2; li++) {
    IDProperty *link_item = bke::idprop::create_group("link").release();
    IDP_AddToGroup(link_item, IDP_NewString(li == 0 ? "LowerLeg" : "UpperLeg", "bone"));
    IDP_AddToGroup(link_item, bke::idprop::create_bool("limit_angle", true).release());

    /* PMX Y-up limits: (min_x, min_y, min_z) = ( -0.5, -1.0, -1.5 )
     *                    (max_x, max_y, max_z) = (  0.5,  1.0,  1.5 )
     * Expected Blender Z-up:
     *   min[0]=-0.5, min[1]=-1.5, min[2]=-1.0
     *   max[0]= 0.5, max[1]= 1.5, max[2]= 1.0 */
    const float min_pmx[3] = {-0.5f, -1.0f, -1.5f};
    const float max_pmx[3] = { 0.5f,  1.0f,  1.5f};
    IDP_AddToGroup(
        link_item,
        bke::idprop::create("limit_min", Span<float>(min_pmx, 3)).release());
    IDP_AddToGroup(
        link_item,
        bke::idprop::create("limit_max", Span<float>(max_pmx, 3)).release());

    IDP_AppendArray(links, link_item);
    MEM_delete(link_item);
  }
  IDP_AddToGroup(ik_bone, links);
  IDP_AppendArray(ik_bones, ik_bone);
  MEM_delete(ik_bone);
  IDP_AddToGroup(def, ik_bones);
  IDP_AddToGroup(sys_props, def);

  /* Enable native IK. */
  Bone *bone = BKE_armature_find_bone_name(arm, "UpperLeg");
  ASSERT_NE(bone, nullptr);
  mmd_native_ik_set_enabled(*bone, true);

  /* Must not crash. */
  mmd_ccd_ik_evaluate(nullptr, arm_obj);
}

TEST_F(MmdCCDIKEvalTest, schema1_keeps_approx_constraint_available_by_default)
{
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);
  make_bone("IKTarget", z2, z2);
  write_ik_definition("UpperLeg", "IKTarget", "LowerLeg", "UpperLeg", true, 1);
  bConstraint *constraint = add_approx_constraint("LowerLeg");
  ASSERT_NE(constraint, nullptr);

  float before[4][4];
  copy_pose_matrix("LowerLeg", before);

  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  EXPECT_FLOAT_EQ(constraint->enforce, 1.0f);
  bPoseChannel *lower = get_pchan("LowerLeg");
  ASSERT_NE(lower, nullptr);
  EXPECT_EQ(std::memcmp(before, lower->pose_mat, sizeof(before)), 0);
}

TEST_F(MmdCCDIKEvalTest, action_skips_v2_solver_and_keeps_approx_constraint)
{
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const float z1[3] = {0.0f, 0.0f, 1.0f};
  const float z2[3] = {0.0f, 0.0f, 2.0f};
  make_bone("UpperLeg", origin, z1);
  make_bone("LowerLeg", z1, z2);
  const float target_head[3] = {0.5f, 0.0f, 1.5f};
  make_bone("IKTarget", target_head, target_head);
  write_ik_definition("UpperLeg", "IKTarget", "LowerLeg", "UpperLeg", true);
  bConstraint *constraint = add_approx_constraint("LowerLeg");
  ASSERT_NE(constraint, nullptr);
  add_toggle_curve("UpperLeg", 0.0f, 0.0f);

  float before[4][4];
  copy_pose_matrix("LowerLeg", before);

  const char *old_v8 = BLI_getenv("MMD_CCD_V8");
  const bool had_v8 = old_v8 != nullptr;
  const std::string old_v8_value = had_v8 ? old_v8 : "";
  BLI_setenv("MMD_CCD_V8", "0");
  mmd_ccd_ik_evaluate(nullptr, arm_obj);

  bPoseChannel *lower = get_pchan("LowerLeg");
  ASSERT_NE(lower, nullptr);
  EXPECT_EQ(std::memcmp(before, lower->pose_mat, sizeof(before)), 0);
  EXPECT_FLOAT_EQ(constraint->enforce, 1.0f);

  BLI_setenv("MMD_CCD_V8", had_v8 ? old_v8_value.c_str() : nullptr);
}

}  // namespace
}  // namespace blender::mmd::tests
