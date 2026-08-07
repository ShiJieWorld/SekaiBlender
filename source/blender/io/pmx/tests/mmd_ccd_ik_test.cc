/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "BKE_action.hh"
#include "BKE_armature.hh"
#include "BKE_gtest_base.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_string.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_object_types.h"

#include "mmd_ccd_ik.hh"

#include "MEM_guardedalloc.h"

#include <cstring>

namespace blender::mmd::tests {
namespace {

class MmdCCDIKTest : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *arm_obj = nullptr;
  bArmature *arm = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();

    /* Create armature */
    arm = BKE_armature_add(bmain, "CCDTestArm");
    arm_obj = BKE_object_add_only_object(bmain, OB_ARMATURE, "CCDTest");
    arm_obj->data = id_cast<ID *>(arm);

    /* Ensure pose channels */
    BKE_pose_ensure(bmain, arm_obj, arm, false);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  /** Create a bone in the armature. Returns the Bone pointer. */
  Bone *make_bone(const char *name,
                  const float head[3],
                  const float tail[3],
                  Bone *parent = nullptr)
  {
    Bone *bone = MEM_new<Bone>(__func__);
    STRNCPY(bone->name, name);
    copy_v3_v3(bone->head, head);
    copy_v3_v3(bone->tail, tail);
    copy_v3_v3(bone->arm_head, head);
    copy_v3_v3(bone->arm_tail, tail);
    bone->parent = parent;
    BLI_addtail(parent ? &parent->childbase : &arm->bonebase, bone);
    return bone;
  }

  /** Build a simple two-bone chain: parent at origin pointing +Z, child extending further. */
  void build_two_bone_chain()
  {
    float head0[3] = {0.0f, 0.0f, 0.0f};
    float tail0[3] = {0.0f, 0.0f, 1.0f};
    Bone *root = make_bone("Root", head0, tail0);

    float head1[3] = {0.0f, 0.0f, 1.0f};
    float tail1[3] = {0.0f, 0.0f, 2.0f};
    make_bone("Tip", head1, tail1, root);

    BKE_armature_where_is_bone(root, nullptr, true);
    BKE_pose_rebuild(bmain, arm_obj, arm, true);
    for (bPoseChannel *pchan = static_cast<bPoseChannel *>(arm_obj->pose->chanbase.first);
         pchan != nullptr;
         pchan = pchan->next)
    {
      Bone *bone = pchan->bone_get(*arm_obj);
      if (bone != nullptr) {
        unit_m4(pchan->chan_mat);
        BKE_armature_mat_bone_to_pose({pchan, bone}, pchan->chan_mat, pchan->pose_mat);
        copy_v3_v3(pchan->pose_head, pchan->pose_mat[3]);
        BKE_pose_where_is_bone_tail({pchan, bone});
      }
    }
  }

  /** Get pose channel by name. */
  bPoseChannel *get_pchan(const char *name)
  {
    return BKE_pose_channel_find_name(arm_obj->pose, name);
  }

  void solver_tail(bPoseChannel *pchan, float r_tail[3])
  {
    Bone *bone = pchan->bone_get(*arm_obj);
    ASSERT_NE(bone, nullptr);
    copy_v3_v3(r_tail, pchan->pose_mat[1]);
    mul_v3_fl(r_tail, bone->length);
    add_v3_v3(r_tail, pchan->pose_mat[3]);
  }
};

class MmdCCDIKV2Test : public bke::BlenderGTestBase {
 protected:
  Main *bmain = nullptr;
  Object *arm_obj = nullptr;
  bArmature *arm = nullptr;

  void SetUp() override
  {
    bmain = BKE_main_new();
    arm = BKE_armature_add(bmain, "CCDV2TestArm");
    arm_obj = BKE_object_add_only_object(bmain, OB_ARMATURE, "CCDV2Test");
    arm_obj->data = id_cast<ID *>(arm);
    BKE_pose_ensure(bmain, arm_obj, arm, false);
  }

  void TearDown() override
  {
    BKE_main_free(bmain);
  }

  Bone *make_bone(const char *name,
                  const float head[3],
                  const float tail[3],
                  Bone *parent = nullptr)
  {
    Bone *bone = MEM_new<Bone>(__func__);
    STRNCPY(bone->name, name);
    copy_v3_v3(bone->head, head);
    copy_v3_v3(bone->tail, tail);
    copy_v3_v3(bone->arm_head, head);
    copy_v3_v3(bone->arm_tail, tail);
    bone->parent = parent;
    BLI_addtail(parent ? &parent->childbase : &arm->bonebase, bone);
    return bone;
  }

  void build_v2_chain()
  {
    const float root_head[3] = {0.0f, 0.0f, 0.0f};
    const float root_tail[3] = {0.0f, 0.0f, 1.0f};
    Bone *root = make_bone("Root", root_head, root_tail);
    const float tip_head[3] = {0.0f, 0.0f, 1.0f};
    const float tip_tail[3] = {0.0f, 0.0f, 2.0f};
    Bone *tip = make_bone("Tip", tip_head, tip_tail, root);
    const float effector_head[3] = {0.0f, 0.0f, 2.0f};
    const float effector_tail[3] = {0.0f, 0.0f, 2.25f};
    make_bone("Effector", effector_head, effector_tail, tip);

    BKE_armature_where_is_bone(root, nullptr, true);
    BKE_pose_rebuild(bmain, arm_obj, arm, true);
    for (bPoseChannel *pchan = static_cast<bPoseChannel *>(arm_obj->pose->chanbase.first);
         pchan != nullptr;
         pchan = pchan->next)
    {
      Bone *bone = pchan->bone_get(*arm_obj);
      if (bone != nullptr) {
        unit_m4(pchan->chan_mat);
        BKE_armature_mat_bone_to_pose({pchan, bone}, pchan->chan_mat, pchan->pose_mat);
        copy_v3_v3(pchan->pose_head, pchan->pose_mat[3]);
        BKE_pose_where_is_bone_tail({pchan, bone});
      }
    }
  }

  bPoseChannel *get_pchan(const char *name)
  {
    return BKE_pose_channel_find_name(arm_obj->pose, name);
  }
};

TEST_F(MmdCCDIKTest, two_bone_converges)
{
  build_two_bone_chain();

  /* Build the chain: Tip (index 0) → Root (index 1). */
  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  ASSERT_NE(chain[0].pchan, nullptr);
  ASSERT_NE(chain[1].pchan, nullptr);

  /* Target: bend the chain to the right (+X). */
  float target[3] = {0.5f, 0.0f, 1.5f};

  CCDIKStats stats;
  bool ok = mmd_ccd_solve_chain(*arm_obj, chain, target, 40, 0.5f, &stats);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(stats.converged);
  EXPECT_LT(stats.final_error, 0.03f); /* Legacy CCD gets close but not sub-mm exact. */

  /* Tip tail should be close to target. */
  float tip_tail[3];
  solver_tail(chain[0].pchan, tip_tail);
  EXPECT_NEAR(tip_tail[0], target[0], 0.03f);
  EXPECT_NEAR(tip_tail[2], target[2], 0.03f);
}

TEST_F(MmdCCDIKTest, degenerate_chain_no_crash)
{
  /* Empty chain. */
  std::vector<CCDIKChainLink> empty;
  EXPECT_FALSE(mmd_ccd_solve_chain(*arm_obj, empty, nullptr, 10, 0.5f));

  /* Single bone chain. */
  build_two_bone_chain();
  std::vector<CCDIKChainLink> single(1);
  single[0].pchan = get_pchan("Root");
  float target[3] = {1.0f, 0.0f, 1.0f};
  EXPECT_FALSE(mmd_ccd_solve_chain(*arm_obj, single, target, 10, 0.5f));
}

TEST_F(MmdCCDIKTest, max_iterations_respected)
{
  build_two_bone_chain();
  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  /* Target far away, only 2 iterations — should not converge. */
  float target[3] = {5.0f, 0.0f, 1.0f};
  CCDIKStats stats;
  bool ok = mmd_ccd_solve_chain(*arm_obj, chain, target, 2, 0.1f, &stats);
  EXPECT_FALSE(ok);
  EXPECT_EQ(stats.iterations, 2);
  EXPECT_FALSE(stats.converged);
}

TEST_F(MmdCCDIKTest, angle_limits_respected)
{
  build_two_bone_chain();

  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  /* Tight limits on Tip bone: only -0.1..+0.1 rad on X, zero on Y/Z. */
  chain[0].limit_angle = true;
  chain[0].limit_min[0] = -0.1f;
  chain[0].limit_min[1] = 0.0f;
  chain[0].limit_min[2] = 0.0f;
  chain[0].limit_max[0] = 0.1f;
  chain[0].limit_max[1] = 0.0f;
  chain[0].limit_max[2] = 0.0f;

  /* Target far to +X — should not reach due to tight limits. */
  float target[3] = {3.0f, 0.0f, 1.5f};
  CCDIKStats stats;
  bool ok = mmd_ccd_solve_chain(*arm_obj, chain, target, 40, 0.5f, &stats);
  EXPECT_FALSE(ok);
  /* Error should remain large since limits prevent reaching target. */
  EXPECT_GT(stats.final_error, 0.1f);
}

TEST_F(MmdCCDIKTest, coordinate_conversion_limits)
{
  /* Test that MMD Y-up → Blender Z-up limit conversion is handled.
   * In Blender Z-up, Y axis is depth. A limit on Blender Y should not
   * affect a rotation around Blender Z. */
  build_two_bone_chain();

  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  /* Only allow rotation around Z (Blender up). Block X and Y. */
  chain[0].limit_angle = true;
  chain[0].limit_min[0] = 0.0f;
  chain[0].limit_min[1] = 0.0f;
  chain[0].limit_min[2] = -0.5f;
  chain[0].limit_max[0] = 0.0f;
  chain[0].limit_max[1] = 0.0f;
  chain[0].limit_max[2] = 0.5f;

  /* Target in X-Z plane. With X rotation blocked, should only rotate Z. */
  float target[3] = {1.0f, 0.0f, 1.5f};
  CCDIKStats stats;
  mmd_ccd_solve_chain(*arm_obj, chain, target, 40, 0.5f, &stats);

  /* Tip should have moved (not stuck). */
  EXPECT_LT(stats.final_error, 1.0f);
}

TEST_F(MmdCCDIKTest, large_angle_still_attempts)
{
  build_two_bone_chain();

  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  /* Target requiring ~90° bend. */
  float target[3] = {2.0f, 0.0f, 0.5f};
  CCDIKStats stats;
  mmd_ccd_solve_chain(*arm_obj, chain, target, 100, 0.3f, &stats);

  /* Should make significant progress even if not perfectly converged. */
  float tip_tail[3];
  solver_tail(chain[0].pchan, tip_tail);

  /* Tip should be closer to target than its original position at (0,0,2). */
  float orig[3] = {0.0f, 0.0f, 2.0f};
  float dist_orig = len_v3v3(orig, target);
  float dist_final = len_v3v3(tip_tail, target);
  EXPECT_LT(dist_final, dist_orig * 0.5f); /* At least 50% closer. */
}

TEST_F(MmdCCDIKTest, target_at_rest_position_no_change)
{
  build_two_bone_chain();

  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");

  /* Target exactly at tip tail rest position. */
  float target[3];
  solver_tail(chain[0].pchan, target);
  CCDIKStats stats;
  bool ok = mmd_ccd_solve_chain(*arm_obj, chain, target, 5, 0.5f, &stats);
  EXPECT_TRUE(ok);

  /* Tip should still be at rest. */
  float tip_tail[3];
  solver_tail(chain[0].pchan, tip_tail);
  EXPECT_NEAR(tip_tail[0], target[0], 0.01f);
  EXPECT_NEAR(tip_tail[2], target[2], 0.01f);
}

TEST_F(MmdCCDIKV2Test, uses_explicit_effector_and_converges)
{
  build_v2_chain();
  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");
  bPoseChannel *effector = get_pchan("Effector");
  ASSERT_NE(effector, nullptr);

  float target[3] = {0.5f, 0.0f, 1.5f};
  CCDIKStats stats;
  const float original_error = len_v3v3(effector->pose_mat[3], target);
  mmd_ccd_v2_solve_chain(*arm_obj, chain, *effector, target, 80, 0.5f, &stats);
  EXPECT_LT(stats.final_error, original_error * 0.05f);
  EXPECT_LT(stats.final_error, 0.05f);
  EXPECT_NEAR(effector->pose_mat[3][0], target[0], 0.02f);
  EXPECT_NEAR(effector->pose_mat[3][2], target[2], 0.05f);
}

TEST_F(MmdCCDIKV2Test, limits_complete_local_pose)
{
  build_v2_chain();
  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[1].pchan = get_pchan("Root");
  chain[0].limit_angle = true;
  chain[0].limit_min[0] = -0.1f;
  chain[0].limit_max[0] = 0.1f;
  chain[0].limit_min[1] = chain[0].limit_max[1] = 0.0f;
  chain[0].limit_min[2] = chain[0].limit_max[2] = 0.0f;
  bPoseChannel *effector = get_pchan("Effector");
  ASSERT_NE(effector, nullptr);

  float target[3] = {1.5f, 0.0f, 0.5f};
  CCDIKStats stats;
  mmd_ccd_v2_solve_chain(*arm_obj, chain, *effector, target, 40, 0.5f, &stats);

  float rotation[4], euler[3];
  mat4_to_quat(rotation, chain[0].pchan->chan_mat);
  quat_to_eulO(euler, ROT_MODE_XYZ, rotation);
  EXPECT_GE(euler[0], -0.1001f);
  EXPECT_LE(euler[0], 0.1001f);
  EXPECT_NEAR(euler[1], 0.0f, 1.0e-4f);
  EXPECT_NEAR(euler[2], 0.0f, 1.0e-4f);
}

TEST_F(MmdCCDIKV2Test, rejects_physics_owned_link_without_writing)
{
  build_v2_chain();
  std::vector<CCDIKChainLink> chain(2);
  chain[0].pchan = get_pchan("Tip");
  chain[0].physics_owned = true;
  chain[1].pchan = get_pchan("Root");
  bPoseChannel *effector = get_pchan("Effector");
  ASSERT_NE(effector, nullptr);
  float before[4][4];
  copy_m4_m4(before, chain[0].pchan->pose_mat);

  float target[3] = {0.5f, 0.0f, 1.5f};
  EXPECT_FALSE(mmd_ccd_v2_solve_chain(
      *arm_obj, chain, *effector, target, 40, 0.5f, nullptr));
  EXPECT_EQ(memcmp(before, chain[0].pchan->pose_mat, sizeof(before)), 0);
}

}  // namespace
}  // namespace blender::mmd::tests
