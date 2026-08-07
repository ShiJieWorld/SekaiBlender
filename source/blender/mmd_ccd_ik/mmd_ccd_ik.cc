/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * MMD native CCD IK solver implementation.
 *
 * FROZEN (2026-07-25): VMD import 路径不再使用此 solver，改用
 * iTaSC + influence F-Curve 方案。此代码仅用于实时 IK 模式。
 * 详见 mmd_ccd_ik_eval.cc 头注释和 project_memory.md。
 */

#include "mmd_ccd_ik.hh"

#include "BKE_action.hh"
#include "BKE_armature.hh"
#include "BKE_idprop.hh"

#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_path_utils.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_object_types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace blender::mmd {

/* -------------------------------------------------------------------- */
/* E1: Native IK toggle — persisted on Bone::system_properties           */
/* -------------------------------------------------------------------- */

static constexpr char kNativeIKEnabledProp[] = "mmd_native_ik_enabled";

void mmd_native_ik_set_enabled(Bone &bone, const bool enabled)
{
  IDProperty *bone_props = bone.system_properties;
  if (bone_props == nullptr) {
    bone_props = bke::idprop::create_group("mmd_native_ik").release();
    bone.system_properties = bone_props;
  }
  IDProperty *old = IDP_GetPropertyFromGroup_null(bone_props, kNativeIKEnabledProp);
  if (old != nullptr) {
    IDP_FreeFromGroup(bone_props, old);
  }
  IDP_AddToGroup(bone_props,
                 bke::idprop::create_bool(kNativeIKEnabledProp, enabled).release());
}

bool mmd_native_ik_is_enabled(const Bone &bone)
{
  const IDProperty *props = bone.system_properties;
  if (props == nullptr) {
    return true;
  }
  const IDProperty *prop = IDP_GetPropertyFromGroup_null(props, kNativeIKEnabledProp);
  if (prop == nullptr || prop->type != IDP_BOOLEAN) {
    return true;
  }
  return IDP_bool_get(prop) != 0;
}

bool mmd_ccd_use_legacy_solver()
{
  const char *value = BLI_getenv("MMD_IK_LEGACY");
  return value != nullptr && std::strcmp(value, "1") == 0;
}

bool mmd_ccd_use_v8_solver()
{
  const char *value = BLI_getenv("MMD_CCD_V8");
  /* V8 is the native default.  Set MMD_CCD_V8=0 for the old V2 path. */
  return value == nullptr || std::strcmp(value, "0") != 0;
}

static bool mmd_ccd_v2_trace_enabled()
{
  const char *value = BLI_getenv("MMD_CCD_V2_TRACE");
  return value != nullptr && std::strcmp(value, "1") == 0;
}

static void mmd_ccd_v2_trace_matrix(const char *stage,
                                     const char *bone_name,
                                     const float matrix[4][4])
{
  if (!mmd_ccd_v2_trace_enabled()) {
    return;
  }
  std::fprintf(stderr,
               "[MMD CCD V2] stage=%s bone=%s pose="
               "[% .7g,% .7g,% .7g,% .7g;"
               "% .7g,% .7g,% .7g,% .7g;"
               "% .7g,% .7g,% .7g,% .7g;"
               "% .7g,% .7g,% .7g,% .7g]\n",
               stage,
               bone_name != nullptr ? bone_name : "<null>",
               matrix[0][0],
               matrix[0][1],
               matrix[0][2],
               matrix[0][3],
               matrix[1][0],
               matrix[1][1],
               matrix[1][2],
               matrix[1][3],
               matrix[2][0],
               matrix[2][1],
               matrix[2][2],
               matrix[2][3],
               matrix[3][0],
               matrix[3][1],
               matrix[3][2],
               matrix[3][3]);
}

static void mmd_ccd_v2_trace_vec3(const char *stage, const char *bone_name, const float value[3])
{
  if (!mmd_ccd_v2_trace_enabled()) {
    return;
  }
  std::fprintf(stderr,
               "[MMD CCD V2] stage=%s bone=%s vec=(% .7g,% .7g,% .7g)\n",
               stage,
               bone_name != nullptr ? bone_name : "<null>",
               value[0],
               value[1],
               value[2]);
}

static void mmd_ccd_v2_trace_quat(const char *stage, const char *bone_name, const float value[4])
{
  if (!mmd_ccd_v2_trace_enabled()) {
    return;
  }
  std::fprintf(stderr,
               "[MMD CCD V2] stage=%s bone=%s quat=(% .7g,% .7g,% .7g,% .7g)\n",
               stage,
               bone_name != nullptr ? bone_name : "<null>",
               value[0],
               value[1],
               value[2],
               value[3]);
}

/* -------------------------------------------------------------------- */
/* E2: CCD solver                                                        */
/* -------------------------------------------------------------------- */

static constexpr float kConvergeThreshold = 1e-3f;
static constexpr float kDotEpsilon = 0.9999f;

/** Compute world-space head and tail for a pose channel.
 *
 * pose_mat is already an absolute pose-space matrix. Bone::arm_tail is also
 * an absolute rest-space point, so multiplying it by pose_mat would apply
 * the rest hierarchy twice. Blender's own pose-tail calculation uses the
 * posed bone Y axis and the bone length; keep CCD on that same contract. */
static void pose_world_head_tail(Object &armature_obj,
                                 bPoseChannel *pchan,
                                 float r_head[3],
                                 float r_tail[3])
{
  const float *head = pchan->pose_mat[3];
  if (r_head) {
    copy_v3_v3(r_head, head);
  }

  Bone *bone = pchan->bone_get(armature_obj);
  if (bone) {
    copy_v3_v3(r_tail, pchan->pose_mat[1]);
    mul_v3_fl(r_tail, bone->length);
    add_v3_v3(r_tail, head);
  }
  else {
    copy_v3_v3(r_tail, head);
  }
}

/** Clamp a world-space rotation quaternion by per-link Euler angle limits. */
static void clamp_to_limits(const CCDIKChainLink &link,
                            const float parent_world_mat[4][4],
                            float r_quat[4])
{
  if (!link.limit_angle) {
    return;
  }

  /* 1. World → bone-local. */
  float parent_quat[4];
  mat4_to_quat(parent_quat, parent_world_mat);
  normalize_qt(parent_quat);

  float parent_inv[4];
  invert_qt_qt_normalized(parent_inv, parent_quat);

  float local_quat[4];
  mul_qt_qtqt(local_quat, parent_inv, r_quat);
  normalize_qt(local_quat);

  /* 2. Decompose to XYZ Euler. */
  float euler[3];
  quat_to_eulO(euler, ROT_MODE_XYZ, local_quat);

  /* 3. Clamp. */
  bool clamped = false;
  for (int i = 0; i < 3; i++) {
    if (euler[i] < link.limit_min[i]) {
      euler[i] = link.limit_min[i];
      clamped = true;
    }
    else if (euler[i] > link.limit_max[i]) {
      euler[i] = link.limit_max[i];
      clamped = true;
    }
  }

  if (!clamped) {
    return;
  }

  /* 4. Re-assemble + 5. local → world. */
  float restricted[4];
  eulO_to_quat(restricted, euler, ROT_MODE_XYZ);
  normalize_qt(restricted);

  mul_qt_qtqt(r_quat, parent_quat, restricted);
  normalize_qt(r_quat);
}

/** Update the derived endpoints after changing pose_mat. */
static void update_pose_endpoints(Object &armature_obj, bPoseChannel &pchan)
{
  Bone *bone = pchan.bone_get(armature_obj);
  if (bone == nullptr) {
    return;
  }
  copy_v3_v3(pchan.pose_head, pchan.pose_mat[3]);
  BKE_pose_where_is_bone_tail({&pchan, bone});
}

/** Recompute FK for bones from start_idx down to tip (0).
 *
 * The channel matrix is the bone-local animated transform. Bone::arm_mat is
 * the absolute rest-pose matrix and must not be multiplied directly onto a
 * parent pose matrix: that would apply the parent's rest hierarchy twice.
 * Use Blender's normal parent-transform path instead, which also respects
 * connected offsets and inherit-scale/hinge flags. */
static void propagate_chain_fk(Object &armature_obj,
                               const std::vector<CCDIKChainLink> &chain,
                               int start_idx)
{
  for (int i = start_idx; i >= 0; i--) {
    bPoseChannel *child = chain[i].pchan;
    Bone *child_bone = child->bone_get(armature_obj);
    if (child_bone == nullptr) {
      continue;
    }

    BKE_armature_mat_bone_to_pose(
        {child, child_bone}, child->chan_mat, child->pose_mat);
    update_pose_endpoints(armature_obj, *child);
  }
}

/** Rebuild the pose-channel caches after CCD changed pose_mat.
 *
 * CCD runs from POSE_DONE, after Blender normally computes chan_mat and the
 * deform dual quaternion. Keep those caches and the original evaluated pose
 * channel consistent with the final matrix, otherwise the viewport may read
 * pose_mat from one state and pose_head/chan_mat from another until a later
 * depsgraph refresh. */
void mmd_ccd_ik_sync_pose_channel(Object &armature_obj,
                                  bPoseChannel &pchan,
                                  const bool publish_to_original)
{
  Bone *bone = pchan.bone_get(armature_obj);
  if (bone == nullptr) {
    return;
  }

  update_pose_endpoints(armature_obj, pchan);

  /* Convert the final pose matrix back to the bone-local channel matrix
   * using the current parent pose. A manual pose_mat * inverse(arm_mat)
   * conversion is only valid for a simple root and loses parent offsets for
   * ordinary connected bones. */
  BKE_armature_mat_pose_to_bone({&pchan, bone}, pchan.pose_mat, pchan.chan_mat);

  /* Keep the local channel matrix for the remaining pose-tree refresh, but
   * build deformation from the absolute pose/rest delta.  Blender's normal
   * pose-bone-done step uses this matrix for dual-quaternion skinning; using
   * the local channel matrix here makes weighted meshes jump even when the
   * evaluated pose matrices are correct. */
  float rest_inverse[4][4];
  float deform_mat[4][4];
  invert_m4_m4(rest_inverse, bone->arm_mat);
  mul_m4_m4m4(deform_mat, pchan.pose_mat, rest_inverse);
  if (!(bone->flag & BONE_NO_DEFORM)) {
    mat4_to_dquat(&pchan.runtime.deform_dual_quat, bone->arm_mat, deform_mat);
  }

  if (publish_to_original && pchan.orig_pchan != nullptr) {
    copy_m4_m4(pchan.orig_pchan->pose_mat, pchan.pose_mat);
    copy_m4_m4(pchan.orig_pchan->chan_mat, pchan.chan_mat);
    copy_v3_v3(pchan.orig_pchan->loc, pchan.loc);
    copy_qt_qt(pchan.orig_pchan->quat, pchan.quat);
    copy_v3_v3(pchan.orig_pchan->eul, pchan.eul);
    copy_v3_v3(pchan.orig_pchan->rotAxis, pchan.rotAxis);
    pchan.orig_pchan->rotAngle = pchan.rotAngle;
    copy_v3_v3(pchan.orig_pchan->scale, pchan.scale);
    copy_v3_v3(pchan.orig_pchan->pose_head, pchan.pose_head);
    copy_v3_v3(pchan.orig_pchan->pose_tail, pchan.pose_tail);
    pchan.orig_pchan->constflag = pchan.constflag;
  }
}

void mmd_ccd_ik_finalize_pose_deform(Object &armature_obj, const bool publish_to_original)
{
  if (armature_obj.pose == nullptr) {
    return;
  }

  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(armature_obj.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    Bone *bone = pchan->bone_get(armature_obj);
    if (bone == nullptr) {
      continue;
    }

    /* Armature modifiers use the absolute rest-to-pose delta here.  Keep this
     * final pass separate from the local channel matrices needed while the
     * pose tree is being rebuilt. */
    float rest_inverse[4][4];
    float deform_mat[4][4];
    invert_m4_m4(rest_inverse, bone->arm_mat);
    mul_m4_m4m4(deform_mat, pchan->pose_mat, rest_inverse);
    copy_m4_m4(pchan->chan_mat, deform_mat);
    if (!(bone->flag & BONE_NO_DEFORM)) {
      mat4_to_dquat(&pchan->runtime.deform_dual_quat, bone->arm_mat, deform_mat);
    }

    if (publish_to_original && pchan->orig_pchan != nullptr) {
      copy_m4_m4(pchan->orig_pchan->pose_mat, pchan->pose_mat);
      copy_m4_m4(pchan->orig_pchan->chan_mat, pchan->chan_mat);
      copy_v3_v3(pchan->orig_pchan->loc, pchan->loc);
      copy_qt_qt(pchan->orig_pchan->quat, pchan->quat);
      copy_v3_v3(pchan->orig_pchan->eul, pchan->eul);
      copy_v3_v3(pchan->orig_pchan->rotAxis, pchan->rotAxis);
      pchan->orig_pchan->rotAngle = pchan->rotAngle;
      copy_v3_v3(pchan->orig_pchan->scale, pchan->scale);
      copy_v3_v3(pchan->orig_pchan->pose_head, pchan->pose_head);
      copy_v3_v3(pchan->orig_pchan->pose_tail, pchan->pose_tail);
      pchan->orig_pchan->constflag = pchan->constflag;
    }
  }
}

static void append_modified_channel(std::vector<bPoseChannel *> *r_modified_channels,
                                     bPoseChannel &pchan)
{
  if (r_modified_channels != nullptr &&
      std::find(r_modified_channels->begin(), r_modified_channels->end(), &pchan) ==
          r_modified_channels->end())
  {
    r_modified_channels->push_back(&pchan);
  }
}

static void propagate_descendants_fk(Object &armature_obj,
                                     Bone &parent_bone,
                                     std::vector<bPoseChannel *> *r_modified_channels)
{
  for (Bone *bone = static_cast<Bone *>(parent_bone.childbase.first); bone != nullptr;
       bone = bone->next)
  {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature_obj.pose, bone->name);
    if (pchan == nullptr) {
      continue;
    }
    BKE_armature_mat_bone_to_pose({pchan, bone}, pchan->chan_mat, pchan->pose_mat);
    update_pose_endpoints(armature_obj, *pchan);
    append_modified_channel(r_modified_channels, *pchan);
    propagate_descendants_fk(armature_obj, *bone, r_modified_channels);
  }
}

/** Apply a world/pose-space CCD delta, then limit the resulting complete
 * channel-local rotation. This is the important V2 distinction: PMX limits
 * constrain the candidate pose, not the incremental quaternion. */
static void apply_v2_rotation(Object &armature_obj,
                              const CCDIKChainLink &link,
                              const float delta[4],
                              std::vector<bPoseChannel *> *r_modified_channels)
{
  bPoseChannel &pchan = *link.pchan;
  Bone *bone = pchan.bone_get(armature_obj);
  if (bone == nullptr) {
    return;
  }

  const size_t modified_count_before = r_modified_channels != nullptr ?
                                           r_modified_channels->size() :
                                           0;
  mmd_ccd_v2_trace_quat("before_apply_delta", pchan.name, delta);
  mmd_ccd_v2_trace_matrix("before_apply", pchan.name, pchan.pose_mat);

  float delta_mat[3][3];
  quat_to_mat3(delta_mat, delta);
  float old_orient[3][3], new_orient[3][3];
  copy_m3_m4(old_orient, pchan.pose_mat);
  mul_m3_m3m3(new_orient, delta_mat, old_orient);

  float candidate_pose[4][4];
  copy_m4_m4(candidate_pose, pchan.pose_mat);
  for (int row = 0; row < 3; row++) {
    for (int column = 0; column < 3; column++) {
      candidate_pose[row][column] = new_orient[row][column];
    }
  }

  float candidate_channel[4][4];
  BKE_armature_mat_pose_to_bone({&pchan, bone}, candidate_pose, candidate_channel);

  if (link.limit_angle) {
    float location[3], rotation[4], scale[3];
    mat4_to_loc_quat(location, rotation, candidate_channel);
    mat4_to_size(scale, candidate_channel);
    normalize_qt(rotation);

    float euler[3];
    quat_to_eulO(euler, ROT_MODE_XYZ, rotation);
    for (int axis = 0; axis < 3; axis++) {
      if (!link.limit_axis[axis]) {
        continue;
      }
      euler[axis] = clamp_f(euler[axis], link.limit_min[axis], link.limit_max[axis]);
    }
    eulO_to_quat(rotation, euler, ROT_MODE_XYZ);
    normalize_qt(rotation);
    loc_quat_size_to_mat4(candidate_channel, location, rotation, scale);
  }

  copy_m4_m4(pchan.chan_mat, candidate_channel);
  BKE_armature_mat_bone_to_pose({&pchan, bone}, pchan.chan_mat, pchan.pose_mat);
  update_pose_endpoints(armature_obj, pchan);
  append_modified_channel(r_modified_channels, pchan);
  mmd_ccd_v2_trace_matrix("after_apply", pchan.name, pchan.pose_mat);
  if (mmd_ccd_v2_trace_enabled()) {
    const size_t modified_count_after = r_modified_channels != nullptr ?
                                             r_modified_channels->size() :
                                             0;
    std::fprintf(stderr,
                 "[MMD CCD V3] stage=apply_result bone=%s modified_before=%zu "
                 "modified_after=%zu\n",
                 pchan.name,
                 modified_count_before,
                 modified_count_after);
  }
}

bool mmd_ccd_v2_solve_chain(Object &armature_obj,
                            const std::vector<CCDIKChainLink> &chain,
                            bPoseChannel &effector,
                            const float target_pose[3],
                            const int loop_count,
                            const float angle_limit,
                            CCDIKStats *r_stats,
                            std::vector<bPoseChannel *> *r_modified_channels)
{
  if (chain.empty() || loop_count < 1 || target_pose == nullptr) {
    if (r_stats) {
      *r_stats = {};
    }
    return false;
  }
  for (const CCDIKChainLink &link : chain) {
    if (link.pchan == nullptr || link.physics_owned) {
      if (r_stats) {
        *r_stats = {};
      }
      return false;
    }
  }

  bool converged = false;
  int iteration = 0;
  for (iteration = 1; iteration <= loop_count; iteration++) {
    const float error = len_v3v3(effector.pose_mat[3], target_pose);
    if (error <= kConvergeThreshold) {
      converged = true;
      break;
    }

    for (int link_index = 0; link_index < int(chain.size()); link_index++) {
      bPoseChannel &pchan = *chain[link_index].pchan;
      float to_effector[3], to_target[3];
      sub_v3_v3v3(to_effector, effector.pose_mat[3], pchan.pose_mat[3]);
      sub_v3_v3v3(to_target, target_pose, pchan.pose_mat[3]);
      if (normalize_v3(to_effector) < 1e-8f || normalize_v3(to_target) < 1e-8f) {
        continue;
      }

      const float cosine = clamp_f(dot_v3v3(to_effector, to_target), -1.0f, 1.0f);
      if (cosine > kDotEpsilon) {
        continue;
      }
      float axis[3];
      cross_v3_v3v3(axis, to_effector, to_target);
      if (normalize_v3(axis) < 1e-8f) {
        continue;
      }

      const float angle = acosf(cosine);
      const float max_step = std::max(0.0f, angle_limit);
      const float applied_angle = max_step > 0.0f ? std::min(angle, max_step) : angle;
      float delta[4];
      axis_angle_to_quat(delta, axis, applied_angle);
      normalize_qt(delta);
      apply_v2_rotation(armature_obj, chain[link_index], delta, r_modified_channels);
      if (Bone *bone = pchan.bone_get(armature_obj)) {
        propagate_descendants_fk(armature_obj, *bone, r_modified_channels);
      }
    }
  }

  if (r_stats) {
    r_stats->iterations = iteration - 1;
    r_stats->converged = converged;
    r_stats->final_error = len_v3v3(effector.pose_mat[3], target_pose);
  }
  return converged;
}

bool mmd_ccd_solve_chain(Object &armature_obj,
                         const std::vector<CCDIKChainLink> &chain,
                         const float target_world[3],
                         const int loop_count,
                         const float angle_limit,
                         CCDIKStats *r_stats)
{
  if (chain.size() < 2 || loop_count < 1) {
    if (r_stats) {
      r_stats->converged = false;
      r_stats->iterations = 0;
      r_stats->final_error = 0.0f;
    }
    return false;
  }

  /* Sanity: every link must have a valid pose channel. */
  for (const auto &link : chain) {
    if (!link.pchan) {
      if (r_stats) {
        r_stats->converged = false;
        r_stats->iterations = 0;
        r_stats->final_error = 0.0f;
      }
      return false;
    }
  }

  const int chain_len = int(chain.size());

  bool converged = false;
  int iter;
  for (iter = 1; iter <= loop_count; iter++) {
    /* Check convergence. */
    float tip_tail[3];
    pose_world_head_tail(armature_obj, chain[0].pchan, nullptr, tip_tail);
    const float error = len_v3v3(tip_tail, target_world);
    if (error <= kConvergeThreshold) {
      converged = true;
      break;
    }

    /* Iterate bones from tip toward root. */
    for (int bone_idx = 0; bone_idx < chain_len; bone_idx++) {
      bPoseChannel *pchan = chain[bone_idx].pchan;

      float cur_tip[3];
      pose_world_head_tail(armature_obj, chain[0].pchan, nullptr, cur_tip);

      float head_world[3], tail_world[3];
      pose_world_head_tail(armature_obj, pchan, head_world, tail_world);

      float dir_tip[3], dir_target[3];
      sub_v3_v3v3(dir_tip, cur_tip, head_world);
      sub_v3_v3v3(dir_target, target_world, head_world);

      const float len_tip = normalize_v3(dir_tip);
      const float len_target = normalize_v3(dir_target);

      if (len_tip < 1e-8f || len_target < 1e-8f) {
        continue;
      }

      const float cos_a = dot_v3v3(dir_tip, dir_target);
      if (cos_a > kDotEpsilon) {
        continue;
      }

      float axis[3];
      cross_v3_v3v3(axis, dir_tip, dir_target);
      const float axis_len = normalize_v3(axis);
      if (axis_len < 1e-8f) {
        continue;
      }
      const float angle = acosf(clamp_f(cos_a, -1.0f, 1.0f));
      float rotation[4];
      axis_angle_to_quat(rotation, axis, angle);
      normalize_qt(rotation);

      /* Step-size limit. */
      const float step = (iter > 1) ? (angle_limit / float(iter)) : angle_limit;
      if (angle > step) {
        float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        interp_qt_qtqt(rotation, identity, rotation, step / angle);
        normalize_qt(rotation);
      }

      /* Angle limits. */
      if (chain[bone_idx].limit_angle) {
        float parent_world[4][4];
        if (bone_idx < chain_len - 1) {
          copy_m4_m4(parent_world, chain[bone_idx + 1].pchan->pose_mat);
        }
        else {
          unit_m4(parent_world);
        }
        clamp_to_limits(chain[bone_idx], parent_world, rotation);
      }

      /* Apply rotation to bone. */
      float rot_mat[3][3];
      quat_to_mat3(rot_mat, rotation);

      float old_orient[3][3], new_orient[3][3];
      copy_m3_m4(old_orient, pchan->pose_mat);
      mul_m3_m3m3(new_orient, rot_mat, old_orient);
      /* Copy 3x3 orientation back into the upper-left of the 4x4 pose_mat. */
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          pchan->pose_mat[r][c] = new_orient[r][c];
        }
      }
      /* Keep the bone-local channel transform in lockstep with the direct
       * pose_mat edit. Otherwise a later parent propagation rebuilds this
       * bone from stale animation data and turns the chain into a whip. */
      Bone *pchan_bone = pchan->bone_get(armature_obj);
      if (pchan_bone != nullptr) {
        BKE_armature_mat_pose_to_bone(
            {pchan, pchan_bone}, pchan->pose_mat, pchan->chan_mat);
        copy_v3_v3(pchan->pose_head, pchan->pose_mat[3]);
        BKE_pose_where_is_bone_tail({pchan, pchan_bone});
      }

      /* Propagate FK to children. */
      propagate_chain_fk(armature_obj, chain, bone_idx - 1);
    }
  }

  if (r_stats) {
    r_stats->iterations = iter - 1;
    r_stats->converged = converged;
    float tip_tail[3];
    pose_world_head_tail(armature_obj, chain[0].pchan, nullptr, tip_tail);
    r_stats->final_error = len_v3v3(tip_tail, target_world);
  }

  return converged;
}

}  // namespace blender::mmd
