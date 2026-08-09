/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * E3: Depsgraph integration — called from BKE_pose_eval_done() to run
 * MMD native CCD IK after FCurves and constraints have been applied.
 *
 * ─────────────────────────────────────────────────────────────────
 * FROZEN (2026-07-25): VMD import no longer uses this CCD solver.
 * VMD 大角度兼容性已改用 iTaSC + influence F-Curve 方案（见
 * io_vmd_ops.cc vmd_suspend_mmd_approx_constraints + vmd_import.cc
 * apply_vmd_ik_toggle）。该方案已稳定复现 mmd_tools 导入效果。
 *
 * CCD 求解器目前仅用于实时 IK 模式（手动拖动 IK 控制骨），通过
 * itasc_plugin.cc 的 is_native_mmd_ik_approx() 排除 iTaSC 后由
 * POSE_DONE 阶段接管。不要在 VMD 路径重新启用此 solver。
 *
 * 若未来遇到 iTaSC 无法处理的特殊 IK 链，可重新激活此代码。
 * 详见 project_memory.md「iTaSC influence 驱动方案」段。
 * ─────────────────────────────────────────────────────────────────
 */

#include "mmd_ccd_ik_eval.hh"
#include "mmd_ccd_ik.hh"
#include "mmd_ccd_ik_v8.hh"

#include "ANIM_action.hh"
#include "ANIM_action_legacy.hh"
#include "BKE_action.hh"
#include "BKE_armature.hh"
#include "BKE_fcurve.hh"
#include "BKE_idprop.hh"
#include "BKE_scene.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"

#include "DNA_anim_types.h"
#include "DNA_armature_types.h"
#include "DNA_constraint_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph_query.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace blender::mmd {

/* -------------------------------------------------------------------- */
/* IDProperty schema constants (schema version 1)                        */
/* -------------------------------------------------------------------- */

static constexpr char kIKDefinitionProp[] = "mmd_pmx_bone_ik_definition";
static constexpr char kIKBonesField[] = "ik_bones";
static constexpr char kBoneNameField[] = "name";
static constexpr char kTargetField[] = "target";
static constexpr char kLoopCountField[] = "loop_count";
static constexpr char kAngleLimitField[] = "angle_limit";
static constexpr char kLinksField[] = "links";
static constexpr char kLinkBoneField[] = "bone";
static constexpr char kLinkPhysicsOwnedField[] = "physics_owned";
static constexpr char kLimitAngleField[] = "limit_angle";
static constexpr char kLimitMinField[] = "limit_min";
static constexpr char kLimitMaxField[] = "limit_max";
static constexpr char kApproxConstraintName[] = "MMD_IK_Approx";

/* -------------------------------------------------------------------- */
/* Helpers                                                                */
/* -------------------------------------------------------------------- */

/** Mute all MMD_IK_Approx constraints and record old enforce values. */
struct ConstraintRestore {
  bConstraint *con;
  float old_enforce;
};

static void mute_approx_constraints(Object &armature_obj,
                                    std::vector<ConstraintRestore> &r_restore)
{
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(armature_obj.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first);
         con != nullptr;
         con = con->next)
    {
      if (std::strcmp(con->name, kApproxConstraintName) == 0) {
        r_restore.push_back({con, con->enforce});
        con->enforce = 0.0f;
      }
    }
  }
}

/** Restore previously muted constraint enforce values. */
static void restore_approx_constraints(const std::vector<ConstraintRestore> &restore)
{
  for (const auto &r : restore) {
    r.con->enforce = r.old_enforce;
  }
}

/** Check if any IK bone in the definition has native CCD enabled. */
static bool has_any_native_ik_enabled(Object &armature_obj, IDProperty &ik_bones_arr)
{
  bArmature *arm = reinterpret_cast<bArmature *>(armature_obj.data);
  for (int i = 0; i < ik_bones_arr.len; i++) {
    IDProperty *item = IDP_GetIndexArray(&ik_bones_arr, i);
    if (!item || item->type != IDP_GROUP) {
      continue;
    }
    IDProperty *name_prop = IDP_GetPropertyTypeFromGroup(item, kBoneNameField, IDP_STRING);
    if (!name_prop) {
      continue;
    }
    Bone *bone = BKE_armature_find_bone_name(arm, IDP_string_get(name_prop));
    if (bone && mmd_native_ik_is_enabled(*bone)) {
      return true;
    }
  }
  return false;
}

/* -------------------------------------------------------------------- */
/* V8 evaluation path                                                    */
/* -------------------------------------------------------------------- */

/** 收集 pchan 及其所有祖先到 involved set。 */
static void v8_collect_pchan_ancestors(bPoseChannel *pchan,
                                       std::set<bPoseChannel *> &r_involved)
{
  for (bPoseChannel *p = pchan; p != nullptr; p = p->parent) {
    r_involved.insert(p);
  }
}

/** V8 link 限制数据（PMX 原始，不做 YZ swap/符号翻转）。 */
struct V8LinkLimitData {
  bool has_limit = false;
  float limit_min[3] = {};
  float limit_max[3] = {};
};

/** V8 链定义临时存储（从 IDProperty 解析）。 */
struct V8ChainDef {
  bPoseChannel *target_pchan = nullptr;
  bPoseChannel *effector_pchan = nullptr;
  std::vector<bPoseChannel *> link_pchans;
  std::vector<V8LinkLimitData> link_limits;
  float angle_limit = float(M_PI_4);
};

/** 从 armature root 做 DFS，按拓扑顺序（从根到叶）收集 involved 骨骼到 bone pool。
 *  preorder DFS 保证父骨骼在子骨骼之前被加入 pool。 */
static void v8_build_bone_pool_topological(
    bArmature *arm,
    bPose *pose,
    const std::set<bPoseChannel *> &involved,
    std::vector<bPoseChannel *> &r_bone_pool,
    std::map<bPoseChannel *, int> &r_bone_index)
{
  std::vector<Bone *> stack;
  for (Bone *bone = static_cast<Bone *>(arm->bonebase.first); bone != nullptr;
       bone = bone->next)
  {
    stack.push_back(bone);
  }
  while (!stack.empty()) {
    Bone *bone = stack.back();
    stack.pop_back();
    bPoseChannel *pchan = BKE_pose_channel_find_name(pose, bone->name);
    if (pchan != nullptr && involved.count(pchan) != 0) {
      r_bone_index[pchan] = int(r_bone_pool.size());
      r_bone_pool.push_back(pchan);
    }
    for (Bone *child = static_cast<Bone *>(bone->childbase.first); child != nullptr;
         child = child->next)
    {
      stack.push_back(child);
    }
  }
}

/** 递归刷新 IK link 骨骼所有后代并发布。
 *
 * 后代可能是 Append/Copy/D/shadow 骨，只从旧 chan_mat 做 FK 会跳过它们
 * 的约束，导致网格使用的 pose_mat 与骨架链脱节。使用标准 bone evaluation
 * 重新执行通道和约束，再同步 deform/original 缓存。 */
static bool v8_has_approx_ik(const bPoseChannel &pchan)
{
  for (const bConstraint &constraint : pchan.constraints) {
    if (std::strcmp(constraint.name, kApproxConstraintName) == 0 ||
        std::strcmp(constraint.name, "MMD_IK_Limit") == 0)
    {
      return true;
    }
  }
  return false;
}

static void v8_propagate_descendants_fk(Depsgraph *depsgraph,
                                         Scene *scene,
                                         Object &armature_obj,
                                         Bone &parent_bone,
                                         bool publish_to_original)
{
  for (Bone *child = static_cast<Bone *>(parent_bone.childbase.first); child != nullptr;
       child = child->next)
  {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature_obj.pose, child->name);
    if (pchan == nullptr) {
      continue;
    }
    if (v8_has_approx_ik(*pchan)) {
      /* Native V8 already owns this link.  Re-running the persisted
       * MMD_IK_Approx constraint here would solve the same chain a second
       * time and overwrite the V8 result. */
      BKE_pchan_calc_mat({pchan, child});
      BKE_armature_mat_bone_to_pose({pchan, child}, pchan->chan_mat, pchan->pose_mat);
      BKE_pose_where_is_bone_tail({pchan, child});
    }
    else {
      BKE_pose_where_is_bone(depsgraph,
                             scene,
                             &armature_obj,
                             pchan,
                             scene != nullptr ? BKE_scene_ctime_get(scene) : 0.0f,
                             true);
    }
    mmd_ccd_ik_sync_pose_channel(armature_obj, *pchan, publish_to_original);
    v8_propagate_descendants_fk(
        depsgraph, scene, armature_obj, *child, publish_to_original);
  }
}

static void v8_refresh_pose_tree_recursive(Depsgraph *depsgraph,
                                           Scene *scene,
                                           Object &armature_obj,
                                           Bone *bone,
                                           const std::set<bPoseChannel *> &v8_links,
                                           bool publish_to_original)
{
  for (Bone *cur = bone; cur != nullptr; cur = cur->next) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature_obj.pose, cur->name);
    if (pchan != nullptr) {
      if (v8_links.count(pchan) != 0 || v8_has_approx_ik(*pchan)) {
        BKE_armature_mat_bone_to_pose({pchan, cur}, pchan->chan_mat, pchan->pose_mat);
        BKE_pose_where_is_bone_tail({pchan, cur});
      }
      else {
        BKE_pose_where_is_bone(depsgraph,
                               scene,
                               &armature_obj,
                               pchan,
                               scene != nullptr ? BKE_scene_ctime_get(scene) : 0.0f,
                               true);
      }
      mmd_ccd_ik_sync_pose_channel(armature_obj, *pchan, publish_to_original);
    }
    v8_refresh_pose_tree_recursive(
        depsgraph, scene, armature_obj, static_cast<Bone *>(cur->childbase.first), v8_links, publish_to_original);
  }
}

static void v8_refresh_pose_tree_after_solve(Depsgraph *depsgraph,
                                             Scene *scene,
                                             Object &armature_obj,
                                             const std::set<bPoseChannel *> &v8_links,
                                             bool publish_to_original)
{
  bArmature *arm = reinterpret_cast<bArmature *>(armature_obj.data);
  if (arm == nullptr || armature_obj.pose == nullptr) {
    return;
  }
  v8_refresh_pose_tree_recursive(depsgraph,
                                 scene,
                                 armature_obj,
                                 static_cast<Bone *>(arm->bonebase.first),
                                 v8_links,
                                 publish_to_original);
}

/** 构造 BoneConverter 转换四元数 q_conv。
 *  q_conv 用于 bl↔mmd 四元数转换：
 *    bl→mmd: conj(q_conv) * bl_q * q_conv
 *    mmd→bl: q_conv * mmd_q * conj(q_conv) */
static void v8_compute_bone_converter(Bone &bone, float r_q_conv[4])
{
  /* 从 Bone::arm_mat 提取 3x3 旋转 */
  float arm_rot[3][3];
  copy_m3_m4(arm_rot, bone.arm_mat);
  normalize_m3(arm_rot);
  /* YZ 列交换（与 vmd_action.cc BoneConverter 保持一致，用于四元数共轭转换） */
  for (int i = 0; i < 3; i++) {
    std::swap(arm_rot[i][1], arm_rot[i][2]);
  }
  /* 转置 = 逆变换矩阵 */
  float conv_mat[3][3];
  transpose_m3_m3(conv_mat, arm_rot);
  /* 转四元数 */
  mat3_to_quat(r_q_conv, conv_mat);
  normalize_qt(r_q_conv);
}

/** Read the animation-layer rotation without consuming constraint-mutated pose data. */
static bool v8_read_action_rotation(const Object &armature_obj,
                                    const bPoseChannel &pchan,
                                    float eval_time,
                                    float r_quat[4])
{
  if (armature_obj.adt == nullptr || armature_obj.adt->action == nullptr)
  {
    return false;
  }

  const std::string prefix = "pose.bones[\"" + std::string(pchan.name) + "\"].";
  const animrig::Action &action = armature_obj.adt->action->wrap();
  blender::Span<const FCurve *> fcurves;
  blender::Vector<const FCurve *> all_fcurves;
  if (armature_obj.adt->slot_handle != animrig::Slot::unassigned) {
    fcurves = animrig::fcurves_for_action_slot(action, armature_obj.adt->slot_handle);
  }
  if (fcurves.is_empty()) {
    all_fcurves = animrig::legacy::fcurves_all(
        static_cast<const bAction *>(armature_obj.adt->action));
    fcurves = all_fcurves.as_span();
  }

  auto read_curves = [&](const char *property, int component_count, float *values) {
    bool found = false;
    const std::string path = prefix + property;
    for (const FCurve *fcurve : fcurves) {
      if (fcurve != nullptr && !fcurve->rna_path().is_empty() && fcurve->rna_path() == path &&
          fcurve->array_index >= 0 && fcurve->array_index < component_count)
      {
        values[fcurve->array_index] = evaluate_fcurve_only_curve(fcurve, eval_time);
        found = true;
      }
    }
    return found;
  };

  float values[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  if (read_curves("rotation_quaternion", 4, values)) {
    copy_qt_qt(r_quat, values);
    normalize_qt(r_quat);
    return true;
  }

  float euler[3] = {0.0f, 0.0f, 0.0f};
  if (read_curves("rotation_euler", 3, euler)) {
    eulO_to_quat(r_quat, euler, pchan.rotmode);
    normalize_qt(r_quat);
    return true;
  }

  float axis_angle[4] = {0.0f, 0.0f, 1.0f, 0.0f};
  if (read_curves("rotation_axis_angle", 4, axis_angle)) {
    axis_angle_to_quat(r_quat, &axis_angle[1], axis_angle[0]);
    normalize_qt(r_quat);
    return true;
  }
  return false;
}

/** 从 pchan->pose_mat 构造 MMD m0（row-major，YZ swap）。
 *  m0_translation = world_pos_mmd - base_pos_mmd * R_mmd（row-vector）
 *  m0_mmd = [R_mmd | m0_translation; 0 0 0 1] */
static void v8_build_initial_m0(bPoseChannel &pchan,
                                CCDIKV8Bone &vb,
                                float global_scale)
{
  /* 1. 从 pose_mat 提取旋转 R_bl，YZ swap → R_mmd
   *   R_mmd = S * R_bl * S（先交换行，再交换列），S=YZ swap。
   *   只交换列会得到 det=-1 的反射矩阵，必须行列都交换。 */
  float r_mmd[3][3];
  copy_m3_m4(r_mmd, pchan.pose_mat);
  normalize_m3(r_mmd);
  for (int i = 0; i < 3; i++) {
    std::swap(r_mmd[i][1], r_mmd[i][2]);  /* 交换列 */
  }
  for (int i = 0; i < 3; i++) {
    std::swap(r_mmd[1][i], r_mmd[2][i]);  /* 交换行 */
  }

  /* 2. 世界位置_mmd = (pose_mat[3][0]/scale, pose_mat[3][2]/scale, pose_mat[3][1]/scale) */
  float world_pos_mmd[3] = {
      pchan.pose_mat[3][0] / global_scale,
      pchan.pose_mat[3][2] / global_scale,
      pchan.pose_mat[3][1] / global_scale,
  };

  /* 3. base_pos_mmd * R_mmd (row-vector) */
  float base_times_r[3];
  for (int c = 0; c < 3; c++) {
    base_times_r[c] = vb.base_pos_mmd[0] * r_mmd[0][c] +
                      vb.base_pos_mmd[1] * r_mmd[1][c] +
                      vb.base_pos_mmd[2] * r_mmd[2][c];
  }

  /* 4/5. m0_mmd = [R_mmd | m0_translation; 0 0 0 1] */
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      vb.initial_m0_mmd[r][c] = r_mmd[r][c];
    }
    vb.initial_m0_mmd[r][3] = 0.0f;
  }
  for (int c = 0; c < 3; c++) {
    vb.initial_m0_mmd[3][c] = world_pos_mmd[c] - base_times_r[c];
  }
  vb.initial_m0_mmd[3][3] = 1.0f;
}

static bool v8_trace_enabled()
{
  const char *t = BLI_getenv("MMD_CCD_V8_TRACE");
  return t != nullptr && std::strcmp(t, "1") == 0;
}

static void v8_trace_matrix4(const char *stage, const char *bone_name, const float m[4][4])
{
  if (!v8_trace_enabled()) {
    return;
  }
  std::fprintf(stderr,
               "[V8W] %s bone=%s rows=[%+.6f,%+.6f,%+.6f,%+.6f;"
               "%+.6f,%+.6f,%+.6f,%+.6f;"
               "%+.6f,%+.6f,%+.6f,%+.6f;"
               "%+.6f,%+.6f,%+.6f,%+.6f]\n",
               stage,
               bone_name,
               m[0][0], m[0][1], m[0][2], m[0][3],
               m[1][0], m[1][1], m[1][2], m[1][3],
               m[2][0], m[2][1], m[2][2], m[2][3],
               m[3][0], m[3][1], m[3][2], m[3][3]);
}

static bool v8_trace_right_leg_bone(const char *name)
{
  return std::strcmp(name, "右足") == 0 || std::strcmp(name, "右ひざ") == 0 ||
         std::strcmp(name, "右足首") == 0 || std::strcmp(name, "右つま先") == 0 ||
         std::strcmp(name, "右足D") == 0 || std::strcmp(name, "右ひざD") == 0 ||
         std::strcmp(name, "右足首D") == 0;
}

/** V8 评估入口：采集数据 → 调用 V8 求解 → 发布结果。
 *  1. 从 Blender 采集数据并转换到 MMD 空间
 *  2. 建立 bone pool（拓扑排序）
 *  3. 调用 V8 求解
 *  4. 把 V8 结果转回 Blender 并发布 */
static void mmd_ccd_v8_evaluate(Depsgraph *depsgraph,
                                Object &armature_obj,
                                IDProperty &ik_bones_arr,
                                float eval_time,
                                bool publish_to_original)
{
  (void)depsgraph;  /* V8 不依赖 depsgraph */

  bPose *pose = armature_obj.pose;
  bArmature *arm = reinterpret_cast<bArmature *>(armature_obj.data);
  if (pose == nullptr || arm == nullptr) {
    return;
  }

  /* 3a. 读取 global_scale（缺失回退 0.08f） */
  float global_scale = 0.08f;
  IDProperty *scale_prop = IDP_GetPropertyTypeFromGroup(
      armature_obj.id.system_properties, "mmd_pmx_global_scale", IDP_FLOAT);
  if (scale_prop != nullptr) {
    global_scale = IDP_float_get(scale_prop);
  }
  if (global_scale == 0.0f) {
    global_scale = 0.08f;
  }

  /* 3b. 解析所有 IK 链定义，收集涉及骨骼 */
  std::vector<V8ChainDef> chain_defs;
  std::set<bPoseChannel *> involved;

  for (int i = 0; i < ik_bones_arr.len; i++) {
    IDProperty *item = IDP_GetIndexArray(&ik_bones_arr, i);
    if (item == nullptr || item->type != IDP_GROUP) {
      continue;
    }

    /* target = IK 控制骨 name */
    IDProperty *name_prop = IDP_GetPropertyTypeFromGroup(item, kBoneNameField, IDP_STRING);
    if (name_prop == nullptr) {
      continue;
    }
    bPoseChannel *target_pchan = BKE_pose_channel_find_name(pose, IDP_string_get(name_prop));
    if (target_pchan == nullptr) {
      continue;
    }

    /* effector = target field（链末端骨） */
    IDProperty *target_prop = IDP_GetPropertyTypeFromGroup(item, kTargetField, IDP_STRING);
    if (target_prop == nullptr) {
      continue;
    }
    bPoseChannel *effector_pchan = BKE_pose_channel_find_name(pose, IDP_string_get(target_prop));
    if (effector_pchan == nullptr) {
      continue;
    }

    /* links */
    IDProperty *links_arr = IDP_GetPropertyTypeFromGroup(item, kLinksField, IDP_IDPARRAY);
    if (links_arr == nullptr || links_arr->len < 1) {
      continue;
    }

    /* angle_limit */
    float angle_limit = float(M_PI_4);
    IDProperty *angle_prop = IDP_GetPropertyTypeFromGroup(item, kAngleLimitField, IDP_FLOAT);
    if (angle_prop != nullptr) {
      angle_limit = IDP_float_get(angle_prop);
    }
    if (angle_limit <= 0.0f) {
      angle_limit = float(M_PI_4);
    }

    V8ChainDef def;
    def.target_pchan = target_pchan;
    def.effector_pchan = effector_pchan;
    def.angle_limit = angle_limit;

    bool chain_valid = true;
    for (int li = 0; li < links_arr->len; li++) {
      IDProperty *link_item = IDP_GetIndexArray(links_arr, li);
      if (link_item == nullptr || link_item->type != IDP_GROUP) {
        chain_valid = false;
        break;
      }
      IDProperty *link_name = IDP_GetPropertyTypeFromGroup(
          link_item, kLinkBoneField, IDP_STRING);
      if (link_name == nullptr) {
        chain_valid = false;
        break;
      }
      bPoseChannel *link_pchan = BKE_pose_channel_find_name(pose, IDP_string_get(link_name));
      if (link_pchan == nullptr) {
        chain_valid = false;
        break;
      }
      def.link_pchans.push_back(link_pchan);

      /* 读取 link 限制（PMX 原始数据，不做 YZ swap/符号翻转） */
      V8LinkLimitData limit;
      IDProperty *limit_prop = IDP_GetPropertyTypeFromGroup(
          link_item, kLimitAngleField, IDP_BOOLEAN);
      limit.has_limit = limit_prop != nullptr && IDP_bool_get(limit_prop);
      if (limit.has_limit) {
        IDProperty *min_arr = IDP_GetPropertyTypeFromGroup(
            link_item, kLimitMinField, IDP_ARRAY);
        IDProperty *max_arr = IDP_GetPropertyTypeFromGroup(
            link_item, kLimitMaxField, IDP_ARRAY);
        if (min_arr != nullptr && min_arr->len == 3 && min_arr->subtype == IDP_FLOAT &&
            max_arr != nullptr && max_arr->len == 3 && max_arr->subtype == IDP_FLOAT)
        {
          const float *min_vals = IDP_array_float_get(min_arr);
          const float *max_vals = IDP_array_float_get(max_arr);
          for (int k = 0; k < 3; k++) {
            limit.limit_min[k] = min_vals[k];
            limit.limit_max[k] = max_vals[k];
          }
        }
      }
      def.link_limits.push_back(limit);
    }

    if (!chain_valid || def.link_pchans.empty()) {
      continue;
    }

    /* 收集 target/effector/links 及其祖先 */
    v8_collect_pchan_ancestors(target_pchan, involved);
    v8_collect_pchan_ancestors(effector_pchan, involved);
    for (bPoseChannel *link_pchan : def.link_pchans) {
      v8_collect_pchan_ancestors(link_pchan, involved);
    }

    chain_defs.push_back(std::move(def));
  }

  if (chain_defs.empty()) {
    return;
  }

  /* 建立 bone pool（拓扑顺序，从根到叶） */
  std::vector<bPoseChannel *> bone_pool;
  std::map<bPoseChannel *, int> bone_index;
  v8_build_bone_pool_topological(arm, pose, involved, bone_pool, bone_index);
  if (bone_pool.empty()) {
    return;
  }

  /* 3c. 填充 CCDIKV8Bone 数据 */
  std::vector<CCDIKV8Bone> bones(bone_pool.size());
  for (size_t bi = 0; bi < bone_pool.size(); bi++) {
    bPoseChannel *pchan = bone_pool[bi];
    Bone *bone = pchan->bone_get(armature_obj);
    if (bone == nullptr) {
      continue;
    }
    CCDIKV8Bone &vb = bones[bi];

    /* base_pos_mmd: 从 Bone::arm_head 转换
     *   pmx_x = bl_x / scale
     *   pmx_y = bl_z / scale  (bl_z → pmx_y)
     *   pmx_z = bl_y / scale  (bl_y → pmx_z) */
    vb.base_pos_mmd[0] = bone->arm_head[0] / global_scale;
    vb.base_pos_mmd[1] = bone->arm_head[2] / global_scale;
    vb.base_pos_mmd[2] = bone->arm_head[1] / global_scale;

    /* q_base_mmd: 从 pchan->quat 提取旋转（VMD 动画层 F-Curve 值，未被 constraints/iTaSC 污染），
     *   BoneConverter 逆变换到 MMD 空间。反编译结论 §5: type-4 IK link q_base = 动画插值层 0x154。
     *   注意：不能用 pchan->chan_mat，chan_mat 已被 iTaSC/MMD_IK_Approx 求解污染。
     *   bl→mmd: conj(q_conv) * bl_quat * q_conv */
    float bl_quat[4];
    /* Read the active animation rotation mode.  The raw quat field is not
     * authoritative for Euler and axis-angle pose channels. */
    if (!v8_read_action_rotation(armature_obj, *pchan, eval_time, bl_quat)) {
      const float4 bl_quat_value = BKE_pchan_rot_to_quat(*pchan);
      copy_qt_qt(bl_quat, bl_quat_value);
    }
    normalize_qt(bl_quat);

    float q_conv[4];
    v8_compute_bone_converter(*bone, q_conv);

    float conj_q[4];
    conjugate_qt_qt(conj_q, q_conv);
    float temp[4];
    mul_qt_qtqt(temp, conj_q, bl_quat);
    mul_qt_qtqt(vb.q_base_mmd, temp, q_conv);
    normalize_qt(vb.q_base_mmd);

    /* parent_index: bone pool 中的父索引（-1=根） */
    if (pchan->parent != nullptr) {
      auto it = bone_index.find(pchan->parent);
      vb.parent_index = (it != bone_index.end()) ? it->second : -1;
    }
    else {
      vb.parent_index = -1;
    }

    /* q_current_mmd: 初始化为 identity（跨链共享） */
    vb.q_current_mmd[0] = 1.0f;
    vb.q_current_mmd[1] = 0.0f;
    vb.q_current_mmd[2] = 0.0f;
    vb.q_current_mmd[3] = 0.0f;

    /* Keep the direct pose-derived m0 for external targets and anchor head
     * positions.  The solver propagates the MMD rotation hierarchy for links
     * and reconstructs anchor translation from this direct head position. */
    v8_build_initial_m0(*pchan, vb, global_scale);
  }

  /* 3d. 填充 CCDIKV8Chain 和 CCDIKV8Link */
  std::vector<CCDIKV8Chain> chains;
  std::vector<std::vector<CCDIKV8Link>> links_storage;
  std::set<int> target_indices;

  for (const V8ChainDef &def : chain_defs) {
    CCDIKV8Chain chain = {};

    auto find_idx = [&](bPoseChannel *p) -> int {
      auto it = bone_index.find(p);
      return (it != bone_index.end()) ? it->second : -1;
    };

    chain.target_bone_index = find_idx(def.target_pchan);
    chain.effector_bone_index = find_idx(def.effector_pchan);
    if (chain.target_bone_index < 0 || chain.effector_bone_index < 0) {
      continue;
    }
    target_indices.insert(chain.target_bone_index);

    /* 构造 links */
    std::vector<CCDIKV8Link> links(def.link_pchans.size());
    bool links_valid = true;
    for (size_t li = 0; li < def.link_pchans.size(); li++) {
      links[li].bone_index = find_idx(def.link_pchans[li]);
      if (links[li].bone_index < 0) {
        links_valid = false;
        break;
      }
      links[li].has_limit = def.link_limits[li].has_limit;
      for (int k = 0; k < 3; k++) {
        links[li].limit_min_mmd[k] = def.link_limits[li].limit_min[k];
        links[li].limit_max_mmd[k] = def.link_limits[li].limit_max[k];
      }
    }
    if (!links_valid || links.empty()) {
      continue;
    }

    chain.link_count = int(links.size());
    chain.iterations = 39; /* runtime 实测，非 PMX loop_count */
    chain.runtime_angle = def.angle_limit * 0.25f;

    links_storage.push_back(std::move(links));
    chain.links = links_storage.back().data();
    chains.push_back(chain);
  }

  if (chains.empty()) {
    return;
  }

  /* 设置 target 骨骼的 initial_m0_mmd（非根 target，根 target 已在上方处理） */
  for (int target_idx : target_indices) {
    if (bones[target_idx].parent_index >= 0) {
      bPoseChannel *pchan = bone_pool[target_idx];
      v8_build_initial_m0(*pchan, bones[target_idx], global_scale);
    }
  }

  /* V8 trace: 受 MMD_CCD_V8_TRACE 控制，打印求解前关键数值用于与 fit 脚本对比。 */
  if (const char *t = BLI_getenv("MMD_CCD_V8_TRACE"); t != nullptr && std::strcmp(t, "1") == 0) {
    std::fprintf(stderr, "[V8T] === BEFORE SOLVE === chains=%d bones=%d\n",
                 (int)chains.size(), (int)bones.size());
    for (size_t i = 0; i < bone_pool.size(); i++) {
      const auto &vb = bones[i];
      std::fprintf(stderr,
                   "[V8T] bone[%d] name=%s parent=%d base=(%.5f,%.5f,%.5f) qbase=(%.5f,%.5f,%.5f,%.5f)\n",
                   (int)i,
                   bone_pool[i]->name,
                   vb.parent_index,
                   vb.base_pos_mmd[0],
                   vb.base_pos_mmd[1],
                   vb.base_pos_mmd[2],
                   vb.q_base_mmd[0],
                   vb.q_base_mmd[1],
                   vb.q_base_mmd[2],
                   vb.q_base_mmd[3]);
    }
    for (size_t c = 0; c < chains.size(); c++) {
      const auto &ch = chains[c];
      std::fprintf(stderr,
                   "[V8T] chain[%d] target=%d(%s) eff=%d(%s) links=%d iter=%d angle=%.6f\n",
                   (int)c,
                   ch.target_bone_index,
                   bone_pool[ch.target_bone_index]->name,
                   ch.effector_bone_index,
                   bone_pool[ch.effector_bone_index]->name,
                   ch.link_count,
                   ch.iterations,
                   ch.runtime_angle);
      for (int li = 0; li < ch.link_count; li++) {
        int bi = ch.links[li].bone_index;
        std::fprintf(stderr,
                     "[V8T]   link[%d] bone=%d(%s) has_limit=%d",
                     li,
                     bi,
                     bone_pool[bi]->name,
                     ch.links[li].has_limit ? 1 : 0);
        if (ch.links[li].has_limit) {
          std::fprintf(stderr,
                       " min=(%.5f,%.5f,%.5f) max=(%.5f,%.5f,%.5f)",
                       ch.links[li].limit_min_mmd[0],
                       ch.links[li].limit_min_mmd[1],
                       ch.links[li].limit_min_mmd[2],
                       ch.links[li].limit_max_mmd[0],
                       ch.links[li].limit_max_mmd[1],
                       ch.links[li].limit_max_mmd[2]);
        }
        std::fprintf(stderr, "\n");
      }
      if (ch.target_bone_index >= 0) {
        const auto &tb = bones[ch.target_bone_index];
        std::fprintf(stderr, "[V8T]   target_m0 rows:\n");
        for (int r = 0; r < 4; r++) {
          std::fprintf(stderr,
                       "[V8T]     [%+.5f %+.5f %+.5f %+.5f]\n",
                       tb.initial_m0_mmd[r][0],
                       tb.initial_m0_mmd[r][1],
                       tb.initial_m0_mmd[r][2],
                       tb.initial_m0_mmd[r][3]);
        }
      }
    }
  }

  /* 3e. 调用 V8 求解 */
  mmd_ccd_v8_solve_all_chains(
      chains.data(), int(chains.size()), bones.data(), int(bones.size()));

  /* V8 trace: 求解后 q_current 结果。 */
  if (const char *t = BLI_getenv("MMD_CCD_V8_TRACE"); t != nullptr && std::strcmp(t, "1") == 0) {
    std::fprintf(stderr, "[V8T] === AFTER SOLVE ===\n");
    for (size_t c = 0; c < chains.size(); c++) {
      const auto &ch = chains[c];
      for (int li = 0; li < ch.link_count; li++) {
        int bi = ch.links[li].bone_index;
        const auto &lb = bones[bi];
        std::fprintf(stderr,
                     "[V8T] chain[%d] link[%d] bone=%d(%s) q_current=(%.5f,%.5f,%.5f,%.5f)\n",
                     (int)c,
                     li,
                     bi,
                     bone_pool[bi]->name,
                     lb.q_current_mmd[0],
                     lb.q_current_mmd[1],
                     lb.q_current_mmd[2],
                     lb.q_current_mmd[3]);
      }
    }
  }

  /* 3f. 把 V8 结果转回 Blender 并发布 */
  /* 收集所有 IK link 骨骼索引（去重，按 bone pool 索引升序 = 拓扑顺序） */
  std::vector<int> ik_link_indices;
  for (const CCDIKV8Chain &chain : chains) {
    for (int li = 0; li < chain.link_count; li++) {
      ik_link_indices.push_back(chain.links[li].bone_index);
    }
  }
  std::sort(ik_link_indices.begin(), ik_link_indices.end());
  ik_link_indices.erase(
      std::unique(ik_link_indices.begin(), ik_link_indices.end()),
      ik_link_indices.end());

  /* 按拓扑顺序处理 IK link 骨骼（保证父在子之前） */
  std::set<bPoseChannel *> ik_link_pchans;
  for (int idx : ik_link_indices) {
    bPoseChannel *pchan = bone_pool[idx];
    ik_link_pchans.insert(pchan);
    Bone *bone = pchan->bone_get(armature_obj);
    if (bone == nullptr) {
      continue;
    }
    CCDIKV8Bone &vb = bones[idx];

    /* q_current is the final local MMD rotation for this link.  Publish that
     * local rotation directly; rebuilding an absolute pose from final_m0 and
     * converting it back through the Blender rest hierarchy applies the rest
     * basis and parent transform a second time. */
    float q_conv[4];
    v8_compute_bone_converter(*bone, q_conv);
    float q_conv_inverse[4];
    conjugate_qt_qt(q_conv_inverse, q_conv);
    float converted[4];
    mul_qt_qtqt(converted, q_conv, vb.q_current_mmd);
    float blender_quat[4];
    mul_qt_qtqt(blender_quat, converted, q_conv_inverse);
    normalize_qt(blender_quat);

    float candidate_channel[4][4];
    quat_to_mat4(candidate_channel, blender_quat);
    if (v8_trace_right_leg_bone(pchan->name)) {
      v8_trace_matrix4("final_m0_mmd", pchan->name, vb.final_m0_mmd);
      v8_trace_matrix4("candidate_channel_from_q_current", pchan->name, candidate_channel);
    }

    if (v8_trace_right_leg_bone(pchan->name)) {
      v8_trace_matrix4("candidate_channel", pchan->name, candidate_channel);
    }
    /* The translation embedded in MMD m0 is pivot compensation for rotating
     * around PMX base_pos, not editable pose-channel location.  Writing it to
     * Blender loc disconnects the IK chain and makes D/Append bones chase a
     * false offset.  Preserve animation loc/scale and publish only rotation. */
    copy_v3_v3(candidate_channel[3], pchan->loc);
    copy_m4_m4(pchan->chan_mat, candidate_channel);
    BKE_pchan_apply_mat4(pchan, pchan->chan_mat, true);
    copy_v3_fl(pchan->scale, 1.0f);
    if (v8_trace_enabled() && v8_trace_right_leg_bone(pchan->name)) {
      std::fprintf(stderr,
                   "[V8W] channel_fields bone=%s loc=(%+.6f,%+.6f,%+.6f) quat=(%+.6f,%+.6f,%+.6f,%+.6f) scale=(%+.6f,%+.6f,%+.6f)\n",
                   pchan->name,
                   pchan->loc[0], pchan->loc[1], pchan->loc[2],
                   pchan->quat[0], pchan->quat[1], pchan->quat[2], pchan->quat[3],
                   pchan->scale[0], pchan->scale[1], pchan->scale[2]);
    }

    /* pose_mat = arm_mat * chan_mat（含父级联） */
    BKE_armature_mat_bone_to_pose({pchan, bone}, pchan->chan_mat, pchan->pose_mat);

    /* 发布：处理 dual quaternion、orig_pchan 同步、endpoints 更新 */
    mmd_ccd_ik_sync_pose_channel(armature_obj, *pchan, publish_to_original);
  }

  /* 3g. 刷新整棵 pose 树。
   * D/Append/shadow 骨通常不是 IK link 的子节点，而是兄弟链，通过约束
   * 读取主腿骨通道。只刷新 link descendants 会留下 D 骨和网格用旧姿态。 */
  Scene *scene = depsgraph != nullptr ? DEG_get_evaluated_scene(depsgraph) : nullptr;
  v8_refresh_pose_tree_after_solve(
      depsgraph, scene, armature_obj, ik_link_pchans, publish_to_original);

  /* The refresh uses local channel matrices to propagate pose.  Once the
   * complete tree is stable, convert every bone to the global deformation
   * delta consumed by Armature modifiers. */
  mmd_ccd_ik_finalize_pose_deform(armature_obj, publish_to_original);
}

/** Read a link definition from IDProperty and push into chain vector.
 *  Returns true if the link was valid and added. */
static bool read_link_from_idp(IDProperty &link_item,
                               bPose *pose,
                               const bool use_v2,
                               std::vector<CCDIKChainLink> &chain)
{
  IDProperty *link_name = IDP_GetPropertyTypeFromGroup(&link_item, kLinkBoneField, IDP_STRING);
  if (!link_name) {
    return false;
  }

  bPoseChannel *pchan = BKE_pose_channel_find_name(pose, IDP_string_get(link_name));
  if (!pchan) {
    return false;
  }

  CCDIKChainLink link;
  link.pchan = pchan;
  IDProperty *physics_owned = IDP_GetPropertyTypeFromGroup(
      &link_item, kLinkPhysicsOwnedField, IDP_BOOLEAN);
  link.physics_owned = physics_owned != nullptr && IDP_bool_get(physics_owned);

  IDProperty *limit_prop = IDP_GetPropertyTypeFromGroup(&link_item, kLimitAngleField, IDP_BOOLEAN);
  if (limit_prop && IDP_bool_get(limit_prop)) {
    link.limit_angle = true;

    IDProperty *min_arr = IDP_GetPropertyTypeFromGroup(&link_item, kLimitMinField, IDP_ARRAY);
    IDProperty *max_arr = IDP_GetPropertyTypeFromGroup(&link_item, kLimitMaxField, IDP_ARRAY);

    if (min_arr && min_arr->len == 3 && min_arr->subtype == IDP_FLOAT &&
        max_arr && max_arr->len == 3 && max_arr->subtype == IDP_FLOAT)
    {
      const float *min_vals = IDP_array_float_get(min_arr);
      const float *max_vals = IDP_array_float_get(max_arr);
      if (min_vals && max_vals) {
        if (use_v2) {
          /* PMX -> Blender local rotation: X changes sign and reverses its
           * interval, while PMX Z/Y map to Blender Y/Z. */
          link.limit_min[0] = -max_vals[0];
          link.limit_min[1] = min_vals[2];
          link.limit_min[2] = min_vals[1];
          link.limit_max[0] = -min_vals[0];
          link.limit_max[1] = max_vals[2];
          link.limit_max[2] = max_vals[1];
          link.limit_axis[0] = std::fabs(max_vals[0] - min_vals[0]) > 0.01f;
          link.limit_axis[1] = std::fabs(max_vals[2] - min_vals[2]) > 0.01f;
          link.limit_axis[2] = std::fabs(max_vals[1] - min_vals[1]) > 0.01f;
        }
        else {
          link.limit_min[0] = min_vals[0];
          link.limit_min[1] = min_vals[2];
          link.limit_min[2] = min_vals[1];
          link.limit_max[0] = max_vals[0];
          link.limit_max[1] = max_vals[2];
          link.limit_max[2] = max_vals[1];
        }
      }
    }
  }

  chain.push_back(link);
  return true;
}

static bool v2_chain_enabled(Object &armature_obj, Bone &ik_bone, const float eval_time)
{
  if (!mmd_native_ik_is_enabled(ik_bone)) {
    return false;
  }
  if (armature_obj.adt == nullptr || armature_obj.adt->action == nullptr) {
    return true;
  }

  char escaped_name[sizeof(ik_bone.name) * 2] = {};
  BLI_str_escape(escaped_name, ik_bone.name, sizeof(escaped_name));
  const std::string expected_path = std::string("pose.bones[\"") + escaped_name +
                                    "\"].mmd_ik_toggle";
  const FCurve *toggle_curve = BKE_animadata_fcurve_find_by_rna_path(
      armature_obj.adt, expected_path.c_str(), 0, nullptr, nullptr);
  return toggle_curve == nullptr || evaluate_fcurve(toggle_curve, eval_time) >= 0.5f;
}

/* -------------------------------------------------------------------- */
/* Main evaluation entry point                                           */
/* -------------------------------------------------------------------- */

void mmd_ccd_ik_evaluate(Depsgraph *depsgraph, Object *armature_obj)
{
  if (!armature_obj || armature_obj->type != OB_ARMATURE) {
    return;
  }

  /* 1. Read IK definition from armature's system properties. */
  IDProperty *def_group = IDP_GetPropertyFromGroup_null(
      armature_obj->id.system_properties, kIKDefinitionProp);
  if (!def_group) {
    return;
  }

  IDProperty *ik_bones_arr = IDP_GetPropertyTypeFromGroup(
      def_group, kIKBonesField, IDP_IDPARRAY);
  if (!ik_bones_arr || ik_bones_arr->len == 0) {
    return;
  }

  /* 2. Early-out: no bone has native CCD enabled. */
  if (!has_any_native_ik_enabled(*armature_obj, *ik_bones_arr)) {
    return;
  }

  const bool use_legacy = mmd_ccd_use_legacy_solver();
  const bool use_v8 = !use_legacy && mmd_ccd_use_v8_solver();
  if (!use_legacy) {
    IDProperty *schema = IDP_GetPropertyTypeFromGroup(def_group, "schema_version", IDP_INT);
    if (schema == nullptr || IDP_int_get(schema) < 2) {
      if (BLI_getenv("MMD_CCD_V2_TRACE") != nullptr) {
        std::fprintf(stderr, "[MMD CCD V2] skipped=legacy_definition_schema\n");
      }
      return;
    }
  }

  /* Legacy rollback preserves the old late mute/restore behavior. V2's
   * approximation constraints are excluded earlier while IK trees are built. */
  std::vector<ConstraintRestore> restore_list;
  if (use_legacy) {
    mute_approx_constraints(*armature_obj, restore_list);
  }

  /* VMD Actions retain MMD_IK_Approx constraints and animate their influence.
   * Leave that path to Blender's normal IK evaluation; CCD must not solve the
   * same chain a second time. */
  if (use_legacy && armature_obj->adt && armature_obj->adt->action) {
    restore_approx_constraints(restore_list);
    return;
  }
  if (!use_legacy && !use_v8 && armature_obj->adt && armature_obj->adt->action) {
    return;
  }

  /* 4. Run CCD for each enabled IK chain. */
  bArmature *arm = reinterpret_cast<bArmature *>(armature_obj->data);
  bPose *pose = armature_obj->pose;
  float eval_time = 0.0f;
  if (depsgraph != nullptr) {
    if (Scene *scene = DEG_get_evaluated_scene(depsgraph)) {
      eval_time = BKE_scene_ctime_get(scene);
    }
  }
  const bool publish_to_original = depsgraph != nullptr && DEG_is_active(depsgraph);
  const char *trace_v2_env = BLI_getenv("MMD_CCD_V2_TRACE");
  const bool trace_v2 = !use_legacy && !use_v8 && trace_v2_env != nullptr &&
                        std::strcmp(trace_v2_env, "1") == 0;

  /* Safety: the COW pose may not have channels during early depsgraph updates.
   * Skip evaluation until the armature is fully ready. */
  if (!pose || !arm || BLI_listbase_is_empty(&pose->chanbase)) {
    restore_approx_constraints(restore_list);
    return;
  }

  /* V8 求解路径：当 use_v8 为 true 时走 V8，跳过旧的逐链循环。
   * V8 一次接收全部 IK 链，跨链共享 q_current/m0（与 MMD 单对象批量求解一致）。 */
  if (use_v8) {
    mmd_ccd_v8_evaluate(depsgraph, *armature_obj, *ik_bones_arr, eval_time, publish_to_original);
    return;
  }

  for (int i = 0; i < ik_bones_arr->len; i++) {
    IDProperty *item = IDP_GetIndexArray(ik_bones_arr, i);
    if (!item || item->type != IDP_GROUP) {
      continue;
    }

    /* --- Read IK bone definition --- */

    IDProperty *name_prop = IDP_GetPropertyTypeFromGroup(item, kBoneNameField, IDP_STRING);
    if (!name_prop) continue;
    const char *ik_bone_name = IDP_string_get(name_prop);

    Bone *bone = BKE_armature_find_bone_name(arm, ik_bone_name);
    if (!bone || !mmd_native_ik_is_enabled(*bone)) {
      continue;
    }
    if (!use_legacy && !use_v8 && !v2_chain_enabled(*armature_obj, *bone, eval_time)) {
      if (trace_v2) {
        std::fprintf(stderr, "[MMD CCD V2] chain=%s enabled=0 frame=%.3f\n", ik_bone_name, eval_time);
      }
      continue;
    }

    /* --- Target: use the IK control bone's world position.
     *     The PMX "target" field stored in the IDProperty is the
     *     chain-end bone (ik_target_index), which moves with the chain.
     *     The IK control bone itself is the external driver — its
     *     position IS the target that the chain tip should reach. --- */
    bPoseChannel *ik_pchan = BKE_pose_channel_find_name(pose, ik_bone_name);
    if (!ik_pchan) {
      continue;
    }
    float target_world[3];
    copy_v3_v3(target_world, ik_pchan->pose_mat[3]);

    bPoseChannel *effector_pchan = nullptr;
    if (!use_legacy) {
      IDProperty *target_prop = IDP_GetPropertyTypeFromGroup(item, kTargetField, IDP_STRING);
      if (target_prop != nullptr) {
        effector_pchan = BKE_pose_channel_find_name(pose, IDP_string_get(target_prop));
      }
      if (effector_pchan == nullptr) {
        if (trace_v2) {
          std::fprintf(stderr, "[MMD CCD V2] chain=%s skipped=missing_effector\n", ik_bone_name);
        }
        continue;
      }
    }

    /* --- Loop count + angle limit --- */
    int loop_count = 40;
    IDProperty *loop_prop = IDP_GetPropertyTypeFromGroup(item, kLoopCountField, IDP_INT);
    if (loop_prop) {
      loop_count = IDP_int_get(loop_prop);
    }
    if (loop_count < 1) {
      continue;
    }

    float angle_limit = float(M_PI_4);
    IDProperty *angle_prop = IDP_GetPropertyTypeFromGroup(item, kAngleLimitField, IDP_FLOAT);
    if (angle_prop) {
      angle_limit = IDP_float_get(angle_prop);
    }
    if (angle_limit <= 0.0f) {
      angle_limit = float(M_PI_4);
    }

    /* --- Build chain --- */
    IDProperty *links_arr = IDP_GetPropertyTypeFromGroup(item, kLinksField, IDP_IDPARRAY);
    if (!links_arr || links_arr->len < (use_legacy ? 2 : 1)) {
      continue;
    }

    std::vector<CCDIKChainLink> chain;
    chain.reserve(links_arr->len);
    bool chain_valid = true;

    for (int li = 0; li < links_arr->len; li++) {
      IDProperty *link_item = IDP_GetIndexArray(links_arr, li);
      if (!link_item || link_item->type != IDP_GROUP) {
        chain_valid = false;
        break;
      }
      if (!read_link_from_idp(*link_item, pose, !use_legacy, chain)) {
        chain_valid = false;
        break;
      }
    }

    if (!chain_valid || chain.size() < (use_legacy ? 2 : 1)) {
      continue;
    }

    /* --- Solve --- */
    CCDIKStats stats;
    std::vector<bPoseChannel *> modified_channels;
    if (use_legacy) {
      mmd_ccd_solve_chain(
          *armature_obj, chain, target_world, loop_count, angle_limit, &stats);
    }
    else {
      const bool physics_owned = std::any_of(
          chain.begin(), chain.end(), [](const CCDIKChainLink &link) { return link.physics_owned; });
      if (physics_owned) {
        if (trace_v2) {
          std::fprintf(stderr, "[MMD CCD V2] chain=%s skipped=physics_owned_link\n", ik_bone_name);
        }
        continue;
      }
      mmd_ccd_v2_solve_chain(*armature_obj,
                              chain,
                              *effector_pchan,
                              target_world,
                              loop_count,
                              angle_limit,
                              &stats,
                              &modified_channels);
      if (trace_v2) {
        std::fprintf(stderr,
                     "[MMD CCD V2] chain=%s enabled=1 frame=%.3f iterations=%d "
                     "converged=%d error=%.9g\n",
                     ik_bone_name,
                     eval_time,
                     stats.iterations,
                     stats.converged ? 1 : 0,
                     stats.final_error);
      }
    }

    /* CCD runs after Blender's normal POSE_DONE bookkeeping. Publish the
     * final pose and all derived channel/deform caches before returning. */
    if (use_legacy) {
      for (const CCDIKChainLink &link : chain) {
        if (link.pchan != nullptr) {
          mmd_ccd_ik_sync_pose_channel(*armature_obj, *link.pchan, publish_to_original);
        }
      }
    }
    else {
      for (bPoseChannel *modified : modified_channels) {
        if (modified != nullptr) {
          mmd_ccd_ik_sync_pose_channel(*armature_obj, *modified, publish_to_original);
        }
      }
    }
  }

  /* 5. Restore constraint enforce values. */
  if (use_legacy) {
    restore_approx_constraints(restore_list);
  }
}

}  // namespace blender::mmd
