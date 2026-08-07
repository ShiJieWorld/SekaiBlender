/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * MMD native CCD IK V8 solver — MMD Y-up row-major independent module.
 *
 * 依据 fit_chinatsu_mmd_ik_algorithm.py 与 mmd-decompile-summary.md §15 实现。
 * 内部只用 MMD Y-up、row-major 显式 Mat4/Quat/Vec3，与 Blender Z-up column-major
 * 完全隔离。一次接收全部 IK 链，跨链共享 q_current/m0（与 MMD 单对象批量求解一致）。
 *
 * V8 算法硬契约（vs 旧 V3 翻车点）：
 * - D3DXQuaternionMultiply 反序：每 link q_cur = delta*q_cur（delta 左乘）；
 *   首轮 q_cur = q_cur*q_base（q_base 右乘）。
 * - clamp cap = (lo+1)*ik_angle*2.0 对称（半角空间）。
 * - 前半迭代（iter < iterations>>1）有限位骨轴钉死 ±坐标轴。
 * - cross_local = M × axis_world（列向量左乘 row-major M），非 v×M。
 * - iterations = 39（runtime 实测，非 PMX loop_count）。
 * - 每 link 后反向刷新 links[0..current] + effector 折叠到 links[0]。
 * - q_current 初始 identity；m0 从 q_base 传播（跨链共享，不每链重置）。
 */

#pragma once

namespace blender::mmd {

/* -------------------------------------------------------------------- */
/* MMD Y-up row-major 数学类型                                          */
/* -------------------------------------------------------------------- */

/** MMD Y-up row-major 4x4 matrix。
 *  row-vector 变换约定：result[c] = sum(point[k] * m[k][c]) + m[3][c]
 *  矩阵乘法：mul4(a, b)[r][c] = sum(a[r][k] * b[k][c])
 *  列向量左乘（cross_local 用）：result[r] = sum(m[r][k] * v[k]) */
struct MmdMat4 {
  float m[4][4];
};

/** MMD Y-up quaternion (w, x, y, z)。 */
struct MmdQuat {
  float w, x, y, z;
};

/** MMD Y-up 3D vector。 */
struct MmdVec3 {
  float v[3];
};

/* -------------------------------------------------------------------- */
/* V8 数据结构                                                          */
/* -------------------------------------------------------------------- */

/**
 * 一个骨骼在 V8 bone pool 中的条目。
 * 所有数据均为 MMD Y-up 空间（由调用方转换）。
 *
 * \note bones 数组必须按拓扑顺序排列（从根到叶），
 *       V8 内部按数组顺序传播 m0，不做拓扑排序。
 */
struct CCDIKV8Bone {
  /** PMX 模型空间基础位置（base_pos, PMX 文件原样读入，Y-up unscaled）。 */
  float base_pos_mmd[3];
  /** q_base（动画四元数层），MMD Y-up 空间 (w, x, y, z)。 */
  float q_base_mmd[4];
  /**
   * 初始 m0（由调用方从 pchan->pose_mat 转换到 MMD 空间）。
   * solver 用它保留 target 的 direct pose，以及独立 anchor 的 head
   * 平移；link 的 MMD 旋转由 q_base 层级传播。
   */
  float initial_m0_mmd[4][4];
  /** 父骨骼在 pool 中的索引（-1 = 根骨骼）。 */
  int parent_index;
  /** 输出：V8 求解后的最终 q_current（MMD 空间，调用方初始化为 identity）。 */
  float q_current_mmd[4];
  /** 输出：V8 求解后的最终 m0（MMD Y-up row-major）。 */
  float final_m0_mmd[4][4];
};

/** V8 IK 链中的一个 link。 */
struct CCDIKV8Link {
  /** bone pool 索引。 */
  int bone_index;
  /** 是否有角度限制。 */
  bool has_limit;
  /** PMX 原始角度限制（不做 YZ 交换/符号翻转，V8 在 MMD Y-up 空间算 limit）。 */
  float limit_min_mmd[3];
  float limit_max_mmd[3];
};

/** 一条 V8 IK 链。 */
struct CCDIKV8Chain {
  /** target 骨骼在 pool 中的索引（IK 控制骨，如 178/179）。 */
  int target_bone_index;
  /** effector 骨骼在 pool 中的索引（链末端，如 25/26）。 */
  int effector_bone_index;
  /** links 数组指针（PMX 原始顺序，tip 到 root）。 */
  const CCDIKV8Link *links;
  /** links 数组长度。 */
  int link_count;
  /** runtime 迭代次数（39，非 PMX loop_count）。 */
  int iterations;
  /** runtime 角度 = PMX angle * 0.25。 */
  float runtime_angle;
};

/* -------------------------------------------------------------------- */
/* 批量求解入口                                                          */
/* -------------------------------------------------------------------- */

/**
 * 一次求解当前 armature 的全部 IK 链。
 *
 * - q_current 初始为 identity（由调用方设置）。
 * - m0 从 q_base 内部传播（跨链共享）。
 * - target 的 m0 直接用 initial_m0_mmd（来自 pose_mat，不传播）。
 * - 求解后各骨骼的 q_current_mmd 包含最终 IK 输出。
 *
 * 调用方职责：
 * 1. 收集所有涉及骨骼（links + effectors + targets + 祖先）到 bone pool，
 *    按拓扑顺序排列（从根到叶）。
 * 2. 转换 base_pos/q_base 到 MMD Y-up 空间。
 * 3. 所有涉及骨骼的 initial_m0 从 pchan->pose_mat 转换；solver 仅对
 *    target/anchor 使用其 direct 数据，link 旋转仍从 q_base 传播。
 * 4. 求解后，把 q_current_mmd 转回 Blender bone-local 旋转并发布。
 *
 * \param chains IK 链数组。
 * \param chain_count 链数量。
 * \param bones bone pool 数组（按拓扑顺序）。
 * \param bone_count bone pool 数量。
 */
void mmd_ccd_v8_solve_all_chains(CCDIKV8Chain *chains,
                                 int chain_count,
                                 CCDIKV8Bone *bones,
                                 int bone_count);

}  // namespace blender::mmd
