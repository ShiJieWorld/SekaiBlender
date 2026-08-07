/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * MMD native CCD IK V8 solver implementation.
 *
 * 严格依据 fit_chinatsu_mmd_ik_algorithm.py::solve_chain_v8 与
 * analyze_chinatsu_mmd_ik_fixture.py 的数学 helper 实现。
 *
 * 关键不变量（vs 旧 V3 翻车点）：
 * 1. q_current 初始 identity；m0 从 q_base 传播（跨链共享，不每链重置）。
 * 2. D3DXQuaternionMultiply 反序：delta*q_cur（delta 左乘），首轮 q_cur*q_base（q_base 右乘）。
 * 3. clamp cap = (lo+1)*ik_angle*2.0 对称（半角空间）。
 * 4. 前半迭代（iter < iterations>>1）有限位骨轴钉死 ±坐标轴。
 * 5. cross_local = M × axis_world（列向量左乘 row-major M），非 v×M。
 * 6. iterations = 39（runtime 实测）。
 * 7. 每 link 后反向刷新 links[0..current] + effector 折叠到 links[0]。
 * 8. row-major 层级组合：m0[bone] = m1[bone] * m0[parent]。
 * 9. row-vector 点变换：result[c] = sum(point[k] * M[k][c]) + M[3][c]。
 */

#include "mmd_ccd_ik_v8.hh"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "BLI_path_utils.hh"

namespace blender::mmd {

/* -------------------------------------------------------------------- */
/* MMD Y-up row-major 数学（对照 analyze_chinatsu_mmd_ik_fixture.py）   */
/* -------------------------------------------------------------------- */

namespace {

constexpr float kEps = 1.0e-10f;

struct Mat3 {
  float m[3][3];
};

/* identity4 */
MmdMat4 identity4()
{
  MmdMat4 r;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      r.m[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  return r;
}

Mat3 identity3()
{
  Mat3 r;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  return r;
}

/* mul4: result[r][c] = sum(a[r][k] * b[k][c]) */
MmdMat4 mul4(const MmdMat4 &a, const MmdMat4 &b)
{
  MmdMat4 r;
  for (int ridx = 0; ridx < 4; ridx++) {
    for (int cidx = 0; cidx < 4; cidx++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += a.m[ridx][k] * b.m[k][cidx];
      }
      r.m[ridx][cidx] = sum;
    }
  }
  return r;
}

/* mul3: result[r][c] = sum(a[r][k] * b[k][c]) */
Mat3 mul3(const Mat3 &a, const Mat3 &b)
{
  Mat3 r;
  for (int ridx = 0; ridx < 3; ridx++) {
    for (int cidx = 0; cidx < 3; cidx++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++) {
        sum += a.m[ridx][k] * b.m[k][cidx];
      }
      r.m[ridx][cidx] = sum;
    }
  }
  return r;
}

Mat3 transpose3(const Mat3 &a)
{
  Mat3 r;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      r.m[i][j] = a.m[j][i];
    }
  }
  return r;
}

/* transform_point: row-vector 变换
 * result[c] = sum(point[k] * M[k][c]) + M[3][c] */
void transform_point(const float point[3], const MmdMat4 &matrix, float out[3])
{
  for (int c = 0; c < 3; c++) {
    out[c] = point[0] * matrix.m[0][c] + point[1] * matrix.m[1][c] +
             point[2] * matrix.m[2][c] + matrix.m[3][c];
  }
}

float dot3(const float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross3(const float a[3], const float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float length3(const float v[3])
{
  return std::sqrt(dot3(v, v));
}

/** 归一化，长度过小返回 false。 */
bool normalize3(const float v[3], float out[3])
{
  const float len = length3(v);
  if (len < kEps) {
    return false;
  }
  const float inv = 1.0f / len;
  out[0] = v[0] * inv;
  out[1] = v[1] * inv;
  out[2] = v[2] * inv;
  return true;
}

/* quat (w, x, y, z) */
void quat_normalize(const float q[4], float out[4])
{
  const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (len < kEps) {
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
    return;
  }
  const float inv = 1.0f / len;
  out[0] = q[0] * inv;
  out[1] = q[1] * inv;
  out[2] = q[2] * inv;
  out[3] = q[3] * inv;
}

/** quat_mul: 数学 a * b (Hamilton)。
 *  D3DXQuaternionMultiply(out, a, b) 返回数学 b*a，
 *  fit 脚本的 quat_mul(a, b) 返回数学 a*b。 */
void quat_mul(const float a[4], const float b[4], float out[4])
{
  float aw = a[0], ax = a[1], ay = a[2], az = a[3];
  float bw = b[0], bx = b[1], by = b[2], bz = b[3];
  float r[4] = {
      aw * bw - ax * bx - ay * by - az * bz,
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
  };
  quat_normalize(r, out);
}

/* quat_to_row3: 四元数 → row-major 3x3 旋转矩阵 */
void quat_to_row3(const float q_in[4], Mat3 &out)
{
  float q[4];
  quat_normalize(q_in, q);
  float w = q[0], x = q[1], y = q[2], z = q[3];

  /* 先构造 column-major，再转置得到 row-major */
  Mat3 column;
  column.m[0][0] = 1.0f - 2.0f * (y * y + z * z);
  column.m[0][1] = 2.0f * (x * y - z * w);
  column.m[0][2] = 2.0f * (x * z + y * w);
  column.m[1][0] = 2.0f * (x * y + z * w);
  column.m[1][1] = 1.0f - 2.0f * (x * x + z * z);
  column.m[1][2] = 2.0f * (y * z - x * w);
  column.m[2][0] = 2.0f * (x * z - y * w);
  column.m[2][1] = 2.0f * (y * z + x * w);
  column.m[2][2] = 1.0f - 2.0f * (x * x + y * y);

  out = transpose3(column);
}

/* quat_from_row3: row-major 3x3 旋转矩阵 → 四元数 (w, x, y, z) */
void quat_from_row3(const Mat3 &row_in, float out[4])
{
  /* 先转置到 column-major 再提取 */
  Mat3 m = transpose3(row_in);
  float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
  float q[4];
  if (trace > 0.0f) {
    float s = std::sqrt(trace + 1.0f) * 2.0f;
    q[0] = 0.25f * s;
    q[1] = (m.m[2][1] - m.m[1][2]) / s;
    q[2] = (m.m[0][2] - m.m[2][0]) / s;
    q[3] = (m.m[1][0] - m.m[0][1]) / s;
  }
  else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
    float s = std::sqrt(std::max(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) * 2.0f;
    q[0] = (m.m[2][1] - m.m[1][2]) / s;
    q[1] = 0.25f * s;
    q[2] = (m.m[0][1] + m.m[1][0]) / s;
    q[3] = (m.m[0][2] + m.m[2][0]) / s;
  }
  else if (m.m[1][1] > m.m[2][2]) {
    float s = std::sqrt(std::max(0.0f, 1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2])) * 2.0f;
    q[0] = (m.m[0][2] - m.m[2][0]) / s;
    q[1] = (m.m[0][1] + m.m[1][0]) / s;
    q[2] = 0.25f * s;
    q[3] = (m.m[1][2] + m.m[2][1]) / s;
  }
  else {
    float s = std::sqrt(std::max(0.0f, 1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1])) * 2.0f;
    q[0] = (m.m[1][0] - m.m[0][1]) / s;
    q[1] = (m.m[0][2] + m.m[2][0]) / s;
    q[2] = (m.m[1][2] + m.m[2][1]) / s;
    q[3] = 0.25f * s;
  }
  quat_normalize(q, out);
}

/* axis_rotation_row: 绕单轴旋转的 row-major 3x3 矩阵 */
void axis_rotation_row(int axis, float angle, Mat3 &out)
{
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float half = angle * 0.5f;
  q[0] = std::cos(half);
  q[axis + 1] = std::sin(half);
  quat_to_row3(q, out);
}

/* pivot_matrix: 构造 pivot 矩阵 (T(pos) * R * T(-pos) 的 row-major 形式)
 * 3x3 = rotation_row
 * translation[c] = position[c] - sum(position[k] * rotation_row[k][c]) */
MmdMat4 pivot_matrix(const float position[3], const Mat3 &rotation_row)
{
  MmdMat4 result = identity4();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      result.m[r][c] = rotation_row.m[r][c];
    }
  }
  for (int c = 0; c < 3; c++) {
    float trans = position[c];
    for (int k = 0; k < 3; k++) {
      trans -= position[k] * rotation_row.m[k][c];
    }
    result.m[3][c] = trans;
  }
  return result;
}

/* -------------------------------------------------------------------- */
/* Euler limit 分支 A/B/C（对照 analyze 脚本）                          */
/* -------------------------------------------------------------------- */

char branch_for_limits(const float limit_min[3], const float limit_max[3])
{
  if (limit_min[0] <= -float(M_PI) * 0.5f || limit_max[0] >= float(M_PI) * 0.5f) {
    if (limit_min[1] <= -float(M_PI) * 0.5f || limit_max[1] >= float(M_PI) * 0.5f) {
      return 'A';
    }
    return 'B';
  }
  return 'C';
}

void decompose_branch_euler(char branch, const Mat3 &matrix, float angles[3])
{
  if (branch == 'A') {
    float z = std::asin(std::max(-1.0f, std::min(1.0f, -matrix.m[1][0])));
    float cz = std::cos(z);
    if (std::fabs(cz) < 1.0e-8f) {
      angles[0] = 0.0f;
      angles[1] = std::atan2(-matrix.m[0][2], matrix.m[2][2]);
    }
    else {
      angles[1] = std::atan2(matrix.m[2][0], matrix.m[0][0]);
      angles[0] = std::atan2(matrix.m[1][2], matrix.m[1][1]);
    }
    angles[2] = z;
  }
  else if (branch == 'B') {
    float y = std::asin(std::max(-1.0f, std::min(1.0f, -matrix.m[0][2])));
    float cy = std::cos(y);
    if (std::fabs(cy) < 1.0e-8f) {
      /* y 仍为 asin 结果，仅 z=0（对照 analyze 脚本）。 */
      angles[0] = std::atan2(-matrix.m[2][1], matrix.m[1][1]);
      angles[1] = y;
      angles[2] = 0.0f;
    }
    else {
      angles[0] = std::atan2(matrix.m[1][2], matrix.m[2][2]);
      angles[1] = y;
      angles[2] = std::atan2(matrix.m[0][1], matrix.m[0][0]);
    }
  }
  else { /* 'C' */
    float x = std::asin(std::max(-1.0f, std::min(1.0f, -matrix.m[2][1])));
    float cx = std::cos(x);
    if (std::fabs(cx) < 1.0e-8f) {
      angles[1] = 0.0f;
      angles[2] = std::atan2(-matrix.m[1][0], matrix.m[0][0]);
      angles[0] = x;
    }
    else {
      angles[1] = std::atan2(matrix.m[2][0], matrix.m[2][2]);
      angles[2] = std::atan2(matrix.m[0][1], matrix.m[1][1]);
      angles[0] = x;
    }
  }
}

void compose_branch_euler(char branch, const float angles[3], Mat3 &out)
{
  float x = angles[0], y = angles[1], z = angles[2];
  Mat3 rx, ry, rz;
  axis_rotation_row(0, x, rx);
  axis_rotation_row(1, y, ry);
  axis_rotation_row(2, z, rz);
  if (branch == 'A') {
    out = mul3(mul3(ry, rz), rx); /* Ry * Rz * Rx */
  }
  else if (branch == 'B') {
    out = mul3(mul3(rx, ry), rz); /* Rx * Ry * Rz */
  }
  else { /* 'C' */
    out = mul3(mul3(rz, rx), ry); /* Rz * Rx * Ry */
  }
}

/** 应用 MMD link limit。
 *  前半迭代（iter < loop_count/2）：反射；后半：硬贴。 */
void apply_mmd_link_limit(const float limit_min[3],
                          const float limit_max[3],
                          const float q[4],
                          int iteration_index,
                          int loop_count,
                          float out[4])
{
  char branch = branch_for_limits(limit_min, limit_max);
  Mat3 row;
  quat_to_row3(q, row);
  float angles[3];
  decompose_branch_euler(branch, row, angles);

  bool hard_clamp = (loop_count / 2) <= iteration_index;
  for (int axis = 0; axis < 3; axis++) {
    float value = angles[axis];
    float low = limit_min[axis];
    float high = limit_max[axis];
    if (value < low) {
      float reflected = low * 2.0f - value;
      if (hard_clamp || reflected < low || reflected > high) {
        value = low;
      }
      else {
        value = reflected;
      }
    }
    else if (value > high) {
      float reflected = high * 2.0f - value;
      if (hard_clamp || reflected < low || reflected > high) {
        value = high;
      }
      else {
        value = reflected;
      }
    }
    angles[axis] = value;
  }
  Mat3 composed;
  compose_branch_euler(branch, angles, composed);
  quat_from_row3(composed, out);
}

/* -------------------------------------------------------------------- */
/* V8 求解核心                                                          */
/* -------------------------------------------------------------------- */

/** 方向是否收敛（归一化方向差平方和 < 1e-7）。 */
bool direction_converged(const float to_effector[3], const float to_target[3])
{
  float diff[3] = {
      to_target[0] - to_effector[0],
      to_target[1] - to_effector[1],
      to_target[2] - to_effector[2],
  };
  return dot3(diff, diff) < 1.0e-7f;
}

/** 求解一条 IK 链。
 *  跨链共享 q_current（bones[i].q_current_mmd）和 m0 数组。 */
void solve_chain_v8(const CCDIKV8Chain &chain,
                    CCDIKV8Bone *bones,
                    int bone_count,
                    MmdMat4 *m0)
{
  (void)bone_count;
  const int target_idx = chain.target_bone_index;
  const int effector_idx = chain.effector_bone_index;
  const int iterations = chain.iterations;
  const int half_iter = iterations >> 1;
  const float ik_angle = chain.runtime_angle;
  const CCDIKV8Link *links = chain.links;
  const int link_count = chain.link_count;

  /* target 世界位置：target_m0 用 initial_m0（来自 pose_mat，不传播）。
   * m0[target_idx] 在初始化时已设为 initial_m0_mmd。 */
  float target[3];
  transform_point(bones[target_idx].base_pos_mmd, m0[target_idx], target);

  for (int iteration_index = 0; iteration_index < iterations; iteration_index++) {
    for (int runtime_order = 0; runtime_order < link_count; runtime_order++) {
      const CCDIKV8Link &link = links[runtime_order];
      const int link_idx = link.bone_index;

      /* joint = transform_point(base_pos[link], m0[link]) */
      float joint[3];
      transform_point(bones[link_idx].base_pos_mmd, m0[link_idx], joint);

      /* effector = transform_point(base_pos[effector], m0[effector]) */
      float effector[3];
      transform_point(bones[effector_idx].base_pos_mmd, m0[effector_idx], effector);

      /* to_effector = normalize(effector - joint) */
      float to_effector_raw[3] = {
          effector[0] - joint[0],
          effector[1] - joint[1],
          effector[2] - joint[2],
      };
      float to_effector[3];
      if (!normalize3(to_effector_raw, to_effector)) {
        continue;
      }

      /* to_target = normalize(target - joint) */
      float to_target_raw[3] = {
          target[0] - joint[0],
          target[1] - joint[1],
          target[2] - joint[2],
      };
      float to_target[3];
      if (!normalize3(to_target_raw, to_target)) {
        continue;
      }

      if (direction_converged(to_effector, to_target)) {
        continue;
      }

      /* axis_world = cross(to_effector, to_target)（未归一化） */
      float axis_world[3];
      cross3(to_effector, to_target, axis_world);

      /* parent_m0 = m0[parent_of_link] */
      const int parent_idx = bones[link_idx].parent_index;
      const MmdMat4 &parent_m0 = m0[parent_idx];

      /* cross_local = M × axis_world（列向量左乘 row-major M）
       * cl[r] = sum(parent_m0[r][k] * axis_world[k]) */
      float cl[3] = {
          parent_m0.m[0][0] * axis_world[0] + parent_m0.m[0][1] * axis_world[1] +
              parent_m0.m[0][2] * axis_world[2],
          parent_m0.m[1][0] * axis_world[0] + parent_m0.m[1][1] * axis_world[1] +
              parent_m0.m[1][2] * axis_world[2],
          parent_m0.m[2][0] * axis_world[0] + parent_m0.m[2][1] * axis_world[1] +
              parent_m0.m[2][2] * axis_world[2],
      };

      /* 前半迭代有限位骨：钉死 ±坐标轴 */
      float axis_local[3];
      if (link.has_limit && iteration_index < half_iter) {
        const float *lmin = link.limit_min_mmd;
        const float *lmax = link.limit_max_mmd;
        if (lmin[1] == 0.0f && lmax[1] == 0.0f && lmin[2] == 0.0f && lmax[2] == 0.0f) {
          axis_local[0] = (cl[0] >= 0.0f) ? 1.0f : -1.0f;
          axis_local[1] = 0.0f;
          axis_local[2] = 0.0f;
        }
        else if (lmin[0] == 0.0f && lmax[0] == 0.0f && lmin[2] == 0.0f && lmax[2] == 0.0f) {
          axis_local[0] = 0.0f;
          axis_local[1] = (cl[1] >= 0.0f) ? 1.0f : -1.0f;
          axis_local[2] = 0.0f;
        }
        else {
          axis_local[0] = 0.0f;
          axis_local[1] = 0.0f;
          axis_local[2] = (cl[2] >= 0.0f) ? 1.0f : -1.0f;
        }
      }
      else {
        if (!normalize3(cl, axis_local)) {
          continue;
        }
      }

      const char *trace = BLI_getenv("MMD_CCD_V8_TRACE");
      if (iteration_index == 0 && trace != nullptr && std::strcmp(trace, "1") == 0) {
        std::fprintf(stderr,
                     "[V8S] iter0 order=%d link_idx=%d has_limit=%d joint=(%+.6f,%+.6f,%+.6f) eff=(%+.6f,%+.6f,%+.6f) target=(%+.6f,%+.6f,%+.6f) axis_world=(%+.6f,%+.6f,%+.6f) cl=(%+.6f,%+.6f,%+.6f) axis_local=(%+.6f,%+.6f,%+.6f)\n",
                     runtime_order,
                     link_idx,
                     link.has_limit ? 1 : 0,
                     joint[0], joint[1], joint[2],
                     effector[0], effector[1], effector[2],
                     target[0], target[1], target[2],
                     axis_world[0], axis_world[1], axis_world[2],
                     cl[0], cl[1], cl[2],
                     axis_local[0], axis_local[1], axis_local[2]);
      }

      /* half = 0.5 * acos(clamp(dot(to_effector, to_target))) */
      float cosine = std::max(-1.0f, std::min(1.0f, dot3(to_effector, to_target)));
      float half = 0.5f * std::acos(cosine);

      /* clamp cap（半角空间，对称 ±2*(lo+1)*angle） */
      float cap = float(runtime_order + 1) * ik_angle * 2.0f;
      if (half > cap) {
        half = cap;
      }
      else if (half < -cap) {
        half = -cap;
      }

      /* delta = (cos(half), axis_local * sin(half)) */
      float delta[4] = {
          std::cos(half),
          axis_local[0] * std::sin(half),
          axis_local[1] * std::sin(half),
          axis_local[2] * std::sin(half),
      };

      /* D3DX 反序：q_cur = delta * q_cur（delta 左乘） */
      float q_cur[4];
      std::memcpy(q_cur, bones[link_idx].q_current_mmd, sizeof(float[4]));
      float tmp[4];
      quat_mul(delta, q_cur, tmp);
      std::memcpy(q_cur, tmp, sizeof(float[4]));

      /* 首轮：q_cur = q_cur * q_base（q_base 右乘） */
      if (iteration_index == 0) {
        quat_mul(q_cur, bones[link_idx].q_base_mmd, tmp);
        std::memcpy(q_cur, tmp, sizeof(float[4]));
      }

      /* Euler limit 全程执行：前半反射、后半硬贴 */
      if (link.has_limit) {
        apply_mmd_link_limit(link.limit_min_mmd,
                             link.limit_max_mmd,
                             q_cur,
                             iteration_index,
                             iterations,
                             tmp);
        std::memcpy(q_cur, tmp, sizeof(float[4]));
      }

      std::memcpy(bones[link_idx].q_current_mmd, q_cur, sizeof(float[4]));

      /* 刷新 links[0..current] 反向 + effector 折叠到 links[0] */
      for (int refresh_index = runtime_order; refresh_index >= 0; refresh_index--) {
        int bone_idx = links[refresh_index].bone_index;
        int parent = bones[bone_idx].parent_index;
        Mat3 rot3;
        quat_to_row3(bones[bone_idx].q_current_mmd, rot3);
        MmdMat4 m1 = pivot_matrix(bones[bone_idx].base_pos_mmd, rot3);
        m0[bone_idx] = mul4(m1, m0[parent]);
      }
      m0[effector_idx] = m0[links[0].bone_index];
    }
  }
}

}  // namespace

/* -------------------------------------------------------------------- */
/* 公开入口                                                              */
/* -------------------------------------------------------------------- */

void mmd_ccd_v8_solve_all_chains(CCDIKV8Chain *chains,
                                 int chain_count,
                                 CCDIKV8Bone *bones,
                                 int bone_count)
{
  if (chains == nullptr || bones == nullptr || chain_count <= 0 || bone_count <= 0) {
    return;
  }

  /* Seed m0 from the MMD animation hierarchy below.  Direct pose matrices are
   * retained only for external IK targets and anchor head positions; link
   * rotations must be propagated through q_base to avoid importing Blender's
   * rest-bone basis.  q_current iterations still refresh m0 below. */
  std::vector<MmdMat4> m0(bone_count);
  for (int i = 0; i < bone_count; i++) {
    std::memcpy(m0[i].m, bones[i].initial_m0_mmd, sizeof(float[4][4]));
  }

  std::vector<bool> is_target(bone_count, false);
  std::vector<bool> is_link(bone_count, false);
  for (int c = 0; c < chain_count; c++) {
    if (chains[c].target_bone_index >= 0 && chains[c].target_bone_index < bone_count) {
      is_target[chains[c].target_bone_index] = true;
    }
    for (int li = 0; li < chains[c].link_count; li++) {
      const int link_idx = chains[c].links[li].bone_index;
      if (link_idx >= 0 && link_idx < bone_count) {
        is_link[link_idx] = true;
      }
    }
  }

  std::vector<bool> is_anchor(bone_count, false);
  for (int c = 0; c < chain_count; c++) {
    if (chains[c].link_count <= 0) {
      continue;
    }
    const int root_link = chains[c].links[chains[c].link_count - 1].bone_index;
    if (root_link < 0 || root_link >= bone_count) {
      continue;
    }
    const int root_parent = bones[root_link].parent_index;
    if (root_parent >= 0 && root_parent < bone_count && !is_link[root_parent] &&
        !is_target[root_parent])
    {
      is_anchor[root_parent] = true;
    }
  }

  /* Initial skeletal m0 follows MMD's animation layer propagation.  IK targets
   * are external controls, so keep their direct pose matrix.  An independent
   * chain-root parent is a hybrid anchor: use q_base propagation for its MMD
   * rotation, but use the direct pose head for world translation.  This keeps
   * upstream VMD translations while removing Blender's rest-bone basis from
   * the MMD rotation frame. */
  for (int i = 0; i < bone_count; i++) {
    const int parent = bones[i].parent_index;
    if (is_target[i]) {
      std::memcpy(m0[i].m, bones[i].initial_m0_mmd, sizeof(float[4][4]));
      continue;
    }
    Mat3 base_rot;
    quat_to_row3(bones[i].q_base_mmd, base_rot);
    const MmdMat4 m1 = pivot_matrix(bones[i].base_pos_mmd, base_rot);
    if (parent >= 0) {
      m0[i] = mul4(m1, m0[parent]);
    }
    else {
      m0[i] = m1;
    }
    if (is_anchor[i]) {
      float head_mmd[3];
      for (int c = 0; c < 3; c++) {
        head_mmd[c] = bones[i].base_pos_mmd[0] * bones[i].initial_m0_mmd[0][c] +
                      bones[i].base_pos_mmd[1] * bones[i].initial_m0_mmd[1][c] +
                      bones[i].base_pos_mmd[2] * bones[i].initial_m0_mmd[2][c] +
                      bones[i].initial_m0_mmd[3][c];
      }
      for (int c = 0; c < 3; c++) {
        float base_times_r = 0.0f;
        for (int k = 0; k < 3; k++) {
          base_times_r += bones[i].base_pos_mmd[k] * m0[i].m[k][c];
        }
        m0[i].m[3][c] = head_mmd[c] - base_times_r;
      }
    }
  }

  /* 按顺序求解每条链（跨链共享 q_current 和 m0） */
  for (int c = 0; c < chain_count; c++) {
    solve_chain_v8(chains[c], bones, bone_count, m0.data());
  }

  for (int i = 0; i < bone_count; i++) {
    std::memcpy(bones[i].final_m0_mmd, m0[i].m, sizeof(float[4][4]));
  }
}

}  // namespace blender::mmd
