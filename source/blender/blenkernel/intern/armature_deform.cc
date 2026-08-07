/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * Deform coordinates by a armature object (used by modifier).
 */

#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "BLI_array.hh"
#include "BLI_listbase.hh"
#include "BLI_listbase_wrapper.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_task.hh"
#include "BLI_task_c.hh"
#include "BLI_vector.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_lattice_types.h"
#include "DNA_listBase.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"

#include "BKE_action.hh"
#include "BKE_armature.hh"
#include "BKE_attribute.hh"
#include "BKE_customdata.hh"
#include "BKE_deform.hh"
#include "BKE_editmesh.hh"
#include "BKE_lattice.hh"
#include "BKE_mesh.hh"
#include "BKE_pose.hh"

#include "MEM_guardedalloc.h"

#include "CLG_log.h"

namespace blender {

static CLG_LogRef LOG = {"geom.armature_deform"};

/* -------------------------------------------------------------------- */
/** \name Armature Deform Internal Utilities
 * \{ */

static float bone_envelope_falloff(const float distance_squared,
                                   const float closest_radius,
                                   const float falloff_distance)
{
  if (distance_squared < closest_radius * closest_radius) {
    return 1.0f;
  }

  /* Zero influence beyond falloff distance. */
  if (falloff_distance == 0.0f ||
      distance_squared >= math::square(closest_radius + falloff_distance))
  {
    return 0.0f;
  }

  /* Compute influence from envelope over the falloff distance. */
  const float dist_envelope = sqrtf(distance_squared) - closest_radius;
  return 1.0f - (dist_envelope * dist_envelope) / (falloff_distance * falloff_distance);
}

float distfactor_to_bone(const float3 &position,
                         const float3 &head,
                         const float3 &tail,
                         const float radius_head,
                         const float radius_tail,
                         const float falloff_distance)
{
  float bone_length;
  const float3 bone_axis = math::normalize_and_get_length(tail - head, bone_length);
  /* Distance along the bone axis from head. */
  const float height = math::dot(position - head, bone_axis);

  if (height < 0.0f) {
    /* Below the start of the bone use the head radius. */
    const float distance_squared = math::distance_squared(position, head);
    return bone_envelope_falloff(distance_squared, radius_head, falloff_distance);
  }
  if (height > bone_length) {
    /* After the end of the bone use the tail radius. */
    const float distance_squared = math::distance_squared(tail, position);
    return bone_envelope_falloff(distance_squared, radius_tail, falloff_distance);
  }
  /* Interpolate radius. */
  const float distance_squared = math::distance_squared(position, head) - height * height;
  const float closest_radius = bone_length != 0.0f ? math::interpolate(radius_head,
                                                                       radius_tail,
                                                                       height / bone_length) :
                                                     radius_head;
  return bone_envelope_falloff(distance_squared, closest_radius, falloff_distance);
}

namespace bke {

/**
 * Utility class for accumulating linear bone deformation.
 * If full_deform is true the deformation matrix is also computed.
 */
template<bool full_deform> struct BoneDeformLinearMixer {

  float3 position_delta = float3(0.0f);
  float3x3 deform = float3x3::zero();

  void accumulate(const bPoseChannel &pchan, const float3 &co, const float weight)
  {
    const float4x4 &pose_mat = float4x4(pchan.chan_mat);

    position_delta += weight * (math::transform_point(pose_mat, co) - co);
    if constexpr (full_deform) {
      deform += weight * pose_mat.view<3, 3>();
    }
  }

  void accumulate_bbone(const bPoseChannel &pchan,
                        const float3 &co,
                        const float weight,
                        const int index)
  {
    const Span<float4x4> pose_mats = Span<Mat4>(pchan.runtime.bbone_deform_mats,
                                                pchan.runtime.bbone_segments + 2)
                                         .cast<float4x4>();
    const float4x4 &pose_mat = pose_mats[index + 1];

    position_delta += weight * (math::transform_point(pose_mat, co) - co);
    if constexpr (full_deform) {
      deform += weight * pose_mat.view<3, 3>();
    }
  }

  void finalize(const float3 & /*co*/,
                float total,
                float armature_weight,
                float3 &r_delta_co,
                float3x3 &r_deform_mat)
  {
    const float scale_factor = armature_weight / total;
    r_delta_co = position_delta * scale_factor;
    r_deform_mat = deform * scale_factor;
  };
};

/**
 * Utility class for accumulating dual quaternion bone deformation.
 * If full_deform is true the deformation matrix is also computed.
 */
template<bool full_deform> struct BoneDeformDualQuaternionMixer {
  DualQuat dq = {};

  void accumulate(const bPoseChannel &pchan, const float3 &co, const float weight)
  {
    const DualQuat &deform_quat = pchan.runtime.deform_dual_quat;

    add_weighted_dq_dq_pivot(&dq, &deform_quat, co, weight, full_deform);
  }

  void accumulate_bbone(const bPoseChannel &pchan,
                        const float3 &co,
                        const float weight,
                        const int index)
  {
    const Span<DualQuat> quats = {pchan.runtime.bbone_dual_quats,
                                  pchan.runtime.bbone_segments + 1};
    const DualQuat &deform_quat = quats[index];

    add_weighted_dq_dq_pivot(&dq, &deform_quat, co, weight, full_deform);
  }

  void finalize(const float3 &co,
                float total,
                float armature_weight,
                float3 &r_delta_co,
                float3x3 &r_deform_mat)
  {
    normalize_dq(&dq, total);
    float3 dco = co;
    float3x3 dmat;
    mul_v3m3_dq(dco, dmat.ptr(), &dq);
    r_delta_co = (dco - co) * armature_weight;
    /* Quaternion already is scale corrected. */
    r_deform_mat = dmat;
  }
};

/* Add interpolated deformation along a b-bone segment of the pose channel. */
template<typename MixerT>
static void b_bone_deform(const bke::PChanBoneConst pchanbone,
                          const float3 &co,
                          const float weight,
                          MixerT &mixer)
{
  /* Calculate the indices of the 2 affecting b_bone segments. */
  int index;
  float blend;
  BKE_pchan_bbone_deform_segment_index(pchanbone, co, &index, &blend);

  mixer.accumulate_bbone(*pchanbone.pchan, co, weight * (1.0f - blend), index);
  mixer.accumulate_bbone(*pchanbone.pchan, co, weight * blend, index + 1);
}

/* Add bone deformation based on envelope distance. */
template<typename MixerT>
static float dist_bone_deform(const bke::PChanBoneConst pchanbone, const float3 &co, MixerT &mixer)
{
  const Bone *bone = pchanbone.bone;
  const bPoseChannel &pchan = *pchanbone.pchan;

  if (bone == nullptr || bone->weight == 0.0f) {
    return 0.0f;
  }

  const float fac = distfactor_to_bone(co,
                                       float3(bone->arm_head),
                                       float3(bone->arm_tail),
                                       bone->rad_head,
                                       bone->rad_tail,
                                       bone->dist);
  if (fac == 0.0f) {
    return 0.0f;
  }

  const float weight = fac * bone->weight;
  if (bone->segments > 1 && pchan.runtime.bbone_segments == bone->segments) {
    b_bone_deform(pchanbone, co, weight, mixer);
  }
  else {
    mixer.accumulate(pchan, co, weight);
  }

  return weight;
}

/* Add bone deformation based on vertex group weight. */
template<typename MixerT>
static float pchan_bone_deform(const PChanBone pchanbone,
                               const float weight,
                               const float3 &co,
                               MixerT &mixer)
{
  const Bone *bone = pchanbone.bone;

  if (!weight) {
    return 0.0f;
  }

  if (bone->segments > 1 && pchanbone.pchan->runtime.bbone_segments == bone->segments) {
    b_bone_deform(pchanbone, co, weight, mixer);
  }
  else {
    mixer.accumulate(*pchanbone.pchan, co, weight);
  }

  return weight;
}

}  // namespace bke

/** \} */

/* -------------------------------------------------------------------- */
/** \name Armature Deform #BKE_armature_deform_coords API
 *
 * #BKE_armature_deform_coords and related functions.
 * \{ */

namespace bke {

struct ArmatureDeformParams {
  bArmature *armature;
  MutableSpan<float3> vert_coords;
  std::optional<MutableSpan<float3x3>> vert_deform_mats;
  std::optional<Span<float3>> vert_coords_prev;

  bool use_envelope;
  bool invert_vgroup;
  bool use_dverts;

  int armature_def_nr;

  /* List of all pose channels on the target object. */
  ConstListBaseWrapper<bPoseChannel> pose_channels = {{nullptr, nullptr}};
  /* Maps vertex group index (def_nr) to pose channels, if vertex groups are used.
   * Vertex groups used for deform can be different from the target object vertex groups list,
   * the def_nr needs to be mapped to the correct pose channel first. */
  Array<bke::PChanBone> pose_channel_by_vertex_group;

  float4x4 target_to_armature;
  float4x4 armature_to_target;

  /* Optional PMX SDEF point attributes. They are only populated by the PMX importer and are
   * intentionally read here rather than introducing an MMD-specific modifier type. */
  VArray<int8_t> pmx_weight_type;
  VArray<float3> pmx_sdef_c;
  VArray<float3> pmx_sdef_r0;
  VArray<float3> pmx_sdef_r1;
  VArray<int> pmx_sdef_bone0;
  VArray<int> pmx_sdef_bone1;
};

static ArmatureDeformParams get_armature_deform_params(
    const Object &ob_arm,
    const Object &ob_target,
    const ListBaseT<bDeformGroup> *defbase,
    MutableSpan<float3> vert_coords,
    std::optional<Span<float3>> vert_coords_prev,
    std::optional<MutableSpan<float3x3>> vert_deform_mats,
    const int deformflag,
    StringRefNull defgrp_name,
    const bool try_use_dverts)
{
  const bool dverts_supported = BKE_object_supports_vertex_groups(&ob_target);

  /* TODO(Sybren): call the still-to-be-written function to assert that the pose is up to date. The
   * armature deform code doesn't use pchan->bone_get(object) but rather takes the armature (for
   * speed), and so cannot assert this itself. */

  bArmature *armature = id_cast<bArmature *>(ob_arm.data);
  ArmatureDeformParams deform_params;
  deform_params.armature = armature;
  deform_params.vert_coords = vert_coords;
  deform_params.vert_deform_mats = vert_deform_mats;
  deform_params.vert_coords_prev = vert_coords_prev;
  deform_params.use_envelope = bool(deformflag & ARM_DEF_ENVELOPE);
  deform_params.invert_vgroup = bool(deformflag & ARM_DEF_INVERT_VGROUP);

  deform_params.pose_channels = {ob_arm.pose->chanbase};
  deform_params.use_dverts = try_use_dverts && dverts_supported && (deformflag & ARM_DEF_VGROUP);
  if (deform_params.use_dverts) {
    const int defbase_len = defbase->count();
    deform_params.pose_channel_by_vertex_group.reinitialize(defbase_len);
    /* TODO(sergey): Some considerations here:
     *
     * - Check whether keeping this consistent across frames gives speedup.
     */

    BKE_pose_ensure_bone_indices(ob_arm);
    for (const auto [i, dg] : (defbase)->enumerate()) {
      bPoseChannel *pchan = BKE_pose_channel_find_name(ob_arm.pose, dg.name);
      /* Exclude non-deforming bones. */
      Bone *bone = pchan ? pchan->bone_get(*armature) : nullptr;
      if (pchan && !(bone->flag & BONE_NO_DEFORM)) {
        deform_params.pose_channel_by_vertex_group[i] = {pchan, bone};
      }
    }
  }

  /* Index of singular vertex group, if used. */
  deform_params.armature_def_nr = dverts_supported ?
                                      BKE_defgroup_name_index(defbase, defgrp_name) :
                                      -1;

/* TODO using the existing matrices directly is better, but fails tests because old code was
 * doing a double-inverse of the object matrix, leading to small differences on the order of 10^-5.
 * Test data needs to be updated if the transforms change. */
#if 0
  deform_params.target_to_armature = ob_arm.world_to_object() * ob_target.object_to_world();
  deform_params.armature_to_target = ob_target.world_to_object() * ob_arm.object_to_world();
#else
  deform_params.armature_to_target = ob_target.world_to_object() * ob_arm.object_to_world();
  deform_params.target_to_armature = math::invert(deform_params.armature_to_target);
#endif

  return deform_params;
}

/* Accumulate bone deformations using the mixer implementation. */
template<typename MixerT>
static void armature_vert_task_with_mixer(const ArmatureDeformParams &params,
                                          const int i,
                                          const MDeformVert *dvert,
                                          MixerT &mixer)
{
  const bool full_deform = params.vert_deform_mats.has_value();

  /* Overall influence, can change by masking with a vertex group. */
  float armature_weight = 1.0f;
  float prevco_weight = 0.0f; /* weight for optional cached vertexcos */
  if (params.armature_def_nr != -1 && dvert) {
    const float mask_weight = BKE_defvert_find_weight(dvert, params.armature_def_nr);
    /* On multi-modifier the mask is used to blend with previous coordinates. */
    if (params.vert_coords_prev) {
      prevco_weight = params.invert_vgroup ? mask_weight : 1.0f - mask_weight;
      if (prevco_weight == 1.0f) {
        return;
      }
    }
    else {
      armature_weight = params.invert_vgroup ? 1.0f - mask_weight : mask_weight;
      if (armature_weight == 0.0f) {
        return;
      }
    }
  }

  /* Input coordinates to start from. */
  float3 co = params.vert_coords_prev ? (*params.vert_coords_prev)[i] : params.vert_coords[i];
  /* Transform to armature space. */
  co = math::transform_point(params.target_to_armature, co);

  float contrib = 0.0f;
  bool deformed = false;
  /* Apply vertex group deformation if enabled. */
  if (params.use_dverts && dvert) {
    /* Range of valid def_nr in MDeformWeight. */
    const IndexRange def_nr_range = params.pose_channel_by_vertex_group.index_range();
    const Span<MDeformWeight> dweights(dvert->dw, dvert->totweight);
    for (const auto &dw : dweights) {
      const PChanBone pchanbone = def_nr_range.contains(dw.def_nr) ?
                                      params.pose_channel_by_vertex_group[dw.def_nr] :
                                      PChanBone(nullptr, nullptr);
      const bPoseChannel *pchan = pchanbone.pchan;
      if (pchan == nullptr) {
        continue;
      }

      float weight = dw.weight;

      /* Bone option to mix with envelope weight. */
      const Bone *bone = pchanbone.bone;
      if (bone && bone->flag & BONE_MULT_VG_ENV) {
        weight *= distfactor_to_bone(co,
                                     float3(bone->arm_head),
                                     float3(bone->arm_tail),
                                     bone->rad_head,
                                     bone->rad_tail,
                                     bone->dist);
      }

      contrib += pchan_bone_deform(pchanbone, weight, co, mixer);
      deformed = true;
    }
  }
  /* Use envelope if enabled and no bone deformed the vertex yet. */
  if (!deformed && params.use_envelope) {
    for (const bPoseChannel *pchan : params.pose_channels) {
      const Bone *bone = pchan->bone_get(*params.armature);
      if (!(bone->flag & BONE_NO_DEFORM)) {
        contrib += dist_bone_deform({pchan, bone}, co, mixer);
      }
    }
  }

  /* TODO Actually should be EPSILON? Weight values and contrib can be like 10e-39 small. */
  constexpr float contrib_threshold = 0.0001f;
  if (contrib > contrib_threshold) {
    float3 delta_co;
    float3x3 local_deform_mat;
    mixer.finalize(co, contrib, armature_weight, delta_co, local_deform_mat);

    co += delta_co;
    if (full_deform) {
      float3x3 &deform_mat = (*params.vert_deform_mats)[i];
      const float3x3 armature_to_target = params.armature_to_target.view<3, 3>();
      const float3x3 target_to_armature = params.target_to_armature.view<3, 3>();
      deform_mat = armature_to_target * local_deform_mat * target_to_armature * deform_mat;
    }
  }

  /* Transform back to target object space. */
  co = math::transform_point(params.armature_to_target, co);

  /* Multi-modifier: Interpolate with previous modifier position using the vertex group mask. */
  if (params.vert_coords_prev) {
    copy_v3_v3(params.vert_coords[i], math::interpolate(co, params.vert_coords[i], prevco_weight));
  }
  else {
    copy_v3_v3(params.vert_coords[i], co);
  }
}

/* PMX SDEF is a two-bone skinning method with a per-vertex rotation center and correction
 * points. This follows the PMX SDEF reference formula used by mmd_tools. */
static bool pmx_sdef_deform_point(const ArmatureDeformParams &params,
                                   const int vert_index,
                                   const MDeformVert &dvert,
                                   const float3 &co,
                                   float3 &r_co,
                                   float3x3 &r_rotation)
{
  constexpr int8_t pmx_sdef_weight_type = 3;
  if (params.pmx_weight_type.size() <= vert_index || params.pmx_sdef_c.size() <= vert_index ||
      params.pmx_sdef_r0.size() <= vert_index || params.pmx_sdef_r1.size() <= vert_index ||
      params.pmx_sdef_bone0.size() <= vert_index || params.pmx_sdef_bone1.size() <= vert_index ||
      params.pmx_weight_type[vert_index] != pmx_sdef_weight_type)
  {
    return false;
  }

  const int bone_indices[2] = {params.pmx_sdef_bone0[vert_index],
                               params.pmx_sdef_bone1[vert_index]};
  const IndexRange def_nr_range = params.pose_channel_by_vertex_group.index_range();
  if (!def_nr_range.contains(bone_indices[0]) || !def_nr_range.contains(bone_indices[1]) ||
      bone_indices[0] == bone_indices[1])
  {
    return false;
  }
  const PChanBone pchanbones[2] = {params.pose_channel_by_vertex_group[bone_indices[0]],
                                   params.pose_channel_by_vertex_group[bone_indices[1]]};
  if (pchanbones[0].pchan == nullptr || pchanbones[1].pchan == nullptr ||
      pchanbones[0].bone->segments > 1 || pchanbones[1].bone->segments > 1 ||
      (pchanbones[0].bone->flag & BONE_MULT_VG_ENV) ||
      (pchanbones[1].bone->flag & BONE_MULT_VG_ENV))
  {
    return false;
  }
  const MDeformWeight *weight0_entry = BKE_defvert_find_index(&dvert, bone_indices[0]);
  const MDeformWeight *weight1_entry = BKE_defvert_find_index(&dvert, bone_indices[1]);
  if (weight0_entry == nullptr || weight1_entry == nullptr || weight0_entry->weight <= 0.0f ||
      weight1_entry->weight <= 0.0f)
  {
    return false;
  }
  const float weights[2] = {weight0_entry->weight, weight1_entry->weight};
  const float total_weight = weights[0] + weights[1];
  if (total_weight <= 0.0f) {
    return false;
  }
  const float weight0 = weights[0] / total_weight;
  const float weight1 = 1.0f - weight0;

  /* PMX SDEF parameters are stored in target mesh space, matching the rest position. */
  const float3 c = math::transform_point(params.target_to_armature, params.pmx_sdef_c[vert_index]);
  const float3 r0 = math::transform_point(params.target_to_armature,
                                           params.pmx_sdef_r0[vert_index]);
  const float3 r1 = math::transform_point(params.target_to_armature,
                                           params.pmx_sdef_r1[vert_index]);
  const float3 rw = r0 * weight0 + r1 * weight1;
  /* PMX SDEF preprocesses R0/R1 around their weighted center, then averages the result with C. */
  const float3 cr0 = (c * 2.0f + r0 - rw) * 0.5f;
  const float3 cr1 = (c * 2.0f + r1 - rw) * 0.5f;

  float rotation0[3][3];
  float rotation1[3][3];
  copy_m3_m4(rotation0, pchanbones[0].pchan->chan_mat);
  copy_m3_m4(rotation1, pchanbones[1].pchan->chan_mat);
  normalize_m3(rotation0);
  normalize_m3(rotation1);

  float quat0[4];
  float quat1[4];
  mat3_to_quat(quat0, rotation0);
  mat3_to_quat(quat1, rotation1);
  if (dot_qtqt(quat0, quat1) < 0.0f) {
    negate_v4(quat1);
  }
  float blended_quat[4];
  interp_qt_qtqt(blended_quat, quat0, quat1, weight1);

  float blended_rotation[3][3];
  quat_to_mat3(blended_rotation, blended_quat);
  r_rotation = float3x3(blended_rotation);
  float3 rotated = co - c;
  mul_m3_v3(blended_rotation, rotated);
  r_co = rotated + math::transform_point(float4x4(pchanbones[0].pchan->chan_mat), cr0) * weight0 +
         math::transform_point(float4x4(pchanbones[1].pchan->chan_mat), cr1) * weight1;
  return true;
}

/* Apply SDEF when the importer supplied complete PMX data. Return false to use ordinary Blender
 * deformation, including for malformed attributes and unsupported B-Bones. */
static bool armature_vert_task_with_sdef(const ArmatureDeformParams &params,
                                         const int i,
                                         const MDeformVert *dvert)
{
  if (dvert == nullptr || !params.use_dverts) {
    return false;
  }

  float armature_weight = 1.0f;
  float prevco_weight = 0.0f;
  if (params.armature_def_nr != -1) {
    const float mask_weight = BKE_defvert_find_weight(dvert, params.armature_def_nr);
    if (params.vert_coords_prev) {
      prevco_weight = params.invert_vgroup ? mask_weight : 1.0f - mask_weight;
      if (prevco_weight == 1.0f) {
        return true;
      }
    }
    else {
      armature_weight = params.invert_vgroup ? 1.0f - mask_weight : mask_weight;
      if (armature_weight == 0.0f) {
        return true;
      }
    }
  }

  float3 co = params.vert_coords_prev ? (*params.vert_coords_prev)[i] : params.vert_coords[i];
  co = math::transform_point(params.target_to_armature, co);
  float3 sdef_co;
  float3x3 sdef_rotation;
  if (!pmx_sdef_deform_point(params, i, *dvert, co, sdef_co, sdef_rotation)) {
    return false;
  }

  co = math::interpolate(co, sdef_co, armature_weight);
  if (params.vert_deform_mats) {
    float3x3 &deform_mat = (*params.vert_deform_mats)[i];
    const float3x3 armature_to_target = params.armature_to_target.view<3, 3>();
    const float3x3 target_to_armature = params.target_to_armature.view<3, 3>();
    const float3x3 local_deform = float3x3::identity() * (1.0f - armature_weight) +
                                  sdef_rotation * armature_weight;
    deform_mat = armature_to_target * local_deform * target_to_armature * deform_mat;
  }
  co = math::transform_point(params.armature_to_target, co);
  if (params.vert_coords_prev) {
    copy_v3_v3(params.vert_coords[i], math::interpolate(co, params.vert_coords[i], prevco_weight));
  }
  else {
    copy_v3_v3(params.vert_coords[i], co);
  }
  return true;
}

/* Accumulate bone deformations for a vertex. */
static void armature_vert_task_with_dvert(const ArmatureDeformParams &deform_params,
                                          const int i,
                                          const MDeformVert *dvert,
                                          const bool use_quaternion)
{
  if (armature_vert_task_with_sdef(deform_params, i, dvert)) {
    return;
  }

  const bool full_deform = deform_params.vert_deform_mats.has_value();
  if (use_quaternion) {
    if (full_deform) {
      bke::BoneDeformDualQuaternionMixer<true> mixer;
      armature_vert_task_with_mixer(deform_params, i, dvert, mixer);
    }
    else {
      bke::BoneDeformDualQuaternionMixer<false> mixer;
      armature_vert_task_with_mixer(deform_params, i, dvert, mixer);
    }
  }
  else {
    if (full_deform) {
      bke::BoneDeformLinearMixer<true> mixer;
      armature_vert_task_with_mixer(deform_params, i, dvert, mixer);
    }
    else {
      bke::BoneDeformLinearMixer<false> mixer;
      armature_vert_task_with_mixer(deform_params, i, dvert, mixer);
    }
  }
}

/*
 * PMX models can contain isolated vertices whose weight field disagrees with
 * every topological neighbor.  Under a large pose rotation that creates a
 * discontinuity in the surface even though each vertex is correctly evaluated
 * by ordinary LBS.  Repair only those vertices whose current incident edge is
 * stretched and whose weight vector is also a local outlier.  The original
 * vertex groups remain unchanged; this is a per-pose deformation correction.
 */
static bool pmx_weight_corrected_deform_point(const ArmatureDeformParams &params,
                                              const float3 &target_co,
                                              const Span<float> weights,
                                              float3 &r_target_co)
{
  const float3 armature_co = math::transform_point(params.target_to_armature, target_co);

  BoneDeformLinearMixer<false> mixer;
  float contrib = 0.0f;
  for (const int def_nr : params.pose_channel_by_vertex_group.index_range()) {
    float weight = weights[def_nr];
    if (weight <= 0.0f) {
      continue;
    }

    const PChanBone pchanbone = params.pose_channel_by_vertex_group[def_nr];
    if (pchanbone.pchan == nullptr || pchanbone.bone == nullptr) {
      continue;
    }

    if (pchanbone.bone->flag & BONE_MULT_VG_ENV) {
      const Bone *bone = pchanbone.bone;
      weight *= distfactor_to_bone(armature_co,
                                   float3(bone->arm_head),
                                   float3(bone->arm_tail),
                                   bone->rad_head,
                                   bone->rad_tail,
                                   bone->dist);
    }
    contrib += pchan_bone_deform(pchanbone, weight, armature_co, mixer);
  }

  constexpr float contrib_threshold = 0.0001f;
  if (contrib <= contrib_threshold) {
    return false;
  }

  float3 delta_co;
  float3x3 deform_mat;
  mixer.finalize(armature_co, contrib, 1.0f, delta_co, deform_mat);
  r_target_co = math::transform_point(params.armature_to_target, armature_co + delta_co);
  return true;
}

static void pmx_correct_weight_outliers(const ArmatureDeformParams &params,
                                        const Mesh &mesh,
                                        const Span<MDeformVert> dverts,
                                        const Span<float3> source_coords,
                                        const MutableSpan<float3> vert_coords)
{
  constexpr int8_t pmx_sdef_weight_type = 3;
  constexpr float edge_stretch_threshold = 1.5f;
  constexpr float weight_outlier_threshold = 0.6f;
  constexpr float correction_strength = 0.75f;
  constexpr float epsilon = 1e-6f;

  if (source_coords.size() != mesh.verts_num || dverts.size() != mesh.verts_num ||
      params.pmx_weight_type.size() != mesh.verts_num ||
      params.pose_channel_by_vertex_group.is_empty())
  {
    return;
  }

  const int vert_num = mesh.verts_num;
  const int defgroup_num = params.pose_channel_by_vertex_group.size();
  Array<Vector<int>> neighbors(vert_num);
  for (const int2 edge : mesh.edges()) {
    if (edge.x < 0 || edge.x >= vert_num || edge.y < 0 || edge.y >= vert_num || edge.x == edge.y)
    {
      continue;
    }
    neighbors[edge.x].append(edge.y);
    neighbors[edge.y].append(edge.x);
  }

  Array<bool> stretched(vert_num, false);
  for (const int2 edge : mesh.edges()) {
    const int a = edge.x;
    const int b = edge.y;
    if (params.pmx_weight_type[a] == pmx_sdef_weight_type ||
        params.pmx_weight_type[b] == pmx_sdef_weight_type)
    {
      continue;
    }

    const float rest_length = math::distance(source_coords[a], source_coords[b]);
    if (rest_length <= epsilon) {
      continue;
    }
    const float pose_length = math::distance(vert_coords[a], vert_coords[b]);
    if (pose_length / rest_length > edge_stretch_threshold) {
      stretched[a] = true;
      stretched[b] = true;
    }
  }

  Array<float> self_weights(defgroup_num, 0.0f);
  Array<float> neighbor_weights(defgroup_num, 0.0f);
  Array<float> corrected_weights(defgroup_num, 0.0f);

  for (const int i : IndexRange(vert_num)) {
    if (!stretched[i] || params.pmx_weight_type[i] == pmx_sdef_weight_type ||
        neighbors[i].is_empty())
    {
      continue;
    }

    self_weights.as_mutable_span().fill(0.0f);
    neighbor_weights.as_mutable_span().fill(0.0f);

    float self_sum = 0.0f;
    for (const MDeformWeight &dw : Span(dverts[i].dw, dverts[i].totweight)) {
      if (dw.def_nr < 0 || dw.def_nr >= defgroup_num ||
          params.pose_channel_by_vertex_group[dw.def_nr].pchan == nullptr)
      {
        continue;
      }
      self_weights[dw.def_nr] += dw.weight;
      self_sum += dw.weight;
    }
    if (self_sum <= epsilon) {
      continue;
    }
    for (const int def_nr : IndexRange(defgroup_num)) {
      self_weights[def_nr] /= self_sum;
    }

    int valid_neighbor_count = 0;
    for (const int neighbor : neighbors[i]) {
      if (params.pmx_weight_type[neighbor] == pmx_sdef_weight_type) {
        continue;
      }

      float neighbor_sum = 0.0f;
      for (const MDeformWeight &dw : Span(dverts[neighbor].dw, dverts[neighbor].totweight)) {
        if (dw.def_nr >= 0 && dw.def_nr < defgroup_num &&
            params.pose_channel_by_vertex_group[dw.def_nr].pchan != nullptr)
        {
          neighbor_sum += dw.weight;
        }
      }
      if (neighbor_sum <= epsilon) {
        continue;
      }

      const float inverse_neighbor_sum = 1.0f / neighbor_sum;
      for (const MDeformWeight &dw : Span(dverts[neighbor].dw, dverts[neighbor].totweight)) {
        if (dw.def_nr >= 0 && dw.def_nr < defgroup_num &&
            params.pose_channel_by_vertex_group[dw.def_nr].pchan != nullptr)
        {
          neighbor_weights[dw.def_nr] += dw.weight * inverse_neighbor_sum;
        }
      }
      valid_neighbor_count++;
    }
    if (valid_neighbor_count == 0) {
      continue;
    }

    const float inverse_neighbor_count = 1.0f / float(valid_neighbor_count);
    for (const int def_nr : IndexRange(defgroup_num)) {
      neighbor_weights[def_nr] *= inverse_neighbor_count;
    }

    float weight_l1 = 0.0f;
    for (const int def_nr : IndexRange(defgroup_num)) {
      weight_l1 += math::abs(self_weights[def_nr] - neighbor_weights[def_nr]);
    }
    if (weight_l1 <= weight_outlier_threshold) {
      continue;
    }

    for (const int def_nr : IndexRange(defgroup_num)) {
      corrected_weights[def_nr] = (1.0f - correction_strength) * self_weights[def_nr] +
                                  correction_strength * neighbor_weights[def_nr];
    }

    float corrected_sum = 0.0f;
    for (const int def_nr : IndexRange(defgroup_num)) {
      corrected_sum += corrected_weights[def_nr];
    }
    if (corrected_sum <= epsilon) {
      continue;
    }
    for (const int def_nr : IndexRange(defgroup_num)) {
      corrected_weights[def_nr] /= corrected_sum;
    }

    float3 corrected_co;
    if (pmx_weight_corrected_deform_point(
            params, source_coords[i], corrected_weights.as_span(), corrected_co))
    {
      vert_coords[i] = corrected_co;
    }
  }
}

static void armature_deform_coords(const Object &ob_arm,
                                   const Object &ob_target,
                                   const ListBaseT<bDeformGroup> *defbase,
                                   const MutableSpan<float3> vert_coords,
                                   const std::optional<MutableSpan<float3x3>> vert_deform_mats,
                                   const int deformflag,
                                   const std::optional<Span<float3>> vert_coords_prev,
                                   StringRefNull defgrp_name,
                                   const std::optional<Span<MDeformVert>> dverts,
                                   const Mesh *me_target)
{
  ArmatureDeformParams deform_params = get_armature_deform_params(ob_arm,
                                                                  ob_target,
                                                                  defbase,
                                                                  vert_coords,
                                                                  vert_coords_prev,
                                                                  vert_deform_mats,
                                                                  deformflag,
                                                                  defgrp_name,
                                                                  dverts.has_value());

  if (me_target) {
    const bke::AttributeAccessor attributes = me_target->attributes();
    deform_params.pmx_weight_type = attributes.lookup<int8_t>("pmx_weight_type",
                                                                bke::AttrDomain::Point)
                                        .varray;
    deform_params.pmx_sdef_c = attributes.lookup<float3>("pmx_sdef_c", bke::AttrDomain::Point)
                                    .varray;
    deform_params.pmx_sdef_r0 = attributes.lookup<float3>("pmx_sdef_r0", bke::AttrDomain::Point)
                                     .varray;
    deform_params.pmx_sdef_r1 = attributes.lookup<float3>("pmx_sdef_r1", bke::AttrDomain::Point)
                                     .varray;
    deform_params.pmx_sdef_bone0 = attributes.lookup<int>("pmx_sdef_bone0",
                                                            bke::AttrDomain::Point)
                                        .varray;
    deform_params.pmx_sdef_bone1 = attributes.lookup<int>("pmx_sdef_bone1",
                                                            bke::AttrDomain::Point)
                                        .varray;
  }

  const bool use_quaternion = bool(deformflag & ARM_DEF_QUATERNION);
  const bool use_pmx_weight_correction = me_target != nullptr && dverts.has_value() &&
                                         deform_params.use_dverts && !use_quaternion &&
                                         !vert_coords_prev.has_value() &&
                                         !vert_deform_mats.has_value() &&
                                         deform_params.pmx_weight_type.size() == vert_coords.size();
  Array<float3> source_coords;
  if (use_pmx_weight_correction) {
    source_coords = Array<float3>(vert_coords.as_span());
  }

  constexpr int grain_size = 32;
  threading::parallel_for(vert_coords.index_range(), grain_size, [&](const IndexRange range) {
    for (const int i : range) {
      const MDeformVert *dvert = nullptr;
      if (deform_params.use_dverts || deform_params.armature_def_nr >= 0) {
        if (me_target) {
          BLI_assert(i < me_target->verts_num);
          if (dverts) {
            dvert = &(*dverts)[i];
          }
        }
        else if (dverts && i < dverts->size()) {
          dvert = &(*dverts)[i];
        }
      }

      armature_vert_task_with_dvert(deform_params, i, dvert, use_quaternion);
    }
  });

  if (use_pmx_weight_correction) {
    pmx_correct_weight_outliers(deform_params,
                                *me_target,
                                *dverts,
                                source_coords.as_span(),
                                vert_coords);
  }
}

struct ArmatureEditMeshUserdata {
  bool use_quaternion = false;
  int cd_dvert_offset = -1;

  ArmatureDeformParams deform_params;
};

template<bool use_dvert>
static void armature_vert_task_editmesh(void *__restrict userdata,
                                        MempoolIterData *iter,
                                        const TaskParallelTLS *__restrict /*tls*/)
{
  const ArmatureEditMeshUserdata &data = *static_cast<const ArmatureEditMeshUserdata *>(userdata);
  BMVert *v = reinterpret_cast<BMVert *>(iter);
  const MDeformVert *dvert = use_dvert ? static_cast<const MDeformVert *>(
                                             BM_ELEM_CD_GET_VOID_P(v, data.cd_dvert_offset)) :
                                         nullptr;
  armature_vert_task_with_dvert(
      data.deform_params, BM_elem_index_get(v), dvert, data.use_quaternion);
}

static void armature_deform_editmesh(const Object &ob_arm,
                                     const Object &ob_target,
                                     const ListBaseT<bDeformGroup> *defbase,
                                     const MutableSpan<float3> vert_coords,
                                     const std::optional<MutableSpan<float3x3>> vert_deform_mats,
                                     const int deformflag,
                                     const std::optional<Span<float3>> vert_coords_prev,
                                     StringRefNull defgrp_name,
                                     const BMEditMesh &em_target,
                                     const int cd_dvert_offset)
{
  ArmatureDeformParams deform_params = get_armature_deform_params(ob_arm,
                                                                  ob_target,
                                                                  defbase,
                                                                  vert_coords,
                                                                  vert_coords_prev,
                                                                  vert_deform_mats,
                                                                  deformflag,
                                                                  defgrp_name,
                                                                  cd_dvert_offset >= 0);

  ArmatureEditMeshUserdata data{};
  data.use_quaternion = bool(deformflag & ARM_DEF_QUATERNION);
  data.cd_dvert_offset = cd_dvert_offset;
  data.deform_params = std::move(deform_params);

  /* While this could cause an extra loop over mesh data, in most cases this will
   * have already been properly set. */
  BM_mesh_elem_index_ensure(em_target.bm, BM_VERT);

  TaskParallelSettings settings;
  BLI_parallel_mempool_settings_defaults(&settings);

  if (data.deform_params.use_dverts) {
    BLI_task_parallel_mempool(
        em_target.bm->vpool, &data, armature_vert_task_editmesh<true>, &settings);
  }
  else {
    BLI_task_parallel_mempool(
        em_target.bm->vpool, &data, armature_vert_task_editmesh<false>, &settings);
  }
}

static bool verify_armature_deform_valid(const Object &ob_arm)
{
  /* Not supported in armature edit mode or without pose data. */
  const bArmature *arm = id_cast<const bArmature *>(ob_arm.data);
  if (arm->edbo || (ob_arm.pose == nullptr)) {
    return false;
  }
  if ((ob_arm.pose->flag & POSE_RECALC) != 0) {
    CLOG_ERROR(&LOG,
               "Trying to evaluate influence of armature '%s' which needs Pose recalc!",
               ob_arm.id.name);
    BLI_assert_unreachable();
  }
  return true;
}

}  // namespace bke

void BKE_armature_deform_coords_with_curves(const Object &ob_arm,
                                            const Object &ob_target,
                                            const ListBaseT<bDeformGroup> *defbase,
                                            MutableSpan<float3> vert_coords,
                                            std::optional<Span<float3>> vert_coords_prev,
                                            std::optional<MutableSpan<float3x3>> vert_deform_mats,
                                            Span<MDeformVert> dverts,
                                            int deformflag,
                                            StringRefNull defgrp_name)
{
  if (!bke::verify_armature_deform_valid(ob_arm)) {
    return;
  }

  /* Vertex groups must be provided explicitly, cannot rely on object vertex groups since this is
   * used for Grease Pencil layers as well. */
  BLI_assert(dverts.size() == vert_coords.size());

  bke::armature_deform_coords(ob_arm,
                              ob_target,
                              defbase,
                              vert_coords,
                              vert_deform_mats,
                              deformflag,
                              vert_coords_prev,
                              defgrp_name,
                              dverts,
                              nullptr);
}

void BKE_armature_deform_coords_with_mesh(const Object &ob_arm,
                                          const Object &ob_target,
                                          MutableSpan<float3> vert_coords,
                                          std::optional<Span<float3>> vert_coords_prev,
                                          std::optional<MutableSpan<float3x3>> vert_deform_mats,
                                          int deformflag,
                                          StringRefNull defgrp_name,
                                          const Mesh *me_target)
{
  if (!bke::verify_armature_deform_valid(ob_arm)) {
    return;
  }

  /* Note armature modifier on legacy curves calls this, so vertex groups are not guaranteed to
   * exist. */
  const ID *id_target = static_cast<const ID *>(ob_target.data);
  const ListBaseT<bDeformGroup> *defbase = nullptr;
  if (me_target) {
    /* Use the vertex groups from the evaluated mesh that is being deformed. */
    defbase = BKE_id_defgroup_list_get(&me_target->id);
  }
  else if (BKE_id_supports_vertex_groups(id_target)) {
    /* Take the vertex groups from the original object data. */
    defbase = BKE_id_defgroup_list_get(id_target);
  }

  Span<MDeformVert> dverts;
  if (ob_target.type == OB_MESH) {
    if (me_target == nullptr) {
      me_target = id_cast<const Mesh *>(ob_target.data);
    }
    dverts = me_target->deform_verts();
  }
  else if (ob_target.type == OB_LATTICE) {
    const Lattice *lt = id_cast<const Lattice *>(ob_target.data);
    if (lt->dvert != nullptr) {
      dverts = Span<MDeformVert>(lt->dvert, lt->pntsu * lt->pntsv * lt->pntsw);
    }
  }

  std::optional<Span<MDeformVert>> dverts_opt;
  if ((me_target && !me_target->deform_verts().is_empty()) || dverts.size() == vert_coords.size())
  {
    dverts_opt = dverts;
  }

  bke::armature_deform_coords(ob_arm,
                              ob_target,
                              defbase,
                              vert_coords,
                              vert_deform_mats,
                              deformflag,
                              vert_coords_prev,
                              defgrp_name,
                              dverts_opt,
                              me_target);
}

void BKE_armature_deform_coords_with_editmesh(
    const Object &ob_arm,
    const Object &ob_target,
    MutableSpan<float3> vert_coords,
    std::optional<Span<float3>> vert_coords_prev,
    std::optional<MutableSpan<float3x3>> vert_deform_mats,
    int deformflag,
    StringRefNull defgrp_name,
    const BMEditMesh &em_target)
{
  if (!bke::verify_armature_deform_valid(ob_arm)) {
    return;
  }

  const ListBaseT<bDeformGroup> *defbase = BKE_id_defgroup_list_get(
      static_cast<const ID *>(ob_target.data));
  const int cd_dvert_offset = CustomData_get_offset(&em_target.bm->vdata, CD_MDEFORMVERT);
  bke::armature_deform_editmesh(ob_arm,
                                ob_target,
                                defbase,
                                vert_coords,
                                vert_deform_mats,
                                deformflag,
                                vert_coords_prev,
                                defgrp_name,
                                em_target,
                                cd_dvert_offset);
}

/** \} */

}  // namespace blender
