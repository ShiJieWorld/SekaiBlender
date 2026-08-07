/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_action.hh"

#include "ANIM_action.hh"
#include "ANIM_fcurve.hh"
#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_fcurve.hh"
#include "BKE_lib_id.hh"
#include "BKE_pose.hh"
#include "BKE_report.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_anim_types.h"
#include "DNA_object_types.h"

#include "BLI_string.hh"

#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace blender::io::vmd {
namespace {

constexpr int kLocationChannels = 3;
constexpr int kRotationChannels = 4;
constexpr int kMaxActionFrame = std::numeric_limits<int>::max();

void add_error(VMDActionReport &result, ReportList *reports, const std::string &message)
{
  result.errors.push_back(message);
  BKE_report(reports, RPT_ERROR, message.c_str());
}

void add_warning(VMDActionReport &result, ReportList *reports, const std::string &message)
{
  result.warnings.push_back(message);
  BKE_report(reports, RPT_INFO, message.c_str());
}

std::string escaped_pose_bone_path(const std::string &bone_name, const char *property)
{
  char escaped_name[128] = {};
  BLI_str_escape(escaped_name, bone_name.c_str(), sizeof(escaped_name));
  return std::string("pose.bones[\"") + escaped_name + "\"]." + property;
}

bool checked_frame(const VMDBoneKeyframe &keyframe,
                   const int frame_offset,
                   int &r_frame,
                   VMDActionReport &result,
                   ReportList *reports)
{
  const int64_t frame = int64_t(keyframe.frame) + int64_t(frame_offset);
  if (frame < std::numeric_limits<int>::min() || frame > kMaxActionFrame) {
    add_error(result,
              reports,
              "VMD frame plus frame offset is outside Blender's signed frame range");
    return false;
  }
  r_frame = int(frame);
  return true;
}

bool finite_quaternion(const std::array<float, 4> &rotation)
{
  for (const float value : rotation) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool write_keyframe(FCurve &fcurve,
                    const int frame,
                    const float value,
                    const animrig::KeyframeSettings &settings)
{
  if (!std::isfinite(value)) {
    return false;
  }
  return animrig::insert_vert_fcurve(&fcurve, {float(frame), value}, settings, INSERTKEY_FAST) ==
         animrig::SingleKeyingResult::SUCCESS;
}

struct TrackWriteInfo {
  const VMDMappedBoneTrack *mapped = nullptr;
  bPoseChannel *pose_bone = nullptr;
  std::vector<const VMDBoneKeyframe *> keyframes;
  std::string location_path;
  std::string rotation_path;
  BoneConverter converter;
};

}  // namespace

void apply_vmd_bezier_to_curve(FCurve &curve,
                               const std::vector<const uint8_t *> &interpolation_data,
                               const int vmd_channel,
                               const int channel_stride,
                               const std::array<int, 4> &control_offsets,
                               const VMDBezierWarningCallback &warning_callback)
{
  if (curve.bezt == nullptr || curve.totvert < 2 || interpolation_data.size() < 2) {
    return; /* Single keyframe: keep linear. */
  }

  const int max_control_offset = *std::max_element(control_offsets.begin(), control_offsets.end());
  if (vmd_channel < 0 || channel_stride <= 0 || max_control_offset < 0) {
    return;
  }

  auto read_interp_norm = [&](const uint8_t *data, const int offset) {
    const int raw = int(data[offset]);
    int value = raw;
    if (raw > 127) {
      if (warning_callback) {
        warning_callback("VMD interpolation byte at offset " + std::to_string(offset) +
                         " exceeds 127 (" + std::to_string(raw) + "); clamped to 127");
      }
      value = 127;
    }
    return float(value) / 127.0f;
  };

  for (int i = 0; i < curve.totvert - 1; i++) {
    /* VMD stores the interpolation from the previous keyframe on the destination keyframe. */
    const uint8_t *interpolation = interpolation_data[size_t(i + 1)];
    const int channel_base = vmd_channel * channel_stride;
    const float ax = read_interp_norm(interpolation, channel_base + control_offsets[0]);
    const float ay = read_interp_norm(interpolation, channel_base + control_offsets[1]);
    const float bx = read_interp_norm(interpolation, channel_base + control_offsets[2]);
    const float by = read_interp_norm(interpolation, channel_base + control_offsets[3]);
    const float f_k = curve.bezt[i].vec[1][0];
    const float v_k = curve.bezt[i].vec[1][1];
    const float df = curve.bezt[i + 1].vec[1][0] - f_k;
    const float dv = curve.bezt[i + 1].vec[1][1] - v_k;
    if (df <= 0.0f) {
      continue; /* Duplicate frame: degrade to linear. */
    }
    /* Right handle of keyframe i = segment start + P1(ax, ay). */
    curve.bezt[i].vec[2][0] = f_k + ax * df;
    curve.bezt[i].vec[2][1] = v_k + ay * dv;
    curve.bezt[i].h2 = HD_FREE;
    /* Left handle of keyframe i+1 = segment start + P2(bx, by). */
    curve.bezt[i + 1].vec[0][0] = f_k + bx * df;
    curve.bezt[i + 1].vec[0][1] = v_k + by * dv;
    curve.bezt[i + 1].h1 = HD_FREE;
  }

  /* Keep the endpoint handles horizontal, matching mmd_tools and the bone importer. */
  const float f0 = curve.bezt[0].vec[1][0], v0 = curve.bezt[0].vec[1][1];
  curve.bezt[0].vec[0][0] = f0 - 1.0f;
  curve.bezt[0].vec[0][1] = v0;
  curve.bezt[0].h1 = HD_FREE;
  const int last = curve.totvert - 1;
  const float fl = curve.bezt[last].vec[1][0], vl = curve.bezt[last].vec[1][1];
  curve.bezt[last].vec[2][0] = fl + 1.0f;
  curve.bezt[last].vec[2][1] = vl;
  curve.bezt[last].h2 = HD_FREE;
  BKE_fcurve_handles_recalc(curve);
}

/* -------------------------------------------------------------------- */
/** \name BoneConverter
 * \{ */

BoneConverter::BoneConverter()
{
  unit_m3(mat);
  unit_qt(q_conv);
}

void BoneConverter::compute_from_pose_bone(const bPoseChannel &pchan, const Object &ob)
{
  const Bone *bone = pchan.bone_get(ob);
  if (bone == nullptr) {
    unit_m3(mat);
    unit_qt(q_conv);
    return;
  }

  /* Extract the 3x3 rotation from Bone::arm_mat, which is the C equivalent of
   * Blender RNA's Bone.matrix_local used by mmd_tools. Bone::bone_mat is only
   * the bone-space basis matrix and is not equivalent for child bones. */
  float arm_rot[3][3];
  copy_m3_m4(arm_rot, bone->arm_mat);
  normalize_m3(arm_rot);

  /* Swap Y and Z columns: converts the bone's rest rotation from Blender's
   * Z-up coordinate system to MMD's Y-up system. After this, arm_rot
   * represents the bone's rest rotation as if it were in MMD coordinates. */
  for (int i = 0; i < 3; i++) {
    std::swap(arm_rot[i][1], arm_rot[i][2]);
  }

  /* Transpose = invert for a proper rotation matrix (or a reflection when
   * combined with the YZ swap). The result transforms from MMD Y-up space
   * to Blender bone-local space. */
  transpose_m3_m3(mat, arm_rot);

  /* Precompute the quaternion form for rotation conjugation. */
  mat3_to_quat(q_conv, mat);
  normalize_qt(q_conv);

  /* Compute per-axis interpolation channel mapping.
   * mat[i][j] quantifies how much VMD channel j influences Blender axis i.
   * Greedily assign each Blender axis to the VMD channel with the largest
   * absolute coefficient, skipping already-matched pairs. This mirrors
   * mmd_tools _InterpolationHelper and handles non-identity arm_mat where
   * the hard-coded {0,2,1} swap is no longer accurate. */
  {
    struct {
      float score; int bl_axis, vmd_chn;
    } pairs[9];
    int pi = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        pairs[pi++] = {std::fabs(mat[i][j]), i, j};
      }
    }
    for (int a = 0; a < 9; a++) {
      for (int b = a + 1; b < 9; b++) {
        if (pairs[b].score > pairs[a].score) {
          std::swap(pairs[a], pairs[b]);
        }
      }
    }
    bool bl_used[3] = {false, false, false};
    bool vmd_used[3] = {false, false, false};
    for (int a = 0; a < 9; a++) {
      if (!bl_used[pairs[a].bl_axis] && !vmd_used[pairs[a].vmd_chn]) {
        interpolation_indices[pairs[a].bl_axis] = pairs[a].vmd_chn;
        bl_used[pairs[a].bl_axis] = true;
        vmd_used[pairs[a].vmd_chn] = true;
      }
    }
  }
}

void BoneConverter::convert_location(const float vmd_loc[3],
                                     float bl_loc[3],
                                     const float scale) const
{
  for (int i = 0; i < 3; i++) {
    bl_loc[i] = (mat[i][0] * vmd_loc[0] + mat[i][1] * vmd_loc[1] +
                 mat[i][2] * vmd_loc[2]) *
                scale;
  }
}

void BoneConverter::convert_rotation(const float vmd_rot[4], float bl_rot[4]) const
{
  /* VMD raw order is (qx, qy, qz, qw). Reorder to Blender (qw, qx, qy, qz). */
  float src_q[4] = {vmd_rot[3], vmd_rot[0], vmd_rot[1], vmd_rot[2]};
  normalize_qt(src_q);

  /* Conjugation: bl_rot = q_conv * src_q * conj(q_conv).
   * This rotates the quaternion's reference frame from MMD Y-up world
   * space to the bone's local space, accounting for both the Y↔Z axis
   * swap and the bone's rest-pose rotation. */
  float conj_q[4];
  conjugate_qt_qt(conj_q, q_conv);
  float temp[4];
  mul_qt_qtqt(temp, q_conv, src_q);
  mul_qt_qtqt(bl_rot, temp, conj_q);
  normalize_qt(bl_rot);
}

void BoneConverter::inverse_location(const float bl_loc[3],
                                     float vmd_loc[3],
                                     const float scale) const
{
  for (int j = 0; j < 3; j++) {
    vmd_loc[j] = (mat[0][j] * bl_loc[0] + mat[1][j] * bl_loc[1] + mat[2][j] * bl_loc[2]) /
                 scale;
  }
}

void BoneConverter::inverse_rotation(const float bl_rot[4], float vmd_rot[4]) const
{
  float normalized[4];
  copy_qt_qt(normalized, bl_rot);
  normalize_qt(normalized);

  float conjugate[4];
  conjugate_qt_qt(conjugate, q_conv);
  float temp[4], source[4];
  mul_qt_qtqt(temp, conjugate, normalized);
  mul_qt_qtqt(source, temp, q_conv);
  normalize_qt(source);
  vmd_rot[0] = source[1];
  vmd_rot[1] = source[2];
  vmd_rot[2] = source[3];
  vmd_rot[3] = source[0];
}

/** \} */

bool build_vmd_action(Main *bmain,
                      Object &target_armature,
                      const VMDModel &model,
                      const VMDMappingReport &mapping,
                      const std::string &action_name,
                      const VMDActionOptions &options,
                      ReportList *reports,
                      VMDActionReport &r_result)
{
  r_result = {};
  r_result.action_name = action_name;
  r_result.mapped_track_count = mapping.mapped_track_count;
  r_result.missing_track_count = mapping.missing_track_count;

  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD Action build requires a valid Main database");
    return false;
  }
  if (action_name.empty()) {
    add_error(r_result, reports, "VMD Action name must not be empty");
    return false;
  }
  if (target_armature.type != OB_ARMATURE || target_armature.data == nullptr) {
    add_error(r_result, reports, "VMD Action target is not a valid Armature Object");
    return false;
  }
  if (!mapping.mapping_valid || mapping.mapped_tracks.empty()) {
    add_error(r_result, reports, "VMD bone mapping is invalid or contains no mapped tracks");
    return false;
  }
  if (options.coordinate_scale <= 0.0f || !std::isfinite(options.coordinate_scale)) {
    add_error(r_result, reports, "VMD coordinate scale must be finite and greater than zero");
    return false;
  }
  const AnimData *existing_anim_data = BKE_animdata_from_id(&target_armature.id);
  if (!options.replace_existing_action && existing_anim_data != nullptr &&
      existing_anim_data->action != nullptr) {
    add_error(r_result,
              reports,
              "Target Armature already has an Action; refusing to replace it in C1-C");
    return false;
  }
  if (!options.use_linear_interpolation && !options.use_vmd_bezier_interpolation) {
    add_warning(r_result,
                reports,
                "C1-C linear interpolation is disabled; this option is reserved for later tests");
  }

  std::vector<TrackWriteInfo> tracks;
  tracks.reserve(mapping.mapped_tracks.size());

  for (const VMDMappedBoneTrack &mapped : mapping.mapped_tracks) {
    bPoseChannel *pose_bone = target_armature.pose ?
                                  BKE_pose_channel_find_name(target_armature.pose,
                                                             mapped.armature_bone_name.c_str()) :
                                  nullptr;
    if (pose_bone == nullptr) {
      add_error(r_result,
                reports,
                "Mapped VMD bone is missing from target Armature: " +
                    mapped.armature_bone_name);
      return false;
    }

    TrackWriteInfo info;
    info.mapped = &mapped;
    info.pose_bone = pose_bone;
    info.converter.compute_from_pose_bone(*pose_bone, target_armature);
    info.location_path = escaped_pose_bone_path(mapped.armature_bone_name, "location");
    info.rotation_path = escaped_pose_bone_path(mapped.armature_bone_name,
                                                "rotation_quaternion");
    info.keyframes.reserve(mapped.keyframe_indices.size());

    for (const size_t keyframe_index : mapped.keyframe_indices) {
      if (keyframe_index >= model.bone_keyframes.size()) {
        add_error(r_result, reports, "VMD mapping contains an out-of-range keyframe index");
        return false;
      }
      const VMDBoneKeyframe &keyframe = model.bone_keyframes[keyframe_index];
      int frame = 0;
      if (!checked_frame(keyframe, options.frame_offset, frame, r_result, reports)) {
        return false;
      }
      if (!finite_quaternion(keyframe.rotation)) {
        add_error(r_result, reports, "VMD keyframe contains a non-finite quaternion");
        return false;
      }
      info.keyframes.push_back(&keyframe);
      if (r_result.first_frame < 0 || frame < r_result.first_frame) {
        r_result.first_frame = frame;
      }
      if (r_result.last_frame < 0 || frame > r_result.last_frame) {
        r_result.last_frame = frame;
      }
    }
    if (info.keyframes.empty()) {
      add_error(r_result, reports, "Mapped VMD bone track contains no keyframes");
      return false;
    }
    tracks.push_back(std::move(info));
  }

  if (!model.morph_frame_count && !model.camera_frame_count && !model.light_frame_count &&
      !model.shadow_frame_count) {
    /* No warning is needed for the common bone-only file. */
  }
  else {
    add_warning(r_result,
                reports,
                "VMD contains Morph/Camera/Light/Self Shadow data that this bone Action does not "
                "apply; use the dedicated VMD camera import for camera tracks");
  }
  add_warning(r_result, reports, "VMD has no FPS field; the current scene FPS is used");
  if (r_result.missing_track_count > 0) {
    add_warning(r_result,
                reports,
                "Some VMD bone tracks are missing from the target Armature and were skipped");
  }

  animrig::Action &action = animrig::action_add(*bmain, action_name);
  animrig::Slot &slot = action.slot_add_for_id(target_armature.id);
  action.layer_keystrip_ensure();
  if (action.layers().is_empty() || action.layer(0)->strips().is_empty()) {
    add_error(r_result, reports, "Failed to initialize the VMD Action keyframe strip");
    BKE_id_free(bmain, &action.id);
    return false;
  }
  animrig::Strip &strip = *action.layer(0)->strip(0);
  animrig::Channelbag &channelbag = strip.data<animrig::StripKeyframeData>(action)
                                        .channelbag_for_slot_add(slot);

  const animrig::KeyframeSettings key_settings = {
      BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, options.use_linear_interpolation ? BEZT_IPO_LIN :
                                                                                BEZT_IPO_BEZ};
  std::vector<std::pair<bPoseChannel *, eRotationModes>> old_rotation_modes;
  old_rotation_modes.reserve(tracks.size());

  for (TrackWriteInfo &track : tracks) {
    std::vector<FCurve *> location_curves;
    std::vector<FCurve *> rotation_curves;
    for (int index = 0; index < kLocationChannels; index++) {
      animrig::FCurveDescriptor descriptor;
      descriptor.rna_path = track.location_path;
      descriptor.array_index = index;
      descriptor.prop_type = PROP_FLOAT;
      descriptor.prop_subtype = PROP_NONE;
      FCurve &curve = channelbag.fcurve_ensure(nullptr, descriptor);
      location_curves.push_back(&curve);
      r_result.location_fcurve_count++;
    }
    for (int index = 0; index < kRotationChannels; index++) {
      animrig::FCurveDescriptor descriptor;
      descriptor.rna_path = track.rotation_path;
      descriptor.array_index = index;
      descriptor.prop_type = PROP_FLOAT;
      descriptor.prop_subtype = PROP_NONE;
      FCurve &curve = channelbag.fcurve_ensure(nullptr, descriptor);
      rotation_curves.push_back(&curve);
      r_result.rotation_fcurve_count++;
    }

    std::array<float, 4> previous_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    bool has_previous_rotation = false;
    for (const VMDBoneKeyframe *keyframe : track.keyframes) {
      int frame = 0;
      if (!checked_frame(*keyframe, options.frame_offset, frame, r_result, reports)) {
        BKE_id_free(bmain, &action.id);
        return false;
      }

      float location_values[3];
      track.converter.convert_location(
          keyframe->translation.data(), location_values, options.coordinate_scale);
      /* Validate source quaternion length BEFORE conversion.
       * convert_rotation normalizes the result, so the post-conversion
       * check would always pass. */
      const float src_rot_len = std::sqrt(
          keyframe->rotation[0] * keyframe->rotation[0] +
          keyframe->rotation[1] * keyframe->rotation[1] +
          keyframe->rotation[2] * keyframe->rotation[2] +
          keyframe->rotation[3] * keyframe->rotation[3]);
      if (!(src_rot_len > 0.0f) || !std::isfinite(src_rot_len)) {
        add_error(r_result, reports, "VMD keyframe contains a zero-length quaternion");
        BKE_id_free(bmain, &action.id);
        return false;
      }
      std::array<float, 4> rotation;
      track.converter.convert_rotation(keyframe->rotation.data(), rotation.data());
      if (has_previous_rotation) {
        const float dot = rotation[0] * previous_rotation[0] +
                          rotation[1] * previous_rotation[1] +
                          rotation[2] * previous_rotation[2] +
                          rotation[3] * previous_rotation[3];
        if (dot < 0.0f) {
          for (float &value : rotation) {
            value = -value;
          }
          r_result.quaternion_sign_flip_count++;
        }
      }
      previous_rotation = rotation;
      has_previous_rotation = true;

      for (int index = 0; index < kLocationChannels; index++) {
        if (!write_keyframe(
                *location_curves[index], frame, location_values[index], key_settings)) {
          add_error(r_result, reports, "Failed to insert VMD location keyframe");
          BKE_id_free(bmain, &action.id);
          return false;
        }
        r_result.location_keyframe_count++;
      }
      for (int index = 0; index < kRotationChannels; index++) {
        if (!write_keyframe(
                *rotation_curves[index], frame, rotation[index], key_settings)) {
          add_error(r_result, reports, "Failed to insert VMD quaternion keyframe");
          BKE_id_free(bmain, &action.id);
          return false;
        }
        r_result.rotation_keyframe_count++;
      }
    }

    for (FCurve *curve : location_curves) {
      BKE_fcurve_handles_recalc(*curve);
    }
    for (FCurve *curve : rotation_curves) {
      BKE_fcurve_handles_recalc(*curve);
    }

    if (options.use_vmd_bezier_interpolation && !options.use_linear_interpolation) {
      std::vector<const uint8_t *> interpolation_data;
      interpolation_data.reserve(track.keyframes.size());
      for (const VMDBoneKeyframe *keyframe : track.keyframes) {
        interpolation_data.push_back(
            reinterpret_cast<const uint8_t *>(keyframe->interpolation.data()));
      }
      const std::array<int, 4> control_offsets = {0, 4, 8, 12};
      const VMDBezierWarningCallback warning_callback = [&](const std::string &message) {
        add_warning(r_result, reports, message);
      };
      const int loc_vmd_channel[3] = {
          track.converter.interpolation_indices[0],
          track.converter.interpolation_indices[1],
          track.converter.interpolation_indices[2]};
      for (int c = 0; c < kLocationChannels; c++) {
        apply_vmd_bezier_to_curve(*location_curves[c],
                                   interpolation_data,
                                   loc_vmd_channel[c],
                                   16,
                                   control_offsets,
                                   warning_callback);
      }
      for (FCurve *curve : rotation_curves) {
        apply_vmd_bezier_to_curve(
            *curve, interpolation_data, 3, 16, control_offsets, warning_callback);
      }
      r_result.bezier_curve_count += int(location_curves.size() + rotation_curves.size());
    }

    old_rotation_modes.emplace_back(track.pose_bone, eRotationModes(track.pose_bone->rotmode));
  }

  for (const auto &[pose_bone, old_mode] : old_rotation_modes) {
    if (old_mode != ROT_MODE_QUAT) {
      pose_bone->rotmode = ROT_MODE_QUAT;
      r_result.rotation_mode_changed_count++;
    }
  }

  if (animrig::assign_action_and_slot(&action, &slot, target_armature.id) !=
      animrig::ActionSlotAssignmentResult::OK) {
    for (const auto &[pose_bone, old_mode] : old_rotation_modes) {
      pose_bone->rotmode = old_mode;
    }
    add_error(r_result, reports, "Failed to bind the completed VMD Action to the Armature");
    BKE_id_free(bmain, &action.id);
    return false;
  }

  r_result.action_bound = true;
  r_result.success = true;
  return true;
}

}  // namespace blender::io::vmd
