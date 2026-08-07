/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include "vmd_mapping.hh"

#include "DNA_action_types.h"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace blender {
struct Main;
struct Object;
struct ReportList;

namespace io::vmd {

/**
 * Per-bone converter that transforms VMD data (MMD Y-up world space)
 * into Blender bone-local space. Mirrors mmd_tools BoneConverter.
 *
 * For each bone, the conversion matrix is derived from the bone's rest-pose
 * armature matrix (Bone::arm_mat). The matrix handles two simultaneous
 * transforms: Y-up ↔ Z-up axis swapping, and bone-local rest-pose rotation
 * compensation.
 *
 * When Bone::arm_mat is identity (root bones or world-aligned bones), the
 * converter naturally degrades to the simple (x,z,y) swap behavior.
 */
struct BoneConverter {
  /** The conversion 3x3 rotation matrix.
   *  Transforms MMD Y-up locations to bone-local space:
   *  bl_loc[i] = sum_j(mat[i][j] * vmd_loc[j]) * scale */
  float mat[3][3];

  /** The conversion quaternion precomputed from mat for rotation conjugation:
   *  bl_rot = q_conv * src_rot * conj(q_conv) */
  float q_conv[4];

  /** Per-axis VMD interpolation channel mapping (mirrors mmd_tools
   *  _InterpolationHelper). Computed greedily from |mat[i][j]|:
   *  interpolation_indices[Blender_axis] = VMD_channel_index. */
  int interpolation_indices[3] = {0, 1, 2};

  /** Default constructor: identity conversion. */
  BoneConverter();

  /** Build the converter from a PoseChannel's rest-pose armature matrix. */
  void compute_from_pose_bone(const bPoseChannel &pchan, const Object &ob);

  /** Convert a VMD location (MMD Y-up) to bone-local space. */
  void convert_location(const float vmd_loc[3], float bl_loc[3], float scale) const;

  /** Convert a VMD rotation (MMD Y-up, VMD raw {qx,qy,qz,qw}) to bone-local
   *  space in Blender quaternion order {w,x,y,z}. */
  void convert_rotation(const float vmd_rot[4], float bl_rot[4]) const;

  /** Convert Blender bone-local location back to VMD MMD Y-up coordinates. */
  void inverse_location(const float bl_loc[3], float vmd_loc[3], float scale) const;

  /** Convert Blender quaternion {w,x,y,z} back to VMD raw {x,y,z,w}. */
  void inverse_rotation(const float bl_rot[4], float vmd_rot[4]) const;
};

struct VMDActionOptions {
  int frame_offset = 0;
  bool replace_existing_action = false;
  bool use_linear_interpolation = true;
  bool use_vmd_bezier_interpolation = false;
  float coordinate_scale = 0.08f;
};

struct VMDActionReport {
  bool success = false;
  bool action_bound = false;
  std::string action_name;
  int mapped_track_count = 0;
  int missing_track_count = 0;
  int location_fcurve_count = 0;
  int rotation_fcurve_count = 0;
  int location_keyframe_count = 0;
  int rotation_keyframe_count = 0;
  int first_frame = -1;
  int last_frame = -1;
  int quaternion_sign_flip_count = 0;
  int rotation_mode_changed_count = 0;
  int bezier_curve_count = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

using VMDBezierWarningCallback = std::function<void(const std::string &)>;

/**
 * Apply VMD Bezier control points to one Blender F-Curve.
 *
 * `interpolation_data` contains one raw interpolation byte block per keyframe. The block for
 * the destination keyframe controls the segment from the preceding keyframe, matching the VMD
 * convention. `channel_stride` and `control_offsets` describe the format-specific packing: bone
 * channels use a 16-byte stride and offsets {0, 4, 8, 12}; camera channels use a 4-byte stride
 * and offsets {0, 2, 1, 3} because their records store {ax, bx, ay, by}.
 */
void apply_vmd_bezier_to_curve(FCurve &curve,
                               const std::vector<const uint8_t *> &interpolation_data,
                               int vmd_channel,
                               int channel_stride,
                               const std::array<int, 4> &control_offsets,
                               const VMDBezierWarningCallback &warning_callback = {});

/**
 * Build a new VMD Action for one explicit Armature Object.
 *
 * The Action is assigned only after all curves and keyframes have been written successfully.
 * Partial missing tracks are allowed when the mapping report is valid; no global object lookup is
 * performed.
 */
bool build_vmd_action(Main *bmain,
                      Object &target_armature,
                      const VMDModel &model,
                      const VMDMappingReport &mapping,
                      const std::string &action_name,
                      const VMDActionOptions &options,
                      ReportList *reports,
                      VMDActionReport &r_result);

}  // namespace io::vmd
}  // namespace blender
