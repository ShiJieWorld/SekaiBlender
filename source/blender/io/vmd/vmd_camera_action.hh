/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include "vmd_action.hh"

#include <string>
#include <vector>

namespace blender {
struct Collection;
struct Main;
struct Object;
struct ReportList;

namespace io::vmd {

struct VMDCameraActionOptions {
  int frame_offset = 0;
  bool replace_existing_action = false;
  bool use_linear_interpolation = true;
  bool use_vmd_bezier_interpolation = false;
  float coordinate_scale = 0.08f;
};

struct VMDCameraActionReport {
  bool success = false;
  bool parent_action_bound = false;
  bool camera_action_bound = false;
  bool camera_data_action_bound = false;
  std::string action_name;
  std::string target_empty_name;
  std::string target_camera_name;
  int fcurve_count = 0;
  int keyframe_count = 0;
  int first_frame = -1;
  int last_frame = -1;
  int bezier_curve_count = 0;
  int duplicate_frame_count = 0;
  int clamped_angle_count = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

/** Create the parent Empty and child Camera used by the native VMD camera importer. */
bool create_vmd_camera_rig(Main *bmain,
                           Collection &collection,
                           const std::string &base_name,
                           float coordinate_scale,
                           Object *&r_target_empty,
                           Object *&r_camera,
                           ReportList *reports,
                           VMDCameraActionReport &r_result,
                           Object *existing_camera = nullptr);

/** Build and bind the parent, child-object, and Camera-data Actions for one VMD camera stream. */
bool build_vmd_camera_action(Main *bmain,
                             Object &target_empty,
                             Object &target_camera,
                             const VMDModel &model,
                             const std::string &action_name,
                             const VMDCameraActionOptions &options,
                             ReportList *reports,
                             VMDCameraActionReport &r_result);

}  // namespace io::vmd
}  // namespace blender
