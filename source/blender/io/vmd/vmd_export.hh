/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>
#include <vector>

namespace blender {
struct Object;
struct ReportList;

namespace io::vmd {

struct VMDExportOptions {
  int frame_start = 0;
  int frame_end = 0;
  float coordinate_scale = 0.08f;
  std::string model_name;
  const Object *morph_controller = nullptr;
};

struct VMDExportReport {
  bool success = false;
  int bone_count = 0;
  int bone_frame_count = 0;
  int morph_count = 0;
  int morph_frame_count = 0;
  int skipped_curve_count = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

bool export_vmd_action(const Object &armature,
                       const std::string &filepath,
                       const VMDExportOptions &options,
                       ReportList *reports,
                       VMDExportReport &r_report);

/**
 * MikuMikuDance stores camera and lighting motion under this fixed model name. Writing it keeps
 * exported files recognizable as camera motion in MMD and MikuMikuMoving.
 */
extern const char *const VMD_CAMERA_MODEL_NAME;

struct VMDCameraExportOptions {
  int frame_start = 0;
  int frame_end = 0;
  float coordinate_scale = 0.08f;
  std::string model_name;
};

struct VMDCameraExportReport {
  bool success = false;
  /** True when the camera has the Empty parent created by the native VMD camera importer. */
  bool used_camera_rig = false;
  int camera_frame_count = 0;
  int bezier_segment_count = 0;
  int clamped_angle_count = 0;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

/**
 * Export one Camera Object's Action as the camera section of a VMD file.
 *
 * Two source layouts are supported, both evaluated from F-Curves rather than the depsgraph:
 *
 * - Camera rig: the Camera is parented to the Empty produced by `create_vmd_camera_rig()`. The
 *   Empty carries the MMD look-at center and rotation, the Camera's local Y carries distance, and
 *   the Camera data carries view angle and projection. This is the exact inverse of the importer,
 *   so VMD Bezier control bytes round-trip.
 * - Standalone Camera: an unparented Camera authored in Blender. Its world pose is written with
 *   distance zero, which MMD replays identically. Rotation cannot keep per-channel Bezier data
 *   because the MMD rotation is a non-linear recomposition of the Camera's own Euler channels, so
 *   rotation degrades to linear.
 *
 * A Camera parented to anything other than a conforming Empty is rejected instead of guessed.
 */
bool export_vmd_camera(const Object &camera,
                       const std::string &filepath,
                       const VMDCameraExportOptions &options,
                       ReportList *reports,
                       VMDCameraExportReport &r_report);

}  // namespace io::vmd
}  // namespace blender
