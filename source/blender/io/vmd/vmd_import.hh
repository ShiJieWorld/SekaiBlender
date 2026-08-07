/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include "vmd_action.hh"
#include "vmd_camera_action.hh"
#include "vmd_morph_action.hh"
#include "vmd_morph_mapping.hh"

#include <string>
#include <vector>

namespace blender {
struct Main;
struct Collection;
struct Object;
struct ReportList;

namespace io::vmd {

struct VMDImportOptions {
  int frame_offset = 0;
  bool replace_existing_action = false;
  float coordinate_scale = 0.08f;
  bool use_linear_interpolation = true;
  bool use_vmd_bezier_interpolation = false;
};

struct VMDImportReport {
  bool success = false;
  VMDReadReport read;
  VMDMappingReport mapping;
  VMDActionReport action;
  VMDCameraActionReport camera_action;
  VMDMorphMappingReport morph_mapping;
  VMDMorphActionReport morph_action;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

/**
 * Import the bone animation from one VMD file into one explicit Armature Object.
 *
 * This is the only orchestration entry point for the editor operator. It keeps the
 * Reader, mapping, and Action stages in the VMD module and never searches for a
 * target object globally.
 */
bool import_vmd_action(Main *bmain,
                       Object &target_armature,
                       const std::string &filepath,
                       const VMDImportOptions &options,
                       ReportList *reports,
                       VMDImportReport &r_result);

/**
 * Import bone and vertex-morph animation into explicit PMX model targets.
 *
 * Unlike the legacy bone-only entry point, this function requires the caller to
 * provide the PMXMorphController object. It never searches Blender data globally.
 */
bool import_vmd_action_with_morphs(Main *bmain,
                                   Object &target_armature,
                                   Object &target_morph_controller,
                                   const std::string &filepath,
                                   const VMDImportOptions &options,
                                   ReportList *reports,
                                   VMDImportReport &r_result);

/** Import the camera section of one VMD file into a new native MMD camera rig. */
bool import_vmd_camera(Main *bmain,
                       Collection &target_collection,
                       const std::string &filepath,
                       const VMDImportOptions &options,
                       ReportList *reports,
                       VMDImportReport &r_result,
                       Object *target_camera = nullptr);

}  // namespace io::vmd
}  // namespace blender
