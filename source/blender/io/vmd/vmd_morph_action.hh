/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include "vmd_morph_mapping.hh"

#include <string>
#include <vector>

namespace blender {
struct Key;
struct Main;
struct ReportList;

namespace io::vmd {

struct VMDMorphActionOptions {
  int frame_offset = 0;
  bool replace_existing_action = false;
  bool use_linear_interpolation = true;
};

struct VMDMorphActionReport {
  bool success = false;
  bool skipped = false;
  bool action_bound = false;
  std::string action_name;
  int mapped_track_count = 0;
  int missing_track_count = 0;
  int fcurve_count = 0;
  int keyframe_count = 0;
  int first_frame = -1;
  int last_frame = -1;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
};

/**
 * Build a new VMD Morph Action for one explicit Key ID.
 *
 * The Action is assigned only after all curves and keyframes have been written successfully.
 * The function does not look up an Object, Controller, or Mesh; ownership of the Key is explicit.
 */
bool build_vmd_morph_action(Main *bmain,
                            Key &target_key,
                            const VMDModel &model,
                            const VMDMorphMappingReport &mapping,
                            const std::string &action_name,
                            const VMDMorphActionOptions &options,
                            ReportList *reports,
                            VMDMorphActionReport &r_result);

}  // namespace io::vmd
}  // namespace blender
