/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#pragma once

#include "vmd_mapping.hh"

#include <cstdint>
#include <string>
#include <vector>

namespace blender::io::vmd {

struct VMDMappedMorphTrack {
  std::string vmd_morph_name;
  std::string target_morph_name;
  /* C2-2E: true when the mapped target is a Group Morph raw channel (not a
   * Vertex Morph raw channel). Group raw channels are expanded into vertex
   * changes by the C2-2D Driver, not directly by this Action. */
  bool is_group_target = false;
  /* R1-VMD: true when the mapped Group raw channel has no direct vertex output
   * (its PMX definition records vertex_output == false). A Group raw channel
   * always lacks direct vertex output; its vertex effect, if any, is produced
   * by the C2-2D Driver expansion. Group Morphs that only reference unsupported
   * morph types therefore yield no vertex effect from this Action. */
  bool is_group_no_vertex_output = false;
  int keyframe_count = 0;
  uint32_t first_frame = 0;
  uint32_t last_frame = 0;
  std::vector<size_t> keyframe_indices;
};

struct VMDMissingMorphTrack {
  std::string vmd_morph_name;
  int keyframe_count = 0;
  uint32_t first_frame = 0;
  uint32_t last_frame = 0;
  uint64_t first_source_offset = 0;
};

struct VMDMorphMappingReport {
  bool target_valid = false;
  bool mapping_valid = false;
  int target_morph_count = 0;
  int vmd_track_count = 0;
  int mapped_track_count = 0;
  int missing_track_count = 0;
  int mapped_keyframe_count = 0;
  int ignored_keyframe_count = 0;
  int duplicate_track_frame_count = 0;
  int empty_name_track_count = 0;
  int first_frame = -1;
  int last_frame = -1;
  std::vector<std::string> target_morph_names;
  std::vector<VMDMappedMorphTrack> mapped_tracks;
  std::vector<VMDMissingMorphTrack> missing_tracks;
  std::vector<VMDMappingIssue> issues;
};

/**
 * Map VMD morph tracks to one explicit Shape Key name snapshot.
 *
 * The function only compares UTF-8 names and returns a pure in-memory report.
 * It does not access or modify Blender data, create Actions, or create F-Curves.
 */
VMDMorphMappingReport map_morph_tracks(const VMDModel &model,
                                       const std::vector<std::string> &target_morph_names);

}  // namespace blender::io::vmd
