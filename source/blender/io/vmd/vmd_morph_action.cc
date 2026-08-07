/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_morph_action.hh"

#include "ANIM_action.hh"
#include "ANIM_fcurve.hh"
#include "BKE_anim_data.hh"
#include "BKE_fcurve.hh"
#include "BKE_key.hh"
#include "BKE_lib_id.hh"
#include "BKE_report.hh"

#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_key_types.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace blender::io::vmd {
namespace {

constexpr int64_t kMinFrame = std::numeric_limits<int>::min();
constexpr int64_t kMaxFrame = std::numeric_limits<int>::max();

void add_error(VMDMorphActionReport &result, ReportList *reports, const std::string &message)
{
  result.errors.push_back(message);
  BKE_report(reports, RPT_ERROR, message.c_str());
}

void add_warning(VMDMorphActionReport &result, ReportList *reports, const std::string &message)
{
  result.warnings.push_back(message);
  BKE_report(reports, RPT_WARNING, message.c_str());
}

bool checked_frame(const VMDMorphKeyframe &keyframe,
                   const int frame_offset,
                   int &r_frame,
                   VMDMorphActionReport &result,
                   ReportList *reports)
{
  const int64_t frame = int64_t(keyframe.frame) + int64_t(frame_offset);
  if (frame < kMinFrame || frame > kMaxFrame) {
    add_error(result,
              reports,
              "VMD morph frame plus frame offset is outside Blender's signed frame range");
    return false;
  }
  r_frame = int(frame);
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

void free_action(Main *bmain, animrig::Action *action)
{
  if (action != nullptr) {
    BKE_id_free(bmain, &action->id);
  }
}

}  // namespace

bool build_vmd_morph_action(Main *bmain,
                            Key &target_key,
                            const VMDModel &model,
                            const VMDMorphMappingReport &mapping,
                            const std::string &action_name,
                            const VMDMorphActionOptions &options,
                            ReportList *reports,
                            VMDMorphActionReport &r_result)
{
  r_result = {};
  r_result.action_name = action_name;
  r_result.mapped_track_count = mapping.mapped_track_count;
  r_result.missing_track_count = mapping.missing_track_count;

  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD Morph Action build requires a valid Main database");
    return false;
  }
  if (action_name.empty()) {
    add_error(r_result, reports, "VMD Morph Action name must not be empty");
    return false;
  }
  if (GS(target_key.id.name) != ID_KE) {
    add_error(r_result, reports, "VMD Morph Action target is not a valid Key ID");
    return false;
  }
  if (!mapping.mapping_valid) {
    add_error(r_result, reports, "VMD Morph mapping is invalid");
    return false;
  }
  if (mapping.mapped_track_count == 0 && mapping.mapped_tracks.empty()) {
    add_warning(r_result, reports, "No VMD Morph tracks are mapped; Morph Action creation skipped");
    return false;
  }
  if (mapping.mapped_track_count != int(mapping.mapped_tracks.size()) ||
      mapping.mapped_track_count <= 0) {
    add_error(r_result, reports, "VMD Morph mapping track count does not match mapped track data");
    return false;
  }
  if (mapping.missing_track_count > 0) {
    add_warning(r_result,
                reports,
                "Some VMD Morph tracks are missing from the target Shape Keys and were skipped");
  }
  if (target_key.block.first == nullptr) {
    add_error(r_result, reports, "VMD Morph Action target Key has no Shape Keys");
    return false;
  }

  const AnimData *existing_anim_data = BKE_animdata_from_id(&target_key.id);
  if (!options.replace_existing_action && existing_anim_data != nullptr &&
      existing_anim_data->action != nullptr) {
    add_error(r_result,
              reports,
              "Target Key already has an Action; refusing to replace it in C2-1C");
    return false;
  }
  if (!options.use_linear_interpolation) {
    add_warning(r_result,
                reports,
                "C2-1C linear interpolation is disabled; Bezier interpolation is not VMD-accurate");
  }

  struct TrackWriteInfo {
    const VMDMappedMorphTrack *mapped = nullptr;
    std::string rna_path;
    std::vector<const VMDMorphKeyframe *> keyframes;
  };
  std::vector<TrackWriteInfo> tracks;
  tracks.reserve(mapping.mapped_tracks.size());
  std::unordered_set<std::string> target_names;
  target_names.reserve(mapping.mapped_tracks.size());

  for (const VMDMappedMorphTrack &mapped : mapping.mapped_tracks) {
    if (mapped.target_morph_name.empty()) {
      add_error(r_result, reports, "Mapped VMD Morph target name is empty");
      return false;
    }
    if (!target_names.insert(mapped.target_morph_name).second) {
      add_error(r_result,
                reports,
                "VMD Morph mapping contains duplicate target track: " + mapped.target_morph_name);
      return false;
    }
    if (mapped.keyframe_indices.empty()) {
      add_error(r_result,
                reports,
                "Mapped VMD Morph track contains no keyframes: " + mapped.target_morph_name);
      return false;
    }

    KeyBlock *key_block = BKE_keyblock_find_name(&target_key, mapped.target_morph_name.c_str());
    if (key_block == nullptr) {
      add_error(r_result,
                reports,
                "Mapped VMD Morph is missing from target Key: " + mapped.target_morph_name);
      return false;
    }
    if (key_block == target_key.refkey) {
      add_error(r_result,
                reports,
                "Mapped VMD Morph resolves to the Basis Shape Key: " + mapped.target_morph_name);
      return false;
    }
    const std::optional<std::string> rna_path = BKE_keyblock_curval_rnapath_get(&target_key,
                                                                                  key_block);
    if (!rna_path || rna_path->empty()) {
      add_error(r_result,
                reports,
                "Failed to resolve Shape Key value RNA path: " + mapped.target_morph_name);
      return false;
    }

    TrackWriteInfo info;
    info.mapped = &mapped;
    info.rna_path = *rna_path;
    info.keyframes.reserve(mapped.keyframe_indices.size());
    uint32_t previous_frame = 0;
    bool has_previous_frame = false;
    for (const size_t keyframe_index : mapped.keyframe_indices) {
      if (keyframe_index >= model.morph_keyframes.size()) {
        add_error(r_result, reports, "VMD Morph mapping contains an out-of-range keyframe index");
        return false;
      }
      const VMDMorphKeyframe &keyframe = model.morph_keyframes[keyframe_index];
      if (has_previous_frame && keyframe.frame <= previous_frame) {
        add_error(r_result,
                  reports,
                  "VMD Morph mapping keyframes are not strictly ordered for: " +
                      mapped.target_morph_name);
        return false;
      }
      previous_frame = keyframe.frame;
      has_previous_frame = true;
      if (!std::isfinite(keyframe.weight)) {
        add_error(r_result,
                  reports,
                  "VMD Morph keyframe contains a non-finite weight: " +
                      mapped.target_morph_name);
        return false;
      }
      int frame = 0;
      if (!checked_frame(keyframe, options.frame_offset, frame, r_result, reports)) {
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
    tracks.push_back(std::move(info));
  }

  animrig::Action &action = animrig::action_add(*bmain, action_name);
  animrig::Slot &slot = action.slot_add_for_id(target_key.id);
  action.layer_keystrip_ensure();
  if (action.layers().is_empty() || action.layer(0)->strips().is_empty()) {
    add_error(r_result, reports, "Failed to initialize the VMD Morph Action keyframe strip");
    free_action(bmain, &action);
    return false;
  }
  animrig::Strip &strip = *action.layer(0)->strip(0);
  animrig::Channelbag &channelbag = strip.data<animrig::StripKeyframeData>(action)
                                        .channelbag_for_slot_add(slot);

  const animrig::KeyframeSettings key_settings = {
      BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, options.use_linear_interpolation ? BEZT_IPO_LIN :
                                                                                BEZT_IPO_BEZ};

  for (TrackWriteInfo &track : tracks) {
    animrig::FCurveDescriptor descriptor;
    descriptor.rna_path = track.rna_path;
    descriptor.array_index = 0;
    descriptor.prop_type = PROP_FLOAT;
    descriptor.prop_subtype = PROP_NONE;
    FCurve &curve = channelbag.fcurve_ensure(nullptr, descriptor);
    r_result.fcurve_count++;

    for (const VMDMorphKeyframe *keyframe : track.keyframes) {
      int frame = 0;
      if (!checked_frame(*keyframe, options.frame_offset, frame, r_result, reports) ||
          !write_keyframe(curve, frame, keyframe->weight, key_settings)) {
        add_error(r_result,
                  reports,
                  "Failed to insert VMD Morph keyframe: " + track.mapped->target_morph_name);
        free_action(bmain, &action);
        return false;
      }
      r_result.keyframe_count++;
    }
    BKE_fcurve_handles_recalc(curve);
  }

  if (animrig::assign_action_and_slot(&action, &slot, target_key.id) !=
      animrig::ActionSlotAssignmentResult::OK) {
    add_error(r_result, reports, "Failed to bind the completed VMD Morph Action to the Key");
    free_action(bmain, &action);
    return false;
  }

  /* [世界的歌] C2-2E / R1-VMD: report Group Morph raw channels explicitly. A
   * Group raw channel has no direct vertex output; its vertex effect (if any) is
   * produced by the C2-2D Driver expansion. Group Morphs that only reference
   * unsupported morph types (Bone/UV/Material/Flip/Impulse) therefore yield no
   * vertex effect from this Action, which is a legal downgrade, not a failure. */
  int group_target_count = 0;
  int group_no_vertex_count = 0;
  for (const VMDMappedMorphTrack &track : mapping.mapped_tracks) {
    if (track.is_group_target) {
      group_target_count++;
      if (track.is_group_no_vertex_output) {
        group_no_vertex_count++;
      }
    }
  }
  if (group_target_count > 0) {
    std::ostringstream message;
    message << group_target_count
            << " VMD Morph track(s) drive a Group Morph raw channel. A Group raw channel has no "
               "direct vertex output";
    if (group_no_vertex_count == group_target_count) {
      message << "; its vertex effect (if any) is produced by the C2-2D Driver expansion set up "
                 "during PMX import. Group Morphs that only reference unsupported morph types "
                 "(Bone/UV/Material/Flip/Impulse) yield no vertex effect from this Action.";
    }
    else {
      message << "; " << (group_target_count - group_no_vertex_count)
              << " of them reference Vertex Morphs directly, the rest rely on the C2-2D Driver.";
    }
    add_warning(r_result, reports, message.str());
  }

  r_result.action_bound = true;
  r_result.success = true;
  return true;
}

}  // namespace blender::io::vmd
