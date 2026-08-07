/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_morph_mapping.hh"

#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace blender::io::vmd {
namespace {

struct TrackGroup {
  std::string name;
  std::vector<size_t> keyframe_indices;
};

void add_issue(VMDMorphMappingReport &report,
               const VMDMappingIssue::Severity severity,
               std::string path,
               std::string message,
               const uint64_t source_offset = 0)
{
  report.issues.push_back({severity, std::move(path), std::move(message), source_offset});
}

std::string track_path(const char *kind, const size_t index)
{
  return std::string(kind) + "[" + std::to_string(index) + "]";
}

void update_frame_range(VMDMorphMappingReport &report, const uint32_t frame)
{
  if (frame > uint32_t(std::numeric_limits<int>::max())) {
    return;
  }
  const int signed_frame = int(frame);
  if (report.first_frame < 0 || signed_frame < report.first_frame) {
    report.first_frame = signed_frame;
  }
  if (report.last_frame < 0 || signed_frame > report.last_frame) {
    report.last_frame = signed_frame;
  }
}

bool validate_target_names(const std::vector<std::string> &target_morph_names,
                           VMDMorphMappingReport &report,
                           std::unordered_map<std::string, int> &target_indices)
{
  bool valid = true;
  target_indices.reserve(target_morph_names.size());
  for (size_t index = 0; index < target_morph_names.size(); index++) {
    const std::string &name = target_morph_names[index];
    if (name.empty()) {
      add_issue(report,
                VMDMappingIssue::Severity::Error,
                track_path("target_morphs", index),
                "target morph name is empty");
      valid = false;
      continue;
    }
    const auto [it, inserted] = target_indices.emplace(name, int(index));
    if (!inserted) {
      add_issue(report,
                VMDMappingIssue::Severity::Error,
                track_path("target_morphs", index),
                "duplicate target morph name; exact mapping is ambiguous");
      valid = false;
      (void)it;
    }
  }
  return valid;
}

std::vector<size_t> select_unique_keyframes(const TrackGroup &group,
                                            const VMDModel &model,
                                            VMDMorphMappingReport &report)
{
  std::map<uint32_t, std::vector<size_t>> by_frame;
  for (const size_t keyframe_index : group.keyframe_indices) {
    const VMDMorphKeyframe &keyframe = model.morph_keyframes[keyframe_index];
    by_frame[keyframe.frame].push_back(keyframe_index);
  }

  std::vector<size_t> selected;
  selected.reserve(by_frame.size());
  int dup_frames = 0;
  for (const auto &[frame, indices] : by_frame) {
    if (indices.size() > 1) {
      report.duplicate_track_frame_count++;
      report.ignored_keyframe_count += int(indices.size() - 1);
      dup_frames++;
    }
    selected.push_back(indices.back());
  }
  if (dup_frames > 0) {
    add_issue(report,
              VMDMappingIssue::Severity::Warning,
              "morph_tracks[\"" + group.name + "\"]",
              std::to_string(dup_frames) + " duplicate frame(s); last record per frame wins",
              model.morph_keyframes[group.keyframe_indices.front()].source_offset);
  }
  return selected;
}

void fill_track_frame_range(VMDMappedMorphTrack &track,
                            const VMDModel &model,
                            const std::vector<size_t> &indices)
{
  track.keyframe_count = int(indices.size());
  if (indices.empty()) {
    return;
  }
  track.first_frame = model.morph_keyframes[indices.front()].frame;
  track.last_frame = model.morph_keyframes[indices.back()].frame;
}

VMDMissingMorphTrack make_missing_track(const TrackGroup &group,
                                       const VMDModel &model,
                                       const std::vector<size_t> &selected)
{
  VMDMissingMorphTrack track;
  track.vmd_morph_name = group.name;
  track.keyframe_count = int(selected.size());
  track.first_source_offset = model.morph_keyframes[group.keyframe_indices.front()].source_offset;
  if (!selected.empty()) {
    track.first_frame = model.morph_keyframes[selected.front()].frame;
    track.last_frame = model.morph_keyframes[selected.back()].frame;
  }
  return track;
}

}  // namespace

VMDMorphMappingReport map_morph_tracks(
    const VMDModel &model, const std::vector<std::string> &target_morph_names)
{
  VMDMorphMappingReport report;
  report.target_morph_count = int(target_morph_names.size());
  report.target_morph_names = target_morph_names;

  std::unordered_map<std::string, int> target_indices;
  const bool target_names_valid = validate_target_names(
      target_morph_names, report, target_indices);
  report.target_valid = target_names_valid;

  std::map<std::string, TrackGroup> groups;
  bool has_fatal_issue = false;
  bool empty_name_reported = false;
  for (size_t index = 0; index < model.morph_keyframes.size(); index++) {
    const VMDMorphKeyframe &keyframe = model.morph_keyframes[index];
    update_frame_range(report, keyframe.frame);
    if (keyframe.frame > uint32_t(std::numeric_limits<int>::max())) {
      has_fatal_issue = true;
      report.ignored_keyframe_count++;
      add_issue(report,
                VMDMappingIssue::Severity::Error,
                "morph_keyframes[" + std::to_string(index) + "]",
                "frame number exceeds Blender signed frame range",
                keyframe.source_offset);
      continue;
    }
    if (keyframe.morph_name.empty()) {
      if (!empty_name_reported) {
        report.empty_name_track_count++;
        empty_name_reported = true;
      }
      report.ignored_keyframe_count++;
      add_issue(report,
                VMDMappingIssue::Severity::Warning,
                "morph_keyframes[" + std::to_string(index) + "]",
                "empty VMD morph name; keyframe ignored",
                keyframe.source_offset);
      continue;
    }
    auto [it, inserted] = groups.try_emplace(keyframe.morph_name);
    if (inserted) {
      it->second.name = keyframe.morph_name;
    }
    it->second.keyframe_indices.push_back(index);
  }

  report.vmd_track_count = int(groups.size());
  report.mapped_tracks.reserve(groups.size());
  report.missing_tracks.reserve(groups.size());

  for (const auto &[name, group] : groups) {
    const std::vector<size_t> selected = select_unique_keyframes(group, model, report);
    const auto target_it = target_indices.find(name);
    if (target_it == target_indices.end()) {
      report.missing_track_count++;
      report.missing_tracks.push_back(make_missing_track(group, model, selected));
      add_issue(report,
                VMDMappingIssue::Severity::Warning,
                "morph_tracks[\"" + name + "\"]",
                "VMD morph is missing in target Shape Keys",
                model.morph_keyframes[group.keyframe_indices.front()].source_offset);
      continue;
    }

    VMDMappedMorphTrack track;
    track.vmd_morph_name = name;
    track.target_morph_name = target_morph_names[size_t(target_it->second)];
    track.keyframe_indices = selected;
    fill_track_frame_range(track, model, selected);
    report.mapped_keyframe_count += int(selected.size());
    report.mapped_track_count++;
    report.mapped_tracks.push_back(std::move(track));
  }

  report.mapping_valid = target_names_valid && !has_fatal_issue;
  return report;
}

}  // namespace blender::io::vmd
