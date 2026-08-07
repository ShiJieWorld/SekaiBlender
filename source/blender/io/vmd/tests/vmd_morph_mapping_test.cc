/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_morph_mapping.hh"

#include "testing/testing.h"

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

namespace blender::io::vmd::tests {
namespace {

VMDMorphKeyframe morph_keyframe(const char *name,
                                const uint32_t frame,
                                const float weight,
                                const uint64_t source_offset)
{
  VMDMorphKeyframe key;
  key.morph_name = name;
  key.frame = frame;
  key.weight = weight;
  key.source_offset = source_offset;
  return key;
}

VMDModel make_model(std::initializer_list<VMDMorphKeyframe> keyframes)
{
  VMDModel model;
  model.morph_keyframes.assign(keyframes.begin(), keyframes.end());
  model.morph_frame_count = uint32_t(model.morph_keyframes.size());
  return model;
}

bool has_issue(const VMDMorphMappingReport &report,
               const VMDMappingIssue::Severity severity,
               const std::string &needle)
{
  return std::any_of(report.issues.begin(), report.issues.end(), [&](const VMDMappingIssue &issue) {
    return issue.severity == severity && issue.message.find(needle) != std::string::npos;
  });
}

TEST(VMDMorphMapping, exact_names_and_frame_order)
{
  const VMDModel model = make_model({
      morph_keyframe("Smile", 20, 0.8f, 100),
      morph_keyframe("Blink", 4, 1.0f, 123),
      morph_keyframe("Smile", 2, 0.2f, 146),
  });
  const std::vector<std::string> targets = {"Smile", "Blink"};

  const VMDMorphMappingReport report = map_morph_tracks(model, targets);

  EXPECT_TRUE(report.target_valid);
  EXPECT_TRUE(report.mapping_valid);
  EXPECT_EQ(report.target_morph_count, 2);
  EXPECT_EQ(report.vmd_track_count, 2);
  EXPECT_EQ(report.mapped_track_count, 2);
  EXPECT_EQ(report.missing_track_count, 0);
  EXPECT_EQ(report.mapped_keyframe_count, 3);
  EXPECT_EQ(report.first_frame, 2);
  EXPECT_EQ(report.last_frame, 20);
  ASSERT_EQ(report.mapped_tracks.size(), 2);

  const VMDMappedMorphTrack &blink = report.mapped_tracks[0];
  EXPECT_EQ(blink.vmd_morph_name, "Blink");
  EXPECT_EQ(blink.target_morph_name, "Blink");
  ASSERT_EQ(blink.keyframe_indices.size(), 1);
  EXPECT_EQ(blink.keyframe_indices[0], 1);

  const VMDMappedMorphTrack &smile = report.mapped_tracks[1];
  EXPECT_EQ(smile.vmd_morph_name, "Smile");
  ASSERT_EQ(smile.keyframe_indices.size(), 2);
  EXPECT_EQ(smile.keyframe_indices[0], 2);
  EXPECT_EQ(smile.keyframe_indices[1], 0);
  EXPECT_EQ(smile.first_frame, 2);
  EXPECT_EQ(smile.last_frame, 20);
  EXPECT_EQ(model.morph_keyframes[0].weight, 0.8f);
  EXPECT_EQ(model.morph_keyframes[2].weight, 0.2f);
  EXPECT_EQ(report.target_morph_names, targets);
}

TEST(VMDMorphMapping, missing_is_warning_and_zero_mapping_is_allowed)
{
  const VMDModel model = make_model({morph_keyframe("Missing", 8, 0.5f, 200)});

  const VMDMorphMappingReport report = map_morph_tracks(model, {"Smile"});

  EXPECT_TRUE(report.target_valid);
  EXPECT_TRUE(report.mapping_valid);
  EXPECT_EQ(report.vmd_track_count, 1);
  EXPECT_EQ(report.mapped_track_count, 0);
  EXPECT_EQ(report.missing_track_count, 1);
  EXPECT_EQ(report.mapped_keyframe_count, 0);
  ASSERT_EQ(report.missing_tracks.size(), 1);
  EXPECT_EQ(report.missing_tracks[0].vmd_morph_name, "Missing");
  EXPECT_EQ(report.missing_tracks[0].keyframe_count, 1);
  EXPECT_TRUE(has_issue(
      report, VMDMappingIssue::Severity::Warning, "missing in target Shape Keys"));

  const VMDMorphMappingReport all_missing = map_morph_tracks(model, {});
  EXPECT_TRUE(all_missing.target_valid);
  EXPECT_TRUE(all_missing.mapping_valid);
  EXPECT_EQ(all_missing.mapped_track_count, 0);
  EXPECT_EQ(all_missing.missing_track_count, 1);
}

TEST(VMDMorphMapping, duplicate_frame_last_record_wins)
{
  const VMDModel model = make_model({
      morph_keyframe("Smile", 10, 0.1f, 300),
      morph_keyframe("Smile", 4, 0.4f, 323),
      morph_keyframe("Smile", 10, 0.9f, 346),
      morph_keyframe("Smile", 10, 1.0f, 369),
  });

  const VMDMorphMappingReport report = map_morph_tracks(model, {"Smile"});

  ASSERT_TRUE(report.mapping_valid);
  EXPECT_EQ(report.duplicate_track_frame_count, 1);
  EXPECT_EQ(report.ignored_keyframe_count, 2);
  EXPECT_EQ(report.mapped_keyframe_count, 2);
  ASSERT_EQ(report.mapped_tracks.size(), 1);
  ASSERT_EQ(report.mapped_tracks[0].keyframe_indices.size(), 2);
  EXPECT_EQ(report.mapped_tracks[0].keyframe_indices[0], 1);
  EXPECT_EQ(report.mapped_tracks[0].keyframe_indices[1], 3);
  EXPECT_EQ(model.morph_keyframes[3].weight, 1.0f);
  EXPECT_TRUE(
      has_issue(report, VMDMappingIssue::Severity::Warning, "last record per frame wins"));
}

TEST(VMDMorphMapping, empty_name_is_warning_and_ignored)
{
  const VMDModel model = make_model({
      morph_keyframe("", 1, 0.2f, 400),
      morph_keyframe("", 2, 0.3f, 423),
      morph_keyframe("Smile", 3, 0.4f, 446),
  });

  const VMDMorphMappingReport report = map_morph_tracks(model, {"Smile"});

  EXPECT_TRUE(report.mapping_valid);
  EXPECT_EQ(report.empty_name_track_count, 1);
  EXPECT_EQ(report.ignored_keyframe_count, 2);
  EXPECT_EQ(report.vmd_track_count, 1);
  EXPECT_EQ(report.mapped_track_count, 1);
  EXPECT_TRUE(has_issue(report, VMDMappingIssue::Severity::Warning, "empty VMD morph name"));
}

TEST(VMDMorphMapping, invalid_target_names_are_errors)
{
  const VMDModel model = make_model({morph_keyframe("Smile", 1, 0.2f, 500)});

  const VMDMorphMappingReport empty_target = map_morph_tracks(model, {""});
  EXPECT_FALSE(empty_target.target_valid);
  EXPECT_FALSE(empty_target.mapping_valid);
  EXPECT_TRUE(has_issue(empty_target, VMDMappingIssue::Severity::Error, "target morph name is empty"));

  const VMDMorphMappingReport duplicate_target = map_morph_tracks(model, {"Smile", "Smile"});
  EXPECT_FALSE(duplicate_target.target_valid);
  EXPECT_FALSE(duplicate_target.mapping_valid);
  EXPECT_TRUE(has_issue(
      duplicate_target, VMDMappingIssue::Severity::Error, "duplicate target morph name"));
}

TEST(VMDMorphMapping, signed_frame_range_is_checked)
{
  const uint32_t max_frame = uint32_t(std::numeric_limits<int>::max());
  const VMDModel valid_model = make_model({
      morph_keyframe("Smile", max_frame, 0.5f, 600),
  });
  const VMDMorphMappingReport valid = map_morph_tracks(valid_model, {"Smile"});
  EXPECT_TRUE(valid.mapping_valid);
  EXPECT_EQ(valid.first_frame, std::numeric_limits<int>::max());
  EXPECT_EQ(valid.last_frame, std::numeric_limits<int>::max());

  const VMDModel invalid_model = make_model({
      morph_keyframe("Smile", max_frame + 1u, 0.5f, 623),
  });
  const VMDMorphMappingReport invalid = map_morph_tracks(invalid_model, {"Smile"});
  EXPECT_FALSE(invalid.mapping_valid);
  EXPECT_EQ(invalid.ignored_keyframe_count, 1);
  EXPECT_EQ(invalid.mapped_track_count, 0);
  EXPECT_TRUE(has_issue(
      invalid, VMDMappingIssue::Severity::Error, "exceeds Blender signed frame range"));
}

TEST(VMDMorphMapping, no_morph_records_is_stable)
{
  const VMDModel model;
  const VMDMorphMappingReport report = map_morph_tracks(model, {"Smile"});

  EXPECT_TRUE(report.target_valid);
  EXPECT_TRUE(report.mapping_valid);
  EXPECT_EQ(report.vmd_track_count, 0);
  EXPECT_EQ(report.mapped_track_count, 0);
  EXPECT_EQ(report.missing_track_count, 0);
  EXPECT_EQ(report.mapped_keyframe_count, 0);
  EXPECT_EQ(report.first_frame, -1);
  EXPECT_EQ(report.last_frame, -1);
  EXPECT_TRUE(report.issues.empty());
}

TEST(VMDMorphMapping, input_is_not_modified)
{
  const VMDModel model = make_model({
      morph_keyframe("Smile", 10, 0.1f, 700),
      morph_keyframe("Smile", 2, 0.2f, 723),
      morph_keyframe("Smile", 10, 0.9f, 746),
  });
  const std::vector<VMDMorphKeyframe> original_keyframes = model.morph_keyframes;
  const std::vector<std::string> targets = {"Smile"};

  (void)map_morph_tracks(model, targets);

  EXPECT_EQ(model.morph_keyframes.size(), original_keyframes.size());
  for (size_t i = 0; i < model.morph_keyframes.size(); i++) {
    EXPECT_EQ(model.morph_keyframes[i].morph_name, original_keyframes[i].morph_name);
    EXPECT_EQ(model.morph_keyframes[i].frame, original_keyframes[i].frame);
    EXPECT_EQ(model.morph_keyframes[i].weight, original_keyframes[i].weight);
    EXPECT_EQ(model.morph_keyframes[i].source_offset, original_keyframes[i].source_offset);
  }
  EXPECT_EQ(targets, (std::vector<std::string>{"Smile"}));
}

}  // namespace
}  // namespace blender::io::vmd::tests
