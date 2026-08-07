/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "testing/testing.h"

#include "pmx_group_morph.hh"

#include <limits>
#include <string>
#include <vector>

namespace blender::io::pmx::tests {
namespace {

static PMXMorph make_morph(const MorphType type, const char *name)
{
  PMXMorph morph;
  morph.type = type;
  morph.name_local = name;
  return morph;
}

static void add_edge(PMXMorph &group, const int target, const float influence)
{
  group.group_offsets.push_back({target, influence});
}

TEST(PMXGroupMorph, BuildsStableDefaultChannels)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));

  const std::vector<PMXMorphChannel> channels = build_default_morph_channels(model);
  ASSERT_EQ(channels.size(), 2);
  EXPECT_EQ(channels[0].morph_index, 0);
  EXPECT_EQ(channels[0].source_name, "Vertex");
  EXPECT_TRUE(channels[0].controller_channel);
  EXPECT_TRUE(channels[0].vertex_output);
  EXPECT_EQ(channels[1].morph_index, 1);
  EXPECT_TRUE(channels[1].controller_channel);
  EXPECT_FALSE(channels[1].vertex_output);
}

TEST(PMXGroupMorph, FinalizesStableControllerChannelNames)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Smile"));
  model.morphs.push_back(make_morph(MorphType::Group, "Smile"));
  model.morphs.push_back(make_morph(MorphType::Group, ""));

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = finalize_controller_channels(
      model, graph, std::vector<int>{0}, std::vector<std::string>{"Smile"});
  ASSERT_TRUE(report.valid);
  ASSERT_EQ(report.channels.size(), 3);
  EXPECT_EQ(report.channels[0].blender_name, "Smile");
  EXPECT_EQ(report.channels[1].blender_name, "Smile.1");
  EXPECT_EQ(report.channels[2].blender_name, "Group");
  EXPECT_EQ(report.warning_count, 2);
}

TEST(PMXGroupMorph, RejectsMismatchedVertexChannelRegistry)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = finalize_controller_channels(
      model, graph, std::vector<int>{0}, std::vector<std::string>{});
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 1);
}

TEST(PMXGroupMorph, RejectsDuplicateVertexChannelRegistry)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = finalize_controller_channels(
      model, graph, std::vector<int>{0, 0}, std::vector<std::string>{"A", "B"});
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 1);
}

TEST(PMXGroupMorph, AcceptsNestedAndMergedSupportedGraph)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "GroupA"));
  model.morphs.push_back(make_morph(MorphType::Group, "GroupB"));
  add_edge(model.morphs[1], 0, 0.5f);
  add_edge(model.morphs[2], 1, 0.25f);
  add_edge(model.morphs[2], 0, 0.1f);

  const PMXGroupMorphReport report = analyze_group_morphs(model);
  EXPECT_TRUE(report.valid);
  EXPECT_EQ(report.morph_count, 3);
  EXPECT_EQ(report.group_count, 2);
  EXPECT_EQ(report.vertex_count, 1);
  EXPECT_EQ(report.edges.size(), 3);
  EXPECT_EQ(report.unsupported_edge_count, 0);
  EXPECT_EQ(report.error_count, 0);
  EXPECT_EQ(report.warning_count, 0);
  EXPECT_EQ(report.max_depth, 2);
}

TEST(PMXGroupMorph, WarnsForUnsupportedTargetWithoutFailingGraph)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  model.morphs.push_back(make_morph(MorphType::Bone, "Bone"));
  add_edge(model.morphs[0], 1, 1.0f);

  const PMXGroupMorphReport report = analyze_group_morphs(model);
  EXPECT_TRUE(report.valid);
  EXPECT_EQ(report.unsupported_edge_count, 1);
  EXPECT_EQ(report.warning_count, 1);
  EXPECT_EQ(report.error_count, 0);
  ASSERT_EQ(report.issues.size(), 1);
  EXPECT_EQ(report.issues[0].severity, PMXGroupMorphIssue::Severity::Warning);
}

TEST(PMXGroupMorph, SkipsUnsupportedTargetDuringVertexExpansion)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  model.morphs.push_back(make_morph(MorphType::Bone, "Bone"));
  add_edge(model.morphs[1], 0, 0.5f);
  add_edge(model.morphs[1], 2, 1.0f);

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  ASSERT_TRUE(graph.valid);
  ASSERT_EQ(graph.warning_count, 1);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph);
  ASSERT_TRUE(report.valid);
  ASSERT_EQ(report.warning_count, 1);
  ASSERT_EQ(report.vertex_expressions.size(), 1);
  ASSERT_EQ(report.vertex_expressions[0].raw_morph_indices.size(), 2);
  EXPECT_EQ(report.vertex_expressions[0].raw_morph_indices[0], 0);
  EXPECT_EQ(report.vertex_expressions[0].raw_morph_indices[1], 1);
  EXPECT_FLOAT_EQ(report.vertex_expressions[0].coefficients[0], 1.0f);
  EXPECT_FLOAT_EQ(report.vertex_expressions[0].coefficients[1], 0.5f);
}

TEST(PMXGroupMorph, RejectsCycle)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Group, "GroupA"));
  model.morphs.push_back(make_morph(MorphType::Group, "GroupB"));
  add_edge(model.morphs[0], 1, 1.0f);
  add_edge(model.morphs[1], 0, 1.0f);

  const PMXGroupMorphReport report = analyze_group_morphs(model);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 1);
  ASSERT_EQ(report.issues.size(), 1);
  EXPECT_NE(report.issues[0].message.find("0 -> 1 -> 0"), std::string::npos);
}

TEST(PMXGroupMorph, RejectsInvalidTargetAndNonFiniteInfluence)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  add_edge(model.morphs[0], 4, 1.0f);
  add_edge(model.morphs[0], 0, std::numeric_limits<float>::quiet_NaN());

  const PMXGroupMorphReport report = analyze_group_morphs(model);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 2);
  EXPECT_EQ(report.edges.size(), 0);
}

TEST(PMXGroupMorph, ExpandsNestedAndMergedVertexExpression)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "GroupA"));
  model.morphs.push_back(make_morph(MorphType::Group, "GroupB"));
  add_edge(model.morphs[1], 0, 0.5f);
  add_edge(model.morphs[2], 1, 0.25f);
  add_edge(model.morphs[2], 0, 0.1f);

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph);
  ASSERT_TRUE(report.valid);
  ASSERT_EQ(report.vertex_expressions.size(), 1);
  const PMXVertexMorphExpression &expression = report.vertex_expressions[0];
  ASSERT_EQ(expression.raw_morph_indices.size(), 3);
  EXPECT_EQ(expression.raw_morph_indices[0], 0);
  EXPECT_EQ(expression.raw_morph_indices[1], 1);
  EXPECT_EQ(expression.raw_morph_indices[2], 2);
  EXPECT_FLOAT_EQ(expression.coefficients[0], 1.0f);
  EXPECT_FLOAT_EQ(expression.coefficients[1], 0.5f);
  /* GroupB -> GroupA -> Vertex contributes 0.25 * 0.5 = 0.125,
   * then the direct GroupB -> Vertex path contributes 0.1. */
  EXPECT_FLOAT_EQ(expression.coefficients[2], 0.125f + 0.1f);
  EXPECT_EQ(report.max_terms, 3);
}

TEST(PMXGroupMorph, ExpandsDirectVertexAndEmptyGroup)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "EmptyGroup"));

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph);
  ASSERT_TRUE(report.valid);
  ASSERT_EQ(report.vertex_expressions.size(), 1);
  ASSERT_EQ(report.vertex_expressions[0].raw_morph_indices.size(), 1);
  EXPECT_EQ(report.vertex_expressions[0].raw_morph_indices[0], 0);
  EXPECT_FLOAT_EQ(report.vertex_expressions[0].coefficients[0], 1.0f);
}

TEST(PMXGroupMorph, RejectsExpressionTermLimitWithoutPartialOutput)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  add_edge(model.morphs[1], 0, 0.5f);

  PMXGroupMorphOptions options;
  options.max_expression_terms = 1;
  const PMXGroupMorphReport graph = analyze_group_morphs(model, {}, options);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph, options);
  EXPECT_FALSE(report.valid);
  EXPECT_TRUE(report.vertex_expressions.empty());
  EXPECT_EQ(report.max_terms, 0);
  EXPECT_EQ(report.error_count, 1);
}

TEST(PMXGroupMorph, RejectsExpansionCoefficientLimitWithoutPartialOutput)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  add_edge(model.morphs[1], 0, 2.0f);

  PMXGroupMorphOptions options;
  options.max_abs_coefficient = 1.5f;
  const PMXGroupMorphReport graph = analyze_group_morphs(model, {}, options);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph, options);
  EXPECT_FALSE(report.valid);
  EXPECT_TRUE(report.vertex_expressions.empty());
  EXPECT_EQ(report.error_count, 1);
}

TEST(PMXGroupMorph, DoesNotMutateInputDuringExpansion)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  add_edge(model.morphs[1], 0, -0.5f);
  const PMXModel before = model;

  const PMXGroupMorphReport graph = analyze_group_morphs(model);
  const PMXGroupMorphReport report = expand_group_morph_expressions(model, graph);
  EXPECT_TRUE(report.valid);
  EXPECT_EQ(model.morphs[1].group_offsets[0].morph_index,
            before.morphs[1].group_offsets[0].morph_index);
  EXPECT_FLOAT_EQ(model.morphs[1].group_offsets[0].influence,
                  before.morphs[1].group_offsets[0].influence);
}

TEST(PMXGroupMorph, RejectsGraphEdgeLimit)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  add_edge(model.morphs[0], 1, 1.0f);
  add_edge(model.morphs[0], 1, 0.5f);

  PMXGroupMorphOptions options;
  options.max_graph_edges = 1;
  const PMXGroupMorphReport report = analyze_group_morphs(model, {}, options);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 1);
  EXPECT_EQ(report.edges.size(), 1);
  EXPECT_NE(report.issues[0].message.find("edge count"), std::string::npos);
}

TEST(PMXGroupMorph, RejectsDepthLimit)
{
  PMXModel model;
  for (int i = 0; i < 4; i++) {
    model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  }
  add_edge(model.morphs[0], 1, 1.0f);
  add_edge(model.morphs[1], 2, 1.0f);
  add_edge(model.morphs[2], 3, 1.0f);

  PMXGroupMorphOptions options;
  options.max_depth = 2;
  const PMXGroupMorphReport report = analyze_group_morphs(model, {}, options);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 1);
  EXPECT_NE(report.issues[0].message.find("depth"), std::string::npos);
}

TEST(PMXGroupMorph, RejectsInvalidChannelRegistry)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));

  std::vector<PMXMorphChannel> channels = build_default_morph_channels(model);
  channels[0].morph_index = 99;
  const PMXGroupMorphReport report = analyze_group_morphs(model, channels);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ(report.error_count, 2);
}

TEST(PMXGroupMorph, DoesNotMutateInputModel)
{
  PMXModel model;
  model.morphs.push_back(make_morph(MorphType::Vertex, "Vertex"));
  model.morphs.push_back(make_morph(MorphType::Group, "Group"));
  add_edge(model.morphs[1], 0, 0.5f);

  const PMXModel before = model;
  const PMXGroupMorphReport report = analyze_group_morphs(model);
  EXPECT_TRUE(report.valid);
  EXPECT_EQ(model.morphs.size(), before.morphs.size());
  EXPECT_EQ(model.morphs[1].group_offsets.size(), before.morphs[1].group_offsets.size());
  EXPECT_EQ(model.morphs[1].group_offsets[0].morph_index,
            before.morphs[1].group_offsets[0].morph_index);
  EXPECT_EQ(model.morphs[1].group_offsets[0].influence,
            before.morphs[1].group_offsets[0].influence);
}

}  // namespace
}  // namespace blender::io::pmx::tests
