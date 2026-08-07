/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "pmx_group_morph.hh"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace blender::io::pmx {
namespace {

enum class VisitState : uint8_t {
  White,
  Gray,
  Black,
};

static bool is_known_morph_type(const MorphType type)
{
  switch (type) {
    case MorphType::Group:
    case MorphType::Vertex:
    case MorphType::Bone:
    case MorphType::UV:
    case MorphType::UV_2nd:
    case MorphType::UV_3rd:
    case MorphType::UV_4th:
    case MorphType::Material:
    case MorphType::Flip:
    case MorphType::Impulse:
      return true;
  }
  return false;
}

static const char *morph_type_name(const MorphType type)
{
  switch (type) {
    case MorphType::Group:
      return "Group";
    case MorphType::Vertex:
      return "Vertex";
    case MorphType::Bone:
      return "Bone";
    case MorphType::UV:
      return "UV";
    case MorphType::UV_2nd:
      return "UV_2nd";
    case MorphType::UV_3rd:
      return "UV_3rd";
    case MorphType::UV_4th:
      return "UV_4th";
    case MorphType::Material:
      return "Material";
    case MorphType::Flip:
      return "Flip";
    case MorphType::Impulse:
      return "Impulse";
  }
  return "Unknown";
}

static void add_issue(PMXGroupMorphReport &report,
                      const PMXGroupMorphIssue::Severity severity,
                      const int morph_index,
                      const int target_morph_index,
                      std::string message)
{
  PMXGroupMorphIssue issue;
  issue.severity = severity;
  issue.morph_index = morph_index;
  issue.target_morph_index = target_morph_index;
  issue.message = std::move(message);
  report.issues.push_back(std::move(issue));
  if (severity == PMXGroupMorphIssue::Severity::Error) {
    report.error_count++;
  }
  else {
    report.warning_count++;
  }
}

static std::vector<PMXMorphChannel> make_default_channels(const PMXModel &model)
{
  std::vector<PMXMorphChannel> channels;
  channels.reserve(model.morphs.size());
  for (int index = 0; index < int(model.morphs.size()); index++) {
    const PMXMorph &morph = model.morphs[index];
    PMXMorphChannel channel;
    channel.morph_index = index;
    channel.source_name = morph.name_local;
    channel.source_name_universal = morph.name_universal;
    channel.blender_name = morph.name_local;
    channel.type = morph.type;
    channel.controller_channel = morph.type == MorphType::Group || morph.type == MorphType::Vertex;
    channel.vertex_output = morph.type == MorphType::Vertex;
    channels.push_back(std::move(channel));
  }
  return channels;
}

static bool validate_channel_registry(const PMXModel &model,
                                      const std::vector<PMXMorphChannel> &channels,
                                      PMXGroupMorphReport &report)
{
  std::vector<bool> seen(model.morphs.size(), false);
  bool valid = true;
  for (const PMXMorphChannel &channel : channels) {
    if (channel.morph_index < 0 || channel.morph_index >= int(model.morphs.size())) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                channel.morph_index,
                -1,
                "Morph channel index is out of range");
      valid = false;
      continue;
    }
    if (seen[channel.morph_index]) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                channel.morph_index,
                -1,
                "Duplicate Morph channel index");
      valid = false;
    }
    seen[channel.morph_index] = true;
    if (!is_known_morph_type(channel.type)) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                channel.morph_index,
                -1,
                "Morph channel has an unknown Morph type");
      valid = false;
    }
    else if (channel.type != model.morphs[channel.morph_index].type) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                channel.morph_index,
                -1,
                "Morph channel type does not match PMX Morph type");
      valid = false;
    }
  }
  for (int morph_index = 0; morph_index < int(model.morphs.size()); morph_index++) {
    if (!seen[morph_index]) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                "Morph channel registry is missing a PMX Morph index");
      valid = false;
    }
  }
  return valid;
}

static bool dfs_group(const PMXModel &model,
                      const PMXGroupMorphOptions &options,
                      const int morph_index,
                      std::vector<VisitState> &states,
                      std::vector<int> &path,
                      std::vector<int> &group_heights,
                      PMXGroupMorphReport &report,
                      int &graph_edges,
                      int depth)
{
  if (depth > options.max_depth) {
    add_issue(report,
              PMXGroupMorphIssue::Severity::Error,
              morph_index,
              -1,
              "Group Morph recursion depth exceeds the configured limit");
    return false;
  }

  states[morph_index] = VisitState::Gray;
  path.push_back(morph_index);
  report.max_depth = std::max(report.max_depth, depth);
  bool valid = true;
  int group_height = 1;
  const PMXMorph &morph = model.morphs[morph_index];
  for (const PMXGroupMorphOffset &offset : morph.group_offsets) {
    if (graph_edges >= options.max_graph_edges) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                "Group Morph edge count exceeds the configured limit");
      valid = false;
      break;
    }
    graph_edges++;
    const int target = offset.morph_index;
    if (target < 0 || target >= int(model.morphs.size())) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                target,
                "Group Morph target index is out of range");
      valid = false;
      continue;
    }
    if (!std::isfinite(offset.influence) ||
        std::abs(offset.influence) > options.max_abs_influence) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                target,
                "Group Morph influence is non-finite or exceeds the configured limit");
      valid = false;
      continue;
    }

    report.edges.push_back({morph_index, target, offset.influence});
    const MorphType target_type = model.morphs[target].type;
    if (target_type != MorphType::Group && target_type != MorphType::Vertex) {
      report.unsupported_edge_count++;
      switch (target_type) {
        case MorphType::Bone:
          report.unsupported_bone_count++;
          break;
        case MorphType::UV:
        case MorphType::UV_2nd:
        case MorphType::UV_3rd:
        case MorphType::UV_4th:
          report.unsupported_uv_count++;
          break;
        case MorphType::Material:
          report.unsupported_material_count++;
          break;
        case MorphType::Flip:
          report.unsupported_flip_count++;
          break;
        case MorphType::Impulse:
          report.unsupported_impulse_count++;
          break;
        default:
          break;
      }
      std::ostringstream message;
      message << "Group Morph edge targets unsupported Morph type " << morph_type_name(target_type);
      add_issue(report,
                PMXGroupMorphIssue::Severity::Warning,
                morph_index,
                target,
                message.str());
      continue;
    }
    if (target_type != MorphType::Group) {
      continue;
    }
    if (states[target] == VisitState::Gray) {
      auto cycle_begin = std::find(path.begin(), path.end(), target);
      std::ostringstream message;
      message << "Group Morph cycle:";
      for (auto it = cycle_begin; it != path.end(); ++it) {
        message << ' ' << *it << " ->";
      }
      message << ' ' << target;
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                target,
                message.str());
      valid = false;
      continue;
    }
    if (states[target] == VisitState::White &&
        !dfs_group(
            model, options, target, states, path, group_heights, report, graph_edges, depth + 1)) {
      valid = false;
    }
    if (states[target] == VisitState::Black) {
      group_height = std::max(group_height, 1 + group_heights[target]);
      const int candidate_depth = depth + group_heights[target];
      if (candidate_depth > options.max_depth) {
        add_issue(report,
                  PMXGroupMorphIssue::Severity::Error,
                  morph_index,
                  target,
                  "Group Morph recursion depth exceeds the configured limit");
        valid = false;
      }
      else {
        report.max_depth = std::max(report.max_depth, candidate_depth);
      }
    }
  }
  path.pop_back();
  states[morph_index] = VisitState::Black;
  group_heights[morph_index] = group_height;
  return valid;
}

}  // namespace

std::vector<PMXMorphChannel> build_default_morph_channels(const PMXModel &model)
{
  return make_default_channels(model);
}

PMXGroupMorphReport analyze_group_morphs(const PMXModel &model,
                                         const std::vector<PMXMorphChannel> &channels,
                                         const PMXGroupMorphOptions &options)
{
  PMXGroupMorphReport report;
  report.morph_count = int(model.morphs.size());
  if (report.morph_count > options.max_morph_count) {
    add_issue(report,
              PMXGroupMorphIssue::Severity::Error,
              -1,
              -1,
              "Morph count exceeds the configured limit");
    return report;
  }

  report.channels = channels.empty() ? make_default_channels(model) : channels;
  if (!validate_channel_registry(model, report.channels, report)) {
    return report;
  }

  std::vector<VisitState> states(model.morphs.size(), VisitState::White);
  std::vector<int> path;
  std::vector<int> group_heights(model.morphs.size(), 0);
  int graph_edges = 0;
  for (int morph_index = 0; morph_index < report.morph_count; morph_index++) {
    const PMXMorph &morph = model.morphs[morph_index];
    if (morph.type == MorphType::Group) {
      report.group_count++;
      if (states[morph_index] == VisitState::White &&
          !dfs_group(
              model, options, morph_index, states, path, group_heights, report, graph_edges, 1)) {
        /* Errors are already recorded with their precise edge/path. */
      }
    }
    if (morph.type == MorphType::Vertex) {
      report.vertex_count++;
    }
  }

  for (const PMXMorphChannel &channel : report.channels) {
    if (channel.controller_channel) {
      report.supported_channel_count++;
    }
  }
  report.valid = report.error_count == 0;
  return report;
}

PMXGroupMorphReport expand_group_morph_expressions(const PMXModel &model,
                                                   const PMXGroupMorphReport &validated_report,
                                                   const PMXGroupMorphOptions &options)
{
  PMXGroupMorphReport report = validated_report;
  report.vertex_expressions.clear();
  report.max_terms = 0;
  if (!validated_report.valid || validated_report.morph_count != int(model.morphs.size()) ||
      validated_report.max_depth > options.max_depth) {
    add_issue(report,
              PMXGroupMorphIssue::Severity::Error,
              -1,
              -1,
              "Cannot expand an invalid, mismatched, or too-deep Group Morph report");
    report.valid = false;
    return report;
  }

  struct IncomingEdge {
    int group_index;
    float influence;
  };
  std::vector<std::vector<IncomingEdge>> incoming(model.morphs.size());
  for (const PMXGroupMorphEdge &edge : validated_report.edges) {
    if (edge.group_morph_index < 0 || edge.group_morph_index >= int(model.morphs.size()) ||
        edge.target_morph_index < 0 || edge.target_morph_index >= int(model.morphs.size()) ||
        model.morphs[edge.group_morph_index].type != MorphType::Group) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                edge.group_morph_index,
                edge.target_morph_index,
                "Validated Group Morph edge is inconsistent with the PMX model");
      report.valid = false;
      report.vertex_expressions.clear();
      return report;
    }
    const MorphType target_type = model.morphs[edge.target_morph_index].type;
    if (target_type != MorphType::Group && target_type != MorphType::Vertex) {
      /* The graph validator has already recorded this as an unsupported
       * warning. It must not block the supported Group/Vertex subgraph from
       * producing Geometry expressions. */
      continue;
    }
    incoming[edge.target_morph_index].push_back({edge.group_morph_index, edge.influence});
  }

  std::vector<std::map<int, double>> cache(model.morphs.size());
  std::vector<VisitState> states(model.morphs.size(), VisitState::White);
  bool expansion_valid = true;
  std::function<const std::map<int, double> &(int, int)> expand_node;
  expand_node = [&](const int morph_index, const int group_depth) -> const std::map<int, double> & {
    if (states[morph_index] == VisitState::Black) {
      return cache[morph_index];
    }
    if (states[morph_index] == VisitState::Gray) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                "Group Morph expansion encountered a cycle");
      expansion_valid = false;
      return cache[morph_index];
    }
    if (group_depth > options.max_depth) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                "Group Morph expansion depth exceeds the configured limit");
      expansion_valid = false;
      return cache[morph_index];
    }

    states[morph_index] = VisitState::Gray;
    cache[morph_index][morph_index] = 1.0;
    for (const IncomingEdge &edge : incoming[morph_index]) {
      const std::map<int, double> &parent = expand_node(edge.group_index, group_depth + 1);
      for (const auto &[raw_index, coefficient] : parent) {
        const double contribution = coefficient * double(edge.influence);
        const double merged = cache[morph_index][raw_index] + contribution;
        if (!std::isfinite(merged) || std::abs(merged) > double(options.max_abs_coefficient)) {
          add_issue(report,
                    PMXGroupMorphIssue::Severity::Error,
                    morph_index,
                    edge.group_index,
                    "Group Morph expansion coefficient is non-finite or exceeds the limit");
          expansion_valid = false;
          continue;
        }
        cache[morph_index][raw_index] = merged;
      }
    }
    states[morph_index] = VisitState::Black;
    return cache[morph_index];
  };

  for (int morph_index = 0; morph_index < int(model.morphs.size()); morph_index++) {
    if (model.morphs[morph_index].type != MorphType::Vertex) {
      continue;
    }
    const std::map<int, double> &coefficients = expand_node(morph_index, 0);
    PMXVertexMorphExpression expression;
    expression.vertex_morph_index = morph_index;
    expression.vertex_morph_name = model.morphs[morph_index].name_local;
    for (const auto &[raw_index, coefficient] : coefficients) {
      if (!std::isfinite(coefficient) || std::abs(coefficient) > double(options.max_abs_coefficient)) {
        add_issue(report,
                  PMXGroupMorphIssue::Severity::Error,
                  morph_index,
                  raw_index,
                  "Group Morph expression coefficient is invalid");
        expansion_valid = false;
        continue;
      }
      if (std::abs(coefficient) <= double(options.coefficient_epsilon)) {
        continue;
      }
      expression.raw_morph_indices.push_back(raw_index);
      expression.coefficients.push_back(float(coefficient));
    }
    if (int(expression.raw_morph_indices.size()) > options.max_expression_terms) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                "Group Morph expression term count exceeds the configured limit");
      expansion_valid = false;
      continue;
    }
    report.max_terms = std::max(report.max_terms, int(expression.raw_morph_indices.size()));
    report.vertex_expressions.push_back(std::move(expression));
  }

  if (!expansion_valid) {
    report.vertex_expressions.clear();
    report.valid = false;
  }
  return report;
}

PMXGroupMorphReport finalize_controller_channels(
    const PMXModel &model,
    const PMXGroupMorphReport &validated_report,
    Span<int> vertex_morph_indices,
    Span<std::string> vertex_names)
{
  PMXGroupMorphReport report = validated_report;
  if (!validated_report.valid || validated_report.morph_count != int(model.morphs.size())) {
    add_issue(report,
              PMXGroupMorphIssue::Severity::Error,
              -1,
              -1,
              "Cannot finalize Controller channels from an invalid or mismatched report");
    report.valid = false;
    return report;
  }
  if (vertex_morph_indices.size() != vertex_names.size()) {
    add_issue(report,
              PMXGroupMorphIssue::Severity::Error,
              -1,
              -1,
              "Vertex morph index/name registry has mismatched lengths");
    report.valid = false;
    return report;
  }

  std::vector<std::string> used_names;
  used_names.reserve(vertex_names.size() + report.group_count);
  std::vector<bool> seen_vertex_indices(model.morphs.size(), false);
  for (int index = 0; index < int(vertex_morph_indices.size()); index++) {
    const int morph_index = vertex_morph_indices[index];
    const bool valid_index = morph_index >= 0 && morph_index < int(model.morphs.size()) &&
                             model.morphs[morph_index].type == MorphType::Vertex;
    const bool duplicate_index = valid_index && seen_vertex_indices[morph_index];
    if (!valid_index || duplicate_index) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Error,
                morph_index,
                -1,
                duplicate_index ?
                    "Vertex Controller channel registry contains a duplicate PMX Morph index" :
                    "Vertex Controller channel has an invalid PMX Morph index");
      report.valid = false;
      continue;
    }
    seen_vertex_indices[morph_index] = true;
    const std::string &name = vertex_names[index];
    std::string final_name = name.empty() ? "Morph" : name;
    if (name.empty()) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Warning,
                morph_index,
                -1,
                "Vertex Controller channel has an empty Blender name");
    }
    int suffix = 1;
    while (std::find(used_names.begin(), used_names.end(), final_name) != used_names.end()) {
      final_name = (name.empty() ? "Morph" : name) + "." + std::to_string(suffix++);
    }
    if (final_name != name && !name.empty()) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Warning,
                morph_index,
                -1,
                "Vertex Controller channel name collides with an existing channel");
    }
    used_names.push_back(final_name);
    for (PMXMorphChannel &channel : report.channels) {
      if (channel.morph_index == morph_index) {
        channel.blender_name = final_name;
        break;
      }
    }
  }

  for (PMXMorphChannel &channel : report.channels) {
    if (channel.type != MorphType::Group || !channel.controller_channel) {
      continue;
    }
    const std::string base_name = channel.source_name.empty() ? "Group" : channel.source_name;
    std::string blender_name = base_name;
    int suffix = 1;
    while (std::find(used_names.begin(), used_names.end(), blender_name) != used_names.end()) {
      blender_name = base_name + "." + std::to_string(suffix++);
    }
    if (blender_name != base_name || channel.source_name.empty()) {
      add_issue(report,
                PMXGroupMorphIssue::Severity::Warning,
                channel.morph_index,
                -1,
                channel.source_name.empty() ? "Group Controller channel has an empty PMX name" :
                                               "Group Controller channel name collides with an existing channel");
    }
    channel.blender_name = blender_name;
    used_names.push_back(blender_name);
  }

  return report;
}

}  // namespace blender::io::pmx
