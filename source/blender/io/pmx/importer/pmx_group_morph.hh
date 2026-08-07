/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "intern/pmx_types.h"

#include "BLI_span.hh"

#include <cstdint>
#include <string>
#include <vector>

namespace blender::io::pmx {

struct PMXGroupMorphOptions {
  int max_morph_count = 1'000'000;
  int max_depth = 256;
  int max_graph_edges = 8'000'000;
  int max_expression_terms = 4096;
  float max_abs_influence = 1'000'000.0f;
  float max_abs_coefficient = 1'000'000'000.0f;
  float coefficient_epsilon = 1.0e-7f;
};

struct PMXGroupMorphIssue {
  enum class Severity : uint8_t {
    Warning,
    Error,
  };

  Severity severity = Severity::Warning;
  int morph_index = -1;
  int target_morph_index = -1;
  std::string message;
};

struct PMXMorphChannel {
  int morph_index = -1;
  std::string source_name;
  std::string source_name_universal;
  std::string blender_name;
  MorphType type = MorphType::Vertex;
  bool controller_channel = false;
  bool vertex_output = false;
};

struct PMXGroupMorphEdge {
  int group_morph_index = -1;
  int target_morph_index = -1;
  float influence = 0.0f;
};

struct PMXVertexMorphExpression {
  int vertex_morph_index = -1;
  std::string vertex_morph_name;
  std::vector<int> raw_morph_indices;
  std::vector<float> coefficients;
};

struct PMXGroupMorphReport {
  bool valid = false;
  int morph_count = 0;
  int group_count = 0;
  int vertex_count = 0;
  int supported_channel_count = 0;
  int unsupported_edge_count = 0;
  /* R1-PMX: unsupported Group Morph edges aggregated by target morph type.
   * Only morph types other than Group/Vertex are unsupported (Bone, UV,
   * Material, Flip, Impulse); the UV_* variants collapse into unsupported_uv_count.
   * These counts summarize the per-edge warnings emitted during analysis. */
  int unsupported_bone_count = 0;
  int unsupported_uv_count = 0;
  int unsupported_material_count = 0;
  int unsupported_flip_count = 0;
  int unsupported_impulse_count = 0;
  int warning_count = 0;
  int error_count = 0;
  int max_depth = 0;
  int max_terms = 0;
  std::vector<PMXMorphChannel> channels;
  std::vector<PMXGroupMorphEdge> edges;
  std::vector<PMXVertexMorphExpression> vertex_expressions;
  std::vector<PMXGroupMorphIssue> issues;
};

/**
 * Build a deterministic raw-channel registry for all PMX morphs.
 *
 * The returned names are only a temporary mapping for the pure in-memory
 * analysis. Blender KeyBlock naming and collision handling are performed by
 * the importer layer in a later C2-2 stage.
 */
std::vector<PMXMorphChannel> build_default_morph_channels(const PMXModel &model);

/**
 * Validate the PMX Group Morph graph without touching Blender data.
 *
 * C2-2A builds and validates Group/Vertex edges. C2-2B expands a valid
 * report into deterministic pure-memory vertex expressions.
 */
PMXGroupMorphReport analyze_group_morphs(
    const PMXModel &model,
    const std::vector<PMXMorphChannel> &channels = {},
    const PMXGroupMorphOptions &options = {});

/**
 * Expand a validated Group Morph graph into deterministic Vertex expressions.
 *
 * The returned report is a copy of the validated report. The input model and
 * report are not modified. Any expansion error invalidates the returned
 * report and clears all expressions so callers cannot consume partial output.
 */
PMXGroupMorphReport expand_group_morph_expressions(
    const PMXModel &model,
    const PMXGroupMorphReport &validated_report,
    const PMXGroupMorphOptions &options = {});

/**
 * Assign deterministic Blender KeyBlock names to the supported raw channels.
 * Existing Vertex names are preserved from the importer; Group names are
 * allocated after them in PMX index order. The input report is not modified.
 */
PMXGroupMorphReport finalize_controller_channels(
    const PMXModel &model,
    const PMXGroupMorphReport &validated_report,
    Span<int> vertex_morph_indices,
    Span<std::string> vertex_names);

}  // namespace blender::io::pmx
