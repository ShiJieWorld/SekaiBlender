/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 *
 * Real-sample PMX round-trip invariants.
 *
 * These are the assertion half of the PMX export scaffolding. They are gated on
 * environment variables and skip when unset, following the same convention as
 * the VMD real-sample tests (`VMD_C1D_SAMPLE`), because the repository does not
 * ship multi-megabyte MMD models.
 *
 *   PMX_SAMPLE
 *     Path to a real .pmx file. Enables the writer round-trip: the file is
 *     parsed, written, and parsed again, and the two models must be *bit*
 *     identical. Synthetic fixtures cannot cover what a production model
 *     contains (tens of thousands of vertices, hundreds of bones, additional UV
 *     sets, every Morph kind, hundreds of rigid bodies), so this is the check
 *     that the writer is correct on real data rather than only on a fixture.
 *
 *   PMX_EXPORTED
 *     Path to a .pmx produced by exporting `PMX_SAMPLE` through Blender.
 *     Enables the full-chain invariant. This is the one that catches the silent
 *     coordinate-space failures: axis mapping, scale, UV V-flip, triangle
 *     winding, SDEF frame, and the MMD/Blender physics space swap. Produce the
 *     file with a headless import+export run, then point this at it.
 */

#include "testing/testing.h"

#include "BLI_fileops.hh"

#include "intern/pmx_model_diff.h"
#include "intern/pmx_reader.h"
#include "intern/pmx_types.h"
#include "intern/pmx_writer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace blender::io::pmx::tests {
namespace {

/** Read an environment variable, returning an empty string when unset. */
std::string env_path(const char *name)
{
  const char *value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

/** A compact description of what a model contains, for test output. */
std::string describe(const PMXModel &model)
{
  return "version " + std::to_string(model.header.version) + ", " +
         std::to_string(model.vertices.size()) + " vertices, " +
         std::to_string(model.face_indices.size() / 3) + " triangles, " +
         std::to_string(model.textures.size()) + " textures, " +
         std::to_string(model.materials.size()) + " materials, " +
         std::to_string(model.bones.size()) + " bones, " +
         std::to_string(model.morphs.size()) + " morphs, " +
         std::to_string(model.display_frames.size()) + " display frames, " +
         std::to_string(model.rigid_bodies.size()) + " rigid bodies, " +
         std::to_string(model.joints.size()) + " joints, additional UV " +
         std::to_string(unsigned(model.header.add_uv_cnt));
}

/**
 * Group recorded issues by field, collapsing array subscripts.
 *
 * A wrong inversion produces one issue per affected element: on a real model
 * that is tens of thousands of lines saying the same thing. The individual
 * paths are noise. What identifies the bug is *which field* differs, how often,
 * and one worked example. This collapses `vertices[9].bone_indices` and
 * `vertices[10].bone_indices` into a single counted line.
 *
 * Reading this per field is also what distinguishes a real loss from a benign
 * one: `bone_indices` differing while `bone_weights` does not means the
 * differing slots carry zero weight and cannot affect deformation.
 */
std::string tally_by_field(const PMXModelDiffReport &report)
{
  struct Entry {
    int count = 0;
    std::string example;
  };
  std::map<std::string, Entry> by_field;

  for (const PMXModelDiffIssue &issue : report.issues) {
    std::string field;
    field.reserve(issue.path.size());
    bool in_subscript = false;
    for (const char c : issue.path) {
      if (c == '[') {
        in_subscript = true;
      }
      else if (c == ']') {
        in_subscript = false;
      }
      else if (!in_subscript) {
        field.push_back(c);
      }
    }
    Entry &entry = by_field[field];
    entry.count++;
    if (entry.example.empty()) {
      entry.example = issue.path + ": " + issue.message;
    }
  }

  std::ostringstream out;
  out << "  differences by field:\n";
  for (const auto &item : by_field) {
    out << "    " << item.first << " x" << item.second.count << "\n"
        << "      e.g. " << item.second.example << "\n";
  }
  return out.str();
}

class PMXRoundTripTest : public ::testing::Test {
 protected:
  /**
   * Resolve a sample path, skipping the test when the variable is unset.
   *
   * A path that is set but missing is a configuration error, not a code
   * failure, so it fails loudly rather than skipping silently.
   */
  static bool resolve_sample(const char *variable, std::string &r_path)
  {
    r_path = env_path(variable);
    if (r_path.empty()) {
      return false;
    }
    if (!BLI_exists(r_path.c_str())) {
      ADD_FAILURE() << variable << " is set but does not exist: " << r_path;
      return false;
    }
    return true;
  }
};

/**
 * Parsing, writing and re-parsing a real model must reproduce it exactly.
 *
 * Tolerances are all zero here on purpose. This path applies no scale and no
 * axis change, so every value must survive bit-exact; any difference is a
 * writer defect, not rounding.
 */
TEST_F(PMXRoundTripTest, writer_reproduces_real_model_exactly)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping real-model writer round-trip";
  }

  const PMXModel original = PMXReader::read(sample);
  testing::Test::RecordProperty("model", describe(original));

  const std::vector<uint8_t> written = PMXWriter::write_to_memory(original);
  ASSERT_FALSE(written.empty());

  const PMXModel reparsed = PMXReader::read_from_memory(
      written.data(), written.size(), sample + " (written)");

  PMXModelDiffOptions options;
  options.geometry_tolerance = 0.0f;
  options.unit_tolerance = 0.0f;
  options.metadata_tolerance = 0.0f;

  const PMXModelDiffReport report = diff_pmx_models(original, reparsed, options);
  EXPECT_TRUE(report.equal()) << "sample: " << sample << "\n"
                              << describe(original) << "\n"
                              << report.to_string();
}

/** Writing twice through a parse must be byte-identical on a real model. */
TEST_F(PMXRoundTripTest, writer_is_idempotent_on_real_model)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping real-model idempotence check";
  }

  const PMXModel original = PMXReader::read(sample);
  const std::vector<uint8_t> first = PMXWriter::write_to_memory(original);
  const PMXModel reparsed = PMXReader::read_from_memory(first.data(), first.size(), sample);
  const std::vector<uint8_t> second = PMXWriter::write_to_memory(reparsed);

  ASSERT_EQ(first.size(), second.size()) << "sample: " << sample;
  EXPECT_EQ(first, second) << "sample: " << sample;
}

/**
 * The full-chain invariant: exporting an imported model must reproduce it.
 *
 * Every value here has crossed into Blender space and back, so geometry carries
 * a scale round-trip tolerance while everything restored from retention is
 * required to be exact.
 */
TEST_F(PMXRoundTripTest, export_reproduces_imported_model)
{
  std::string sample;
  std::string exported;
  if (!resolve_sample("PMX_SAMPLE", sample) || !resolve_sample("PMX_EXPORTED", exported)) {
    GTEST_SKIP() << "PMX_SAMPLE and PMX_EXPORTED must both be set; skipping full-chain check";
  }

  const PMXModel original = PMXReader::read(sample);
  const PMXModel round_tripped = PMXReader::read(exported);

  /* Record everything rather than the default per-section cap. A real model
   * produces differences by the ten thousand, and the default "showing 8" is
   * not enough to tell which *field* is wrong -- which is the only thing that
   * localizes a broken inversion. The tally below is printed instead of the
   * raw issue list, so the output stays readable. */
  PMXModelDiffOptions options;
  options.max_issues_per_section = 1000000;

  const PMXModelDiffReport report = diff_pmx_models(original, round_tripped, options);
  EXPECT_TRUE(report.equal()) << "sample:   " << sample << "\n"
                              << "  " << describe(original) << "\n"
                              << "exported: " << exported << "\n"
                              << "  " << describe(round_tripped) << "\n"
                              << tally_by_field(report);
}

/**
 * Characterize the sample's bone weight sums, per weight type.
 *
 * This exists to separate two very different explanations for an export weight
 * mismatch. PMX does not require a vertex's weights to sum to 1, and the
 * importer normalizes any vertex whose sum falls below 0.999
 * (`pmx_import_weights.cc`). That normalization is not invertible from Blender
 * data, so if the file itself carries unnormalized weights, an exact weight
 * round-trip is impossible without retaining something extra.
 *
 * The alternative -- that import silently drops nonzero influences -- would be a
 * far more serious bug. Measuring the file tells them apart instead of inferring
 * it from the reader's code.
 *
 * Reported, never asserted: this describes the input, not a requirement.
 */
TEST_F(PMXRoundTripTest, reports_unnormalized_weight_distribution)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping weight distribution report";
  }

  const PMXModel model = PMXReader::read(sample);

  struct Bucket {
    int vertices = 0;
    int unnormalized = 0;
    float min_sum = 2.0f;
  };
  /* Indexed by BoneWeightType: BDEF1, BDEF2, BDEF4, SDEF, QDEF. */
  Bucket buckets[5];
  const char *names[5] = {"BDEF1", "BDEF2", "BDEF4", "SDEF", "QDEF"};

  for (const PMXVertex &vertex : model.vertices) {
    const int type = int(vertex.weight_type);
    if (type < 0 || type > 4) {
      continue;
    }
    Bucket &bucket = buckets[type];
    bucket.vertices++;
    float sum = 0.0f;
    for (const float weight : vertex.bone_weights) {
      sum += weight;
    }
    bucket.min_sum = sum < bucket.min_sum ? sum : bucket.min_sum;
    /* Same threshold the importer uses to decide whether to normalize. */
    if (sum < 0.999f) {
      bucket.unnormalized++;
    }
  }

  std::ostringstream out;
  out << "  weight sums in " << sample << "\n";
  for (const int type : {0, 1, 2, 3, 4}) {
    const Bucket &bucket = buckets[type];
    if (bucket.vertices == 0) {
      continue;
    }
    out << "    " << names[type] << ": " << bucket.vertices << " vertices, "
        << bucket.unnormalized << " with sum < 0.999";
    if (bucket.vertices > 0) {
      out << ", min sum " << bucket.min_sum;
    }
    out << "\n";
  }
  std::cout << out.str();
}

/**
 * Count Vertex Morph offsets that cannot survive the Shape Key round-trip.
 *
 * Import stores a Vertex Morph as `keyblock_coord = basis_coord + delta`, both in
 * float32 Blender space. Export recovers `delta = keyblock_coord - basis_coord`.
 * Two distinct things make a source offset unrecoverable:
 *
 *   - it is exactly zero. Legal PMX, but identical to the vertex not being in
 *     the morph, so the zero delta export reads back is indistinguishable from an
 *     untouched vertex.
 *   - it is non-zero but small enough relative to the vertex's coordinate
 *     magnitude that `float(basis + delta) == basis`. The addition rounds it away
 *     at import time and no export arithmetic can bring it back.
 *
 * This is a measurement, not an assertion: it is what justifies `pmx_model_diff`
 * comparing Vertex Morph offsets as a map keyed by vertex index with zero-valued
 * entries ignored, rather than element-wise, and it bounds how far the offset
 * count may legitimately shrink.
 */
TEST_F(PMXRoundTripTest, reports_vertex_morph_offsets_lost_to_float32)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping offset-loss report";
  }

  const PMXModel model = PMXReader::read(sample);

  /* The import scale the driver uses. Absorption depends on it: a larger scale
   * moves both basis and delta up together, so the ratio that decides whether
   * the addition rounds away is unchanged, but the absolute threshold is not. */
  const float scale = 0.08f;

  /* Import maps PMX (x,y,z) to Blender (x,z,y). Both basis and delta go through
   * the same permutation, so pair each Blender axis with its PMX source. */
  const int pmx_axis_for_blender[3] = {0, 2, 1};

  int vertex_morphs = 0;
  int total_offsets = 0;
  int zero_offsets = 0;
  int absorbed_offsets = 0;
  int morphs_with_loss = 0;
  float largest_absorbed = 0.0f;

  for (const PMXMorph &morph : model.morphs) {
    if (morph.type != MorphType::Vertex) {
      continue;
    }
    vertex_morphs++;
    int lost_here = 0;

    for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
      total_offsets++;

      if (offset.offset[0] == 0.0f && offset.offset[1] == 0.0f && offset.offset[2] == 0.0f) {
        zero_offsets++;
        lost_here++;
        continue;
      }
      if (offset.vertex_index < 0 || size_t(offset.vertex_index) >= model.vertices.size()) {
        continue;
      }
      const float *pos = model.vertices[offset.vertex_index].pos;

      bool all_absorbed = true;
      float magnitude = 0.0f;
      for (const int axis : {0, 1, 2}) {
        const int pmx_axis = pmx_axis_for_blender[axis];
        const float basis = pos[pmx_axis] * scale;
        const float delta = offset.offset[pmx_axis] * scale;
        magnitude += delta * delta;
        if (float(basis + delta) != basis) {
          all_absorbed = false;
        }
      }
      if (all_absorbed) {
        absorbed_offsets++;
        lost_here++;
        const float length = std::sqrt(magnitude);
        largest_absorbed = length > largest_absorbed ? length : largest_absorbed;
      }
    }
    if (lost_here > 0) {
      morphs_with_loss++;
    }
  }

  std::cout << "  Vertex Morph offset loss in " << sample << "\n"
            << "    " << vertex_morphs << " Vertex Morphs, " << total_offsets << " offsets total\n"
            << "    " << zero_offsets << " exactly zero\n"
            << "    " << absorbed_offsets << " absorbed by float32 (largest "
            << largest_absorbed << " Blender units)\n"
            << "    " << morphs_with_loss << " Morph(s) lose at least one offset\n";
}

/**
 * Count the ways a Vertex Morph offset cannot survive Blender's Shape Keys.
 *
 * Distinct from `reports_vertex_morph_offsets_lost_to_float32`, which measures
 * arithmetic loss. These three are structural: the offset either never reaches a
 * Shape Key, or reaches one in a form that cannot be told apart from a different
 * source arrangement.
 *
 *   1. Rejected at import. `pmx_import_morph.cc` drops any offset that is
 *      non-finite or fails `reasonable_offset` (limit
 *      `max(1000, 10000 * scale)`, checked on the *pre-transform* PMX value).
 *      A rejected offset never lands in the Shape Key, so export cannot recover
 *      it and the exported count is legitimately lower.
 *   2. The same `vertex_index` listed more than once in one Morph. Legal PMX,
 *      and import accumulates them (`data[index] += ...`). A Shape Key holds one
 *      value per vertex, so export emits a single summed offset. Deformation is
 *      identical -- applying two offsets to a vertex equals applying their sum --
 *      but the count shrinks by the number of extra entries.
 *   3. Source offsets not ascending by `vertex_index`. Export walks vertices in
 *      source-index order, so it always emits ascending. If the source file used
 *      any other order, an element-wise comparison of the two offset arrays
 *      reports differences that are only a permutation.
 *
 * (1) and (2) bound how far the offset count may legitimately shrink. (3)
 * decides whether the offsets can be compared element-wise at all, or have to be
 * compared as a `vertex_index -> summed offset` map, the same way skinning is.
 *
 * Reported, never asserted: this describes the input file.
 */
TEST_F(PMXRoundTripTest, reports_vertex_morph_offsets_blender_cannot_represent)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping Vertex Morph representation report";
  }

  const PMXModel model = PMXReader::read(sample);

  /* Same scale the driver imports with, and the same limit
   * `pmx_import_morph.cc` computes from it. */
  const float scale = 0.08f;
  const float limit = std::max(1000.0f, 10000.0f * scale);

  int vertex_morphs = 0;
  int total_offsets = 0;
  int rejected_offsets = 0;
  int non_finite_offsets = 0;
  float largest_rejected = 0.0f;
  int morphs_with_duplicates = 0;
  int duplicate_entries = 0;
  int unordered_morphs = 0;
  int out_of_range = 0;

  for (const PMXMorph &morph : model.morphs) {
    if (morph.type != MorphType::Vertex) {
      continue;
    }
    vertex_morphs++;

    std::map<int, int> seen;
    int duplicates_here = 0;
    bool ascending = true;
    int previous_index = -1;

    for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
      total_offsets++;

      if (offset.vertex_index < 0 || size_t(offset.vertex_index) >= model.vertices.size()) {
        out_of_range++;
        continue;
      }
      if (offset.vertex_index <= previous_index) {
        ascending = false;
      }
      previous_index = offset.vertex_index;

      if (++seen[offset.vertex_index] > 1) {
        duplicates_here++;
      }

      const bool finite = std::isfinite(offset.offset[0]) && std::isfinite(offset.offset[1]) &&
                          std::isfinite(offset.offset[2]);
      if (!finite) {
        non_finite_offsets++;
        rejected_offsets++;
        continue;
      }
      const float largest = std::max(std::max(std::fabs(offset.offset[0]),
                                              std::fabs(offset.offset[1])),
                                     std::fabs(offset.offset[2]));
      if (largest > limit) {
        rejected_offsets++;
        largest_rejected = largest > largest_rejected ? largest : largest_rejected;
      }
    }

    if (duplicates_here > 0) {
      morphs_with_duplicates++;
      duplicate_entries += duplicates_here;
    }
    if (!ascending) {
      unordered_morphs++;
    }
  }

  std::cout << "  Vertex Morph offsets Blender cannot represent, in " << sample << "\n"
            << "    " << vertex_morphs << " Vertex Morphs, " << total_offsets
            << " offsets total\n"
            << "    rejected at import (limit " << limit << " PMX units): " << rejected_offsets
            << " (largest rejected component " << largest_rejected << ", " << non_finite_offsets
            << " non-finite)\n"
            << "    duplicate vertex_index within one Morph: " << morphs_with_duplicates
            << " Morph(s), " << duplicate_entries << " extra entries\n"
            << "    Morphs whose offsets are not ascending by vertex_index: " << unordered_morphs
            << " of " << vertex_morphs << "\n"
            << "    offsets with an out-of-range vertex_index: " << out_of_range << "\n";
}

/**
 * Count the two ways a PMX skinning slot cannot survive Blender's vertex groups.
 *
 * A `MDeformVert` is a set of (group, weight) pairs. That representation cannot
 * hold either of these, both of which are legal PMX:
 *
 *   1. Zero-weight padding. BDEF4/QDEF always store four slots; unused ones
 *      carry weight 0 and an arbitrary bone index. Import drops zero weights
 *      (`pmx_import_weights.cc`), so the index is gone and export pads with 0.
 *   2. The same bone in more than one slot. Blender has one weight per group, so
 *      import's `dw->weight += weight` sums them, and export sees a single
 *      influence carrying the total.
 *
 * Both are semantics-preserving: a zero-weight slot cannot deform, and a bone
 * applied twice deforms identically to one applied once with the summed weight.
 * Neither is recoverable, so `pmx_model_diff` has to compare skinning as a
 * bone -> total-weight map instead of element-wise. This measures how much of
 * the real model depends on that rule.
 */
TEST_F(PMXRoundTripTest, reports_skinning_slots_blender_cannot_represent)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping skinning representation report";
  }

  const PMXModel model = PMXReader::read(sample);

  int vertices_with_padding = 0;
  int padding_slots = 0;
  int vertices_with_duplicate_bone = 0;
  int collapsed_slots = 0;
  float largest_collapsed_sum = 0.0f;

  for (const PMXVertex &vertex : model.vertices) {
    const size_t count = vertex.bone_indices.size() < vertex.bone_weights.size() ?
                             vertex.bone_indices.size() :
                             vertex.bone_weights.size();

    /* (1) Zero-weight slots whose index export cannot reproduce. Export writes 0
     * for a padded slot, so only a non-zero index is actually a difference. */
    int padding_here = 0;
    for (size_t i = 0; i < count; i++) {
      if (vertex.bone_weights[i] == 0.0f && vertex.bone_indices[i] != 0) {
        padding_here++;
      }
    }
    if (padding_here > 0) {
      vertices_with_padding++;
      padding_slots += padding_here;
    }

    /* (2) The same bone in two or more slots, counting only slots that carry
     * weight -- a duplicate among zero-weight slots is already case (1). */
    int duplicates_here = 0;
    float duplicated_sum = 0.0f;
    for (size_t i = 0; i < count; i++) {
      if (vertex.bone_weights[i] <= 0.0f) {
        continue;
      }
      for (size_t j = 0; j < i; j++) {
        if (vertex.bone_weights[j] > 0.0f && vertex.bone_indices[j] == vertex.bone_indices[i]) {
          duplicates_here++;
          duplicated_sum = vertex.bone_weights[i] + vertex.bone_weights[j];
          break;
        }
      }
    }
    if (duplicates_here > 0) {
      vertices_with_duplicate_bone++;
      collapsed_slots += duplicates_here;
      largest_collapsed_sum = duplicated_sum > largest_collapsed_sum ? duplicated_sum :
                                                                      largest_collapsed_sum;
    }
  }

  std::cout << "  skinning slots Blender cannot represent, in " << sample << "\n"
            << "    " << model.vertices.size() << " vertices total\n"
            << "    zero-weight padding with non-zero index: " << vertices_with_padding
            << " vertices, " << padding_slots << " slots\n"
            << "    same bone in >1 weighted slot: " << vertices_with_duplicate_bone
            << " vertices, " << collapsed_slots << " slots collapsed"
            << " (largest summed weight " << largest_collapsed_sum << ")\n";
}

/**
 * Distribution of per-vertex normal error across a real export round-trip.
 *
 * `normal_tolerance` is set from a synthetic measurement
 * (`PMXNormalFidelityTest`). This measures the real thing, which is the only way
 * to know whether that bound is complete.
 *
 * The shape of the distribution is what matters. A continuous tail reaching the
 * tolerance means the bound is simply the encoding's cost and the tolerance has
 * to cover it. A cluster of isolated outliers far above an otherwise tight body
 * means something specific to those vertices is wrong, and raising the tolerance
 * would bury it.
 *
 * Needs both PMX_SAMPLE and PMX_EXPORTED, so it runs after the driver script.
 */
TEST_F(PMXRoundTripTest, reports_normal_error_distribution)
{
  std::string sample;
  std::string exported;
  if (!resolve_sample("PMX_SAMPLE", sample) || !resolve_sample("PMX_EXPORTED", exported)) {
    GTEST_SKIP() << "PMX_SAMPLE and PMX_EXPORTED must both be set; skipping normal distribution";
  }

  const PMXModel original = PMXReader::read(sample);
  const PMXModel round_tripped = PMXReader::read(exported);
  const size_t count = original.vertices.size() < round_tripped.vertices.size() ?
                           original.vertices.size() :
                           round_tripped.vertices.size();

  const float edges[6] = {1.0e-5f, 1.0e-4f, 1.0e-3f, 1.0e-2f, 1.0e-1f, 2.0f};
  const char *labels[6] = {"< 1e-5", "< 1e-4", "< 1e-3", "< 1e-2", "< 1e-1", ">= 1e-1"};
  int buckets[6] = {0, 0, 0, 0, 0, 0};

  struct Outlier {
    int vertex = -1;
    float error = 0.0f;
    float expected[3] = {0.0f, 0.0f, 0.0f};
    float actual[3] = {0.0f, 0.0f, 0.0f};
  };
  std::vector<Outlier> worst;

  for (size_t i = 0; i < count; i++) {
    const PMXVertex &a = original.vertices[i];
    const PMXVertex &b = round_tripped.vertices[i];
    float error = 0.0f;
    for (const int axis : {0, 1, 2}) {
      const float d = std::fabs(a.normal[axis] - b.normal[axis]);
      error = d > error ? d : error;
    }
    for (const int bucket : {0, 1, 2, 3, 4, 5}) {
      if (error < edges[bucket]) {
        buckets[bucket]++;
        break;
      }
    }
    if (error >= 1.0e-3f) {
      Outlier outlier;
      outlier.vertex = int(i);
      outlier.error = error;
      for (const int axis : {0, 1, 2}) {
        outlier.expected[axis] = a.normal[axis];
        outlier.actual[axis] = b.normal[axis];
      }
      worst.push_back(outlier);
    }
  }

  std::sort(worst.begin(), worst.end(), [](const Outlier &x, const Outlier &y) {
    return x.error > y.error;
  });

  std::ostringstream out;
  out << "  normal error over " << count << " vertices\n";
  for (const int bucket : {0, 1, 2, 3, 4, 5}) {
    out << "    " << labels[bucket] << ": " << buckets[bucket] << "\n";
  }
  out << "    " << worst.size() << " vertices at or above 1e-3\n";
  const size_t shown = worst.size() < 5 ? worst.size() : 5;
  for (size_t i = 0; i < shown; i++) {
    const Outlier &outlier = worst[i];
    out << "      vertex " << outlier.vertex << " err " << outlier.error << "  expected ("
        << outlier.expected[0] << ", " << outlier.expected[1] << ", " << outlier.expected[2]
        << ")  got (" << outlier.actual[0] << ", " << outlier.actual[1] << ", "
        << outlier.actual[2] << ")\n";
  }
  std::cout << out.str();
}

/**
 * Measure physics-field error over a real import/export round-trip.
 *
 * The persisted definition is Blender-space float32 data. Positions, sizes and
 * linear limits cross the import scale and therefore use the geometry budget;
 * rotations and the Y/Z-only spring vectors do not scale and should stay within
 * the unit budget. Rotation is intentionally reported separately: if the
 * coordinate convention were wrong, the error would be O(1), not a rounding
 * tail, and increasing a tolerance would hide the bug.
 *
 * Reported, never asserted: this is the empirical basis for the comparator
 * budgets, not a second implementation of the physics inversion.
 */
TEST_F(PMXRoundTripTest, reports_physics_error_distribution)
{
  std::string sample;
  std::string exported;
  if (!resolve_sample("PMX_SAMPLE", sample) || !resolve_sample("PMX_EXPORTED", exported)) {
    GTEST_SKIP() << "PMX_SAMPLE and PMX_EXPORTED must both be set; skipping physics distribution";
  }

  const PMXModel original = PMXReader::read(sample);
  const PMXModel round_tripped = PMXReader::read(exported);
  const size_t rigid_count = std::min(original.rigid_bodies.size(),
                                      round_tripped.rigid_bodies.size());
  const size_t joint_count = std::min(original.joints.size(), round_tripped.joints.size());

  auto max_vec_error = [](const float *a, const float *b, const int count) {
    float result = 0.0f;
    for (int i = 0; i < count; i++) {
      result = std::max(result, std::fabs(a[i] - b[i]));
    }
    return result;
  };

  float rigid_position_max = 0.0f;
  float rigid_size_max = 0.0f;
  float rigid_rotation_max = 0.0f;
  float joint_position_max = 0.0f;
  float joint_rotation_max = 0.0f;
  float joint_translation_limit_max = 0.0f;
  float joint_rotation_limit_max = 0.0f;
  float joint_spring_translation_max = 0.0f;
  float joint_spring_rotation_max = 0.0f;

  const float rotation_edges[6] = {1.0e-7f, 1.0e-6f, 1.0e-5f, 1.0e-4f, 1.0e-3f, 2.0f};
  const char *rotation_labels[6] = {
      "< 1e-7", "< 1e-6", "< 1e-5", "< 1e-4", "< 1e-3", ">= 1e-3"};
  int rotation_buckets[6] = {0, 0, 0, 0, 0, 0};
  auto record_rotation = [&](const float *expected, const float *actual) {
    const float error = max_vec_error(expected, actual, 3);
    for (int bucket = 0; bucket < 6; bucket++) {
      if (error < rotation_edges[bucket] || bucket == 5) {
        rotation_buckets[bucket]++;
        break;
      }
    }
    return error;
  };

  for (size_t i = 0; i < rigid_count; i++) {
    const PMXRigidBody &a = original.rigid_bodies[i];
    const PMXRigidBody &b = round_tripped.rigid_bodies[i];
    rigid_position_max = std::max(rigid_position_max, max_vec_error(a.pos, b.pos, 3));
    rigid_size_max = std::max(rigid_size_max, max_vec_error(a.shape_size, b.shape_size, 3));
    rigid_rotation_max = std::max(rigid_rotation_max, record_rotation(a.rot, b.rot));
  }
  for (size_t i = 0; i < joint_count; i++) {
    const PMXJoint &a = original.joints[i];
    const PMXJoint &b = round_tripped.joints[i];
    joint_position_max = std::max(joint_position_max, max_vec_error(a.pos, b.pos, 3));
    joint_rotation_max = std::max(joint_rotation_max, record_rotation(a.rot, b.rot));
    joint_translation_limit_max = std::max(
        joint_translation_limit_max,
        std::max(max_vec_error(a.translation_limit_min, b.translation_limit_min, 3),
                 max_vec_error(a.translation_limit_max, b.translation_limit_max, 3)));
    joint_rotation_limit_max = std::max(
        joint_rotation_limit_max,
        std::max(max_vec_error(a.rotation_limit_min, b.rotation_limit_min, 3),
                 max_vec_error(a.rotation_limit_max, b.rotation_limit_max, 3)));
    joint_spring_translation_max = std::max(
        joint_spring_translation_max, max_vec_error(a.spring_translation, b.spring_translation, 3));
    joint_spring_rotation_max = std::max(
        joint_spring_rotation_max, max_vec_error(a.spring_rotation, b.spring_rotation, 3));
  }

  std::cout << "  physics error over " << rigid_count << " rigid bodies and " << joint_count
            << " joints\n"
            << "    rigid position max: " << rigid_position_max
            << ", shape size max: " << rigid_size_max
            << ", rotation max: " << rigid_rotation_max << "\n"
            << "    joint position max: " << joint_position_max
            << ", rotation max: " << joint_rotation_max
            << ", translation limits max: " << joint_translation_limit_max
            << ", rotation limits max: " << joint_rotation_limit_max
            << ", spring translation max: " << joint_spring_translation_max
            << ", spring rotation max: " << joint_spring_rotation_max << "\n"
            << "    rotation buckets:\n";
  for (int bucket = 0; bucket < 6; bucket++) {
    std::cout << "      " << rotation_labels[bucket] << ": " << rotation_buckets[bucket]
              << "\n";
  }
}

/** Report whether the real sample actually exercises PMX 2.1 Impulse Morphs. */
TEST_F(PMXRoundTripTest, reports_impulse_morph_usage)
{
  std::string sample;
  if (!resolve_sample("PMX_SAMPLE", sample)) {
    GTEST_SKIP() << "PMX_SAMPLE is not set; skipping Impulse Morph usage report";
  }

  const PMXModel model = PMXReader::read(sample);
  int impulse_morphs = 0;
  int impulse_offsets = 0;
  int local_offsets = 0;
  for (const PMXMorph &morph : model.morphs) {
    if (morph.type != MorphType::Impulse) {
      continue;
    }
    impulse_morphs++;
    impulse_offsets += int(morph.impulse_offsets.size());
    for (const PMXImpulseMorphOffset &offset : morph.impulse_offsets) {
      local_offsets += offset.local_flag != 0 ? 1 : 0;
    }
  }

  std::cout << "  Impulse Morph use in " << sample << "\n"
            << "    PMX version " << model.header.version << ", " << impulse_morphs
            << " Impulse Morph(s), " << impulse_offsets << " offsets (" << local_offsets
            << " local)\n";
}

}  // namespace
}  // namespace blender::io::pmx::tests
