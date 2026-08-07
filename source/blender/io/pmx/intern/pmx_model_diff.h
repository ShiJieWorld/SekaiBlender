#ifndef PMX_MODEL_DIFF_H
#define PMX_MODEL_DIFF_H

#include "pmx_types.h"

#include <string>
#include <vector>

/*
 * Section-by-section comparison of two parsed PMXModel values.
 *
 * This exists for the PMX export round-trip invariant: parse a file, import it,
 * export it, parse the result, and require the two models to agree. Every
 * coordinate-space inversion the exporter has to perform (axis mapping, scale,
 * UV V-flip, triangle winding, SDEF frame) fails *silently* when it is wrong —
 * the file still opens and the model still renders. Comparing the parsed models
 * field by field is what turns those into named, located failures.
 *
 * Both sides are expected to come from PMXReader. That matters: the reader
 * normalizes the format's discriminated fields (an unset `toon_internal_value`
 * is always 0, an unset `tail_pos_offset` is always zero, the offset vector for
 * a non-matching Morph type is always empty). Because both sides are normalized
 * the same way, every field can be compared unconditionally without producing
 * false differences from the writer's conditional emission.
 */

struct PMXModelDiffOptions {
  /**
   * Absolute tolerance for values that survive a scale round-trip.
   *
   * Positions, rigid-body sizes, and joint linear limits are multiplied by the
   * import scale on the way in and divided by it on the way out, in float32
   * both times, so they cannot be expected to be bit-exact. Expressed in PMX
   * units.
   */
  float geometry_tolerance = 1.0e-3f;

  /**
   * Absolute tolerance for values that are not scaled: UV, weights, axis
   * vectors, physics Euler triplets, angular limits, and spring vectors. These
   * only lose precision to storage and sign/axis permutation, not scale
   * arithmetic. A 403-rigid / 602-joint real round-trip measured zero error for
   * every rotation, angular limit, and spring vector.
   */
  float unit_tolerance = 1.0e-5f;

  /**
   * Absolute tolerance for per-vertex normals.
   *
   * Normals need their own, much looser budget. Import stores them through
   * `bke::mesh_set_custom_normals`, which encodes each normal as a `short2` in
   * the corner's tangent frame rather than keeping the `float3`. Reading them
   * back therefore costs real precision, unrelated to any export error.
   *
   * The default comes from measuring that encode/decode round-trip, both
   * synthetically (`PMXNormalFidelityTest`) and over a full real round-trip
   * (`PMXRoundTripTest.reports_normal_error_distribution`):
   *
   *   - shared-vertex fan, synthetic:       1.15e-4
   *   - adversarial synthetic (normals swept across the whole sphere on
   *     coplanar triangles, i.e. far from the geometric normal, which is where
   *     tangent-frame encoding degrades):   4.83e-3
   *   - real MMD model, 58242 vertices:     1.33e-2 max
   *
   * The real model exceeds the synthetic bound, so the synthetic figure is not
   * a ceiling and must not be used as one. The real distribution decays
   * continuously -- 12336 vertices under 1e-5, 43891 under 1e-4, 1924 under
   * 1e-3, 88 under 1e-2, 3 under 1e-1, and none at or above 1e-1 -- which is
   * what quantization loss looks like. Outliers also come in identical pairs,
   * matching seam vertices duplicated by split-by-material: the loss is
   * deterministic in the source normal, not random corruption.
   *
   * Set to ~4x the largest value observed on a real model. Looseness per vertex
   * is acceptable because magnitude is not what identifies a broken inversion:
   * an axis permutation or sign flip moves a component by O(1) on essentially
   * every vertex at once, so it is caught by the count, not by the threshold.
   */
  float normal_tolerance = 5.0e-2f;

  /**
   * Absolute tolerance for metadata restored from retention.
   *
   * Defaults to exact. Retention stores PMX float32 in IDProperty float32, so
   * any difference here is a lost or corrupted field rather than rounding.
   */
  float metadata_tolerance = 0.0f;

  /** Stop recording (but keep counting) issues once a section reaches this many. */
  int max_issues_per_section = 8;
};

struct PMXModelDiffIssue {
  /** Locates the field, e.g. `materials[3].specular_power`. */
  std::string path;
  std::string message;
};

/** Per-section tally, so a fully broken section reads as one line, not hundreds. */
struct PMXModelDiffSection {
  std::string name;
  /** Total differences found, including ones not recorded in `issues`. */
  int issue_count = 0;
  /** How many of those were recorded. */
  int reported_count = 0;
};

struct PMXModelDiffReport {
  std::vector<PMXModelDiffIssue> issues;
  std::vector<PMXModelDiffSection> sections;
  int total_issues = 0;

  bool equal() const { return total_issues == 0; }

  /** Multi-line human-readable summary; empty when the models agree. */
  std::string to_string() const;
};

/**
 * Compare every section of `expected` against `actual`.
 *
 * When a section's item counts differ, the mismatch is reported and the common
 * prefix is still compared, so a count error does not mask field errors.
 */
PMXModelDiffReport diff_pmx_models(const PMXModel &expected,
                                   const PMXModel &actual,
                                   const PMXModelDiffOptions &options = {});

#endif  // PMX_MODEL_DIFF_H
