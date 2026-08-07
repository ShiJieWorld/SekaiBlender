/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * FSR Diagnostic — Luma Statistics Reduction
 *
 * Compute shader that scans an image2D (RGBA16F) and computes:
 *   min, max, mean, median, stddev of luma = dot(rgb, vec3(0.2126, 0.7152, 0.0722))
 *   count of NaN, Inf, zero-valued pixels
 *
 * Algorithm: two-stage parallel reduction
 *   Stage 1: 16×16 thread group, each thread covers 4×4 pixels
 *            local reduction via shared memory
 *            one output record per group → SSBO
 *   Stage 2: 16×16 threads reduce the group-level SSBO records
 *            to final global stats
 *
 * This shader makes no judgments — it only produces raw statistics.
 */

/* Work group size: 16×16 = 256 threads. Each thread covers 4×4 pixels. */
#define PIXELS_PER_THREAD_X 4
#define PIXELS_PER_THREAD_Y 4

/* Final output SSBO layout (written by Stage 2):
 *   float min_luma;
 *   float max_luma;
 *   float mean_luma;
 *   float stddev_luma;
 *   uint  nan_count;
 *   uint  inf_count;
 *   uint  zero_count;
 *   uint  total_pixels;
 */

/* Stage 1 per-group output: 6 floats per group */
struct GroupReduceResult {
  float min_luma;
  float max_luma;
  float sum_luma;
  float sum_sq_luma;
  uint  nan_count;
  uint  inf_count;
  uint  zero_count;
  uint  pixel_count;
};

/* Compute dispatch: (image_w / 64 + 1) × (image_h / 64 + 1) × 1 groups */

void main()
{
  /* — Stage 1: per-group reduction — */
  /* TODO: implement in shader */
  /* For now, this is a shader skeleton. Full implementation follows the
   * two-stage parallel reduction pattern used in Blender's compositor
   * and EEVEE histogram shaders. */
}
