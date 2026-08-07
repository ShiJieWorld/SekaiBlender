/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * FSR Diagnostic — Sparse Pixel Sampling
 *
 * Compute shader that samples pixels from an image2D (RGBA16F) on a
 * uniform grid and writes (grid_x, grid_y, coord_x, coord_y, r, g, b, a)
 * into an SSBO.
 *
 * Dispatch: grid_cols × grid_rows × 1 threads (1 thread per sample)
 *
 * This shader makes no judgments — it only reads pixel values.
 */

void main()
{
  /* TODO: implement samplers & dispatch */
  /* Each thread loads one pixel from the grid coordinate and writes to SSBO. */
  /* For now, this is a shader skeleton. */
}
