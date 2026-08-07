/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 *
 * FSR Diagnostic — Objective GPU Data Collection
 *
 * This module captures raw data from FSR's GPU pipeline without making
 * any judgments about correctness or performance. All "good/bad" analysis
 * is deferred to external Python tools.
 *
 * Collected data:
 *   1. Shader & pipeline status (bool flags)
 *   2. EASU/RCAS constants (float4 arrays)
 *   3. Texture metadata (dimensions, format)
 *   4. CPU-side timing (microseconds)
 *   5. Luma statistics (min/max/mean/stddev, NaN/Inf/zero counts)
 *   6. Sparse pixel samples (grid readback)
 *
 * Output: JSONL file, one record per captured frame.
 */

#pragma once

namespace blender {
namespace gpu {
class Texture;
}  // namespace gpu

namespace draw {

/* -------------------------------------------------------------------- */
/** \name Public API
 * \{ */

/**
 * Initialize the diagnostic system.
 * Must be called once before any capture.
 * Creates internal textures, SSBOs, and registers GLSL compute shaders.
 */
void FSR_diagnostics_init();

/**
 * Shutdown and release all diagnostic resources.
 */
void FSR_diagnostics_free();

/**
 * Capture one frame of diagnostic data from the FSR output texture.
 *
 * \param output_texture: The final FSR output texture (after RCAS pass).
 * \param intermediate_texture: Intermediate texture (after EASU, before RCAS).
 *        Maybe null if EASU and RCAS are fused.
 * \param output_mode: "viewport" or "render".
 * \param input_width: Width of the input (pre-FSR) texture, for constant recording.
 * \param input_height: Height of the input (pre-FSR) texture.
 * \param frame_number: Frame counter (viewport frame or render batch index).
 *
 * This function may skip capture if diagnostics are disabled or if
 * capture_interval has not elapsed.
 */
void FSR_diagnostics_capture(gpu::Texture *output_texture,
                             gpu::Texture *intermediate_texture,
                             const char *output_mode,
                             int input_width,
                             int input_height,
                             int frame_number);

/**
 * Flush buffered diagnostic data to the JSONL file.
 * Called at diagnostic stop or session end.
 */
void FSR_diagnostics_flush();

/**
 * Reset all diagnostic state (called on Stop).
 */
void FSR_diagnostics_reset();

/**
 * Enable/disable diagnostic capture.
 */
void FSR_diagnostics_enable(bool enable);

/**
 * Set the capture interval (number of frames between captures).
 * Only affects viewport mode.
 */
void FSR_diagnostics_set_interval(int interval);

/**
 * Set the sparse pixel sampling grid dimensions.
 */
void FSR_diagnostics_set_pixel_grid(int cols, int rows);

/**
 * Set the output directory for JSONL files.
 */
void FSR_diagnostics_set_output_dir(const char *dir);

/** \} */

}  // namespace draw
}  // namespace blender
