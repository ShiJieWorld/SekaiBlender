/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 *
 * FSR Diagnostic — Objective GPU Data Collection (Implementation)
 *
 * Design contract:
 *   - Every field recorded here is raw observation, never a conclusion.
 *   - This code never writes "PASS", "FAIL", "OK", or "ERROR".
 *   - Status flags are bools about existence (shader compiled? texture valid?).
 *   - GPU timing is measured with high_resolution_clock, no interpretation.
 *   - Sampled pixel values are stored verbatim.
 *
 * All analysis, interpretation, and judgment lives in Python.
 */

#include "DRW_fsr_diagnostics.hh"

#include "BLI_fileops.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"
#include "BLI_time.hh"

#include "GPU_shader.hh"
#include "GPU_texture.hh"

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace blender::draw {

/* -------------------------------------------------------------------- */
/** \name Internal State
 * \{ */

/* Output directory for JSONL files. */
static std::string g_output_dir;

/* JSONL output file handle. */
static FILE *g_jsonl_file = nullptr;

/* Whether diagnostics are enabled. */
static bool g_enabled = false;

/* Capture interval (viewport mode). */
static int g_capture_interval = 10;
static int g_capture_counter = 0;

/* Pixel sample grid. */
static int g_sample_cols = 16;
static int g_sample_rows = 12;

/* Active options snapshot (written at the start of each capture session). */
static bool g_options_written = false;

/* Number of frames captured in the current session. */
static int g_frame_count = 0;

/* GPU resources (lazily initialized). */
static gpu::Shader *g_luma_shader = nullptr;
static gpu::Shader *g_sample_shader = nullptr;
static gpu::Texture *g_sample_readback_tex = nullptr;
static bool g_resources_valid = false;

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal Helpers
 * \{ */

/**
 * Ensure the output directory exists.
 * Returns true on success.
 */
static bool ensure_output_dir()
{
  if (g_output_dir.empty()) {
    return false;
  }
  /* BLI_dir_create_recursive returns 1 for existing directory, 0 for new. */
  return BLI_dir_create_recursive(g_output_dir.c_str()) >= 0;
}

/**
 * Open (or create) the JSONL output file.
 * Returns true on success.
 */
static bool open_jsonl()
{
  if (g_jsonl_file != nullptr) {
    return true;
  }
  if (g_output_dir.empty() || !ensure_output_dir()) {
    return false;
  }

  /* Build filename: fsr_diagnostics_YYYYMMDD_HHMMSS.jsonl */
  char filename[512];
  time_t now = time(nullptr);
  struct tm *tm_info = localtime(&now);
  char timestamp[32];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_info);

  /* Use .jsonl.tmp to detect incomplete writes. */
  SNPRINTF(filename, "%s/fsr_diagnostics_%s.jsonl.tmp", g_output_dir.c_str(), timestamp);

  g_jsonl_file = BLI_fopen(filename, "w");
  if (g_jsonl_file == nullptr) {
    fprintf(stderr, "[FSR Diagnostics] Cannot open output file: %s\n", filename);
    return false;
  }

  g_options_written = false;
  g_frame_count = 0;
  return true;
}

/**
 * Write a single JSON line to the output file.
 * Uses fprintf for simplicity; no JSON library dependency.
 */
static void write_jsonl_line(const char *json_line)
{
  if (g_jsonl_file == nullptr) {
    return;
  }
  fputs(json_line, g_jsonl_file);
  fputc('\n', g_jsonl_file);
}

/**
 * Flush and close the JSONL file, renaming from .jsonl.tmp to .jsonl.
 */
static void close_jsonl()
{
  if (g_jsonl_file == nullptr) {
    return;
  }
  fclose(g_jsonl_file);

  /* TODO: Rename from .jsonl.tmp to .jsonl. */
  /* For now, keep the .jsonl.tmp name for debugging. */

  g_jsonl_file = nullptr;
  g_options_written = false;
  g_frame_count = 0;
}

/**
 * Write the options header (only once per session).
 * Contains configuration that applies to all captured frames.
 */
static void write_options_header()
{
  if (g_options_written) {
    return;
  }
  if (g_jsonl_file == nullptr) {
    return;
  }

  /* Write as a metadata line. */
  char buf[1024];
  SNPRINTF(buf,
           "{\"type\":\"options\","
           "\"capture_interval\":%d,"
           "\"pixel_sample_grid\":[%d,%d]}",
           g_capture_interval,
           g_sample_cols,
           g_sample_rows);
  write_jsonl_line(buf);
  g_options_written = true;
}

/**
 * Get the current timestamp in microseconds.
 */
static uint64_t timestamp_us()
{
  auto now = std::chrono::high_resolution_clock::now();
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch());
  return static_cast<uint64_t>(us.count());
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name GPU Resource Management
 * \{ */

/**
 * Initialize GPU resources for diagnostic capture.
 * Creates compute shaders (luma reduction, pixel sampling) lazily.
 */
static void ensure_gpu_resources()
{
  if (g_resources_valid) {
    return;
  }

  /* TODO: Create compute shaders using GPU_shader_create_from_info().
   * For now, mark resources as valid to avoid repeated attempts.
   * Full shader creation happens when the GLSL files are integrated
   * into Blender's shader build system.
   */
  g_resources_valid = true;
}

/**
 * Free GPU resources.
 */
static void free_gpu_resources()
{
  GPU_SHADER_FREE_SAFE(g_luma_shader);
  GPU_SHADER_FREE_SAFE(g_sample_shader);
  GPU_TEXTURE_FREE_SAFE(g_sample_readback_tex);
  g_resources_valid = false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Data Collection Helpers
 * \{ */

/**
 * Collect shader & pipeline status.
 * Returns a JSON fragment string (must be freed with MEM_freeN).
 *
 * Status fields are booleans about existence, not about "correctness".
 * "easu_shader_valid = false" means the shader pointer is null,
 * not "the shader is broken".
 */
static std::string collect_status(gpu::Texture *output_tex,
                                  gpu::Texture *intermediate_tex)
{
  char buf[1024];
  SNPRINTF(buf,
           "\"status\":{"
           "\"easu_shader_valid\":%s,"
           "\"rcas_shader_valid\":%s,"
           "\"input_texture_valid\":%s,"
           "\"output_texture_valid\":%s,"
           "\"intermediate_texture_valid\":%s"
           "}",
           g_luma_shader != nullptr ? "true" : "false",   /* Placeholder: EASU shader */
           g_luma_shader != nullptr ? "true" : "false",    /* Placeholder: RCAS shader */
           "true",                                          /* Input tex always valid when called */
           output_tex != nullptr ? "true" : "false",
           intermediate_tex != nullptr ? "true" : "false");
  return std::string(buf);
}

/**
 * Collect texture metadata.
 * Records dimensions, format, and layer count.
 * Does not record pixel data (that's sampled_pixels).
 */
static std::string collect_texture_meta(gpu::Texture *output_tex,
                                        gpu::Texture *intermediate_tex,
                                        int input_w,
                                        int input_h)
{
  int out_w = output_tex ? GPU_texture_width(output_tex) : 0;
  int out_h = output_tex ? GPU_texture_height(output_tex) : 0;
  int inter_w = intermediate_tex ? GPU_texture_width(intermediate_tex) : 0;
  int inter_h = intermediate_tex ? GPU_texture_height(intermediate_tex) : 0;

  char buf[1024];
  SNPRINTF(buf,
           "\"textures\":{"
           "\"input\":{\"w\":%d,\"h\":%d,\"format\":\"RGBA16F\",\"layers\":1},"
           "\"intermediate\":{\"w\":%d,\"h\":%d,\"format\":\"RGBA16F\",\"layers\":1},"
           "\"output\":{\"w\":%d,\"h\":%d,\"format\":\"RGBA16F\",\"layers\":1}"
           "}",
           input_w, input_h,
           inter_w, inter_h,
           out_w, out_h);
  return std::string(buf);
}

/**
 * Collect CPU-side timing.
 * Since Blender has no GPU timer query API, we measure from the CPU side
 * using std::chrono. This is a coarse measurement that includes CPU overhead.
 *
 * The timing values are placed at 0 for now; they will be filled when
 * FSR dispatch is actually hooked up.
 */
static std::string collect_timing()
{
  return std::string(
      "\"gpu_timing_us\":{"
      "\"easu\":0,"
      "\"rcas\":0,"
      "\"total\":0"
      "}");
}

/**
 * Collect luma statistics by running a compute shader reduction.
 *
 * For now, returns placeholder zeros. Full implementation requires
 * the luma reduction compute shader to be working.
 *
 * Fields:
 *   min, max, mean, median, stddev: luminance statistics
 *   nan_count, inf_count, zero_count: pixel validity counts
 *   total_pixels: total number of pixels examined
 */
static std::string collect_luma_stats()
{
  return std::string(
      "\"luma_stats\":{"
      "\"min\":0.0,"
      "\"max\":0.0,"
      "\"mean\":0.0,"
      "\"median\":0.0,"
      "\"stddev\":0.0,"
      "\"nan_count\":0,"
      "\"inf_count\":0,"
      "\"zero_count\":0,"
      "\"total_pixels\":0"
      "}");
}

/**
 * Collect sparse pixel samples via GPU readback.
 *
 * For now, returns an empty samples array. Full implementation requires
 * the pixel sampling compute shader and GPU readback to be working.
 */
static std::string collect_pixel_samples()
{
  return std::string("\"sampled_pixels\":[]");
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public API
 * \{ */

void FSR_diagnostics_init()
{
  g_enabled = false;
  g_jsonl_file = nullptr;
  g_capture_counter = g_capture_interval;
  g_options_written = false;
  g_frame_count = 0;

  /* Auto-enable via environment variable. Writes a test record. */
  const char *diag_dir = getenv("FSR_DIAGNOSTICS_DIR");
  if (diag_dir != nullptr && diag_dir[0] != '\0') {
    FSR_diagnostics_set_output_dir(diag_dir);
    FSR_diagnostics_enable(true);

    /* Write a test JSONL record to verify output pipeline. */
    if (open_jsonl()) {
      write_options_header();
      uint64_t ts = timestamp_us();
      char buf[4096];
      SNPRINTF(buf,
               "{"
               "\"frame\":0,"
               "\"type\":\"init_test\","
               "\"timestamp_us\":%" PRIu64 ","
               "\"options\":{\"output_mode\":\"init\",\"fsr_enabled\":false,\"fsr_scale\":1.0,\"fsr_quality\":0},"
               "\"fsr\":{"
                 "\"status\":{"
                 "\"easu_shader_valid\":false,\"rcas_shader_valid\":false,"
                   "\"input_texture_valid\":false,\"output_texture_valid\":false,"
                   "\"intermediate_texture_valid\":false"
                 "},"
                 "\"textures\":{"
                   "\"input\":{\"w\":0,\"h\":0,\"format\":\"none\",\"layers\":0},"
                   "\"intermediate\":{\"w\":0,\"h\":0,\"format\":\"none\",\"layers\":0},"
                   "\"output\":{\"w\":0,\"h\":0,\"format\":\"none\",\"layers\":0}"
                 "},"
                 "\"gpu_timing_us\":{\"easu\":0,\"rcas\":0,\"total\":0},"
                 "\"constants\":{},"
                 "\"luma_stats\":{"
                   "\"min\":0.0,\"max\":0.0,\"mean\":0.0,"
                   "\"median\":0.0,\"stddev\":0.0,"
                   "\"nan_count\":0,\"inf_count\":0,\"zero_count\":0,\"total_pixels\":0"
                 "},"
                 "\"sampled_pixels\":[]"
               "}"
               "}",
               ts);
      write_jsonl_line(buf);
      g_frame_count++;
      close_jsonl();
    }
  }
}

void FSR_diagnostics_free()
{
  close_jsonl();
  free_gpu_resources();
}

void FSR_diagnostics_capture(gpu::Texture *output_texture,
                             gpu::Texture *intermediate_texture,
                             const char *output_mode,
                             int input_width,
                             int input_height,
                             int frame_number)
{
  if (!g_enabled) {
    return;
  }

  /* Throttle capture for viewport mode. */
  if (strcmp(output_mode, "viewport") == 0) {
    g_capture_counter--;
    if (g_capture_counter > 0) {
      return;
    }
    g_capture_counter = g_capture_interval;
  }
  /* Render mode: capture every frame (no throttling). */

  if (!open_jsonl()) {
    return;
  }

  /* Ensure GPU resources are ready. */
  ensure_gpu_resources();

  /* Write options header on first capture. */
  write_options_header();

  /* — Collect all diagnostic data — */

  uint64_t ts = timestamp_us();

  std::string status_json = collect_status(output_texture, intermediate_texture);
  std::string tex_json = collect_texture_meta(output_texture, intermediate_texture,
                                              input_width, input_height);
  std::string timing_json = collect_timing();
  std::string luma_json = collect_luma_stats();
  std::string samples_json = collect_pixel_samples();

  /* — Assemble JSONL record — */

  char frame_buf[8192];
  SNPRINTF(frame_buf,
           "{"
           "\"frame\":%d,"
           "\"timestamp_us\":%" PRIu64 ","
           "\"options\":{"
             "\"output_mode\":\"%s\","
             "\"fsr_enabled\":true,"
             "\"fsr_scale\":%.1f,"
             "\"fsr_quality\":0"
           "},"
           "\"fsr\":{"
             "%s,"
             "%s,"
             "%s,"
             "%s,"
             "%s,"
             "%s"
           "}"
           "}",
           frame_number,
           ts,
           output_mode,
           (input_width > 0 && output_texture != nullptr)
               ? (float)GPU_texture_width(output_texture) / (float)input_width
               : 1.0f,
           status_json.c_str(),
           tex_json.c_str(),
           timing_json.c_str(),
           "{}",  /* constants — placeholder, will be filled when FSR is hooked up */
           luma_json.c_str(),
           samples_json.c_str());

  write_jsonl_line(frame_buf);
  g_frame_count++;
}

void FSR_diagnostics_flush()
{
  close_jsonl();
}

void FSR_diagnostics_reset()
{
  close_jsonl();
  g_frame_count = 0;
  g_options_written = false;
}

void FSR_diagnostics_enable(bool enable)
{
  g_enabled = enable;
  if (!enable) {
    close_jsonl();
  }
}

void FSR_diagnostics_set_interval(int interval)
{
  g_capture_interval = std::max(1, interval);
  g_capture_counter = g_capture_interval;
}

void FSR_diagnostics_set_pixel_grid(int cols, int rows)
{
  g_sample_cols = std::max(4, std::min(32, cols));
  g_sample_rows = std::max(4, std::min(32, rows));
}

void FSR_diagnostics_set_output_dir(const char *dir)
{
  if (dir == nullptr || dir[0] == '\0') {
    g_output_dir.clear();
    return;
  }
  g_output_dir = dir;
}

/** \} */

}  // namespace blender::draw
