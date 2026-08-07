/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 *
 * FSR 1.0 EASU and RCAS compute pipeline.
 */

#include "DRW_fsr.hh"

#include "MEM_guardedalloc.h"

#include "GPU_compute.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace blender::draw {

static gpu::Shader *g_easu_shader = nullptr;
static gpu::Shader *g_rcas_shader = nullptr;

struct EasuConstants {
  float con0[4];
  float con1[4];
  float con2[4];
  float con3[4];
};

static EasuConstants compute_easu_constants(const int input_x,
                                             const int input_y,
                                             const int input_width,
                                             const int input_height,
                                             const int texture_width,
                                             const int texture_height,
                                             const int output_width,
                                             const int output_height)
{
  const float reciprocal_texture_width = 1.0f / float(texture_width);
  const float reciprocal_texture_height = 1.0f / float(texture_height);
  const float scale_x = float(input_width) / float(output_width);
  const float scale_y = float(input_height) / float(output_height);

  EasuConstants constants{};
  constants.con0[0] = scale_x;
  constants.con0[1] = scale_y;
  constants.con0[2] = float(input_x) + 0.5f * scale_x - 0.5f;
  constants.con0[3] = float(input_y) + 0.5f * scale_y - 0.5f;
  constants.con1[0] = reciprocal_texture_width;
  constants.con1[1] = reciprocal_texture_height;
  constants.con1[2] = reciprocal_texture_width;
  constants.con1[3] = -reciprocal_texture_height;
  constants.con2[0] = -reciprocal_texture_width;
  constants.con2[1] = 2.0f * reciprocal_texture_height;
  constants.con2[2] = reciprocal_texture_width;
  constants.con2[3] = 2.0f * reciprocal_texture_height;
  constants.con3[0] = 0.0f;
  constants.con3[1] = 4.0f * reciprocal_texture_height;
  return constants;
}

static gpu::Texture *ensure_texture(gpu::Texture *&texture,
                                    const char *name,
                                    const int width,
                                    const int height,
                                    const bool allow_host_readback)
{
  const bool has_host_readback = texture != nullptr &&
                                 (GPU_texture_usage(texture) & GPU_TEXTURE_USAGE_HOST_READ) != 0;
  if (texture != nullptr &&
      (GPU_texture_width(texture) != width || GPU_texture_height(texture) != height ||
       has_host_readback != allow_host_readback))
  {
    GPU_TEXTURE_FREE_SAFE(texture);
  }
  if (texture == nullptr) {
    const eGPUTextureUsage usage = allow_host_readback ?
                                       eGPUTextureUsage(GPU_TEXTURE_USAGE_SHADER_READ |
                                                        GPU_TEXTURE_USAGE_SHADER_WRITE |
                                                        GPU_TEXTURE_USAGE_HOST_READ) :
                                       eGPUTextureUsage(GPU_TEXTURE_USAGE_SHADER_READ |
                                                        GPU_TEXTURE_USAGE_SHADER_WRITE);
    texture = GPU_texture_create_2d(
        name,
        width,
        height,
        1,
        gpu::TextureFormat::SFLOAT_16_16_16_16,
        usage,
        nullptr);
  }
  return texture;
}

static void bind_compute_textures(gpu::Shader *shader,
                                  gpu::Texture *input,
                                  gpu::Texture *output)
{
  GPU_texture_bind(input, GPU_shader_get_sampler_binding(shader, "input_tx"));
  GPU_texture_image_bind(output, GPU_shader_get_sampler_binding(shader, "output_img"));
}

static void unbind_compute_textures(gpu::Texture *input, gpu::Texture *output)
{
  GPU_texture_unbind(input);
  GPU_texture_image_unbind(output);
}

FsrContext::~FsrContext()
{
  free();
}

bool FsrContext::init()
{
  if (initialized_ && g_easu_shader != nullptr && g_rcas_shader != nullptr) {
    return true;
  }
  initialized_ = false;

  if (GPU_shader_create_info_get("fsr_easu") == nullptr ||
      GPU_shader_create_info_get("fsr_rcas") == nullptr)
  {
    std::fprintf(stderr, "FSR shader create info is not registered\n");
    return false;
  }

  if (g_easu_shader == nullptr) {
    g_easu_shader = GPU_shader_create_from_info_name("fsr_easu");
  }
  if (g_rcas_shader == nullptr) {
    g_rcas_shader = GPU_shader_create_from_info_name("fsr_rcas");
  }
  if (g_easu_shader == nullptr || g_rcas_shader == nullptr) {
    free();
    return false;
  }

  initialized_ = true;
  return true;
}

void FsrContext::free()
{
  GPU_TEXTURE_FREE_SAFE(intermediate_texture_);
  GPU_TEXTURE_FREE_SAFE(output_texture_);
  initialized_ = false;
}

void FsrContext::reset()
{
  free();
}

gpu::Texture *FsrContext::upsample(gpu::Texture *input_texture,
                                   const int input_x,
                                   const int input_y,
                                   const int input_width,
                                   const int input_height,
                                   const int output_width,
                                   const int output_height,
                                   const float sharpness,
                                   const bool allow_host_readback)
{
  if (input_texture == nullptr || input_x < 0 || input_y < 0 || input_width <= 0 ||
      input_height <= 0 || output_width <= 0 || output_height <= 0 || !init())
  {
    return nullptr;
  }

  const int texture_width = GPU_texture_width(input_texture);
  const int texture_height = GPU_texture_height(input_texture);
  if (input_x + input_width > texture_width || input_y + input_height > texture_height) {
    return nullptr;
  }

  gpu::Texture *intermediate = ensure_texture(
      intermediate_texture_, "fsr_easu_output", output_width, output_height, false);
  gpu::Texture *output = ensure_texture(
      output_texture_, "fsr_rcas_output", output_width, output_height, allow_host_readback);
  if (intermediate == nullptr || output == nullptr) {
    return nullptr;
  }

  const int output_size[2] = {output_width, output_height};
  const uint groups_x = uint(output_width + 15) / 16;
  const uint groups_y = uint(output_height + 15) / 16;
  const EasuConstants constants = compute_easu_constants(input_x,
                                                         input_y,
                                                         input_width,
                                                         input_height,
                                                         texture_width,
                                                         texture_height,
                                                         output_width,
                                                         output_height);

  GPU_shader_bind(g_easu_shader);
  GPU_shader_uniform_4fv(g_easu_shader, "easu_con0", constants.con0);
  GPU_shader_uniform_4fv(g_easu_shader, "easu_con1", constants.con1);
  GPU_shader_uniform_4fv(g_easu_shader, "easu_con2", constants.con2);
  GPU_shader_uniform_4fv(g_easu_shader, "easu_con3", constants.con3);
  GPU_shader_uniform_2iv(g_easu_shader, "output_size", output_size);
  bind_compute_textures(g_easu_shader, input_texture, intermediate);
  GPU_compute_dispatch(g_easu_shader, groups_x, groups_y, 1);
  unbind_compute_textures(input_texture, intermediate);

  GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH);

  const float amount = std::clamp(sharpness, 0.0f, 1.0f);
  const float rcas_scale = amount == 0.0f ? 0.0f : std::exp2(-2.0f * (1.0f - amount));
  const float rcas_con[4] = {rcas_scale, 0.0f, 0.0f, 0.0f};
  GPU_shader_bind(g_rcas_shader);
  GPU_shader_uniform_4fv(g_rcas_shader, "rcas_con", rcas_con);
  GPU_shader_uniform_2iv(g_rcas_shader, "output_size", output_size);
  bind_compute_textures(g_rcas_shader, intermediate, output);
  GPU_compute_dispatch(g_rcas_shader, groups_x, groups_y, 1);
  unbind_compute_textures(intermediate, output);
  GPU_shader_unbind();

  GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH);
  return output;
}

bool fsr_self_test()
{
  constexpr int texture_width = 11;
  constexpr int texture_height = 9;
  constexpr int input_x = 1;
  constexpr int input_y = 1;
  constexpr int input_width = 9;
  constexpr int input_height = 7;
  constexpr int output_width = 17;
  constexpr int output_height = 13;

  float input_pixels[texture_width * texture_height * 4];
  for (int y = 0; y < texture_height; y++) {
    for (int x = 0; x < texture_width; x++) {
      const int index = (y * texture_width + x) * 4;
      const bool border = x == 0 || y == 0 || x == texture_width - 1 ||
                          y == texture_height - 1;
      input_pixels[index + 0] = border ? 0.2f : float(x - input_x) / float(input_width - 1);
      input_pixels[index + 1] = border ? 0.3f : float(y - input_y) / float(input_height - 1);
      input_pixels[index + 2] = border ? 0.4f : (((x + y) & 1) ? 0.75f : 0.1f);
      input_pixels[index + 3] = border ? 0.0f : float(x - input_x) / float(input_width - 1);
      if (x == input_x + input_width / 2 && y == input_y + input_height / 2) {
        input_pixels[index + 0] = 4.0f;
        input_pixels[index + 1] = 2.0f;
      }
    }
  }

  gpu::Texture *input = GPU_texture_create_2d(
      "fsr_self_test_input",
      texture_width,
      texture_height,
      1,
      gpu::TextureFormat::SFLOAT_16_16_16_16,
      eGPUTextureUsage(GPU_TEXTURE_USAGE_SHADER_READ),
      input_pixels);

  FsrContext context;
  gpu::Texture *output = input != nullptr ? context.upsample(input,
                                                            input_x,
                                                             input_y,
                                                            input_width,
                                                            input_height,
                                                            output_width,
                                                            output_height,
                                                            0.2f,
                                                            true) :
                                            nullptr;
  float *pixels = output != nullptr ?
                      static_cast<float *>(GPU_texture_read(output, GPU_DATA_FLOAT, 0)) :
                      nullptr;

  bool valid = pixels != nullptr;
  float minimum = pixels != nullptr ? pixels[0] : 0.0f;
  float maximum = minimum;
  double sum = 0.0;
  if (pixels != nullptr) {
    for (int index = 0; index < output_width * output_height * 4; index++) {
      const float value = pixels[index];
      valid &= std::isfinite(value) && value > -2.0f && value < 8.0f;
      minimum = std::min(minimum, value);
      maximum = std::max(maximum, value);
      sum += value;
    }
    valid &= maximum > 2.0f;
  }

  std::printf("FSR_SELF_TEST result=%s input=%dx%d region=%d,%d,%dx%d output=%dx%d "
              "min=%.6f max=%.6f mean=%.6f\n",
              valid ? "PASS" : "FAIL",
              texture_width,
              texture_height,
              input_x,
              input_y,
              input_width,
              input_height,
              output_width,
              output_height,
              minimum,
              maximum,
              pixels != nullptr ? float(sum / (output_width * output_height * 4)) : 0.0f);

  if (pixels != nullptr) {
    MEM_delete(pixels);
  }
  GPU_TEXTURE_FREE_SAFE(input);
  return valid;
}

void fsr_static_shaders_free()
{
  GPU_SHADER_FREE_SAFE(g_easu_shader);
  GPU_SHADER_FREE_SAFE(g_rcas_shader);
}

}  // namespace blender::draw
