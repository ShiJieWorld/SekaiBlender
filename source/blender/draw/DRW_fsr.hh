/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 *
 * FSR 1.0 EASU and RCAS compute pipeline.
 */

#pragma once

namespace blender::gpu {
class Shader;
class Texture;
}  // namespace blender::gpu

namespace blender::draw {

class FsrContext {
 private:
  gpu::Texture *intermediate_texture_ = nullptr;
  gpu::Texture *output_texture_ = nullptr;
  bool initialized_ = false;

  bool init();
  void free();

 public:
  FsrContext() = default;
  ~FsrContext();

  FsrContext(const FsrContext &) = delete;
  FsrContext &operator=(const FsrContext &) = delete;

  /** Release per-context textures while keeping shared FSR shaders available. */
  void reset();

  /**
   * Upscale a region of an RGBA16F texture using EASU followed by RCAS.
   * The returned texture is owned by this context and remains valid until the next call.
   *
   * \param allow_host_readback: Add host-read usage to the output texture for explicit
   *                             GPU readback, such as the self-test. Keep false for rendering.
   */
  gpu::Texture *upsample(gpu::Texture *input_texture,
                         int input_x,
                         int input_y,
                         int input_width,
                         int input_height,
                         int output_width,
                         int output_height,
                         float sharpness,
                         bool allow_host_readback = false);

  gpu::Texture *intermediate_texture_get() const
  {
    return intermediate_texture_;
  }
};

/** Run deterministic GPU dispatch and readback validation with an active GPU context. */
bool fsr_self_test();

/** Release immutable FSR shaders while a GPU context is active. */
void fsr_static_shaders_free();

}  // namespace blender::draw
