/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Float path port of AMD FidelityFX FSR 1.0 RCAS from ffx_fsr1.h. */

#include "fsr_infos.hh"

COMPUTE_SHADER_CREATE_INFO(fsr_rcas)

float4 fsr_sample(int2 p)
{
  int2 size = textureSize(input_tx, 0);
  return texelFetch(input_tx, clamp(p, int2(0), size - int2(1)), 0);
}

float fsr_luma(float3 color)
{
  return color.b * 0.5f + (color.r * 0.5f + color.g);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(pixel, output_size))) {
    return;
  }

  float3 b = fsr_sample(pixel + int2(0, -1)).rgb;
  float3 d = fsr_sample(pixel + int2(-1, 0)).rgb;
  float4 center = fsr_sample(pixel);
  float3 e = center.rgb;
  float3 f = fsr_sample(pixel + int2(1, 0)).rgb;
  float3 h = fsr_sample(pixel + int2(0, 1)).rgb;

  float b_l = fsr_luma(b);
  float d_l = fsr_luma(d);
  float e_l = fsr_luma(e);
  float f_l = fsr_luma(f);
  float h_l = fsr_luma(h);

  float3 min4 = min(e, min(min(b, d), min(f, h)));
  float3 max4 = max(e, max(max(b, d), max(f, h)));
  float lower_luma = min(min(b_l, d_l), min(f_l, h_l));
  float lower_limiter = lower_luma > 0.0f ? clamp(e_l / lower_luma, 0.0f, 1.0f) : 1.0f;

  float3 hit_min = min4 / max(float3(4.0f) * max4, float3(1.0e-6f));
  hit_min *= lower_limiter;
  /* Scene-linear HDR has no fixed upper bound. Limit only against negative undershoot. */
  float3 lobe_rgb = -hit_min;
  float lobe = max(-(0.25f - 1.0f / 16.0f),
                   min(max(lobe_rgb.r, max(lobe_rgb.g, lobe_rgb.b)), 0.0f));
  lobe *= rcas_con.x;

  float reciprocal_weight = 1.0f / (4.0f * lobe + 1.0f);
  float3 result = (lobe * (b + d + f + h) + e) * reciprocal_weight;
  imageStore(output_img, pixel, float4(result, center.a));
}
