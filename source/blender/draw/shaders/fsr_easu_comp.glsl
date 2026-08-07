/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Float path port of AMD FidelityFX FSR 1.0 EASU from ffx_fsr1.h. */

#include "fsr_infos.hh"

COMPUTE_SHADER_CREATE_INFO(fsr_easu)

float3 fsr_sample(int2 p)
{
  int2 size = textureSize(input_tx, 0);
  return texelFetch(input_tx, clamp(p, int2(0), size - 1), 0).rgb;
}

float fsr_luma(float3 color)
{
  return color.b * 0.5f + (color.r * 0.5f + color.g);
}

float3 fsr_easu_set(float3 state,
                    float weight,
                    float luma_a,
                    float luma_b,
                    float luma_c,
                    float luma_d,
                    float luma_e)
{
  float dc = luma_d - luma_c;
  float cb = luma_c - luma_b;
  float length_x = max(abs(dc), abs(cb));
  float direction_x = luma_d - luma_b;
  length_x = length_x > 0.0f ? clamp(abs(direction_x) / length_x, 0.0f, 1.0f) : 0.0f;
  state.x += direction_x * weight;
  state.z += length_x * length_x * weight;

  float ec = luma_e - luma_c;
  float ca = luma_c - luma_a;
  float length_y = max(abs(ec), abs(ca));
  float direction_y = luma_e - luma_a;
  length_y = length_y > 0.0f ? clamp(abs(direction_y) / length_y, 0.0f, 1.0f) : 0.0f;
  state.y += direction_y * weight;
  state.z += length_y * length_y * weight;
  return state;
}

float4 fsr_easu_tap(float4 accumulation,
                    float2 pixel_offset,
                    float2 direction,
                    float2 length_scale,
                    float negative_lobe,
                    float clipping_point,
                    float3 color)
{
  float2 rotated_offset;
  rotated_offset.x = pixel_offset.x * direction.x + pixel_offset.y * direction.y;
  rotated_offset.y = pixel_offset.x * -direction.y + pixel_offset.y * direction.x;
  rotated_offset *= length_scale;

  float distance_squared = min(dot(rotated_offset, rotated_offset), clipping_point);
  float weight_b = 0.4f * distance_squared - 1.0f;
  float weight_a = negative_lobe * distance_squared - 1.0f;
  weight_b *= weight_b;
  weight_a *= weight_a;
  weight_b = (25.0f / 16.0f) * weight_b - (9.0f / 16.0f);
  float weight = weight_b * weight_a;

  accumulation.rgb += color * weight;
  accumulation.a += weight;
  return accumulation;
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(pixel, output_size))) {
    return;
  }

  float2 source_position = float2(pixel) * easu_con0.xy + easu_con0.zw;
  int2 f_position = int2(floor(source_position));
  float2 subpixel = source_position - floor(source_position);

  float3 b = fsr_sample(f_position + int2(0, -1));
  float3 c = fsr_sample(f_position + int2(1, -1));
  float3 e = fsr_sample(f_position + int2(-1, 0));
  float3 f = fsr_sample(f_position);
  float3 g = fsr_sample(f_position + int2(1, 0));
  float3 h = fsr_sample(f_position + int2(2, 0));
  float3 i = fsr_sample(f_position + int2(-1, 1));
  float3 j = fsr_sample(f_position + int2(0, 1));
  float3 k = fsr_sample(f_position + int2(1, 1));
  float3 l = fsr_sample(f_position + int2(2, 1));
  float3 n = fsr_sample(f_position + int2(0, 2));
  float3 o = fsr_sample(f_position + int2(1, 2));

  float b_l = fsr_luma(b);
  float c_l = fsr_luma(c);
  float e_l = fsr_luma(e);
  float f_l = fsr_luma(f);
  float g_l = fsr_luma(g);
  float h_l = fsr_luma(h);
  float i_l = fsr_luma(i);
  float j_l = fsr_luma(j);
  float k_l = fsr_luma(k);
  float l_l = fsr_luma(l);
  float n_l = fsr_luma(n);
  float o_l = fsr_luma(o);

  float3 direction_state = float3(0.0f);
  direction_state = fsr_easu_set(direction_state,
                                 (1.0f - subpixel.x) * (1.0f - subpixel.y),
                                 b_l,
                                 e_l,
                                 f_l,
                                 g_l,
                                 j_l);
  direction_state = fsr_easu_set(direction_state,
                                 subpixel.x * (1.0f - subpixel.y),
                                 c_l,
                                 f_l,
                                 g_l,
                                 h_l,
                                 k_l);
  direction_state = fsr_easu_set(direction_state,
                                 (1.0f - subpixel.x) * subpixel.y,
                                 f_l,
                                 i_l,
                                 j_l,
                                 k_l,
                                 n_l);
  direction_state = fsr_easu_set(direction_state,
                                 subpixel.x * subpixel.y,
                                 g_l,
                                 j_l,
                                 k_l,
                                 l_l,
                                 o_l);

  float2 direction = direction_state.xy;
  float edge_length = direction_state.z;

  float direction_length_squared = dot(direction, direction);
  if (direction_length_squared < (1.0f / 32768.0f)) {
    direction = float2(1.0f, 0.0f);
  }
  else {
    direction *= inversesqrt(direction_length_squared);
  }

  edge_length = edge_length * 0.5f;
  edge_length *= edge_length;
  float stretch = dot(direction, direction) / max(abs(direction.x), abs(direction.y));
  float2 length_scale = float2(1.0f + (stretch - 1.0f) * edge_length,
                               1.0f - 0.5f * edge_length);
  float negative_lobe = 0.5f + (0.21f - 0.5f) * edge_length;
  float clipping_point = 1.0f / negative_lobe;

  float3 min4 = min(min(f, g), min(j, k));
  float3 max4 = max(max(f, g), max(j, k));
  float4 accumulation = float4(0.0f);

  accumulation = fsr_easu_tap(accumulation, float2(0.0f, -1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, b);
  accumulation = fsr_easu_tap(accumulation, float2(1.0f, -1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, c);
  accumulation = fsr_easu_tap(accumulation, float2(-1.0f, 1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, i);
  accumulation = fsr_easu_tap(accumulation, float2(0.0f, 1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, j);
  accumulation = fsr_easu_tap(accumulation, -subpixel, direction, length_scale, negative_lobe, clipping_point, f);
  accumulation = fsr_easu_tap(accumulation, float2(-1.0f, 0.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, e);
  accumulation = fsr_easu_tap(accumulation, float2(1.0f, 1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, k);
  accumulation = fsr_easu_tap(accumulation, float2(2.0f, 1.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, l);
  accumulation = fsr_easu_tap(accumulation, float2(2.0f, 0.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, h);
  accumulation = fsr_easu_tap(accumulation, float2(1.0f, 0.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, g);
  accumulation = fsr_easu_tap(accumulation, float2(1.0f, 2.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, o);
  accumulation = fsr_easu_tap(accumulation, float2(0.0f, 2.0f) - subpixel, direction, length_scale, negative_lobe, clipping_point, n);

  float3 result = clamp(accumulation.rgb / accumulation.a, min4, max4);
  int2 input_max = textureSize(input_tx, 0) - 1;
  float2 alpha_position = clamp(source_position, float2(0.0f), float2(input_max));
  int2 alpha_base = int2(floor(alpha_position));
  float2 alpha_fraction = alpha_position - floor(alpha_position);
  float alpha_bottom = mix(texelFetch(input_tx, clamp(alpha_base, int2(0), input_max), 0).a,
                           texelFetch(input_tx,
                                      clamp(alpha_base + int2(1, 0), int2(0), input_max),
                                      0)
                               .a,
                           alpha_fraction.x);
  float alpha_top = mix(texelFetch(input_tx,
                                   clamp(alpha_base + int2(0, 1), int2(0), input_max),
                                   0)
                            .a,
                        texelFetch(input_tx,
                                   clamp(alpha_base + int2(1, 1), int2(0), input_max),
                                   0)
                            .a,
                        alpha_fraction.x);
  float alpha = mix(alpha_bottom, alpha_top, alpha_fraction.y);
  imageStore(output_img, pixel, float4(result, alpha));
}
