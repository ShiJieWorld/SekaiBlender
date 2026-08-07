/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(fsr_easu)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
PUSH_CONSTANT(float4, easu_con0)
PUSH_CONSTANT(float4, easu_con1)
PUSH_CONSTANT(float4, easu_con2)
PUSH_CONSTANT(float4, easu_con3)
PUSH_CONSTANT(int2, output_size)
COMPUTE_SOURCE("fsr_easu_comp.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(fsr_rcas)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
PUSH_CONSTANT(float4, rcas_con)
PUSH_CONSTANT(int2, output_size)
COMPUTE_SOURCE("fsr_rcas_comp.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()
