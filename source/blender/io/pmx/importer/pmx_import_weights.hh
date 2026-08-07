/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "IO_pmx.hh"
#include "intern/pmx_types.h"
#include "DNA_object_types.h"

#include "BLI_vector.hh"

#include <string>

namespace blender {
struct Mesh;
}

namespace blender::io::pmx {

struct PMXImportContext;

/**
 * Store the PMX per-vertex skinning schema on a mesh. This preserves the
 * original PMX weight type and SDEF parameters for a later MMD skinning
 * evaluator. If new_to_old is provided (split mode), mesh vertices map back
 * to original model vertices through that array.
 */
void write_pmx_vertex_skinning_attributes(Mesh &mesh,
                                          const PMXModel &model,
                                          const Vector<int> *new_to_old,
                                          float global_scale);

/**
 * Create vertex groups and assign PMX bone weights to a single mesh object.
 * bone_names[i] corresponds to PMX bone index i.
 * If new_to_old is provided (split mode), vertex indices are remapped from
 * sub-mesh space to original model space for weight lookup.
 */
void assign_vertex_weights_named(Object *obj,
                                  const PMXModel &model,
                                  const Vector<std::string> &bone_names,
                                  const Vector<int> *new_to_old = nullptr,
                                  float global_scale = 1.0f);

/**
 * Process weight assignment for all mesh objects in the import context.
 * Handles both single-mode (direct) and split-mode (vertex remapped).
 */
void assign_all_vertex_weights(PMXImportContext &ctx,
                                const PMXModel &model,
                                const Vector<std::string> &bone_names);

/**
 * Persist the PMX per-vertex edge scale as an "mmd_edge_scale" vertex group.
 * This is pure metadata: it never deforms the mesh and is only read back when
 * building the toon-edge preview. If new_to_old is provided (split mode), mesh
 * vertices map back to original model vertices through that array.
 */
void write_pmx_edge_scale_group(Object *obj,
                                const PMXModel &model,
                                const Vector<int> *new_to_old = nullptr);

/** Write the "mmd_edge_scale" group for every mesh object in the import context. */
void write_all_pmx_edge_scale_groups(PMXImportContext &ctx, const PMXModel &model);

}  // namespace blender::io::pmx
