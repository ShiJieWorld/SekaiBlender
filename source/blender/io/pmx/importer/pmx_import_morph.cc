/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_idprop.hh"
#include "BKE_key.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"

#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "pmx_import_mesh.hh"
#include "pmx_import_morph.hh"

#include <algorithm>
#include <cmath>
#include <string>

namespace blender::io::pmx {

static bool finite_offset(const float offset[3])
{
  return std::isfinite(offset[0]) && std::isfinite(offset[1]) && std::isfinite(offset[2]);
}

static bool reasonable_offset(const float offset[3], const float scale)
{
  const float limit = std::max(1000.0f, 10000.0f * scale);
  return std::abs(offset[0]) <= limit && std::abs(offset[1]) <= limit &&
         std::abs(offset[2]) <= limit;
}

static void ensure_shape_key_data(Mesh *mesh, Key *key, KeyBlock *key_block)
{
  if (key_block->data == nullptr) {
    BKE_keyblock_convert_from_mesh(mesh, key, key_block);
  }
}

static void apply_morph_to_object(const PMXImportContext &ctx,
                                  const PMXMorph &morph,
                                  Object *obj,
                                  const Vector<int> *new_to_old,
                                  KeyBlock *key_block)
{
  Mesh *mesh = obj ? reinterpret_cast<Mesh *>(obj->data) : nullptr;
  Key *key = mesh ? mesh->key : nullptr;
  if (!mesh || !key || !key_block || !key_block->data) {
    return;
  }

  float3 *data = static_cast<float3 *>(key_block->data);
  const float scale = ctx.params->global_scale;

  if (new_to_old == nullptr) {
    for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
      if (offset.vertex_index < 0 || offset.vertex_index >= mesh->verts_num) {
        continue;
      }
      if (!finite_offset(offset.offset) || !reasonable_offset(offset.offset, scale)) {
        continue;
      }
      float converted[3];
      transform_coord(converted, offset.offset, scale);
      data[offset.vertex_index] += float3(converted[0], converted[1], converted[2]);
    }
    return;
  }

  for (const int new_index : new_to_old->index_range()) {
    const int old_index = (*new_to_old)[new_index];
    for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
      if (offset.vertex_index != old_index || !finite_offset(offset.offset) ||
          !reasonable_offset(offset.offset, scale)) {
        continue;
      }
      float converted[3];
      transform_coord(converted, offset.offset, scale);
      data[new_index] += float3(converted[0], converted[1], converted[2]);
    }
  }
}

Vector<std::string> import_vertex_morphs(PMXImportContext &ctx, const PMXModel &model)
{
  Vector<std::string> morph_names;
  if (ctx.mesh_objects.is_empty()) {
    return morph_names;
  }

  for (const PMXMorph &morph : model.morphs) {
    if (morph.type != MorphType::Vertex) {
      continue;
    }

    const std::string base_name = morph.name_local.empty() ? "Morph" : morph.name_local;
    std::string blender_name = base_name;
    int suffix = 1;
    while (true) {
      bool used = false;
      for (const std::string &existing : morph_names) {
        if (existing == blender_name) {
          used = true;
          break;
        }
      }
      if (!used) {
        break;
      }
      blender_name = base_name + "." + std::to_string(suffix++);
    }
    morph_names.append(blender_name);
    ctx.morph_indices.append(int(&morph - model.morphs.data()));

    for (int object_index : ctx.mesh_objects.index_range()) {
      Object *obj = ctx.mesh_objects[object_index];
      Mesh *mesh = reinterpret_cast<Mesh *>(obj->data);
      if (!mesh) {
        continue;
      }

      Key *key = mesh->key;
      if (!key) {
        key = BKE_key_add(ctx.bmain, &mesh->id);
        key->type = KEY_RELATIVE;
        mesh->key = key;
        KeyBlock *basis = BKE_keyblock_add(key, "Basis");
        BKE_keyblock_convert_from_mesh(mesh, key, basis);
      }
      else if (key->block.first == nullptr) {
        KeyBlock *basis = BKE_keyblock_add(key, "Basis");
        BKE_keyblock_convert_from_mesh(mesh, key, basis);
      }

      KeyBlock *key_block = BKE_keyblock_add(key, blender_name.c_str());
      ensure_shape_key_data(mesh, key, key_block);

      const Vector<int> *mapping = nullptr;
      if (object_index < ctx.sub_meshes.size() && ctx.sub_meshes[object_index].obj == obj) {
        mapping = &ctx.sub_meshes[object_index].new_to_old;
      }
      apply_morph_to_object(ctx, morph, obj, mapping, key_block);
    }
  }

  return morph_names;
}

}  // namespace blender::io::pmx
