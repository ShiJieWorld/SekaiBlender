/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "IO_pmx.hh"
#include "intern/pmx_types.h"
#include "pmx_group_morph.hh"

#include "DNA_object_types.h"

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include <string>

struct Collection;
struct Image;
struct Main;
struct Material;
struct ReportList;
struct Scene;
struct ViewLayer;

namespace blender::io::pmx {

/**
 * Per-sub-mesh information saved during split-mode mesh creation.
 * Used to correctly map vertex weights from the original model to sub-mesh vertices.
 */
struct SubMeshInfo {
  Object *obj;
  Vector<int> new_to_old; /**< Sub-mesh vertex index → original model vertex index. */
};

struct PMXTextureLoadResult {
  Image *image = nullptr;
  bool attempted = false;
};

struct PMXMaterialImportReport {
  int materials_created = 0;
  int loaded_textures = 0;
  int reused_textures = 0;
  int missing_textures = 0;
  int decode_failed_textures = 0;
  int empty_texture_paths = 0;
  int invalid_texture_indices = 0;
  int cached_failures = 0;
};

struct PMXImportContext {
  Main *bmain;
  Scene *scene;
  ViewLayer *view_layer;
  Object *mesh_obj;              /**< Primary mesh object (legacy). */
  Object *root_obj;              /**< Unified PMX model root Empty. */
  Collection *model_collection;   /**< Collection containing this PMX import. */
  Collection *geometry_collection; /**< Child collection for imported Mesh objects. */
  Collection *controls_collection; /**< Child collection for Morph Controller objects. */
  Object *armature_obj;           /**< Created armature object, set by create_armature_object(). */
  Object *morph_controller_obj;   /**< Shape-key controller object for vertex morphs. */
  Vector<Object *> mesh_objects; /**< All mesh objects created in this import. */
  Vector<SubMeshInfo> sub_meshes; /**< Vertex mapping for split mode (empty in single mode). */
  Vector<std::string> morph_names; /**< PMX vertex morph index order → Blender shape key name. */
  Vector<int> morph_indices; /**< PMX morph indices corresponding to morph_names. */
  /** C2-2 raw Controller registry; Vertex-only arrays above keep their legacy meaning. */
  PMXGroupMorphReport group_morph_report;
  /** C2-2D expanded expressions consumed by Geometry Driver creation. */
  PMXGroupMorphReport group_morph_expression_report;
  const PMXImportParams *params;
  ReportList *reports;
  const std::vector<PMXTexture> *model_textures;
  Map<std::string, PMXTextureLoadResult> texture_cache;
  Map<int, Material *> material_cache;
  PMXMaterialImportReport material_report;
};

/**
 * Create mesh objects and materials from a parsed PMXModel.
 * Populates ctx.mesh_obj, ctx.mesh_objects, and ctx.sub_meshes (split mode only).
 * Parents all created Mesh objects to ctx.root_obj.
 * Does NOT assign vertex weights — that is done after armature creation.
 */
void create_mesh_object(PMXImportContext &ctx, PMXModel &model);

/** Create the unified Empty root for one imported PMX model. */
Object *create_model_root(PMXImportContext &ctx, const PMXModel &model);

/**
 * Transform PMX coordinates (Y-up, cm) to Blender coordinates (Z-up, m).
 * Mapped as: Blender X=PMX X, Blender Y=PMX Z, Blender Z=PMX Y.
 */
void transform_coord(float out[3], const float in[3], float scale);

}  // namespace blender::io::pmx
