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

struct Collection;
struct Main;
struct ViewLayer;

namespace blender::io::pmx {

struct PMXImportContext;

/**
 * Result of armature creation: the armature object and a PMX-index→Blender-bone-name mapping.
 */
struct PMXArmatureResult {
  Object *armature_obj = nullptr;
  Vector<std::string> bone_names; /**< PMX bone index → Blender bone name. */
};

/**
 * Create an Armature object with EditBones from PMX bone data.
 *
 * Steps performed:
 *  1. Create bArmature data and Armature Object, add to collection.
 *  2. Enter edit mode; create all EditBones with head/parent/tail.
 *  3. Apply coordinate transform via transform_coord().
 *  4. Fix zero-length bones using fixed_axis, child direction, parent, or Z fallback.
 *  5. Apply roll from PMX local_axis / fixed_axis.
 *  6. Commit edit mode; save PMX index → Blender bone name mapping.
 *
 * Does NOT bind Armature modifier to any mesh — that is done by bind_armature_modifiers().
 */
PMXArmatureResult create_armature_object(Main *bmain,
                                          const PMXModel &model,
                                          const PMXImportParams &params,
                                          const char *base_name,
                                          ViewLayer *view_layer,
                                          Object *root_obj,
                                          Collection *model_collection);

/**
 * Bind the armature object to all mesh objects collected in the import context.
 * Sets Armature modifier and keeps world transform unchanged.
 */
void bind_armature_modifiers(PMXImportContext &ctx);

}  // namespace blender::io::pmx
