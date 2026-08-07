/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "BLI_path_utils.hh"

namespace blender {

struct bContext;
struct Object;

struct PMXImportParams {
  /** Full path to the PMX file being imported. */
  char filepath[FILE_MAX] = "";
  /** Scale factor applied to the imported data.
   * Aligned with mmd_tools default (operators/fileio.py: ImportPmx.scale=0.08)
   * to ensure Bullet world body transforms match MikuMikuPhysics reference. */
  float global_scale = 0.08f;
  bool split_by_material = true;
  /** Output: armature object created by the import (set by importer_main). */
  Object *result_armature = nullptr;
};

void PMX_import(bContext *C, PMXImportParams &params);

}  // namespace blender
