/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "intern/pmx_types.h"

#include "BLI_vector.hh"

#include <string>

namespace blender::io::pmx {

struct PMXImportContext;

/** Import PMX vertex morphs as relative Blender shape keys. */
Vector<std::string> import_vertex_morphs(PMXImportContext &ctx, const PMXModel &model);

}  // namespace blender::io::pmx
