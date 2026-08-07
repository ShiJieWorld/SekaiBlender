/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include "pmx_import_mesh.hh"

namespace blender::io::pmx {

/** Create the shared vertex morph control mesh and connect mesh shape keys to it. */
void create_morph_controller(PMXImportContext &ctx);

}  // namespace blender::io::pmx
