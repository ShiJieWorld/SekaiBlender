/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "IO_pmx.hh"
#include "importer/pmx_import.hh"

namespace blender {

void PMX_import(bContext *C, PMXImportParams &params)
{
  io::pmx::importer_main(C, params);
}

}  // namespace blender
