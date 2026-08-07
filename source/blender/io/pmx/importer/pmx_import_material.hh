/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#pragma once

#include <array>

#include "intern/pmx_types.h"

struct Material;

namespace blender::io::pmx {

struct PMXImportContext;

/* PMX edge metadata persisted on each imported Material. These describe the
 * original PMX toon-edge settings and never affect the material's shader. */
inline constexpr char kPMXEdgeEnabled[] = "mmd_pmx_edge_enabled";
inline constexpr char kPMXEdgeColor[] = "mmd_pmx_edge_color";
inline constexpr char kPMXEdgeWeight[] = "mmd_pmx_edge_weight";

/* Names shared with the toon-edge preview built by the MMD Render panel. These
 * match mmd_tools so a model round-trips between both implementations. */
inline constexpr char kPMXEdgeScaleGroup[] = "mmd_edge_scale";
inline constexpr char kMMDEdgePreviewName[] = "mmd_edge_preview";
inline constexpr char kMMDEdgeMaterialPrefix[] = "mmd_edge.";
inline constexpr char kMMDEdgePreviewNodeGroup[] = "MMDEdgePreview";

/**
 * Create or retrieve one Material for a PMX material index in the current import.
 * The Material is shared by combined and split mesh modes, while Image reuse is
 * handled separately by the import context.
 */
Material *create_pmx_material(PMXImportContext &ctx, const PMXModel &model, int material_index);

/** Read PMX edge data persisted on an imported Material without touching its shader. */
bool read_pmx_material_edge_data(const Material &material,
                                 bool &r_enabled,
                                 std::array<float, 4> &r_color,
                                 float &r_weight);

/** Report the aggregated base-texture outcome for one PMX import. */
void report_pmx_material_import_summary(const PMXImportContext &ctx);

}  // namespace blender::io::pmx
