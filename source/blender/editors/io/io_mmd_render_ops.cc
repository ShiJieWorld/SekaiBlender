/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors/io
 *
 * MMD render features for imported PMX models, plus their own "MMD Render"
 * N-panel tab in the View3D sidebar (kept separate from "MMD Physics").
 *
 * Currently this provides the PMX toon-edge (outline) preview, rebuilt to match
 * `mmd_tools` (operators/material.py: `EdgePreviewSetup`) exactly:
 *
 * - The original PMX material is never modified. Edge geometry is a *separate*
 *   material appended after all original slots, plus one `Solidify` modifier
 *   whose `material_offset` shifts generated back-faces into those new slots.
 * - Edge width comes from `thickness * vertex_group_weight`, where the weight is
 *   `mmd_edge_scale * material_edge_weight * 0.02` (PMX per-vertex edge scale
 *   times the PMX per-material edge size).
 * - The edge material's shader is the `MMDEdgePreview` node group: a
 *   Light Path / Backfacing test that keeps the flipped shell visible only from
 *   camera rays, mixing a Transparent BSDF with a Background of the edge color.
 */

#include "io_mmd_render_ops.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_deform.hh"
#include "BKE_idprop.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_modifier.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_object.hh"
#include "BKE_object_deform.h"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "BLI_index_range.hh"
#include "BLI_listbase.hh"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "importer/pmx_import_material.hh"

namespace blender {

namespace {

using io::pmx::kMMDEdgeMaterialPrefix;
using io::pmx::kMMDEdgePreviewName;
using io::pmx::kMMDEdgePreviewNodeGroup;
using io::pmx::kPMXEdgeScaleGroup;
using io::pmx::read_pmx_material_edge_data;

/* mmd_tools multiplies the PMX edge size by this constant before using it as a
 * Solidify vertex-group weight (operators/material.py:
 * `weight = scale * weight * 0.02`). */
constexpr float kEdgeWeightScale = 0.02f;
/* mmd_tools keeps a small non-zero base thickness so the shell never collapses
 * onto the original faces (`mod.thickness_vertex_group = 1e-3`). */
constexpr float kEdgeThicknessVertexGroupBase = 1e-3f;
/* mmd_tools derives Solidify thickness from the model root Empty display size
 * (`scale = 0.2 * empty_display_size`). */
constexpr float kEdgeThicknessFromRootScale = 0.2f;
/* PMX import default scale, used when a model root Empty is unavailable. */
constexpr float kEdgeThicknessFallback = 0.08f;
/* Use a neutral, broadly useful outline color for the preview. PMX edge color
 * is still parsed as metadata, but does not override this default appearance. */
constexpr std::array<float, 4> kDefaultEdgeColor = {0.0f, 0.0f, 0.0f, 1.0f};

/* ----------------------------------------------------------------- */
/* Panel language. Shared with the MMD Physics panel so both sidebar  */
/* tabs follow one setting.                                          */
/* ----------------------------------------------------------------- */

constexpr const char *kPanelLanguageProperty = "mmd_physics_panel_language";

enum class MMDRenderPanelLanguage : int {
  Chinese = 0,
  English = 1,
  Japanese = 2,
};

static const EnumPropertyItem mmd_render_panel_language_items[] = {
    {int(MMDRenderPanelLanguage::Chinese), "CHINESE", 0, "中文", "使用中文界面"},
    {int(MMDRenderPanelLanguage::English), "ENGLISH", 0, "English", "Use the English interface"},
    {int(MMDRenderPanelLanguage::Japanese), "JAPANESE", 0, "日本語", "日本語のインターフェースを使用"},
    {0, nullptr, 0, nullptr, nullptr},
};

struct MMDRenderPanelText {
  const char *language;
  const char *toon_edge;
  const char *create_edge;
  const char *clean_edge;
  const char *status_format;
  const char *no_model;
};

const MMDRenderPanelText &mmd_render_panel_text(const MMDRenderPanelLanguage language)
{
  static const MMDRenderPanelText chinese = {
      "语言",
      "描边预览",
      "生成描边",
      "清除描边",
      "可描边网格: %d / %d",
      "请选择一个 PMX 模型",
  };
  static const MMDRenderPanelText english = {
      "Language",
      "Toon Edge Preview",
      "Create Edge",
      "Clean Edge",
      "Edge-capable meshes: %d / %d",
      "Select a PMX model",
  };
  static const MMDRenderPanelText japanese = {
      "言語",
      "輪郭プレビュー",
      "輪郭を作成",
      "輪郭を削除",
      "輪郭可能メッシュ: %d / %d",
      "PMX モデルを選択してください",
  };
  switch (language) {
    case MMDRenderPanelLanguage::English:
      return english;
    case MMDRenderPanelLanguage::Japanese:
      return japanese;
    case MMDRenderPanelLanguage::Chinese:
      break;
  }
  return chinese;
}

MMDRenderPanelLanguage scene_mmd_render_panel_language(Scene *scene)
{
  if (scene != nullptr) {
    if (IDProperty *props = IDP_GetProperties(&scene->id)) {
      if (IDProperty *prop = IDP_GetPropertyTypeFromGroup(props, kPanelLanguageProperty, IDP_INT)) {
        return MMDRenderPanelLanguage(std::clamp(IDP_int_get(prop), 0, 2));
      }
    }
  }
  return MMDRenderPanelLanguage::Chinese;
}

void scene_mmd_render_panel_language_set(Scene *scene, const MMDRenderPanelLanguage language)
{
  if (scene == nullptr) {
    return;
  }
  IDProperty *props = IDP_EnsureProperties(&scene->id);
  const int value = std::clamp(int(language), 0, 2);
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(props, kPanelLanguageProperty, IDP_INT);
  if (prop != nullptr) {
    IDP_int_set(prop, value);
  }
  else {
    IDP_AddToGroup(props, IDP_NewInt(value, kPanelLanguageProperty));
  }
}

wmOperatorStatus mmd_render_set_panel_language_exec(bContext *C, wmOperator *op)
{
  scene_mmd_render_panel_language_set(
      CTX_data_scene(C), MMDRenderPanelLanguage(RNA_enum_get(op->ptr, "language")));
  return OPERATOR_FINISHED;
}

/* ----------------------------------------------------------------- */
/* Model resolution.                                                 */
/* ----------------------------------------------------------------- */

/** Walk up the parent chain to the top-most ancestor of `object`. */
Object *model_root_of(Object *object)
{
  if (object == nullptr) {
    return nullptr;
  }
  Object *root = object;
  /* Bound the walk so a corrupt parent cycle cannot hang the UI. */
  for (int step = 0; step < 64 && root->parent != nullptr; step++) {
    root = root->parent;
  }
  return root;
}

/**
 * True when the mesh carries PMX material metadata. This is what separates
 * imported PMX geometry from helper or hand-made meshes in the same hierarchy.
 */
bool mesh_has_pmx_edge_data(Object *object)
{
  if (object == nullptr || object->type != OB_MESH) {
    return false;
  }
  for (const int slot : IndexRange(object->totcol)) {
    Material *material = BKE_object_material_get(object, short(slot + 1));
    if (material == nullptr) {
      continue;
    }
    bool enabled = false;
    std::array<float, 4> color{};
    float weight = 1.0f;
    if (read_pmx_material_edge_data(*material, enabled, color, weight)) {
      return true;
    }
  }
  return false;
}

/** Collect every PMX mesh that shares a model root with `active`. */
Vector<Object *> collect_model_meshes(Main *bmain, Object *active)
{
  Vector<Object *> meshes;
  Object *root = model_root_of(active);
  if (bmain == nullptr || root == nullptr) {
    return meshes;
  }
  for (Object &object : bmain->objects) {
    if (object.type != OB_MESH) {
      continue;
    }
    if (model_root_of(&object) != root) {
      continue;
    }
    if (!mesh_has_pmx_edge_data(&object)) {
      continue;
    }
    meshes.append(&object);
  }
  return meshes;
}

/** Solidify thickness for one model, matching mmd_tools' root-Empty convention. */
float model_edge_thickness(Object *root)
{
  if (root != nullptr && root->type == OB_EMPTY && std::isfinite(root->empty_drawsize) &&
      root->empty_drawsize > 0.0f)
  {
    return kEdgeThicknessFromRootScale * root->empty_drawsize;
  }
  return kEdgeThicknessFallback;
}

/* ----------------------------------------------------------------- */
/* MMDEdgePreview node group.                                        */
/* ----------------------------------------------------------------- */

bNode *add_shader_node(bNodeTree &ntree, const int type, const float x, const float y)
{
  bNode *node = bke::node_add_static_node(nullptr, ntree, type);
  node->location[0] = x;
  node->location[1] = y;
  return node;
}

bNode *add_math_node(
    bNodeTree &ntree, const int operation, const float x, const float y, const bool muted)
{
  bNode *node = add_shader_node(ntree, SH_NODE_MATH, x, y);
  node->custom1 = operation;
  if (muted) {
    node->flag |= NODE_MUTED;
  }
  return node;
}

/**
 * Build (or reuse) the `MMDEdgePreview` shader node group.
 *
 * The topology mirrors mmd_tools' `__get_edge_preview_shader`, including the two
 * intentionally muted nodes: the `MAXIMUM` math node (so only "Is Camera Ray"
 * reaches the comparison) and the legacy `Mix` node (so `Color` passes straight
 * through to the Background shader).
 */
bNodeTree *ensure_edge_preview_node_group(Main *bmain)
{
  if (bmain == nullptr) {
    return nullptr;
  }
  if (ID *existing = BKE_libblock_find_name(bmain, ID_NT, kMMDEdgePreviewNodeGroup)) {
    bNodeTree *group = reinterpret_cast<bNodeTree *>(existing);
    /* Reuse a previously built group, exactly like mmd_tools does. */
    if (group->nodes.first != nullptr) {
      return group;
    }
  }

  bNodeTree *group = bke::node_tree_add_tree(bmain, kMMDEdgePreviewNodeGroup, "ShaderNodeTree");
  if (group == nullptr) {
    return nullptr;
  }

  /* mmd_tools lays nodes out on a (210, 220) grid. */
  const auto grid_x = [](const float x) { return x * 210.0f; };
  const auto grid_y = [](const float y) { return y * 220.0f; };

  /* Interface sockets, in mmd_tools' creation order: Color, Alpha, then Shader. */
  bNodeTreeInterfaceSocket *io_color = group->tree_interface.add_socket(
      "Color", "", "NodeSocketColor", NODE_INTERFACE_SOCKET_INPUT, nullptr);
  bNodeTreeInterfaceSocket *io_alpha = group->tree_interface.add_socket(
      "Alpha", "", "NodeSocketFloat", NODE_INTERFACE_SOCKET_INPUT, nullptr);
  bNodeTreeInterfaceSocket *io_shader = group->tree_interface.add_socket(
      "Shader", "", "NodeSocketShader", NODE_INTERFACE_SOCKET_OUTPUT, nullptr);
  if (io_color == nullptr || io_alpha == nullptr || io_shader == nullptr) {
    return nullptr;
  }
  /* mmd_tools clamps any socket whose name ends with "Alpha" to [0, 1]. */
  if (auto *alpha_data = static_cast<bNodeSocketValueFloat *>(io_alpha->socket_data)) {
    alpha_data->min = 0.0f;
    alpha_data->max = 1.0f;
  }

  bNode *group_input = bke::node_add_node(nullptr, *group, "NodeGroupInput"_ustr);
  bNode *group_output = bke::node_add_node(nullptr, *group, "NodeGroupOutput"_ustr);
  group_input->location[0] = grid_x(-5.0f);
  group_input->location[1] = grid_y(0.0f);
  group_output->location[0] = grid_x(3.0f);
  group_output->location[1] = grid_y(0.0f);

  /* Muted legacy Mix: passes Color1 through unchanged. */
  bNode *color_mix = add_shader_node(*group, SH_NODE_MIX_RGB_LEGACY, grid_x(-1.0f), grid_y(-1.5f));
  color_mix->flag |= NODE_MUTED;

  bNode *light_path = add_shader_node(*group, SH_NODE_LIGHT_PATH, grid_x(-3.0f), grid_y(1.5f));
  bNode *geometry = add_shader_node(*group, SH_NODE_NEW_GEOMETRY, grid_x(-3.0f), grid_y(0.0f));
  bNode *ray_max = add_math_node(*group, NODE_MATH_MAXIMUM, grid_x(-2.0f), grid_y(1.5f), true);
  bNode *front_test = add_math_node(
      *group, NODE_MATH_GREATER_THAN, grid_x(-1.0f), grid_y(1.0f), false);
  bNode *alpha_mul = add_math_node(
      *group, NODE_MATH_MULTIPLY, grid_x(0.0f), grid_y(1.0f), false);
  bNode *transparent = add_shader_node(
      *group, SH_NODE_BSDF_TRANSPARENT, grid_x(0.0f), grid_y(0.0f));
  bNode *background = add_shader_node(*group, SH_NODE_BACKGROUND, grid_x(0.0f), grid_y(-0.5f));
  bNode *mix_shader = add_shader_node(*group, SH_NODE_MIX_SHADER, grid_x(1.0f), grid_y(0.5f));

  /* Group interface sockets exist now; realize them on the I/O nodes. */
  BKE_main_ensure_invariants(*bmain, group->id);

  bNodeSocket *in_color = bke::node_find_socket(*group_input, SOCK_OUT, UString(io_color->identifier));
  bNodeSocket *in_alpha = bke::node_find_socket(*group_input, SOCK_OUT, UString(io_alpha->identifier));
  bNodeSocket *out_shader = bke::node_find_socket(
      *group_output, SOCK_IN, UString(io_shader->identifier));

  bNodeSocket *mix_color_1 = bke::node_find_socket(*color_mix, SOCK_IN, "Color1"_ustr);
  bNodeSocket *mix_color_out = bke::node_find_socket(*color_mix, SOCK_OUT, "Color"_ustr);
  bNodeSocket *camera_ray = bke::node_find_socket(*light_path, SOCK_OUT, "Is Camera Ray"_ustr);
  bNodeSocket *glossy_ray = bke::node_find_socket(*light_path, SOCK_OUT, "Is Glossy Ray"_ustr);
  bNodeSocket *backfacing = bke::node_find_socket(*geometry, SOCK_OUT, "Backfacing"_ustr);
  bNodeSocket *ray_max_a = bke::node_find_socket(*ray_max, SOCK_IN, "Value"_ustr);
  bNodeSocket *ray_max_b = bke::node_find_socket(*ray_max, SOCK_IN, "Value_001"_ustr);
  bNodeSocket *ray_max_out = bke::node_find_socket(*ray_max, SOCK_OUT, "Value"_ustr);
  bNodeSocket *front_a = bke::node_find_socket(*front_test, SOCK_IN, "Value"_ustr);
  bNodeSocket *front_b = bke::node_find_socket(*front_test, SOCK_IN, "Value_001"_ustr);
  bNodeSocket *front_out = bke::node_find_socket(*front_test, SOCK_OUT, "Value"_ustr);
  bNodeSocket *alpha_a = bke::node_find_socket(*alpha_mul, SOCK_IN, "Value"_ustr);
  bNodeSocket *alpha_b = bke::node_find_socket(*alpha_mul, SOCK_IN, "Value_001"_ustr);
  bNodeSocket *alpha_out = bke::node_find_socket(*alpha_mul, SOCK_OUT, "Value"_ustr);
  bNodeSocket *transparent_out = bke::node_find_socket(*transparent, SOCK_OUT, "BSDF"_ustr);
  bNodeSocket *background_color = bke::node_find_socket(*background, SOCK_IN, "Color"_ustr);
  bNodeSocket *background_out = bke::node_find_socket(*background, SOCK_OUT, "Background"_ustr);
  bNodeSocket *mix_fac = bke::node_find_socket(*mix_shader, SOCK_IN, "Fac"_ustr);
  bNodeSocket *mix_shader_1 = bke::node_find_socket(*mix_shader, SOCK_IN, "Shader"_ustr);
  bNodeSocket *mix_shader_2 = bke::node_find_socket(*mix_shader, SOCK_IN, "Shader_001"_ustr);
  bNodeSocket *mix_shader_out = bke::node_find_socket(*mix_shader, SOCK_OUT, "Shader"_ustr);

  const bNodeSocket *required_sockets[] = {
      in_color,     in_alpha,   out_shader,       mix_color_1,      mix_color_out,
      camera_ray,   glossy_ray, backfacing,       ray_max_a,        ray_max_b,
      ray_max_out,  front_a,    front_b,          front_out,        alpha_a,
      alpha_b,      alpha_out,  transparent_out,  background_color, background_out,
      mix_fac,      mix_shader_1, mix_shader_2,   mix_shader_out,
  };
  for (const bNodeSocket *socket : required_sockets) {
    if (socket == nullptr) {
      return nullptr;
    }
  }

  bke::node_add_link(*group, *group_input, *in_color, *color_mix, *mix_color_1);
  bke::node_add_link(*group, *light_path, *camera_ray, *ray_max, *ray_max_a);
  bke::node_add_link(*group, *light_path, *glossy_ray, *ray_max, *ray_max_b);
  bke::node_add_link(*group, *ray_max, *ray_max_out, *front_test, *front_a);
  bke::node_add_link(*group, *geometry, *backfacing, *front_test, *front_b);
  bke::node_add_link(*group, *front_test, *front_out, *alpha_mul, *alpha_a);
  bke::node_add_link(*group, *group_input, *in_alpha, *alpha_mul, *alpha_b);
  bke::node_add_link(*group, *alpha_mul, *alpha_out, *mix_shader, *mix_fac);
  bke::node_add_link(*group, *transparent, *transparent_out, *mix_shader, *mix_shader_1);
  bke::node_add_link(*group, *background, *background_out, *mix_shader, *mix_shader_2);
  bke::node_add_link(*group, *color_mix, *mix_color_out, *background, *background_color);
  bke::node_add_link(*group, *mix_shader, *mix_shader_out, *group_output, *out_shader);

  BKE_main_ensure_invariants(*bmain, group->id);
  return group;
}

/* ----------------------------------------------------------------- */
/* Edge material.                                                    */
/* ----------------------------------------------------------------- */

void clear_node_tree(Main *bmain, bNodeTree &ntree)
{
  while (bNode *node = static_cast<bNode *>(ntree.nodes.first)) {
    bke::node_remove_node(bmain, ntree, *node, true);
  }
}

/**
 * Build the edge material's shader: one `MMDEdgePreview` group node driving the
 * Material Output. Mirrors mmd_tools' `__make_shader`.
 */
void build_edge_material_shader(Main *bmain,
                                Material &material,
                                bNodeTree *group,
                                const std::array<float, 4> &edge_color)
{
  bNodeTree *ntree = material.nodetree;
  if (ntree == nullptr || group == nullptr) {
    return;
  }
  clear_node_tree(bmain, *ntree);

  bNode *group_node = bke::node_add_node(nullptr, *ntree, "ShaderNodeGroup"_ustr);
  if (group_node == nullptr) {
    return;
  }
  STRNCPY_UTF8(group_node->name, kMMDEdgePreviewName);
  group_node->location[0] = 0.0f;
  group_node->location[1] = 0.0f;
  group_node->width = 200.0f;
  group_node->id = &group->id;
  id_us_plus(&group->id);

  bNode *output = add_shader_node(*ntree, SH_NODE_OUTPUT_MATERIAL, 420.0f, 0.0f);
  STRNCPY_UTF8(output->name, "Material Output");

  /* A group node's sockets are generated from the group interface, but only when
   * its declaration is rebuilt. Assigning `node->id` directly does not tag that,
   * so mirror what RNA does (`rna_NodeGroup_update`): tag the node property and
   * update the *node tree* ID, not the owning Material. Without this the group
   * node stays socket-less and the edge material has no surface shader. */
  BKE_ntree_update_tag_node_property(ntree, group_node);
  BKE_main_ensure_invariants(*bmain, ntree->id);

  bNodeSocket *shader_out = bke::node_find_enabled_output_socket(*group_node, "Shader");
  bNodeSocket *surface = bke::node_find_socket(*output, SOCK_IN, "Surface"_ustr);
  if (shader_out != nullptr && surface != nullptr) {
    bke::node_add_link(*ntree, *group_node, *shader_out, *output, *surface);
  }
  bke::node_set_active(*ntree, *output);

  if (bNodeSocket *color = bke::node_find_enabled_input_socket(*group_node, "Color")) {
    bNodeSocketValueRGBA *value = color->default_value_typed<bNodeSocketValueRGBA>();
    for (const int channel : IndexRange(4)) {
      value->value[channel] = edge_color[channel];
    }
  }
  if (bNodeSocket *alpha = bke::node_find_enabled_input_socket(*group_node, "Alpha")) {
    alpha->default_value_typed<bNodeSocketValueFloat>()->value = edge_color[3];
  }

  BKE_main_ensure_invariants(*bmain, ntree->id);
}

/** Create or update one `mmd_edge.*` material for a given edge color. */
Material *ensure_edge_material(Main *bmain,
                               const char *name,
                               const std::array<float, 4> &edge_color)
{
  bNodeTree *group = ensure_edge_preview_node_group(bmain);
  if (group == nullptr) {
    return nullptr;
  }

  Material *material = reinterpret_cast<Material *>(BKE_libblock_find_name(bmain, ID_MA, name));
  if (material == nullptr) {
    material = BKE_material_add(bmain, name);
  }
  if (material == nullptr) {
    return nullptr;
  }

  /* Keep the solid-mode viewport color meaningful; the rendered result comes
   * from the node group above. */
  material->r = edge_color[0];
  material->g = edge_color[1];
  material->b = edge_color[2];
  material->a = edge_color[3];

  /* mmd_tools: edges are single-sided (`is_double_sided = False`) and use the
   * dithered/hashed transparency path. MA_SURFACE_METHOD_DEFERRED is this DNA's
   * name for the "Dithered" render method. */
  material->blend_method = MA_BM_HASHED;
  material->surface_render_method = MA_SURFACE_METHOD_DEFERRED;
  material->blend_flag |= MA_BL_CULL_BACKFACE;

  build_edge_material_shader(bmain, *material, group, edge_color);
  return material;
}

/* ----------------------------------------------------------------- */
/* Per-mesh edge setup.                                              */
/* ----------------------------------------------------------------- */

bool material_name_is_edge(const Material *material)
{
  return material != nullptr && STRPREFIX(material->id.name + 2, kMMDEdgeMaterialPrefix);
}

/** Remove the Solidify modifier, preview vertex group, and `mmd_edge.*` slots. */
void clean_toon_edge(Main *bmain, Object *object)
{
  if (object == nullptr || object->type != OB_MESH) {
    return;
  }

  if (ModifierData *md = BKE_modifiers_findby_name(object, kMMDEdgePreviewName)) {
    BKE_modifier_remove_from_list(object, md);
    BKE_modifier_free(md);
  }

  if (bDeformGroup *group = BKE_object_defgroup_find_name(object, kMMDEdgePreviewName)) {
    BKE_object_defgroup_remove(object, group);
  }

  /* Edge slots are always appended after the originals, so walking backwards
   * removes exactly the generated slots and never touches PMX materials. Faces
   * only reference original indices, so no material-index remap is needed. */
  Mesh *mesh = reinterpret_cast<Mesh *>(object->data);
  if (mesh == nullptr) {
    return;
  }
  for (int slot = mesh->totcol - 1; slot >= 0; slot--) {
    if (!material_name_is_edge(mesh->mat[slot])) {
      continue;
    }
    BKE_id_material_pop(bmain, &mesh->id, slot);
  }
  BKE_object_material_active_index_sanitize(object);
}

/**
 * Fill the `mmd_edge_preview` vertex group.
 *
 * mmd_tools computes, per vertex, `mmd_edge_scale * material_edge_weight * 0.02`
 * where the material is taken from the *first* face using that vertex (its
 * reversed-polygon iteration leaves the lowest face index as the final write).
 */
void build_edge_preview_group(Object *object, const Span<float> material_edge_weights)
{
  Mesh *mesh = reinterpret_cast<Mesh *>(object->data);
  if (mesh == nullptr || mesh->verts_num == 0) {
    return;
  }

  /* Read the PMX per-vertex edge scale saved at import time. */
  Vector<float> edge_scale(mesh->verts_num, 1.0f);
  const int scale_group = BKE_object_defgroup_name_index(object, kPMXEdgeScaleGroup);
  if (scale_group >= 0) {
    const Span<MDeformVert> dverts = mesh->deform_verts();
    if (!dverts.is_empty()) {
      for (const int vi : IndexRange(mesh->verts_num)) {
        if (const MDeformWeight *dw = BKE_defvert_find_index(&dverts[vi], scale_group)) {
          edge_scale[vi] = dw->weight;
        }
        else {
          /* mmd_tools' `scale_map.get(i, 1.0)`: vertices outside the group
           * fall back to an unscaled edge. */
          edge_scale[vi] = 1.0f;
        }
      }
    }
  }

  /* Material of the first face using each vertex. */
  Vector<int> vert_material(mesh->verts_num, -1);
  const OffsetIndices<int> faces = mesh->faces();
  const Span<int> corner_verts = mesh->corner_verts();
  const bke::AttributeAccessor attributes = mesh->attributes();
  const VArray<int> material_indices = *attributes.lookup_or_default<int>(
      "material_index", bke::AttrDomain::Face, 0);
  for (const int face : faces.index_range()) {
    const int material_index = material_indices[face];
    for (const int vert : corner_verts.slice(faces[face])) {
      if (vert >= 0 && vert < mesh->verts_num && vert_material[vert] == -1) {
        vert_material[vert] = material_index;
      }
    }
  }

  if (bDeformGroup *existing = BKE_object_defgroup_find_name(object, kMMDEdgePreviewName)) {
    BKE_object_defgroup_remove(object, existing);
  }
  BKE_object_defgroup_new(object, kMMDEdgePreviewName);
  const int preview_group = BKE_object_defgroup_name_index(object, kMMDEdgePreviewName);
  if (preview_group < 0) {
    return;
  }

  MutableSpan<MDeformVert> dverts = mesh->deform_verts_for_write();
  if (dverts.is_empty()) {
    return;
  }
  for (const int vi : IndexRange(mesh->verts_num)) {
    const int material_index = vert_material[vi];
    const float edge_weight = (material_index >= 0 &&
                               material_index < int(material_edge_weights.size())) ?
                                  material_edge_weights[material_index] :
                                  1.0f;
    const float weight = std::clamp(
        edge_scale[vi] * edge_weight * kEdgeWeightScale, 0.0f, 1.0f);
    if (MDeformWeight *dw = BKE_defvert_ensure_index(&dverts[vi], preview_group)) {
      dw->weight = weight;
    }
  }
}

/** Append or update the `mmd_edge_preview` Solidify modifier. */
void ensure_edge_solidify_modifier(Object *object,
                                   const int material_offset,
                                   const float thickness)
{
  ModifierData *md = BKE_modifiers_findby_name(object, kMMDEdgePreviewName);
  if (md != nullptr && md->type != eModifierType_Solidify) {
    BKE_modifier_remove_from_list(object, md);
    BKE_modifier_free(md);
    md = nullptr;
  }
  if (md == nullptr) {
    md = BKE_modifier_new(eModifierType_Solidify);
    if (md == nullptr) {
      return;
    }
    /* mmd_tools adds the Solidify after the Armature modifier, which is what
     * `add_at_end` reproduces for an imported PMX mesh. */
    BKE_modifiers_add_at_end_if_possible(object, md);
    BKE_modifiers_persistent_uid_init(*object, *md);
    STRNCPY_UTF8(md->name, kMMDEdgePreviewName);
    BKE_modifier_unique_name(&object->modifiers, md);
  }

  SolidifyModifierData *smd = reinterpret_cast<SolidifyModifierData *>(md);
  smd->offset = thickness;                              /* RNA "thickness" */
  smd->offset_fac = 1.0f;                               /* RNA "offset" */
  smd->offset_fac_vg = kEdgeThicknessVertexGroupBase;   /* RNA "thickness_vertex_group" */
  smd->mat_ofs = short(material_offset);
  STRNCPY_UTF8(smd->defgrp_name, kMMDEdgePreviewName);
  /* Flip the shell inside-out and skip rim geometry, matching mmd_tools. */
  smd->flag = (smd->flag & ~MOD_SOLIDIFY_RIM) | MOD_SOLIDIFY_FLIP;
}

/**
 * Create the toon edge for one mesh.
 * \return Number of edge material slots appended (0 when nothing was created).
 */
int create_toon_edge(Main *bmain, Object *object, const float thickness)
{
  clean_toon_edge(bmain, object);

  Mesh *mesh = reinterpret_cast<Mesh *>(object->data);
  if (mesh == nullptr) {
    return 0;
  }
  const int material_offset = mesh->totcol;
  if (material_offset == 0) {
    return 0;
  }

  /* PMX edge size per original material index, used for the vertex group. */
  Vector<float> material_edge_weights(material_offset, 1.0f);
  Vector<Material *> edge_materials;
  edge_materials.reserve(material_offset);

  for (const int slot : IndexRange(material_offset)) {
    Material *material = mesh->mat[slot];
    bool enabled = false;
    std::array<float, 4> pmx_edge_color{0.0f, 0.0f, 0.0f, 0.0f};
    float edge_weight = 1.0f;
    const bool has_pmx_data = material != nullptr &&
                              read_pmx_material_edge_data(
                                  *material, enabled, pmx_edge_color, edge_weight);
    material_edge_weights[slot] = has_pmx_data ? edge_weight : 1.0f;

    if (material != nullptr && has_pmx_data && enabled) {
      char edge_name[MAX_ID_NAME - 2];
      SNPRINTF_UTF8(edge_name, "%s%s", kMMDEdgeMaterialPrefix, material->id.name + 2);
      edge_materials.append(ensure_edge_material(bmain, edge_name, kDefaultEdgeColor));
    }
    else if (material_offset > 1) {
      /* mmd_tools only pads with a fully transparent "disabled" material when
       * the mesh has more than one slot, so material_offset stays aligned. */
      char edge_name[MAX_ID_NAME - 2];
      SNPRINTF_UTF8(edge_name, "%sdisabled", kMMDEdgeMaterialPrefix);
      edge_materials.append(ensure_edge_material(bmain, edge_name, {0.0f, 0.0f, 0.0f, 0.0f}));
    }
  }

  if (edge_materials.is_empty()) {
    return 0;
  }
  for (Material *edge_material : edge_materials) {
    /* A null entry would still have to occupy a slot to keep material_offset
     * aligned with the original slots. */
    BKE_id_material_append(bmain, &mesh->id, edge_material);
  }

  build_edge_preview_group(object, material_edge_weights);
  ensure_edge_solidify_modifier(object, material_offset, thickness);

  DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
  return int(edge_materials.size());
}

/* ----------------------------------------------------------------- */
/* Operators.                                                        */
/* ----------------------------------------------------------------- */

enum class EdgePreviewAction : int {
  Create = 0,
  Clean = 1,
};

const EnumPropertyItem mmd_edge_preview_action_items[] = {
    {int(EdgePreviewAction::Create), "CREATE", 0, "Create", "Create the toon edge preview"},
    {int(EdgePreviewAction::Clean), "CLEAN", 0, "Clean", "Remove the toon edge preview"},
    {0, nullptr, 0, nullptr, nullptr},
};

bool poll_mmd_model(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  return object != nullptr && ELEM(object->type, OB_MESH, OB_EMPTY, OB_ARMATURE);
}

wmOperatorStatus mmd_edge_preview_setup_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *active = CTX_data_active_object(C);
  if (bmain == nullptr || active == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Render: select a PMX model first");
    return OPERATOR_CANCELLED;
  }

  const Vector<Object *> meshes = collect_model_meshes(bmain, active);
  if (meshes.is_empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Render: no PMX mesh found for the active model");
    return OPERATOR_CANCELLED;
  }

  const EdgePreviewAction action = EdgePreviewAction(RNA_enum_get(op->ptr, "action"));
  if (action == EdgePreviewAction::Clean) {
    for (Object *object : meshes) {
      clean_toon_edge(bmain, object);
      DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
    }
    DEG_relations_tag_update(bmain);
    WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
    WM_event_add_notifier(C, NC_MATERIAL | ND_SHADING, nullptr);
    BKE_reportf(op->reports,
                RPT_INFO,
                "MMD Render: cleared the toon edge on %d mesh(es)",
                int(meshes.size()));
    return OPERATOR_FINISHED;
  }

  float thickness = RNA_float_get(op->ptr, "thickness");
  if (!(thickness > 0.0f) || !std::isfinite(thickness)) {
    thickness = model_edge_thickness(model_root_of(active));
  }

  int created = 0;
  for (Object *object : meshes) {
    created += create_toon_edge(bmain, object, thickness);
  }
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  WM_event_add_notifier(C, NC_MATERIAL | ND_SHADING, nullptr);

  if (created == 0) {
    BKE_report(op->reports,
               RPT_WARNING,
               "MMD Render: no material has the PMX toon-edge flag enabled");
    return OPERATOR_CANCELLED;
  }
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Render: created %d toon edge(s) on %d mesh(es), thickness %.4f",
              created,
              int(meshes.size()),
              thickness);
  return OPERATOR_FINISHED;
}

/* ----------------------------------------------------------------- */
/* N-panel (sidebar) UI.                                             */
/* ----------------------------------------------------------------- */

bool mmd_render_panel_poll(const bContext *C, PanelType * /*pt*/)
{
  Object *object = CTX_data_active_object(C);
  return object != nullptr && ELEM(object->type, OB_MESH, OB_EMPTY, OB_ARMATURE);
}

void mmd_render_panel_draw(const bContext *C, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  Main *bmain = CTX_data_main(const_cast<bContext *>(C));
  Object *active = CTX_data_active_object(const_cast<bContext *>(C));
  Scene *scene = CTX_data_scene(const_cast<bContext *>(C));
  const MMDRenderPanelLanguage language = scene_mmd_render_panel_language(scene);
  const MMDRenderPanelText &text = mmd_render_panel_text(language);

  const char *language_name = mmd_render_panel_language_items[int(language)].name;
  layout.op_menu_enum(C,
                      "WM_OT_mmd_render_set_panel_language",
                      "language",
                      std::string(text.language) + ": " + language_name,
                      ICON_WORLD);
  layout.separator();

  const Vector<Object *> meshes = collect_model_meshes(bmain, active);
  if (meshes.is_empty()) {
    layout.label(text.no_model, ICON_INFO);
    return;
  }

  int edge_capable = 0;
  for (Object *object : meshes) {
    Mesh *mesh = reinterpret_cast<Mesh *>(object->data);
    if (mesh == nullptr) {
      continue;
    }
    for (const int slot : IndexRange(mesh->totcol)) {
      Material *material = mesh->mat[slot];
      bool enabled = false;
      std::array<float, 4> color{};
      float weight = 1.0f;
      if (material != nullptr && read_pmx_material_edge_data(*material, enabled, color, weight) &&
          enabled)
      {
        edge_capable++;
        break;
      }
    }
  }

  if (ui::Layout *edge = layout.panel(C, "mmd_render_toon_edge", false, text.toon_edge)) {
    char status[64];
    SNPRINTF(status, text.status_format, edge_capable, int(meshes.size()));
    edge->label(status, ICON_MOD_SOLIDIFY);

    ui::Layout &column = edge->column(true);
    PointerRNA create_props = column.op(
        "WM_OT_mmd_edge_preview_setup", text.create_edge, ICON_MOD_SOLIDIFY);
    RNA_enum_set(&create_props, "action", int(EdgePreviewAction::Create));
    PointerRNA clean_props = column.op("WM_OT_mmd_edge_preview_setup", text.clean_edge, ICON_X);
    RNA_enum_set(&clean_props, "action", int(EdgePreviewAction::Clean));
  }
}

}  // namespace

void WM_OT_mmd_render_set_panel_language(wmOperatorType *ot)
{
  ot->name = "Set MMD Render Panel Language";
  ot->description = "Set the display language used by the MMD Render panel";
  ot->idname = "WM_OT_mmd_render_set_panel_language";
  ot->exec = mmd_render_set_panel_language_exec;
  ot->poll = poll_mmd_model;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  ot->prop = RNA_def_enum(ot->srna,
                          "language",
                          mmd_render_panel_language_items,
                          int(MMDRenderPanelLanguage::Chinese),
                          "Language",
                          "MMD Render panel display language");
}

void WM_OT_mmd_edge_preview_setup(wmOperatorType *ot)
{
  ot->name = "MMD Toon Edge Preview";
  ot->description =
      "Preview PMX toon edge settings of the active model using a Solidify modifier";
  ot->idname = "WM_OT_mmd_edge_preview_setup";
  ot->exec = mmd_edge_preview_setup_exec;
  ot->poll = poll_mmd_model;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  ot->prop = RNA_def_enum(ot->srna,
                          "action",
                          mmd_edge_preview_action_items,
                          int(EdgePreviewAction::Create),
                          "Action",
                          "Create or remove the toon edge preview");
  RNA_def_float(ot->srna,
                "thickness",
                0.0f,
                0.0f,
                10.0f,
                "Thickness",
                "Solidify thickness for the edge shell; 0 derives it from the model root scale",
                0.0f,
                1.0f);
}

void ED_mmd_render_panel_register(ARegionType *art)
{
  if (art == nullptr) {
    return;
  }
  PanelType *pt = MEM_new_zeroed<PanelType>("spacetype view3d panel mmd render");
  STRNCPY_UTF8(pt->idname, "VIEW3D_PT_mmd_render");
  STRNCPY_UTF8(pt->label, N_("MMD Render"));
  /* Own sidebar tab, deliberately separate from the "MMD" physics tab. */
  STRNCPY_UTF8(pt->category, "MMD Render");
  STRNCPY_UTF8(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  pt->draw = mmd_render_panel_draw;
  pt->poll = mmd_render_panel_poll;
  BLI_addtail(&art->paneltypes, pt);
}

}  // namespace blender
