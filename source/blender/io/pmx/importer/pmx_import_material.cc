/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include <algorithm>
#include <cctype>
#include <cmath>

#include "BKE_idprop.hh"
#include "BKE_image.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_report.hh"

#include "BLI_fileops.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector.hh"
#include "BLI_path_utils.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"

#include "DNA_material_types.h"
#include "DNA_node_types.h"

#include "IMB_imbuf_types.hh"

#include "NOD_shader.h"

#include "pmx_import_material.hh"
#include "pmx_import_mesh.hh"

namespace blender::io::pmx {
namespace {

static void replace_system_property(IDProperty *group, IDProperty *property)
{
  if (group == nullptr || property == nullptr) {
    return;
  }
  if (IDProperty *old = IDP_GetPropertyFromGroup_null(group, property->name)) {
    IDP_FreeFromGroup(group, old);
  }
  if (!IDP_AddToGroup(group, property)) {
    IDP_FreeProperty(property);
  }
}

static void write_pmx_edge_data(Material &material, const PMXMaterial &pmx_material)
{
  IDProperty *props = IDP_ID_system_properties_ensure(&material.id);
  const std::array<float, 4> edge_color = {pmx_material.edge_color[0],
                                           pmx_material.edge_color[1],
                                           pmx_material.edge_color[2],
                                           pmx_material.edge_color[3]};
  replace_system_property(
      props,
      bke::idprop::create_bool(kPMXEdgeEnabled,
                               (pmx_material.flag & PMX_MATERIAL_FLAG_EDGE) != 0)
          .release());
  replace_system_property(
      props,
      bke::idprop::create(kPMXEdgeColor, Span<float>(edge_color.data(), edge_color.size())).release());
  replace_system_property(props, bke::idprop::create(kPMXEdgeWeight, pmx_material.edge_size).release());
}

static bool path_is_empty_or_whitespace(const std::string &path)
{
  return std::all_of(path.begin(), path.end(), [](const unsigned char c) { return std::isspace(c); });
}

static bool resolve_texture_path(const PMXImportContext &ctx,
                                 const std::string &raw_path,
                                 char r_path[FILE_MAX])
{
  if (raw_path.empty() || path_is_empty_or_whitespace(raw_path) || raw_path.size() >= FILE_MAX) {
    return false;
  }

  BLI_strncpy(r_path, raw_path.c_str(), FILE_MAX);
  BLI_path_slash_native(r_path);

  if (!BLI_path_is_abs_from_cwd(r_path)) {
    char pmx_dir[FILE_MAX];
    BLI_path_split_dir_part(ctx.params->filepath, pmx_dir, sizeof(pmx_dir));
    if (pmx_dir[0] == '\0' || strlen(pmx_dir) + strlen(r_path) + 1 >= FILE_MAX) {
      return false;
    }
    BLI_path_append(pmx_dir, sizeof(pmx_dir), r_path);
    BLI_strncpy(r_path, pmx_dir, FILE_MAX);
  }

  BLI_path_normalize_native(r_path);
  return r_path[0] != '\0';
}

static Image *load_texture(PMXImportContext &ctx,
                           const PMXMaterial &pmx_material,
                           const int texture_index,
                           const char *texture_role)
{
  if (texture_index == -1) {
    return nullptr;
  }

  const char *material_name = pmx_material.name_local.empty() ? "PMXMaterial" :
                                                                  pmx_material.name_local.c_str();
  if (texture_index < -1 || ctx.model_textures == nullptr ||
      texture_index >= int(ctx.model_textures->size()))
  {
    ctx.material_report.invalid_texture_indices++;
    const size_t texture_count = ctx.model_textures ? ctx.model_textures->size() : 0;
    BKE_reportf(ctx.reports,
                RPT_WARNING,
                "PMX material '%s' has invalid %s texture index %d (texture count: %zu)",
                material_name,
                texture_role,
                texture_index,
                texture_count);
    return nullptr;
  }

  const std::string &raw_path = (*ctx.model_textures)[texture_index].path;
  char normalized_path[FILE_MAX];
  if (!resolve_texture_path(ctx, raw_path, normalized_path)) {
    ctx.material_report.empty_texture_paths++;
    BKE_reportf(ctx.reports,
                RPT_WARNING,
                "PMX material '%s' has an empty or unresolved %s texture path (index: %d)",
                material_name,
                texture_role,
                texture_index);
    return nullptr;
  }

  const std::string cache_key(normalized_path);
  if (const PMXTextureLoadResult *cached = ctx.texture_cache.lookup_ptr(cache_key)) {
    if (cached->image != nullptr) {
      ctx.material_report.reused_textures++;
      return cached->image;
    }
    ctx.material_report.cached_failures++;
    return nullptr;
  }

  PMXTextureLoadResult result;
  result.attempted = true;
  if (!BLI_exists(normalized_path)) {
    ctx.material_report.missing_textures++;
    BKE_reportf(ctx.reports,
                RPT_WARNING,
                "PMX material '%s' %s texture is missing: '%s' (resolved: '%s')",
                material_name,
                texture_role,
                raw_path.c_str(),
                normalized_path);
    ctx.texture_cache.add(cache_key, result);
    return nullptr;
  }

  result.image = BKE_image_load_exists(ctx.bmain, normalized_path);
  if (result.image == nullptr) {
    ctx.material_report.decode_failed_textures++;
    BKE_reportf(ctx.reports,
                RPT_WARNING,
                "PMX material '%s' could not load %s texture: '%s'",
                material_name,
                texture_role,
                normalized_path);
    ctx.texture_cache.add(cache_key, result);
    return nullptr;
  }

  ctx.material_report.loaded_textures++;
  ctx.texture_cache.add(cache_key, result);
  return result.image;
}

struct PMXImageAlphaInfo {
  bool has_alpha_channel = false;
};

static PMXImageAlphaInfo image_alpha_info(Image *image)
{
  PMXImageAlphaInfo result;
  if (image == nullptr) {
    return result;
  }

  void *lock = nullptr;
  ImBuf *ibuf = BKE_image_acquire_ibuf(image, nullptr, &lock);
  if (ibuf != nullptr) {
    result.has_alpha_channel = ibuf->can_contain_alpha();
    BKE_image_release_ibuf(image, ibuf, lock);
  }
  return result;
}

static bNode *add_node(bNodeTree &ntree, const int type, const float x, const float y)
{
  bNode *node = bke::node_add_static_node(nullptr, ntree, type);
  node->location[0] = x;
  node->location[1] = y;
  return node;
}

static void build_pmx_principled_tree(Material &material,
                                      const PMXMaterial &pmx_material,
                                      Image *base_texture,
                                      Image *sphere_texture)
{
  bNodeTree *ntree = material.nodetree;
  if (ntree == nullptr) {
    return;
  }

  bNode *principled = add_node(*ntree, SH_NODE_BSDF_PRINCIPLED, 280.0f, 0.0f);
  BLI_strncpy(principled->name, "Principled BSDF", sizeof(principled->name));
  BLI_strncpy(principled->label, "PMX Principled", sizeof(principled->label));
  bNodeSocket *base_color = bke::node_find_socket(*principled, SOCK_IN, "Base Color"_ustr);
  bNodeSocket *alpha = bke::node_find_socket(*principled, SOCK_IN, "Alpha"_ustr);
  BLI_assert(base_color != nullptr && alpha != nullptr);
  if (base_color == nullptr || alpha == nullptr) {
    return;
  }

  const PMXImageAlphaInfo base_alpha_info = image_alpha_info(base_texture);
  const PMXImageAlphaInfo sphere_alpha_info = image_alpha_info(sphere_texture);
  bNodeSocketValueRGBA *diffuse_value =
      base_color->default_value_typed<bNodeSocketValueRGBA>();
  for (int channel = 0; channel < 4; channel++) {
    diffuse_value->value[channel] = pmx_material.diffuse[channel];
  }
  alpha->default_value_typed<bNodeSocketValueFloat>()->value = pmx_material.diffuse[3];

  bNode *output = add_node(*ntree, SH_NODE_OUTPUT_MATERIAL, 540.0f, 0.0f);
  BLI_strncpy(output->name, "Material Output", sizeof(output->name));
  bNodeSocket *bsdf_output = bke::node_find_socket(*principled, SOCK_OUT, "BSDF"_ustr);
  bNodeSocket *surface_input = bke::node_find_socket(*output, SOCK_IN, "Surface"_ustr);
  BLI_assert(bsdf_output != nullptr && surface_input != nullptr);
  if (bsdf_output == nullptr || surface_input == nullptr) {
    return;
  }
  bke::node_add_link(*ntree, *principled, *bsdf_output, *output, *surface_input);
  bke::node_set_active(*ntree, *output);

  /* Match mmd_tools: alpha textures use the hashed/dithered path by default. Materials whose
   * faces overlap another PMX material are switched to forward blending after mesh creation. */
  material.blend_method = MA_BM_HASHED;
  material.surface_render_method = MA_SURFACE_METHOD_DEFERRED;

  bNode *base_image = nullptr;
  bNodeSocket *base_image_color = nullptr;
  bNodeSocket *base_image_alpha = nullptr;
  if (base_texture != nullptr) {
    base_image = add_node(*ntree, SH_NODE_TEX_IMAGE, -520.0f, 60.0f);
    BLI_strncpy(base_image->name, "PMX Base Texture", sizeof(base_image->name));
    BLI_strncpy(base_image->label, "PMX Base Texture", sizeof(base_image->label));
    base_image->id = &base_texture->id;
    base_image_color = bke::node_find_socket(*base_image, SOCK_OUT, "Color"_ustr);
    base_image_alpha = bke::node_find_socket(*base_image, SOCK_OUT, "Alpha"_ustr);
    BLI_assert(base_image_color != nullptr && base_image_alpha != nullptr);
  }

  bNodeSocket *alpha_source = nullptr;
  bNode *alpha_source_owner = nullptr;
  if (base_image_color != nullptr && sphere_texture == nullptr) {
    bke::node_add_link(*ntree, *base_image, *base_image_color, *principled, *base_color);
  }

  if (base_image_alpha != nullptr && base_alpha_info.has_alpha_channel) {
    if (pmx_material.diffuse[3] < 1.0f) {
      bNode *base_alpha = add_node(*ntree, SH_NODE_MATH, -40.0f, -260.0f);
      BLI_strncpy(base_alpha->name, "PMX Base Alpha", sizeof(base_alpha->name));
      base_alpha->custom1 = NODE_MATH_MULTIPLY;
      bNodeSocket *factor = bke::node_find_socket(*base_alpha, SOCK_IN, "Value"_ustr);
      bNodeSocket *texture_alpha = bke::node_find_socket(
          *base_alpha, SOCK_IN, "Value_001"_ustr);
      bNodeSocket *result = bke::node_find_socket(*base_alpha, SOCK_OUT, "Value"_ustr);
      BLI_assert(factor != nullptr && texture_alpha != nullptr && result != nullptr);
      if (factor != nullptr && texture_alpha != nullptr && result != nullptr) {
        factor->default_value_typed<bNodeSocketValueFloat>()->value = pmx_material.diffuse[3];
        bke::node_add_link(*ntree, *base_image, *base_image_alpha, *base_alpha, *texture_alpha);
        alpha_source = result;
        alpha_source_owner = base_alpha;
      }
    }
    else {
      alpha_source = base_image_alpha;
      alpha_source_owner = base_image;
    }
  }

  if (sphere_texture != nullptr && pmx_material.sphere_mode != SphereMode::None) {
    bNode *sphere_image = add_node(*ntree, SH_NODE_TEX_IMAGE, -520.0f, -360.0f);
    BLI_strncpy(sphere_image->name, "PMX Sphere Texture", sizeof(sphere_image->name));
    BLI_strncpy(sphere_image->label, "PMX Sphere Texture", sizeof(sphere_image->label));
    sphere_image->id = &sphere_texture->id;

    /* mmd_tools samples sphere maps from the view-space normal, remapped from [-1, 1] to [0, 1]. */
    bNode *tex_coord = add_node(*ntree, SH_NODE_TEX_COORD, -1180.0f, -440.0f);
    BLI_strncpy(tex_coord->name, "PMX Sphere Coordinates", sizeof(tex_coord->name));
    bNode *vector_transform = add_node(*ntree, SH_NODE_VECT_TRANSFORM, -980.0f, -440.0f);
    BLI_strncpy(vector_transform->name,
                "PMX Sphere Normal Transform",
                sizeof(vector_transform->name));
    NodeShaderVectTransform *transform = static_cast<NodeShaderVectTransform *>(
        vector_transform->storage);
    if (transform != nullptr) {
      transform->type = SHD_VECT_TRANSFORM_TYPE_NORMAL;
      transform->convert_from = SHD_VECT_TRANSFORM_SPACE_OBJECT;
      transform->convert_to = SHD_VECT_TRANSFORM_SPACE_CAMERA;
    }
    bNode *mapping = add_node(*ntree, SH_NODE_MAPPING, -760.0f, -440.0f);
    BLI_strncpy(mapping->name, "PMX Sphere Mapping", sizeof(mapping->name));
    bNodeSocket *normal = bke::node_find_socket(*tex_coord, SOCK_OUT, "Normal"_ustr);
    bNodeSocket *transform_input = bke::node_find_socket(
        *vector_transform, SOCK_IN, "Vector"_ustr);
    bNodeSocket *transform_output = bke::node_find_socket(
        *vector_transform, SOCK_OUT, "Vector"_ustr);
    bNodeSocket *mapping_input = bke::node_find_socket(*mapping, SOCK_IN, "Vector"_ustr);
    bNodeSocket *mapping_location = bke::node_find_socket(*mapping, SOCK_IN, "Location"_ustr);
    bNodeSocket *mapping_scale = bke::node_find_socket(*mapping, SOCK_IN, "Scale"_ustr);
    bNodeSocket *mapping_output = bke::node_find_socket(*mapping, SOCK_OUT, "Vector"_ustr);
    bNodeSocket *sphere_vector = bke::node_find_socket(*sphere_image, SOCK_IN, "Vector"_ustr);
    BLI_assert(normal != nullptr && transform_input != nullptr && transform_output != nullptr &&
               mapping_input != nullptr && mapping_location != nullptr && mapping_scale != nullptr &&
               mapping_output != nullptr && sphere_vector != nullptr);
    if (normal != nullptr && transform_input != nullptr && transform_output != nullptr &&
        mapping_input != nullptr && mapping_location != nullptr && mapping_scale != nullptr &&
        mapping_output != nullptr && sphere_vector != nullptr)
    {
      bke::node_add_link(*ntree, *tex_coord, *normal, *vector_transform, *transform_input);
      bke::node_add_link(*ntree, *vector_transform, *transform_output, *mapping, *mapping_input);
      bke::node_add_link(*ntree, *mapping, *mapping_output, *sphere_image, *sphere_vector);

      bNodeSocketValueVector *location = mapping_location->default_value_typed<
          bNodeSocketValueVector>();
      bNodeSocketValueVector *scale = mapping_scale->default_value_typed<bNodeSocketValueVector>();
      location->value[0] = 0.5f;
      location->value[1] = 0.5f;
      location->value[2] = 0.0f;
      scale->value[0] = 0.5f;
      scale->value[1] = 0.5f;
      scale->value[2] = 1.0f;
    }

    bNode *sphere_mix = add_node(*ntree, SH_NODE_MIX_RGB_LEGACY, 40.0f, 80.0f);
    BLI_strncpy(sphere_mix->name, "PMX Sphere Mix", sizeof(sphere_mix->name));
    sphere_mix->custom1 = pmx_material.sphere_mode == SphereMode::Cube ? MA_RAMP_ADD :
                                                                         MA_RAMP_MULT;
    bNodeSocket *mix_factor = bke::node_find_socket(*sphere_mix, SOCK_IN, "Fac"_ustr);
    bNodeSocket *mix_color_1 = bke::node_find_socket(*sphere_mix, SOCK_IN, "Color1"_ustr);
    bNodeSocket *mix_color_2 = bke::node_find_socket(*sphere_mix, SOCK_IN, "Color2"_ustr);
    bNodeSocket *mix_output = bke::node_find_socket(*sphere_mix, SOCK_OUT, "Color"_ustr);
    bNodeSocket *sphere_color = bke::node_find_socket(*sphere_image, SOCK_OUT, "Color"_ustr);
    bNodeSocket *sphere_alpha = bke::node_find_socket(*sphere_image, SOCK_OUT, "Alpha"_ustr);
    BLI_assert(mix_factor != nullptr && mix_color_1 != nullptr && mix_color_2 != nullptr &&
               mix_output != nullptr && sphere_color != nullptr && sphere_alpha != nullptr);
    if (mix_factor != nullptr && mix_color_1 != nullptr && mix_color_2 != nullptr &&
        mix_output != nullptr && sphere_color != nullptr && sphere_alpha != nullptr)
    {
      mix_factor->default_value_typed<bNodeSocketValueFloat>()->value = 1.0f;
      if (base_image_color != nullptr) {
        bke::node_add_link(*ntree, *base_image, *base_image_color, *sphere_mix, *mix_color_1);
      }
      else {
        bNodeSocketValueRGBA *mix_color = mix_color_1->default_value_typed<bNodeSocketValueRGBA>();
        for (int channel = 0; channel < 4; channel++) {
          mix_color->value[channel] = pmx_material.diffuse[channel];
        }
      }
      bke::node_add_link(*ntree, *sphere_image, *sphere_color, *sphere_mix, *mix_color_2);
      bke::node_add_link(*ntree, *sphere_mix, *mix_output, *principled, *base_color);

      if (sphere_alpha_info.has_alpha_channel) {
        bNode *sphere_alpha_node = add_node(*ntree, SH_NODE_MATH, 40.0f, -260.0f);
        BLI_strncpy(sphere_alpha_node->name, "PMX Sphere Alpha", sizeof(sphere_alpha_node->name));
        sphere_alpha_node->custom1 = NODE_MATH_MULTIPLY;
        bNodeSocket *alpha_1 = bke::node_find_socket(*sphere_alpha_node, SOCK_IN, "Value"_ustr);
        bNodeSocket *alpha_2 = bke::node_find_socket(
            *sphere_alpha_node, SOCK_IN, "Value_001"_ustr);
        bNodeSocket *alpha_result = bke::node_find_socket(
            *sphere_alpha_node, SOCK_OUT, "Value"_ustr);
        BLI_assert(alpha_1 != nullptr && alpha_2 != nullptr && alpha_result != nullptr);
        if (alpha_1 != nullptr && alpha_2 != nullptr && alpha_result != nullptr) {
          if (alpha_source != nullptr) {
            BLI_assert(alpha_source_owner != nullptr);
            if (alpha_source_owner != nullptr) {
              bke::node_add_link(
                  *ntree, *alpha_source_owner, *alpha_source, *sphere_alpha_node, *alpha_1);
            }
          }
          else {
            alpha_1->default_value_typed<bNodeSocketValueFloat>()->value = pmx_material.diffuse[3];
          }
          bke::node_add_link(*ntree, *sphere_image, *sphere_alpha, *sphere_alpha_node, *alpha_2);
          alpha_source = alpha_result;
          alpha_source_owner = sphere_alpha_node;
        }
      }
    }
  }

  if (alpha_source != nullptr && alpha_source_owner != nullptr) {
    bke::node_add_link(*ntree, *alpha_source_owner, *alpha_source, *principled, *alpha);
  }
  else {
    alpha->default_value_typed<bNodeSocketValueFloat>()->value = pmx_material.diffuse[3];
  }
}

}  // namespace

bool read_pmx_material_edge_data(const Material &material,
                                 bool &r_enabled,
                                 std::array<float, 4> &r_color,
                                 float &r_weight)
{
  r_enabled = false;
  r_color = {0.0f, 0.0f, 0.0f, 0.0f};
  r_weight = 1.0f;

  IDProperty *props = material.id.system_properties;
  if (props == nullptr) {
    return false;
  }
  IDProperty *enabled = IDP_GetPropertyTypeFromGroup(props, kPMXEdgeEnabled, IDP_BOOLEAN);
  IDProperty *color = IDP_GetPropertyTypeFromGroup(props, kPMXEdgeColor, IDP_ARRAY);
  IDProperty *weight = IDP_GetPropertyTypeFromGroup(props, kPMXEdgeWeight, IDP_FLOAT);
  if (enabled == nullptr || color == nullptr || color->len != 4 || color->subtype != IDP_FLOAT ||
      weight == nullptr || !std::isfinite(IDP_float_get(weight)))
  {
    return false;
  }

  const float *color_values = IDP_array_float_get(color);
  for (const int index : IndexRange(4)) {
    if (!std::isfinite(color_values[index])) {
      return false;
    }
    r_color[index] = std::clamp(color_values[index], 0.0f, 1.0f);
  }
  r_enabled = IDP_bool_get(enabled);
  r_weight = std::max(0.0f, IDP_float_get(weight));
  return true;
}

Material *create_pmx_material(PMXImportContext &ctx, const PMXModel &model, const int material_index)
{
  if (material_index < 0 || material_index >= int(model.materials.size())) {
    return nullptr;
  }

  if (Material **cached = ctx.material_cache.lookup_ptr(material_index)) {
    return *cached;
  }

  const PMXMaterial &pmx_material = model.materials[material_index];
  const char *material_name = pmx_material.name_local.empty() ? "PMXMaterial" :
                                                                  pmx_material.name_local.c_str();
  Material *material = BKE_material_add(ctx.bmain, material_name);
  if (material == nullptr) {
    BKE_reportf(ctx.reports, RPT_ERROR, "Could not create PMX material '%s'", material_name);
    return nullptr;
  }

  material->r = pmx_material.diffuse[0];
  material->g = pmx_material.diffuse[1];
  material->b = pmx_material.diffuse[2];
  material->a = pmx_material.diffuse[3];

  /* PMX's material double-sided flag maps to Blender's backface culling. The
   * transparent overlay fix only controls which transparent layers are shown;
   * it must not replace the source material's face-sidedness rule. */
  if ((pmx_material.flag & PMX_MATERIAL_FLAG_DOUBLE_SIDED) == 0) {
    material->blend_flag |= MA_BL_CULL_BACKFACE;
  }
  write_pmx_edge_data(*material, pmx_material);

  Image *base_texture = load_texture(ctx, pmx_material, pmx_material.texture_idx, "base");
  Image *sphere_texture = pmx_material.sphere_mode == SphereMode::None ?
                              nullptr :
                              load_texture(ctx,
                                           pmx_material,
                                           pmx_material.sphere_texture_idx,
                                           "sphere");
  build_pmx_principled_tree(*material, pmx_material, base_texture, sphere_texture);

  ctx.material_cache.add(material_index, material);
  ctx.material_report.materials_created++;
  return material;
}

void report_pmx_material_import_summary(const PMXImportContext &ctx)
{
  BKE_reportf(ctx.reports,
               RPT_INFO,
               "PMX materials: %d created, %d textures loaded, %d reused, %d missing, "
              "%d decode failed, %d empty path, %d invalid texture index, %d cached failures",
              ctx.material_report.materials_created,
              ctx.material_report.loaded_textures,
              ctx.material_report.reused_textures,
              ctx.material_report.missing_textures,
              ctx.material_report.decode_failed_textures,
              ctx.material_report.empty_texture_paths,
              ctx.material_report.invalid_texture_indices,
              ctx.material_report.cached_failures);
}

}  // namespace blender::io::pmx
