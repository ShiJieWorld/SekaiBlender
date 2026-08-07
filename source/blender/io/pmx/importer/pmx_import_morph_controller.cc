/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_anim_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_anim_data.hh"
#include "BKE_idprop.hh"
#include "BKE_collection.hh"
#include "BKE_fcurve.hh"
#include "BKE_fcurve_driver.h"
#include "BKE_key.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "ED_keyframing.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "MEM_guardedalloc.h"

#include "pmx_import_morph_controller.hh"

namespace blender::io::pmx {
namespace {

constexpr char kMorphDefinitionProperty[] = "mmd_pmx_morph_definition";
constexpr int kMorphDefinitionSchemaVersion = 1;
constexpr size_t kMaxDriverExpressionLength = 255;

static void add_property(IDProperty *group, IDProperty *property)
{
  if (!property || !IDP_AddToGroup(group, property)) {
    if (property) {
      IDP_FreeProperty(property);
    }
  }
}

static void add_int(IDProperty *group, const char *name, const int value)
{
  add_property(group, IDP_NewInt(value, name));
}

static void add_bool(IDProperty *group, const char *name, const bool value)
{
  add_property(group, blender::bke::idprop::create_bool(name, value).release());
}

static void add_string(IDProperty *group, const char *name, const std::string &value)
{
  add_property(group, IDP_NewString(value.c_str(), name));
}

static void append_group(IDProperty *array, IDProperty *item)
{
  IDP_ResizeIDPArray(array, array->len + 1);
  IDP_SetIndexArray(array, array->len - 1, item);
  MEM_delete(item);
}

static void persist_morph_definition(PMXImportContext &ctx)
{
  if (!ctx.model_collection) {
    return;
  }
  IDProperty *properties = IDP_ID_system_properties_ensure(&ctx.model_collection->id);
  IDProperty *definition = blender::bke::idprop::create_group(kMorphDefinitionProperty).release();
  add_int(definition, "schema_version", kMorphDefinitionSchemaVersion);
  add_int(definition, "pmx_morph_count", ctx.group_morph_report.morph_count);
  add_bool(definition, "valid", ctx.group_morph_report.valid);
  add_int(definition, "warning_count", ctx.group_morph_report.warning_count);
  add_int(definition, "error_count", ctx.group_morph_report.error_count);
  /* R1-PMX: persist the unsupported-edge summary so the VMD importer can read
   * it and warn about capability limits when a VMD track drives a Group raw
   * channel whose expansion references unsupported morph types. */
  add_int(definition, "unsupported_edge_count", ctx.group_morph_report.unsupported_edge_count);
  add_int(definition, "unsupported_bone_count", ctx.group_morph_report.unsupported_bone_count);
  add_int(definition, "unsupported_uv_count", ctx.group_morph_report.unsupported_uv_count);
  add_int(definition, "unsupported_material_count", ctx.group_morph_report.unsupported_material_count);
  add_int(definition, "unsupported_flip_count", ctx.group_morph_report.unsupported_flip_count);
  add_int(definition, "unsupported_impulse_count", ctx.group_morph_report.unsupported_impulse_count);

  IDProperty *channels = IDP_NewIDPArray("channels");
  for (const PMXMorphChannel &channel : ctx.group_morph_report.channels) {
    if (!channel.controller_channel) {
      continue;
    }
    IDProperty *item = blender::bke::idprop::create_group("channel").release();
    add_int(item, "pmx_morph_index", channel.morph_index);
    add_int(item, "morph_type", int(channel.type));
    add_string(item, "pmx_name_local", channel.source_name);
    add_string(item, "pmx_name_universal", channel.source_name_universal);
    add_string(item, "controller_key_name", channel.blender_name);
    add_bool(item, "controller_channel", channel.controller_channel);
    add_bool(item, "vertex_output", channel.vertex_output);
    append_group(channels, item);
  }
  add_property(definition, channels);

  IDProperty *old_definition = IDP_GetPropertyFromGroup_null(properties, kMorphDefinitionProperty);
  if (old_definition) {
    IDP_FreeFromGroup(properties, old_definition);
  }
  add_property(properties, definition);

  /* [世界的歌] C2-2E: also expose the morph definition on the controller object
   * so the VMD importer can read Group raw channel names directly from the
   * controller without searching model collections. */
  if (ctx.morph_controller_obj != nullptr) {
    IDProperty *controller_props = IDP_ID_system_properties_ensure(&ctx.morph_controller_obj->id);
    IDProperty *old_controller_definition = IDP_GetPropertyFromGroup_null(
        controller_props, kMorphDefinitionProperty);
    if (old_controller_definition) {
      IDP_FreeFromGroup(controller_props, old_controller_definition);
    }
    IDProperty *controller_definition = IDP_CopyProperty(definition);
    IDP_AddToGroup(controller_props, controller_definition);
  }
}

static std::string key_block_path(const Key *key, const KeyBlock *key_block)
{
  const std::optional<std::string> path = BKE_keyblock_curval_rnapath_get(key, key_block);
  return path.value_or(std::string());
}

static KeyBlock *find_or_add_key_block(Main *bmain, Mesh *mesh, const char *name)
{
  Key *key = mesh->key;
  if (!key) {
    key = mesh->key = BKE_key_add(bmain, &mesh->id);
    key->type = KEY_RELATIVE;
  }

  if (!key->block.first) {
    KeyBlock *basis = BKE_keyblock_add(key, "Basis");
    BKE_keyblock_convert_from_mesh(mesh, key, basis);
  }

  KeyBlock *key_block = BKE_keyblock_find_name(key, name);
  if (!key_block) {
    key_block = BKE_keyblock_add(key, name);
    BKE_keyblock_convert_from_mesh(mesh, key, key_block);
  }
  return key_block;
}

struct DriverTerm {
  std::string variable_name;
  std::string source_path;
  float coefficient = 0.0f;
};

struct GeometryDriverPlan {
  Key *dst_key = nullptr;
  KeyBlock *dst_key_block = nullptr;
  std::string dst_path;
  std::vector<DriverTerm> terms;
  std::string expression;
};

struct CreatedDriver {
  Key *key = nullptr;
  FCurve *fcurve = nullptr;
};

static bool add_driver_if_missing(Key *dst_key,
                                   KeyBlock *dst_key_block,
                                   Object *controller_obj,
                                   Key *controller_key,
                                   KeyBlock *controller_key_block,
                                   CreatedDriver *r_created = nullptr)
{
  const std::string dst_path = key_block_path(dst_key, dst_key_block);
  const std::string src_key_path = key_block_path(controller_key, controller_key_block);
  if (dst_path.empty() || src_key_path.empty()) {
    return false;
  }

  AnimData *adt = BKE_animdata_from_id(&dst_key->id);
  if (adt && BKE_fcurve_find(&adt->drivers, dst_path.c_str(), 0)) {
    return true;
  }

  /* DRIVER_FCURVE_EMPTY creates an F-Curve without a ChannelDriver. Such an
   * F-Curve is still added to the driver list and the dependency graph later
   * dereferences its null driver. Use the normal creation mode so the driver
   * object is allocated before adding variables. */
  FCurve *fcu = verify_driver_fcurve(
      &dst_key->id, dst_path.c_str(), 0, DRIVER_FCURVE_KEYFRAMES);
  if (!fcu || !fcu->driver) {
    return false;
  }

  fcu->driver->type = DRIVER_TYPE_AVERAGE;
  DriverVar *var = driver_add_new_variable(fcu->driver);
  if (!var) {
    AnimData *created_adt = BKE_animdata_from_id(&dst_key->id);
    if (created_adt) {
      BLI_remlink(&created_adt->drivers, fcu);
    }
    BKE_fcurve_free(fcu);
    return false;
  }
  /* Initialize target flags and target count through the public driver helper.
   * Assigning type/num_targets directly leaves the target in an invalid state
   * for dependency-graph driver construction. */
  driver_change_variable_type(var, DVAR_TYPE_SINGLE_PROP);
  DriverTarget *target = &var->targets[0];
  /* The RNA path returned by BKE_keyblock_curval_rnapath_get() is relative
   * to the Key datablock, not to the controller Object. Using the Object here
   * makes dependency-graph driver resolution fail with a null RNA property. */
  target->id = &controller_obj->id;
  target->idtype = ID_OB;
  const std::string src_path = "data.shape_keys." + src_key_path;
  target->rna_path = BLI_strdup(src_path.c_str());
  STRNCPY_UTF8(var->name, "morph_value");
  STRNCPY_UTF8(fcu->driver->expression, "morph_value");
  if (r_created) {
    r_created->key = dst_key;
    r_created->fcurve = fcu;
  }
  return true;
}

static std::string format_coefficient(const float coefficient)
{
  std::ostringstream stream;
  stream << std::setprecision(9) << coefficient;
  return stream.str();
}

static void remove_created_driver(const CreatedDriver &created)
{
  if (!created.key || !created.fcurve) {
    return;
  }
  AnimData *adt = BKE_animdata_from_id(&created.key->id);
  if (!adt) {
    return;
  }
  BLI_remlink(&adt->drivers, created.fcurve);
  BKE_fcurve_free(created.fcurve);
}

static bool add_expression_driver(const GeometryDriverPlan &plan,
                                  Object *controller_obj,
                                  std::vector<CreatedDriver> &created)
{
  AnimData *adt = BKE_animdata_from_id(&plan.dst_key->id);
  if (adt && BKE_fcurve_find(&adt->drivers, plan.dst_path.c_str(), 0)) {
    return true;
  }
  FCurve *fcu = verify_driver_fcurve(
      &plan.dst_key->id, plan.dst_path.c_str(), 0, DRIVER_FCURVE_KEYFRAMES);
  if (!fcu || !fcu->driver) {
    return false;
  }
  fcu->driver->type = DRIVER_TYPE_PYTHON;
  for (const DriverTerm &term : plan.terms) {
    DriverVar *var = driver_add_new_variable(fcu->driver);
    if (!var) {
      AnimData *created_adt = BKE_animdata_from_id(&plan.dst_key->id);
      if (created_adt) {
        BLI_remlink(&created_adt->drivers, fcu);
      }
      BKE_fcurve_free(fcu);
      return false;
    }
    driver_change_variable_type(var, DVAR_TYPE_SINGLE_PROP);
    DriverTarget *target = &var->targets[0];
    target->id = &controller_obj->id;
    target->idtype = ID_OB;
    target->rna_path = BLI_strdup(("data.shape_keys." + term.source_path).c_str());
    STRNCPY_UTF8(var->name, term.variable_name.c_str());
  }
  STRNCPY_UTF8(fcu->driver->expression, plan.expression.c_str());
  created.push_back({plan.dst_key, fcu});
  return true;
}

}  // namespace

void create_morph_controller(PMXImportContext &ctx)
{
  const bool has_group_channels = ctx.group_morph_report.valid &&
                                  ctx.group_morph_report.supported_channel_count >
                                      int(ctx.morph_names.size());
  if (!ctx.root_obj || (ctx.morph_names.is_empty() && !has_group_channels)) {
    return;
  }

  Mesh *mesh_data = BKE_mesh_new_nomain(1, 0, 0, 0);
  Mesh *mesh = BKE_mesh_add(ctx.bmain, "PMXMorphControllerMesh");
  BKE_mesh_nomain_to_mesh(mesh_data, mesh, nullptr);

  Object *controller = BKE_object_add_only_object(
      ctx.bmain, OB_MESH, "PMXMorphController");
  controller->data = &mesh->id;
  if (ctx.controls_collection) {
    BKE_collection_object_add(ctx.bmain, ctx.controls_collection, controller);
  }
  controller->parent = ctx.root_obj;
  controller->partype = PAROBJECT;
  unit_m4(controller->parentinv);
  ctx.morph_controller_obj = controller;

  Key *controller_key = nullptr;
  for (int morph_slot : ctx.morph_names.index_range()) {
    const std::string &fallback_name = ctx.morph_names[morph_slot];
    const int morph_index = ctx.morph_indices[morph_slot];
    std::string controller_name = fallback_name;
    for (const PMXMorphChannel &channel : ctx.group_morph_report.channels) {
      if (channel.morph_index == morph_index && channel.controller_channel &&
          !channel.blender_name.empty()) {
        controller_name = channel.blender_name;
        break;
      }
    }
    KeyBlock *key_block = find_or_add_key_block(ctx.bmain, mesh, controller_name.c_str());
    if (!controller_key) {
      controller_key = mesh->key;
    }
    key_block->curval = 0.0f;
  }
  if (ctx.group_morph_report.valid) {
    for (const PMXMorphChannel &channel : ctx.group_morph_report.channels) {
      if (channel.type != MorphType::Group || !channel.controller_channel ||
          channel.blender_name.empty()) {
        continue;
      }
      KeyBlock *key_block = find_or_add_key_block(
          ctx.bmain, mesh, channel.blender_name.c_str());
      if (!controller_key) {
        controller_key = mesh->key;
      }
      key_block->curval = 0.0f;
    }
  }
  if (!controller_key) {
    return;
  }

  auto controller_name_for_index = [&](const int morph_index) -> std::string {
    for (const PMXMorphChannel &channel : ctx.group_morph_report.channels) {
      if (channel.morph_index == morph_index && channel.controller_channel) {
        return channel.blender_name;
      }
    }
    return std::string();
  };

  persist_morph_definition(ctx);

  /* R1-PMX: emit one aggregated summary warning so the user learns, at PMX
   * import time, which Group Morph edges reference unsupported morph types.
   * The per-edge warnings are still emitted by the analyzer; this is the
   * capability-level headline for the whole model. */
  if (ctx.group_morph_report.unsupported_edge_count > 0) {
    std::ostringstream summary;
    summary << "PMX contains " << ctx.group_morph_report.unsupported_edge_count
            << " Group Morph edge(s) targeting unsupported morph types:";
    const auto append_type = [&](const int count, const char *label) {
      if (count > 0) {
        summary << ' ' << label << " x" << count;
      }
    };
    append_type(ctx.group_morph_report.unsupported_bone_count, "Bone");
    append_type(ctx.group_morph_report.unsupported_uv_count, "UV");
    append_type(ctx.group_morph_report.unsupported_material_count, "Material");
    append_type(ctx.group_morph_report.unsupported_flip_count, "Flip");
    append_type(ctx.group_morph_report.unsupported_impulse_count, "Impulse");
    summary << ". These are imported by VMD only as raw channels and produce no vertex effect; "
               "drive those targets through VMD Bone/UV tracks instead.";
    BKE_report(ctx.reports, RPT_WARNING, summary.str().c_str());
  }

  auto expression_for_index = [&](const int morph_index) -> const PMXVertexMorphExpression * {
    if (!ctx.group_morph_expression_report.valid) {
      return nullptr;
    }
    for (const PMXVertexMorphExpression &expression :
         ctx.group_morph_expression_report.vertex_expressions) {
      if (expression.vertex_morph_index == morph_index) {
        return &expression;
      }
    }
    return nullptr;
  };

  /* Build every multi-variable destination before mutating any Geometry Key.
   * This keeps source/target resolution deterministic across split meshes and
   * lets the write phase roll back the complete batch on the first failure. */
  std::vector<GeometryDriverPlan> plans;
  bool plan_valid = true;
  for (int object_index : ctx.mesh_objects.index_range()) {
    Object *obj = ctx.mesh_objects[object_index];
    Mesh *mesh_data = reinterpret_cast<Mesh *>(obj->data);
    if (!mesh_data || !mesh_data->key) {
      continue;
    }
    for (int morph_index : ctx.morph_indices.index_range()) {
      const std::string &name = ctx.morph_names[morph_index];
      KeyBlock *dst_key_block = BKE_keyblock_find_name(mesh_data->key, name.c_str());
      const PMXVertexMorphExpression *expression = expression_for_index(
          ctx.morph_indices[morph_index]);
      const std::string direct_controller_name = controller_name_for_index(
          ctx.morph_indices[morph_index]);
      KeyBlock *direct_controller_key_block = direct_controller_name.empty() ? nullptr :
                                                  BKE_keyblock_find_name(
                                                      controller_key, direct_controller_name.c_str());
      if (!dst_key_block || !direct_controller_key_block) {
        plan_valid = false;
        break;
      }
      if (!expression || expression->raw_morph_indices.size() <= 1) {
        if (key_block_path(mesh_data->key, dst_key_block).empty() ||
            key_block_path(controller_key, direct_controller_key_block).empty()) {
          plan_valid = false;
          break;
        }
        continue;
      }
      if (expression->raw_morph_indices.size() != expression->coefficients.size()) {
        plan_valid = false;
        break;
      }
      GeometryDriverPlan plan;
      plan.dst_key = mesh_data->key;
      plan.dst_key_block = dst_key_block;
      plan.dst_path = key_block_path(mesh_data->key, dst_key_block);
      if (plan.dst_path.empty()) {
        plan_valid = false;
        break;
      }
      for (size_t term_index = 0; term_index < expression->raw_morph_indices.size(); term_index++) {
        const int raw_index = expression->raw_morph_indices[term_index];
        const std::string source_name = controller_name_for_index(raw_index);
        KeyBlock *source_key_block = source_name.empty() ? nullptr :
                                         BKE_keyblock_find_name(controller_key, source_name.c_str());
        const std::string source_path = source_key_block ?
                                            key_block_path(controller_key, source_key_block) :
                                            std::string();
        if (source_path.empty()) {
          plan_valid = false;
          break;
        }
        const std::string variable_name = "m" + std::to_string(raw_index);
        const float coefficient = expression->coefficients[term_index];
        plan.terms.push_back({variable_name, source_path, coefficient});
        if (!plan.expression.empty()) {
          plan.expression += " + ";
        }
        if (std::abs(coefficient - 1.0f) <= 1.0e-7f) {
          plan.expression += variable_name;
        }
        else {
          plan.expression += format_coefficient(coefficient) + " * " + variable_name;
        }
      }
      if (plan.terms.empty() || plan.expression.empty() ||
          plan.expression.size() > kMaxDriverExpressionLength) {
        plan_valid = false;
        break;
      }
      plans.push_back(std::move(plan));
    }
  }

  if (!plan_valid) {
      BKE_report(ctx.reports,
                 RPT_WARNING,
                 "PMX Geometry Morph Driver plan is invalid; no new drivers were created");
    return;
  }
  std::vector<CreatedDriver> created;
  for (const GeometryDriverPlan &plan : plans) {
    if (!add_expression_driver(plan, controller, created)) {
      for (auto it = created.rbegin(); it != created.rend(); ++it) {
        remove_created_driver(*it);
      }
      BKE_report(ctx.reports,
                 RPT_WARNING,
                 "PMX Geometry Morph Driver creation failed; batch rolled back");
      return;
    }
  }

  for (int object_index : ctx.mesh_objects.index_range()) {
    Object *obj = ctx.mesh_objects[object_index];
    Mesh *mesh_data = reinterpret_cast<Mesh *>(obj->data);
    if (!mesh_data || !mesh_data->key) {
      continue;
    }
    for (int morph_index : ctx.morph_indices.index_range()) {
      const std::string &name = ctx.morph_names[morph_index];
      KeyBlock *dst_key_block = BKE_keyblock_find_name(mesh_data->key, name.c_str());
      const std::string controller_name = controller_name_for_index(
          ctx.morph_indices[morph_index]);
      KeyBlock *controller_key_block = controller_name.empty() ? nullptr :
                                           BKE_keyblock_find_name(controller_key,
                                                                   controller_name.c_str());
      const PMXVertexMorphExpression *expression = expression_for_index(
          ctx.morph_indices[morph_index]);
      if (!dst_key_block || !controller_key_block || (expression &&
          expression->raw_morph_indices.size() > 1)) {
        continue;
      }
      CreatedDriver driver;
      if (!add_driver_if_missing(mesh_data->key,
                                 dst_key_block,
                                 controller,
                                 controller_key,
                                 controller_key_block,
                                 &driver)) {
        for (auto it = created.rbegin(); it != created.rend(); ++it) {
          remove_created_driver(*it);
        }
        return;
      }
      if (driver.fcurve) {
        created.push_back(driver);
      }
    }
  }

  DEG_id_tag_update(&controller->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  DEG_relations_tag_update(ctx.bmain);
}

}  // namespace blender::io::pmx
