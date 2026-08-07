/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "BKE_armature.hh"
#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"
#include "BLI_fileops.hh"
#include "BLI_path_utils.hh"
#include "WM_api.hh"
#include "WM_types.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "IO_pmx.hh"
#include "intern/pmx_reader.h"
#include "intern/pmx_types.h"
#include "mmd_physics_definition.hh"
#include "pmx_import.hh"
#include "pmx_import_armature.hh"
#include "pmx_import_bone_ik.hh"
#include "pmx_import_bone_append.hh"
#include "pmx_import_bone_axis.hh"
#include "pmx_import_material.hh"
#include "pmx_import_mesh.hh"
#include "pmx_group_morph.hh"
#include "pmx_import_morph.hh"
#include "pmx_import_morph_controller.hh"
#include "pmx_import_weights.hh"
#include "pmx_source_data.hh"

#include <algorithm>
#include <string>

namespace blender::io::pmx {
namespace {

void report_physics_debug(const mmd_physics::MMDPhysicsDebugReport &debug, ReportList *reports)
{
  const eReportType status_type = debug.static_ready ? RPT_INFO : RPT_WARNING;
  const char *readiness = debug.static_ready ?
                              (debug.warning_count > 0 ? "ready with warnings" : "ready") :
                              "not ready";
  BKE_reportf(reports,
              debug.definition_valid ? RPT_INFO : RPT_WARNING,
              "PMX physics debug: definition %s",
              debug.definition_valid ? "valid" : "invalid");
  BKE_reportf(reports,
              debug.mapping_valid ? RPT_INFO : RPT_WARNING,
              "PMX physics debug: mapping %s",
              debug.mapping_valid ? "valid" : "invalid");
  BKE_reportf(reports, status_type, "PMX physics debug: static readiness: %s", readiness);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: bones %d (resolved %d)",
              debug.bone_count,
              debug.resolved_bone_count);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: rigid bodies %d (bound %d, unbound %d, invalid %d)",
              debug.rigid_body_count,
              debug.bound_rigid_body_count,
              debug.unbound_rigid_body_count,
              debug.invalid_rigid_body_binding_count);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: joints %d (endpoints resolved %d)",
              debug.joint_count,
              debug.resolved_joint_count);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: rigid shapes sphere=%d, box=%d, capsule=%d",
              debug.rigid_shape_counts[0],
              debug.rigid_shape_counts[1],
              debug.rigid_shape_counts[2]);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: physics types static=%d, dynamic=%d, dynamic_bone=%d",
              debug.rigid_type_counts[0],
              debug.rigid_type_counts[1],
              debug.rigid_type_counts[2]);
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: rotation axes limited=(%d,%d,%d), free=(%d,%d,%d)",
              debug.joint_rotation_limited_axes[0],
              debug.joint_rotation_limited_axes[1],
              debug.joint_rotation_limited_axes[2],
              debug.joint_rotation_free_axes[0],
              debug.joint_rotation_free_axes[1],
              debug.joint_rotation_free_axes[2]);

  std::string collision_groups;
  for (int group = 0; group < int(debug.collision_group_counts.size()); group++) {
    if (debug.collision_group_counts[group] == 0) {
      continue;
    }
    if (!collision_groups.empty()) {
      collision_groups += ", ";
    }
    collision_groups += "g" + std::to_string(group) + "=" +
                        std::to_string(debug.collision_group_counts[group]);
  }
  BKE_reportf(reports,
              status_type,
              "PMX physics debug: collision groups %s",
              collision_groups.empty() ? "none" : collision_groups.c_str());
  BKE_reportf(reports,
              debug.joints_with_same_endpoint == 0 && debug.negative_spring_count == 0 ? RPT_INFO :
                                                                                           RPT_WARNING,
              "PMX physics debug: joints same-endpoint=%d, negative-spring-components=%d",
              debug.joints_with_same_endpoint,
              debug.negative_spring_count);
  for (const std::string &diagnostic : debug.diagnostics) {
    BKE_reportf(reports, RPT_WARNING, "PMX physics debug: %s", diagnostic.c_str());
  }
}

void report_physics_mapping(const mmd_physics::MMDPhysicsDefinition &definition,
                            const mmd_physics::MMDPhysicsMappingReport &mapping,
                            ReportList *reports)
{
  const int bone_count = int(definition.bone_mapping.size());
  const int rigid_count = int(definition.rigid_bodies.size());
  const int joint_count = int(definition.joints.size());
  const int valid_rigid_count = std::max(0, rigid_count - mapping.invalid_rigid_bodies);
  int error_count = 0;
  int warning_count = 0;
  for (const mmd_physics::MMDPhysicsMappingIssue &issue : mapping.issues) {
    if (issue.severity == mmd_physics::MMDPhysicsMappingIssueSeverity::Error) {
      error_count++;
    }
    else if (issue.severity == mmd_physics::MMDPhysicsMappingIssueSeverity::Warning) {
      warning_count++;
    }
  }

  BKE_reportf(reports,
              mapping.resolved_bones == bone_count ? RPT_INFO : RPT_WARNING,
              "PMX physics mapping: %d/%d bones resolved",
              mapping.resolved_bones,
              bone_count);
  BKE_reportf(reports,
              mapping.invalid_rigid_bodies == 0 ? RPT_INFO : RPT_WARNING,
              "PMX physics mapping: %d/%d rigid bodies validated",
              valid_rigid_count,
              rigid_count);
  BKE_reportf(reports,
              mapping.resolved_joint_endpoints == joint_count ? RPT_INFO : RPT_WARNING,
              "PMX physics mapping: %d/%d joints resolved",
              mapping.resolved_joint_endpoints,
              joint_count);
  BKE_reportf(reports,
              error_count == 0 && warning_count == 0 ? RPT_INFO : RPT_WARNING,
              "PMX physics mapping complete: %d errors, %d warnings",
              error_count,
              warning_count);

  for (const mmd_physics::MMDPhysicsMappingIssue &issue : mapping.issues) {
    BKE_reportf(reports,
                issue.severity == mmd_physics::MMDPhysicsMappingIssueSeverity::Error ?
                    RPT_WARNING :
                    RPT_INFO,
                "PMX physics mapping: %s: %s",
                issue.path.c_str(),
                issue.message.c_str());
  }
}

}  // namespace

void importer_main(bContext *C, PMXImportParams &params)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);

  if (!BLI_exists(params.filepath)) {
    BKE_reportf(CTX_wm_reports(C), RPT_ERROR, "File not found: %s", params.filepath);
    return;
  }

  /* Read and parse the PMX file. */
  PMXModel model;
  try {
    model = PMXReader::read(params.filepath);
  }
  catch (const std::exception &e) {
    BKE_reportf(CTX_wm_reports(C), RPT_ERROR, "Failed to parse PMX: %s", e.what());
    return;
  }

  /* Set up import context. */
  PMXImportContext ctx;
  ctx.bmain = bmain;
  ctx.scene = scene;
  ctx.view_layer = view_layer;
  ctx.params = &params;
  ctx.mesh_obj = nullptr;
  ctx.root_obj = nullptr;
  ctx.armature_obj = nullptr;
  ctx.morph_controller_obj = nullptr;
  ctx.reports = CTX_wm_reports(C);
  ctx.model_textures = &model.textures;

  BKE_reportf(CTX_wm_reports(C),
              RPT_INFO,
              "PMX parsed: '%s' (%zu vertices, %zu triangles, %zu materials, %zu bones, %zu morphs)",
              model.name_local.c_str(),
              model.vertices.size(),
              model.face_indices.size() / 3,
              model.materials.size(),
              model.bones.size(),
              model.morphs.size());
  BKE_reportf(CTX_wm_reports(C),
              RPT_INFO,
              "PMX header: version %.1f, encoding %s, additional UV %u, file size %zu bytes, parse end %zu",
              model.header.version,
              model.is_utf8() ? "UTF-8" : "UTF-16LE",
              unsigned(model.header.add_uv_cnt),
              model.file_size,
              model.parse_end_offset);
  BKE_reportf(CTX_wm_reports(C),
              RPT_INFO,
              "PMX tail sections: %zu display frames, %zu rigid bodies, %zu joints",
              model.display_frames.size(),
              model.rigid_bodies.size(),
              model.joints.size());

  try {
    bool physics_definition_persisted = false;

    /* --- Phase 0: Create one unified model root. --- */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: model root");
    create_model_root(ctx, model);

    /* --- Phase 1: Create mesh objects. --- */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: mesh");
    create_mesh_object(ctx, model);
    BKE_reportf(CTX_wm_reports(C),
                RPT_INFO,
                "Created mesh (%zu vertices, %zu triangles, %zu objects)",
                model.vertices.size(),
                model.face_indices.size() / 3,
                ctx.mesh_objects.size());
    report_pmx_material_import_summary(ctx);

    /* --- Phase 2: Create armature object and bones. --- */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: armature");
    const char *base_name = model.name_local.empty() ? "PMXModel" : model.name_local.c_str();
    PMXArmatureResult arm_result = create_armature_object(
        bmain, model, params, base_name, view_layer, ctx.root_obj, ctx.model_collection);

    if (arm_result.armature_obj) {
      ctx.armature_obj = arm_result.armature_obj;

      BKE_reportf(CTX_wm_reports(C),
                  RPT_INFO,
                  "Created armature with %d bones", int(arm_result.bone_names.size()));

      /* --- Phase 2A: Build the in-memory MMD physics definition. */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: physics definition");
      const mmd_physics::MMDPhysicsBuildResult physics_result =
          mmd_physics::build_physics_definition(model,
                                                arm_result.bone_names,
                                                base_name,
                                                params.global_scale);
      BKE_reportf(CTX_wm_reports(C),
                  physics_result.success() ? RPT_INFO : RPT_WARNING,
                  "PMX physics definition: %zu rigid bodies, %zu joints, %d validation errors",
                  physics_result.definition.rigid_bodies.size(),
                  physics_result.definition.joints.size(),
                  physics_result.definition.validation.total_errors);
      for (const std::string &error : physics_result.errors) {
        BKE_reportf(CTX_wm_reports(C), RPT_WARNING, "PMX physics definition: %s", error.c_str());
      }
      if (physics_result.success() && ctx.model_collection) {
        if (mmd_physics::serialize_physics_definition(
                *ctx.model_collection, physics_result.definition, CTX_wm_reports(C)))
        {
          physics_definition_persisted = true;
          BKE_reportf(CTX_wm_reports(C),
                      RPT_INFO,
                      "PMX physics definition persisted: schema %d, %zu rigid bodies, %zu joints",
                      physics_result.definition.schema_version,
                      physics_result.definition.rigid_bodies.size(),
                      physics_result.definition.joints.size());
        }
      }

      /* --- Phase 2B: Read back the persisted definition and validate the current armature mapping. */
      if (ctx.model_collection && physics_definition_persisted) {
        BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX physics mapping: reading persisted definition");
        mmd_physics::MMDPhysicsDefinition persisted_definition;
        if (mmd_physics::deserialize_physics_definition(
                *ctx.model_collection, persisted_definition, CTX_wm_reports(C)))
        {
          const mmd_physics::MMDPhysicsMappingReport mapping_report =
              mmd_physics::validate_physics_mapping(
                  persisted_definition,
                  reinterpret_cast<bArmature *>(arm_result.armature_obj->data));
          report_physics_mapping(persisted_definition, mapping_report, CTX_wm_reports(C));
          const mmd_physics::MMDPhysicsDebugReport debug_report =
              mmd_physics::build_physics_debug_report(persisted_definition, mapping_report);
          report_physics_debug(debug_report, CTX_wm_reports(C));
        }
        else {
          BKE_report(CTX_wm_reports(C),
                     RPT_WARNING,
                     "PMX physics mapping: persisted definition could not be read");
        }
      }
      else if (ctx.model_collection) {
        BKE_report(CTX_wm_reports(C),
                   RPT_WARNING,
                   "PMX physics mapping: persisted definition is unavailable");
      }

      /* --- Phase 2C: D1 IK definition data base (no Blender IK constraint). --- */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: IK definition");
      persist_bone_ik_definition(ctx, model);

      /* --- Phase 2D: D2 append-transform definition data base (no Blender
       * constraint, red line D2-a). --- */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: append transform definition");
      persist_bone_append_definition(ctx, model);

      /* --- Phase 2E: D3 axis / deform-layer definition data base (no Blender
       * constraint, no bone-matrix change; red lines D3-a / D3-b / D3-c). --- */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: axis / deform definition");
      persist_bone_axis_definition(ctx, model);

      /* --- Phase 3: Assign vertex weights (handles both single and split mode). */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: vertex weights");
      assign_all_vertex_weights(ctx, model, arm_result.bone_names);
      BKE_reportf(CTX_wm_reports(C),
                  RPT_INFO,
                  "Assigned vertex groups for %zu objects", ctx.mesh_objects.size());

      /* --- Phase 4: Bind armature modifiers. */
      BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: armature modifiers");
      bind_armature_modifiers(ctx);
      BKE_reportf(CTX_wm_reports(C),
                  RPT_INFO,
                  "Bound armature modifier to %zu objects", ctx.mesh_objects.size());
    }

    /* --- Phase 4A: Persist the PMX per-vertex edge scale as vertex-group metadata.
     * This runs whether or not an armature exists because it only records PMX
     * data; the toon-edge preview in the MMD Render panel reads it back later. */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: edge scale metadata");
    write_all_pmx_edge_scale_groups(ctx, model);

    /* --- Phase 5: Import PMX vertex morphs as Blender shape keys. */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: vertex morphs");
    ctx.morph_names = import_vertex_morphs(ctx, model);
    BKE_reportf(CTX_wm_reports(C), RPT_INFO, "Imported %zu vertex morphs", ctx.morph_names.size());

    /* --- Phase 5A: Validate Group Morphs and finalize raw Controller channels. */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: group morph channels");
    const std::vector<PMXMorphChannel> raw_channels = build_default_morph_channels(model);
    const PMXGroupMorphReport graph_report = analyze_group_morphs(model, raw_channels);
    ctx.group_morph_expression_report = expand_group_morph_expressions(model, graph_report);
    std::vector<std::string> vertex_names;
    vertex_names.reserve(ctx.morph_names.size());
    for (const std::string &name : ctx.morph_names) {
      vertex_names.push_back(name);
    }
    ctx.group_morph_report = finalize_controller_channels(
        model, graph_report, ctx.morph_indices.as_span(), vertex_names);
    /* Reuse the finalized Blender names for the already-expanded expressions. */
    ctx.group_morph_expression_report.channels = ctx.group_morph_report.channels;
    ctx.group_morph_expression_report.valid = ctx.group_morph_report.valid &&
                                              ctx.group_morph_expression_report.valid;
    BKE_reportf(CTX_wm_reports(C),
                ctx.group_morph_report.valid ? RPT_INFO : RPT_WARNING,
                "PMX Group Morph channels: %d errors, %d warnings, %d controller channels",
                ctx.group_morph_report.error_count,
                ctx.group_morph_report.warning_count,
                ctx.group_morph_report.supported_channel_count);
    for (const PMXGroupMorphIssue &issue : ctx.group_morph_report.issues) {
      BKE_reportf(CTX_wm_reports(C),
                  issue.severity == PMXGroupMorphIssue::Severity::Error ? RPT_WARNING : RPT_INFO,
                  "PMX Group Morph: %s",
                  issue.message.c_str());
    }

    /* --- Phase 6: Create the shared vertex morph controller and drivers. */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: morph controller");
    create_morph_controller(ctx);

    /* --- Phase 7: Retain the PMX sections that have no Blender representation,
     * plus the per-vertex identity map. This runs last so every Blender name it
     * records (materials, bones, Shape Keys) is already final. It only writes
     * metadata and never changes imported geometry. */
    BKE_report(CTX_wm_reports(C), RPT_INFO, "PMX import phase: source data retention");
    persist_pmx_source_data(ctx, model, arm_result.bone_names);
  }
  catch (const std::exception &e) {
    BKE_reportf(CTX_wm_reports(C), RPT_ERROR, "PMX import failed for '%s': %s", params.filepath, e.what());
    return;
  }

  /* Notify dependency graph of all created objects. */
  if (ctx.root_obj) {
    Base *base = BKE_view_layer_base_find(view_layer, ctx.root_obj);
    if (base) {
      BKE_view_layer_base_select_and_set_active(view_layer, base);
    }
    DEG_id_tag_update_ex(bmain, &ctx.root_obj->id,
                         ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  }
  DEG_relations_tag_update(bmain);

  BKE_reportf(CTX_wm_reports(C), RPT_INFO,
              "PMX import complete: '%s' (%zu vertices, %zu triangles, %zu materials)",
              model.name_local.c_str(),
              model.vertices.size(),
              model.face_indices.size() / 3,
              model.materials.size());

  /* Hand the created armature back to the caller so post-import steps
   * (e.g. auto-apply MMD approximations) can target it. */
  params.result_armature = ctx.armature_obj;
}

}  // namespace blender::io::pmx
