/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "mmd_physics_diagnostics.hh"

#include "mmd_physics_world.hh"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <system_error>

#include "BKE_appdir.hh"
#include "BKE_deform.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"

#include "BLI_fileops.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_path_utils.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"

namespace blender::mmd_physics {

namespace {

bool float3_is_finite(const std::array<float, 3> &value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

uint32_t float_bits(const float value)
{
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value));
  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void write_json_string(FILE *file, const std::string &value)
{
  static constexpr char hex[] = "0123456789abcdef";
  fputc('"', file);
  for (const unsigned char byte : value) {
    switch (byte) {
      case '"':
        fputs("\\\"", file);
        break;
      case '\\':
        fputs("\\\\", file);
        break;
      case '\b':
        fputs("\\b", file);
        break;
      case '\f':
        fputs("\\f", file);
        break;
      case '\n':
        fputs("\\n", file);
        break;
      case '\r':
        fputs("\\r", file);
        break;
      case '\t':
        fputs("\\t", file);
        break;
      default:
        if (byte < 0x20) {
          const char escaped[7] = {'\\', 'u', '0', '0', hex[byte >> 4], hex[byte & 0x0f], '\0'};
          fputs(escaped, file);
        }
        else {
          fputc(byte, file);
        }
        break;
    }
  }
  fputc('"', file);
}

void write_json_float(FILE *file, const float value)
{
  if (!std::isfinite(value)) {
    fputs("null", file);
    return;
  }

  char buffer[32];
  const auto result = std::to_chars(
      buffer, buffer + sizeof(buffer), value, std::chars_format::general, 9);
  if (result.ec == std::errc()) {
    fwrite(buffer, 1, size_t(result.ptr - buffer), file);
  }
  else {
    fputs("null", file);
  }
}

void write_float3(FILE *file, const std::array<float, 3> &value)
{
  fputc('[', file);
  for (int i = 0; i < 3; i++) {
    if (i != 0) {
      fputc(',', file);
    }
    write_json_float(file, value[i]);
  }
  fputc(']', file);
}

void write_float4(FILE *file, const std::array<float, 4> &value)
{
  fputc('[', file);
  for (int i = 0; i < 4; i++) {
    if (i != 0) {
      fputc(',', file);
    }
    write_json_float(file, value[i]);
  }
  fputc(']', file);
}

void write_int3(FILE *file, const std::array<int, 3> &value)
{
  fprintf(file, "[%d,%d,%d]", value[0], value[1], value[2]);
}

void write_body(FILE *file, const MMDDiagnosticBodySample &body)
{
  fputs("{\"runtime_index\":", file);
  fprintf(file, "%d,\"pmx_index\":%d,\"name_local\":", body.runtime_index, body.pmx_index);
  write_json_string(file, body.name_local);
  fputs(",\"bone_name\":", file);
  write_json_string(file, body.bone_name);
  fprintf(file,
          ",\"physics_type\":%d,\"collision_group\":%d,\"no_collision_group\":%u,"
          "\"effective_bullet_group\":%u,\"effective_bullet_mask\":%u,\"mass\":",
          int(body.physics_type),
          int(body.collision_group),
          unsigned(body.no_collision_group),
          unsigned(body.effective_bullet_group),
          unsigned(body.effective_bullet_mask));
  write_json_float(file, body.mass);
  fputs(",\"linear_damping\":", file);
  write_json_float(file, body.linear_damping);
  fputs(",\"angular_damping\":", file);
  write_json_float(file, body.angular_damping);
  fputs(",\"position\":", file);
  write_float3(file, body.position);
  fputs(",\"quaternion\":", file);
  write_float4(file, body.quaternion);
  fputs(",\"linear_velocity\":", file);
  write_float3(file, body.linear_velocity);
  fputs(",\"angular_velocity\":", file);
  write_float3(file, body.angular_velocity);
  fprintf(file, ",\"activation\":%d,\"kinetic_energy\":", body.activation);
  write_json_float(file, body.kinetic_energy);
  fputc('}', file);
}

void write_joint(FILE *file, const MMDDiagnosticJointSample &joint)
{
  fprintf(file,
          "{\"runtime_index\":%d,\"pmx_index\":%d,\"name_local\":",
          joint.runtime_index,
          joint.pmx_index);
  write_json_string(file, joint.name_local);
  fprintf(file, ",\"A\":%d,\"B\":%d,\"frame_a\":{\"position\":", joint.rigid_a, joint.rigid_b);
  write_float3(file, joint.frame_a_position);
  fputs(",\"quaternion\":", file);
  write_float4(file, joint.frame_a_quaternion);
  fputs("},\"frame_b\":{\"position\":", file);
  write_float3(file, joint.frame_b_position);
  fputs(",\"quaternion\":", file);
  write_float4(file, joint.frame_b_quaternion);
  fputs("},\"angle\":", file);
  write_float3(file, joint.angle);
  fputs(",\"linear_diff\":", file);
  write_float3(file, joint.linear_diff);
  fputs(",\"angular_limits\":{\"lower\":", file);
  write_float3(file, joint.angular_lower_limit);
  fputs(",\"upper\":", file);
  write_float3(file, joint.angular_upper_limit);
  fputs("},\"linear_limits\":{\"lower\":", file);
  write_float3(file, joint.linear_lower_limit);
  fputs(",\"upper\":", file);
  write_float3(file, joint.linear_upper_limit);
  fputs("},\"angular_currentLimit\":", file);
  write_int3(file, joint.angular_current_limit);
  fputs(",\"angular_currentLimitError\":", file);
  write_float3(file, joint.angular_current_limit_error);
  fputs(",\"linear_currentLimit\":", file);
  write_int3(file, joint.linear_current_limit);
  fputs(",\"linear_currentLimitError\":", file);
  write_float3(file, joint.linear_current_limit_error);
  fprintf(file,
          ",\"constraint_config\":{\"use_frame_offset\":%s,\"enabled\":%s,"
          "\"needs_feedback\":%s,\"override_solver_iterations\":%d}",
          joint.use_frame_offset ? "true" : "false",
          joint.constraint_enabled ? "true" : "false",
          joint.needs_feedback ? "true" : "false",
          joint.override_solver_iterations);
  fputs(",\"linear_motor\":{\"limit_softness\":", file);
  write_json_float(file, joint.linear_limit_softness);
  fputs(",\"damping\":", file);
  write_json_float(file, joint.linear_damping);
  fputs(",\"restitution\":", file);
  write_json_float(file, joint.linear_restitution);
  fputs(",\"normal_cfm\":", file);
  write_float3(file, joint.linear_normal_cfm);
  fputs(",\"stop_erp\":", file);
  write_float3(file, joint.linear_stop_erp);
  fputs(",\"stop_cfm\":", file);
  write_float3(file, joint.linear_stop_cfm);
  fputs(",\"enable_motor\":", file);
  write_int3(file, joint.linear_enable_motor);
  fputs(",\"target_velocity\":", file);
  write_float3(file, joint.linear_target_velocity);
  fputs(",\"max_motor_force\":", file);
  write_float3(file, joint.linear_max_motor_force);
  fputs("}", file);
  fputs(",\"angular_motor\":{\"target_velocity\":", file);
  write_float3(file, joint.angular_target_velocity);
  fputs(",\"max_motor_force\":", file);
  write_float3(file, joint.angular_max_motor_force);
  fputs(",\"max_limit_force\":", file);
  write_float3(file, joint.angular_max_limit_force);
  fputs(",\"damping\":", file);
  write_float3(file, joint.angular_damping);
  fputs(",\"limit_softness\":", file);
  write_float3(file, joint.angular_limit_softness);
  fputs(",\"normal_cfm\":", file);
  write_float3(file, joint.angular_normal_cfm);
  fputs(",\"stop_erp\":", file);
  write_float3(file, joint.angular_stop_erp);
  fputs(",\"stop_cfm\":", file);
  write_float3(file, joint.angular_stop_cfm);
  fputs(",\"bounce\":", file);
  write_float3(file, joint.angular_bounce);
  fputs(",\"enable_motor\":", file);
  write_int3(file, joint.angular_enable_motor);
  fputs(",\"current_position\":", file);
  write_float3(file, joint.angular_current_position);
  fputs(",\"accumulated_impulse\":", file);
  write_float3(file, joint.angular_accumulated_impulse);
  fputs("}", file);
  fputs(",\"spring\":{\"enabled\":[", file);
  for (int axis = 0; axis < 6; axis++) {
    fprintf(file, "%s%d", axis == 0 ? "" : ",", joint.spring_enabled[axis]);
  }
  fputs("],\"stiffness\":[", file);
  for (int axis = 0; axis < 6; axis++) {
    if (axis != 0) {
      fputc(',', file);
    }
    write_json_float(file, joint.spring_stiffness[axis]);
  }
  fputs("],\"damping\":[", file);
  for (int axis = 0; axis < 6; axis++) {
    if (axis != 0) {
      fputc(',', file);
    }
    write_json_float(file, joint.spring_damping[axis]);
  }
  fputs("],\"equilibrium\":[", file);
  for (int axis = 0; axis < 6; axis++) {
    if (axis != 0) {
      fputc(',', file);
    }
    write_json_float(file, joint.spring_equilibrium[axis]);
  }
  fputs("]}", file);
  fputs(",\"constraint_applied_impulse\":", file);
  write_json_float(file, joint.applied_impulse);
  fputc('}', file);
}

void write_contact(FILE *file, const MMDDiagnosticContactSample &contact)
{
  fprintf(file, "{\"A\":%d,\"B\":%d,\"position_a\":", contact.body_a, contact.body_b);
  write_float3(file, contact.position_a);
  fputs(",\"position_b\":", file);
  write_float3(file, contact.position_b);
  fputs(",\"normal_on_b\":", file);
  write_float3(file, contact.normal_on_b);
  fputs(",\"distance\":", file);
  write_json_float(file, contact.distance);
  fputs(",\"contact_applied_impulse\":", file);
  write_json_float(file, contact.applied_impulse);
  fputc('}', file);
}

void write_mesh_sample(FILE *file, const MMDDiagnosticMeshSample &sample)
{
  fputs("{\"bone\":", file);
  write_json_string(file, sample.bone_name);
  fprintf(file, ",\"vertex\":%d,\"position\":", sample.vertex_index);
  write_float3(file, sample.position_world);
  fputs(",\"inside_body\":", file);
  fprintf(file, "%d,\"inside_body_bone\":", sample.inside_body_index);
  write_json_string(file, sample.inside_body_bone);
  fputs(",\"penetration_depth\":", file);
  write_json_float(file, sample.penetration_depth);
  fputc('}', file);
}

void write_frame(FILE *file, const MMDDiagnosticFrame &frame)
{
  fprintf(file,
          "{\"step\":%d,\"runtime\":{\"timer_elapsed\":%.17g,"
          "\"accumulator\":%.17g,\"fixed_steps\":%d,"
          "\"total_fixed_steps\":%llu,\"writeback\":%s},\"bodies\":[",
          frame.step,
          frame.runtime_timer_elapsed,
          frame.runtime_accumulator,
          frame.runtime_fixed_steps,
          static_cast<unsigned long long>(frame.runtime_total_fixed_steps),
          frame.runtime_writeback ? "true" : "false");
  for (const int i : frame.bodies.index_range()) {
    if (i != 0) {
      fputc(',', file);
    }
    write_body(file, frame.bodies[i]);
  }
  fputs("],\"joints\":[", file);
  for (const int i : frame.joints.index_range()) {
    if (i != 0) {
      fputc(',', file);
    }
    write_joint(file, frame.joints[i]);
  }
  fputs("],\"contacts\":[", file);
  for (const int i : frame.contacts.index_range()) {
    if (i != 0) {
      fputc(',', file);
    }
    write_contact(file, frame.contacts[i]);
  }
  fputs("],\"broadphase_pairs\":[", file);
  for (const int i : frame.broadphase_pairs.index_range()) {
    if (i != 0) {
      fputc(',', file);
    }
    fprintf(file, "[%d,%d]", frame.broadphase_pairs[i][0], frame.broadphase_pairs[i][1]);
  }
  fputs("],\"mesh_samples\":[", file);
  for (const int i : frame.mesh_samples.index_range()) {
    if (i != 0) {
      fputc(',', file);
    }
    write_mesh_sample(file, frame.mesh_samples[i]);
  }
  fputs("]}\n", file);
}

bool create_output_paths(Object *armature,
                         const MMDDiagnosticCaptureOptions &options,
                         std::string &r_jsonl_path,
                         std::string &r_summary_path)
{
  char safe_name[MAX_ID_NAME];
  BLI_strncpy(safe_name, armature->id.name + 2, sizeof(safe_name));
  if (safe_name[0] == '\0') {
    BLI_strncpy(safe_name, "Armature", sizeof(safe_name));
  }
  BLI_path_make_safe_filename(safe_name);

  char variant[208];
  snprintf(variant,
           sizeof(variant),
           "g_%08x_%08x_%08x_i_%08x_hz_%08x_n_%08x_p_%08x_c_%d_s_%d_d_%08x_x_%08x",
           unsigned(float_bits(options.gravity[0])),
           unsigned(float_bits(options.gravity[1])),
           unsigned(float_bits(options.gravity[2])),
           unsigned(options.solver_iterations),
           unsigned(options.fixed_step_hz),
           unsigned(options.steps),
           unsigned(options.startup_prewarm_steps),
           int(options.disable_rigid_body_contacts),
           int(options.disable_joint_springs),
           unsigned(float_bits(options.joint_spring_damping)),
           unsigned(options.joint_collision_exclusion_depth));
  char jsonl_name[FILE_MAXFILE];
  char summary_name[FILE_MAXFILE];
  snprintf(jsonl_name, sizeof(jsonl_name), "%s_%s_latest.jsonl", safe_name, variant);
  snprintf(summary_name, sizeof(summary_name), "%s_%s_latest_summary.json", safe_name, variant);

  char jsonl_path[FILE_MAX];
  char summary_path[FILE_MAX];
  BLI_path_join(jsonl_path,
                sizeof(jsonl_path),
                BKE_tempdir_base(),
                "mmd_physics_diagnostics",
                jsonl_name);
  BLI_path_join(summary_path,
                sizeof(summary_path),
                BKE_tempdir_base(),
                "mmd_physics_diagnostics",
                summary_name);
  if (!BLI_file_ensure_parent_dir_exists(jsonl_path) ||
      !BLI_file_ensure_parent_dir_exists(summary_path))
  {
    return false;
  }
  r_jsonl_path = jsonl_path;
  r_summary_path = summary_path;
  return true;
}

bool write_summary(const std::string &path,
                   const MMDDiagnosticCaptureOptions &options,
                   const int captured_steps,
                   const int body_count,
                   const int joint_count,
                   const std::string &jsonl_path)
{
  FILE *file = BLI_fopen(path.c_str(), "wb");
  if (file == nullptr) {
    return false;
  }

  fputs("{\"success\":true,\"schema_version\":2,\"coordinate_space\":"
        "\"blender_z_up\",\"capture_kind\":\"objective_physics_state\",\"jsonl_path\":",
        file);
  write_json_string(file, jsonl_path);
  fprintf(file,
          ",\"captured_steps\":%d,\"body_count\":%d,\"joint_count\":%d,"
          "\"simulation\":{\"startup_timestep_hz\":30,\"startup_prewarm_steps\":%d,"
          "\"recording_mode\":\"isolated_fixed_step_no_pose_feedback\"},"
          "\"options\":{\"fixed_step_hz\":%d,\"requested_steps\":%d,"
          "\"solver_iterations\":%d,\"disable_rigid_body_contacts\":%s,"
          "\"disable_joint_springs\":%s,\"joint_collision_exclusion_depth\":%d,"
          "\"joint_spring_damping\":",
          captured_steps,
          body_count,
          joint_count,
          options.startup_prewarm_steps,
          options.fixed_step_hz,
          options.steps,
          options.solver_iterations,
          options.disable_rigid_body_contacts ? "true" : "false",
          options.disable_joint_springs ? "true" : "false",
          options.joint_collision_exclusion_depth);
  write_json_float(file, options.joint_spring_damping);
  fputs(",\"gravity\":", file);
  write_float3(file, options.gravity);
  fputs(",\"mesh_sample\":{\"bones\":[", file);
  for (size_t i = 0; i < options.mesh_sample_bones.size(); i++) {
    if (i != 0) {
      fputc(',', file);
    }
    write_json_string(file, options.mesh_sample_bones[i]);
  }
  fprintf(file,
          "],\"interval\":%d,\"max_per_bone\":%d}}}\n",
          options.mesh_sample_interval,
          options.mesh_max_vertices_per_bone);

  const bool write_ok = ferror(file) == 0;
  const bool close_ok = fclose(file) == 0;
  return write_ok && close_ok;
}

bool delete_if_exists(const std::string &path)
{
  return !BLI_exists(path.c_str()) || BLI_delete(path.c_str(), false, false) == 0;
}

bool restore_backup_pair(const std::string &jsonl_path,
                         const std::string &summary_path,
                         const std::string &jsonl_backup_path,
                         const std::string &summary_backup_path,
                         std::string &r_error)
{
  if (!delete_if_exists(jsonl_path) || !delete_if_exists(summary_path)) {
    r_error = "failed to remove an incomplete published capture during rollback";
    return false;
  }
  if (BLI_rename_overwrite(jsonl_backup_path.c_str(), jsonl_path.c_str()) != 0) {
    r_error = "failed to restore the previous JSONL backup";
    return false;
  }
  if (BLI_rename_overwrite(summary_backup_path.c_str(), summary_path.c_str()) != 0) {
    if (BLI_rename_overwrite(jsonl_path.c_str(), jsonl_backup_path.c_str()) != 0) {
      r_error = "failed to restore the previous summary and failed to preserve its JSONL backup";
    }
    else {
      r_error = "failed to restore the previous capture summary";
    }
    return false;
  }
  return true;
}

bool recover_interrupted_publication(const std::string &jsonl_path,
                                     const std::string &summary_path,
                                     const std::string &jsonl_backup_path,
                                     const std::string &summary_backup_path,
                                     std::string &r_error)
{
  const bool has_jsonl = BLI_exists(jsonl_path.c_str());
  const bool has_summary = BLI_exists(summary_path.c_str());
  const bool has_jsonl_backup = BLI_exists(jsonl_backup_path.c_str());
  const bool has_summary_backup = BLI_exists(summary_backup_path.c_str());

  if (has_jsonl && has_summary) {
    if (!delete_if_exists(jsonl_backup_path) || !delete_if_exists(summary_backup_path)) {
      r_error = "failed to remove stale backups after a complete capture";
      return false;
    }
    return true;
  }
  if (has_jsonl_backup != has_summary_backup) {
    r_error = "incomplete backup pair from an interrupted capture";
    return false;
  }
  if (!has_jsonl && !has_summary) {
    if (!has_jsonl_backup) {
      return true;
    }
    return restore_backup_pair(
        jsonl_path, summary_path, jsonl_backup_path, summary_backup_path, r_error);
  }
  if (!has_jsonl_backup) {
    r_error = "incomplete published capture without a recoverable backup pair";
    return false;
  }
  return restore_backup_pair(
      jsonl_path, summary_path, jsonl_backup_path, summary_backup_path, r_error);
}

/* ------------------------------------------------------------------- */
/* Mesh vertex penetration sampling.                                   */
/* ------------------------------------------------------------------- */

/* A pre-selected mesh vertex with its world-space Blender position already
 * computed. Position is computed once from the evaluated mesh (reflecting
 * the current armature pose) and reused for every sampling step, because
 * the diagnostic capture does not feed physics back into the pose. */
struct MeshSampleVertex {
  std::string bone_name;
  int vertex_index = -1;
  std::array<float, 3> blender_position{};
};

struct MeshSampleSet {
  bool valid = false;
  std::vector<MeshSampleVertex> vertices;
};

Object *find_mesh_object_for_armature(Main *bmain, Object *armature)
{
  if (bmain == nullptr || armature == nullptr) {
    return nullptr;
  }
  for (Object &obj_ref : bmain->objects) {
    Object *obj = &obj_ref;
    if (obj->type == OB_MESH && obj->parent == armature) {
      return obj;
    }
  }
  return nullptr;
}

/* Build the pre-selected vertex set. Output volume is strictly bounded:
 *   - Only vertices whose top-weight bone is in `sample_bones` are considered.
 *   - Per bone, at most `max_per_bone` vertices are selected (uniform stride).
 *   - Positions are precomputed in Blender space so per-step sampling is cheap.
 * Returns a set with `valid=false` if no matching vertices were found. */
MeshSampleSet build_mesh_sample_set(Main *bmain,
                                    Depsgraph *depsgraph,
                                    Object *armature,
                                    const std::vector<std::string> &sample_bones,
                                    int max_per_bone)
{
  MeshSampleSet set;
  if (sample_bones.empty() || armature == nullptr || bmain == nullptr) {
    return set;
  }
  if (max_per_bone < 1) {
    max_per_bone = 1;
  }

  Object *mesh_obj = find_mesh_object_for_armature(bmain, armature);
  if (mesh_obj == nullptr) {
    return set;
  }

  const Mesh *mesh = nullptr;
  if (depsgraph != nullptr) {
    const Object *eval_obj = DEG_get_evaluated(depsgraph, mesh_obj);
    if (eval_obj != nullptr) {
      mesh = BKE_object_get_evaluated_mesh(eval_obj);
    }
  }
  if (mesh == nullptr) {
    mesh = reinterpret_cast<const Mesh *>(mesh_obj->data);
  }
  if (mesh == nullptr) {
    return set;
  }

  /* Map bone name -> vertex group index. MMD models use vertex group
   * names that match bone names exactly. */
  std::map<std::string, int> bone_to_vg_index;
  const ListBaseT<bDeformGroup> *defbase = BKE_object_defgroup_list(mesh_obj);
  if (defbase != nullptr) {
    int idx = 0;
    for (const bDeformGroup &dg_ref : *defbase) {
      const bDeformGroup *dg = &dg_ref;
      for (const std::string &bone : sample_bones) {
        if (dg->name == bone) {
          bone_to_vg_index[bone] = idx;
          break;
        }
      }
      idx++;
    }
  }
  if (bone_to_vg_index.empty()) {
    return set;
  }

  const Span<float3> positions = mesh->vert_positions();
  const Span<MDeformVert> dverts = mesh->deform_verts();
  if (positions.is_empty() || dverts.is_empty()) {
    return set;
  }

  /* Collect all vertices whose top-weight bone is in sample_bones,
   * grouped by bone name. */
  std::map<std::string, std::vector<int>> bone_to_all_vertices;
  for (const int vi : dverts.index_range()) {
    const MDeformVert &dv = dverts[vi];
    if (dv.dw == nullptr || dv.totweight == 0) {
      continue;
    }
    /* Find top-weight vertex group. */
    int top_idx = 0;
    float top_weight = dv.dw[0].weight;
    for (int j = 1; j < dv.totweight; j++) {
      if (dv.dw[j].weight > top_weight) {
        top_weight = dv.dw[j].weight;
        top_idx = j;
      }
    }
    const int vg_idx = dv.dw[top_idx].def_nr;

    /* Check if this vg_idx matches any sampled bone. */
    for (const auto &pair : bone_to_vg_index) {
      if (vg_idx == pair.second) {
        bone_to_all_vertices[pair.first].push_back(vi);
        break;
      }
    }
  }

  /* Precompute world-space Blender position for the selected vertices. */
  const float(*obj_mat)[4] = mesh_obj->object_to_world().ptr();

  for (const std::string &bone : sample_bones) {
    auto it = bone_to_all_vertices.find(bone);
    if (it == bone_to_all_vertices.end()) {
      continue;
    }
    const std::vector<int> &verts = it->second;
    const int total = int(verts.size());
    if (total == 0) {
      continue;
    }

    /* Uniform stride selection: pick at most max_per_bone vertices. */
    std::vector<int> selected;
    if (total <= max_per_bone) {
      selected = verts;
    }
    else {
      const int stride = total / max_per_bone;
      for (int i = 0; i < max_per_bone && i * stride < total; i++) {
        selected.push_back(verts[i * stride]);
      }
    }

    for (const int vi : selected) {
      if (vi < 0 || vi >= int(positions.size())) {
        continue;
      }
      const float3 &co = positions[vi];
      float world[3];
      mul_v3_m4v3(world, obj_mat, co);
      MeshSampleVertex sv;
      sv.bone_name = bone;
      sv.vertex_index = vi;
      sv.blender_position = {world[0], world[1], world[2]};
      set.vertices.push_back(std::move(sv));
    }
  }

  set.valid = !set.vertices.empty();
  return set;
}

/* Run penetration tests for all pre-selected vertices and append only
 * the penetrating ones to `r_frame.mesh_samples`. `r_frame.bodies` must
 * already be populated (capture_diagnostic_frame fills it before this is
 * called) so we can look up the body's bound bone name by runtime index. */
void capture_mesh_samples(const MMDPhysicsWorld &world,
                          const MeshSampleSet &sample_set,
                          MMDDiagnosticFrame &r_frame)
{
  if (!sample_set.valid) {
    return;
  }
  for (const MeshSampleVertex &sv : sample_set.vertices) {
    int body_index = -1;
    float depth = 0.0f;
    if (world.find_body_containing_point(sv.blender_position, nullptr, body_index, depth)) {
      MMDDiagnosticMeshSample sample;
      sample.bone_name = sv.bone_name;
      sample.vertex_index = sv.vertex_index;
      sample.position_world = sv.blender_position;
      sample.inside_body_index = body_index;
      sample.penetration_depth = depth;
      if (body_index >= 0 && body_index < int(r_frame.bodies.size())) {
        sample.inside_body_bone = r_frame.bodies[body_index].bone_name;
      }
      r_frame.mesh_samples.append(std::move(sample));
    }
  }
}

}  // namespace

bool append_mmd_physics_diagnostic_frame(const std::string &path,
                                         const MMDDiagnosticFrame &frame)
{
  FILE *file = BLI_fopen(path.c_str(), "ab");
  if (file == nullptr) {
    return false;
  }
  write_frame(file, frame);
  fputc('\n', file);
  const bool write_ok = ferror(file) == 0;
  const bool close_ok = fclose(file) == 0;
  return write_ok && close_ok;
}

MMDDiagnosticCaptureResult capture_mmd_physics_diagnostics(
    const MMDPhysicsDefinition &definition,
    Object *armature,
    Main *bmain,
    Depsgraph *depsgraph,
    const MMDDiagnosticCaptureOptions &options)
{
  MMDDiagnosticCaptureResult result;
  if (armature == nullptr || bmain == nullptr) {
    result.error = "MMD Physics diagnostics: invalid Blender context";
    return result;
  }
  if (options.fixed_step_hz <= 0 || options.steps <= 0 || options.solver_iterations <= 0 ||
      !float3_is_finite(options.gravity) || !std::isfinite(options.joint_spring_damping) ||
      options.joint_spring_damping < 0.0f || options.joint_spring_damping > 1.0f ||
       options.startup_prewarm_steps < 0 || options.startup_prewarm_steps > 16 ||
       options.joint_collision_exclusion_depth < 0 ||
      options.joint_collision_exclusion_depth > 8)
  {
    result.error = "MMD Physics diagnostics: invalid capture options";
    return result;
  }
  if (!create_output_paths(armature, options, result.jsonl_path, result.summary_path)) {
    result.error = "MMD Physics diagnostics: failed to create output directory";
    return result;
  }

  MMDPhysicsWorld world;
  if (!world.initialize(definition,
                        armature,
                        bmain,
                        options.gravity.data(),
                        options.solver_iterations,
                        options.fixed_step_hz,
                        8,
                        options.disable_rigid_body_contacts,
                        options.disable_joint_springs,
                        options.joint_spring_damping,
                        options.joint_collision_exclusion_depth))
  {
    result.error = "MMD Physics diagnostics: temporary world initialization failed";
    return result;
  }
  world.mute_physics_bone_constraints();
  world.disconnect_physics_bones();
  struct RestorePhysicsPose {
    MMDPhysicsWorld &world;
    ~RestorePhysicsPose()
    {
      world.restore_physics_bone_connections();
      world.restore_physics_bone_constraints();
    }
  } restore_physics_pose{world};
  if (!world.temporal_kinematic_init()) {
    result.error = "MMD Physics diagnostics: temporal kinematic initialization failed";
    return result;
  }

  /* Optional: gradually interpolate kinematic bodies from rest pose to VMD
   * pose over N frames, letting spring delta build up smoothly. Mirrors MMP
   * `_sync_to_start_pose`. Default 0 = no-op (bodies stay at snapped pose). */
  world.startup_sync(options.startup_sync_steps);

  /* Match the real-time Start integration sequence while allowing objective
   * probes of individual prewarm stages. The production-equivalent default
   * is two 1/30 s frames. */
  constexpr float startup_timestep = 1.0f / 30.0f;
  constexpr int startup_max_substeps = 8;
  world.prewarm(options.startup_prewarm_steps, startup_timestep, startup_max_substeps);

  const std::string jsonl_temp_path = result.jsonl_path + ".tmp";
  const std::string summary_temp_path = result.summary_path + ".tmp";
  const std::string jsonl_backup_path = result.jsonl_path + ".bak";
  const std::string summary_backup_path = result.summary_path + ".bak";
  std::string recovery_error;
  if (!recover_interrupted_publication(result.jsonl_path,
                                       result.summary_path,
                                       jsonl_backup_path,
                                       summary_backup_path,
                                       recovery_error))
  {
    result.error = "MMD Physics diagnostics: " + recovery_error;
    return result;
  }
  if (!delete_if_exists(jsonl_temp_path) || !delete_if_exists(summary_temp_path)) {
    result.error = "MMD Physics diagnostics: failed to remove stale temporary outputs";
    return result;
  }
  FILE *jsonl = BLI_fopen(jsonl_temp_path.c_str(), "wb");
  if (jsonl == nullptr) {
    result.error = "MMD Physics diagnostics: failed to open JSONL output";
    return result;
  }

  result.body_count = world.body_count();
  result.joint_count = world.joint_count();

  /* Pre-build the mesh vertex sample set once. Positions are computed from
   * the evaluated mesh (current pose) and reused for every sampling step.
   * Invalid (empty) set disables mesh sampling with zero overhead. */
  const MeshSampleSet mesh_set = build_mesh_sample_set(bmain,
                                                        depsgraph,
                                                        armature,
                                                        options.mesh_sample_bones,
                                                        options.mesh_max_vertices_per_bone);

  MMDDiagnosticFrame frame;
  const float timestep = 1.0f / float(options.fixed_step_hz);
  bool capture_ok = true;
  if (options.capture_initial_state) {
    if (!world.capture_diagnostic_frame(-1, frame)) {
      capture_ok = false;
    }
    else {
      write_frame(jsonl, frame);
      capture_ok = ferror(jsonl) == 0;
    }
  }
  for (int step = 0; step < options.steps; step++) {
    if (!capture_ok) {
      break;
    }
    world.step(timestep, 1, false, timestep);
    if (!world.capture_diagnostic_frame(step, frame)) {
      capture_ok = false;
      break;
    }
    /* Mesh vertex penetration sampling: only on the configured interval
     * and only when the sample set is valid. Non-penetrating vertices
     * produce no output, so the JSONL size stays bounded. */
    if (mesh_set.valid && options.mesh_sample_interval > 0 &&
        (step % options.mesh_sample_interval == 0))
    {
      capture_mesh_samples(world, mesh_set, frame);
    }
    write_frame(jsonl, frame);
    if (ferror(jsonl) != 0) {
      capture_ok = false;
      break;
    }
    result.captured_steps++;
  }
  if (fclose(jsonl) != 0) {
    capture_ok = false;
  }
  if (!capture_ok) {
    delete_if_exists(jsonl_temp_path);
    result.error = "MMD Physics diagnostics: failed while writing JSONL output";
    return result;
  }

  /* Write both temporary files before touching the last successful capture.
   * Publishing two files cannot be atomic, so move the old pair to backups
   * and restore it on every ordinary failure path. The summary remains the
   * publication gate consumers use to discover a matching JSONL file. */
  if (!write_summary(summary_temp_path,
                     options,
                     result.captured_steps,
                     result.body_count,
                     result.joint_count,
                     result.jsonl_path))
  {
    delete_if_exists(jsonl_temp_path);
    delete_if_exists(summary_temp_path);
    result.error = "MMD Physics diagnostics: failed to write capture summary";
    return result;
  }

  const bool had_jsonl = BLI_exists(result.jsonl_path.c_str());
  const bool had_summary = BLI_exists(result.summary_path.c_str());
  if (had_jsonl &&
      BLI_rename_overwrite(result.jsonl_path.c_str(), jsonl_backup_path.c_str()) != 0)
  {
    delete_if_exists(jsonl_temp_path);
    delete_if_exists(summary_temp_path);
    result.error = "MMD Physics diagnostics: failed to back up previous JSONL output";
    return result;
  }
  if (had_summary &&
      BLI_rename_overwrite(result.summary_path.c_str(), summary_backup_path.c_str()) != 0)
  {
    const bool rollback_ok = !had_jsonl ||
                             BLI_rename_overwrite(jsonl_backup_path.c_str(),
                                                  result.jsonl_path.c_str()) == 0;
    delete_if_exists(jsonl_temp_path);
    delete_if_exists(summary_temp_path);
    result.error = rollback_ok ?
                       "MMD Physics diagnostics: failed to back up previous capture summary" :
                       "MMD Physics diagnostics: failed to back up summary and restore JSONL";
    return result;
  }

  if (BLI_rename_overwrite(jsonl_temp_path.c_str(), result.jsonl_path.c_str()) != 0) {
    std::string rollback_error;
    const bool rollback_ok = (!had_jsonl && !had_summary) ||
                             restore_backup_pair(result.jsonl_path,
                                                 result.summary_path,
                                                 jsonl_backup_path,
                                                 summary_backup_path,
                                                 rollback_error);
    delete_if_exists(jsonl_temp_path);
    delete_if_exists(summary_temp_path);
    result.error = rollback_ok ? "MMD Physics diagnostics: failed to publish JSONL output" :
                                 "MMD Physics diagnostics: failed to publish JSONL and " +
                                     rollback_error;
    return result;
  }
  if (BLI_rename_overwrite(summary_temp_path.c_str(), result.summary_path.c_str()) != 0) {
    std::string rollback_error;
    const bool rollback_ok = (!had_jsonl && !had_summary) ? delete_if_exists(result.jsonl_path) :
                                                           restore_backup_pair(result.jsonl_path,
                                                                               result.summary_path,
                                                                               jsonl_backup_path,
                                                                               summary_backup_path,
                                                                               rollback_error);
    delete_if_exists(summary_temp_path);
    result.error = rollback_ok ?
                       "MMD Physics diagnostics: failed to publish capture summary" :
                       "MMD Physics diagnostics: failed to publish summary and " + rollback_error;
    return result;
  }
  if (!delete_if_exists(jsonl_backup_path) || !delete_if_exists(summary_backup_path)) {
    result.error = "MMD Physics diagnostics: capture published but failed to remove backups";
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace blender::mmd_physics
