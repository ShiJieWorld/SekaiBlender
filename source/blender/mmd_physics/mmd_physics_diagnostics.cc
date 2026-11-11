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
#include <set>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

#include "BKE_appdir.hh"
#include "BKE_collection.hh"
#include "BKE_customdata.hh"
#include "BKE_deform.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"

#include "BLI_fileops.hh"
#include "BLI_hash_md5.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_path_utils.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_vector.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_layer_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"

namespace blender::mmd_physics {

namespace {

struct MeshSamplingStats {
  bool enabled = false;
  int mesh_object_count = 0;
  int eligible_mesh_object_count = 0;
  int mesh_objects_with_vertex_groups = 0;
  int evaluated_mesh_object_count = 0;
  int bind_pose_mesh_object_count = 0;
  int evaluated_mesh_failure_count = 0;
  int topology_mismatch_count = 0;
  int requested_bone_count = 0;
  int matched_bone_count = 0;
  int sampled_bone_count = 0;
  int selected_vertex_count = 0;
  int sampled_step_count = 0;
  uint64_t vertex_test_count = 0;
  uint64_t nonpenetrating_count = 0;
  uint64_t penetration_sample_count = 0;
  uint64_t emitted_record_count = 0;
  std::vector<std::string> sampled_mesh_objects;
  std::string status = "disabled";
  std::string reason = "mesh_sampling_not_requested";
  std::string pose_mode = "not_applicable";
};

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

std::string string_list_hash(const std::vector<std::string> &values)
{
  std::vector<std::string> sorted_values = values;
  std::sort(sorted_values.begin(), sorted_values.end());
  sorted_values.erase(std::unique(sorted_values.begin(), sorted_values.end()), sorted_values.end());
  std::string identity;
  for (const std::string &value : sorted_values) {
    identity += std::to_string(value.size());
    identity += ':';
    identity += value;
    identity += ';';
  }
  unsigned char digest[16];
  char hex_digest[33];
  BLI_hash_md5_buffer(identity.data(), identity.size(), digest);
  BLI_hash_md5_to_hexdigest(digest, hex_digest);
  return hex_digest;
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

void write_json_double(FILE *file, const double value)
{
  if (!std::isfinite(value)) {
    fputs("null", file);
    return;
  }
  char buffer[64];
  const auto result = std::to_chars(
      buffer, buffer + sizeof(buffer), value, std::chars_format::general, 17);
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
  fputs("{\"bone_name\":", file);
  write_json_string(file, sample.bone_name);
  fputs(",\"mesh_object_name\":", file);
  write_json_string(file, sample.mesh_object_name);
  fprintf(file, ",\"vertex_index\":%d,\"position_world\":", sample.vertex_index);
  write_float3(file, sample.position_world);
  fputs(",\"inside_body_index\":", file);
  fprintf(file, "%d,\"inside_body_bone\":", sample.inside_body_index);
  write_json_string(file, sample.inside_body_bone);
  fputs(",\"penetration_depth\":", file);
  write_json_float(file, sample.penetration_depth);
  fputs(",\"penetration_metric\":\"aabb_nearest_face_containment\"}", file);
}

void write_frame(FILE *file, const MMDDiagnosticFrame &frame)
{
  fprintf(file,
          "{\"step\":%d,\"runtime\":{\"timer_elapsed\":",
          frame.step);
  write_json_double(file, frame.runtime_timer_elapsed);
  fputs(",\"accumulator\":", file);
  write_json_double(file, frame.runtime_accumulator);
  fprintf(file,
          ",\"fixed_steps\":%d,\"total_fixed_steps\":%llu,\"writeback\":%s},\"bodies\":[",
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
  const std::string mesh_bone_hash = string_list_hash(options.mesh_sample_bones);
  char variant[256];
  snprintf(variant,
           sizeof(variant),
           "g_%08x_%08x_%08x_i_%08x_hz_%08x_n_%08x_p_%08x_u_%08x_q_%d_c_%d_s_%d_d_%08x_x_%08x_m_%s_%08x_%08x",
           unsigned(float_bits(options.gravity[0])),
           unsigned(float_bits(options.gravity[1])),
           unsigned(float_bits(options.gravity[2])),
           unsigned(options.solver_iterations),
           unsigned(options.fixed_step_hz),
           unsigned(options.steps),
           unsigned(options.startup_prewarm_steps),
           unsigned(options.startup_sync_steps),
           int(options.capture_initial_state),
           int(options.disable_rigid_body_contacts),
           int(options.disable_joint_springs),
           unsigned(float_bits(options.joint_spring_damping)),
           unsigned(options.joint_collision_exclusion_depth),
           mesh_bone_hash.c_str(),
           unsigned(options.mesh_sample_interval),
           unsigned(options.mesh_max_vertices_per_bone));
  char jsonl_name[FILE_MAXFILE];
  char summary_name[FILE_MAXFILE];
  /* Compact identity avoids filesystem component truncation for long model names.
   * MD5 is a stable local filename identity, not a security/provenance claim. */
  const std::string capture_hash = string_list_hash(
      {std::string(armature->id.name + 2), std::string(variant), "r27_level_a"});
  snprintf(jsonl_name, sizeof(jsonl_name), "capture_%s_latest.jsonl", capture_hash.c_str());
  snprintf(summary_name,
           sizeof(summary_name),
           "capture_%s_latest_summary.json",
           capture_hash.c_str());

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
                   const int first_step,
                   const int last_step,
                   const MeshSamplingStats &mesh_stats,
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
  fputs(",\"canonical_mesh_keys\":[\"bone_name\",\"mesh_object_name\",\"vertex_index\","
        "\"position_world\",\"inside_body_index\",\"inside_body_bone\","
        "\"penetration_depth\",\"penetration_metric\"]", file);
  fprintf(file,
          ",\"schema_revision\":\"r27_level_a\",\"frame_count\":%d,\"physical_step_count\":%d",
          captured_steps + (options.capture_initial_state ? 1 : 0),
          captured_steps);
  fprintf(file,
          ",\"captured_steps\":%d,"
          "\"first_step\":%d,\"last_step\":%d,"
          "\"physical_first_step\":%d,\"physical_last_step\":%d,"
          "\"body_count\":%d,\"joint_count\":%d,"
          "\"simulation\":{\"startup_timestep_hz\":30,\"startup_prewarm_steps\":%d,"
          "\"startup_sync_steps\":%d,"
          "\"recording_mode\":\"isolated_fixed_step_no_pose_feedback\"},"
          "\"options\":{\"fixed_step_hz\":%d,\"requested_steps\":%d,"
          "\"solver_iterations\":%d,\"disable_rigid_body_contacts\":%s,"
          "\"disable_joint_springs\":%s,\"joint_collision_exclusion_depth\":%d,"
          "\"joint_spring_damping\":",
          captured_steps,
          first_step,
          last_step,
          captured_steps > 0 ? 0 : -1,
          captured_steps > 0 ? captured_steps - 1 : -1,
          body_count,
          joint_count,
          options.startup_prewarm_steps,
          options.startup_sync_steps,
          options.fixed_step_hz,
          options.steps,
          options.solver_iterations,
          options.disable_rigid_body_contacts ? "true" : "false",
          options.disable_joint_springs ? "true" : "false",
          options.joint_collision_exclusion_depth);
  write_json_float(file, options.joint_spring_damping);
  fprintf(file, ",\"capture_initial_state\":%s,\"gravity\":",
          options.capture_initial_state ? "true" : "false");
  write_float3(file, options.gravity);
  fputs(",\"mesh_sample\":{\"bones\":[", file);
  for (size_t i = 0; i < options.mesh_sample_bones.size(); i++) {
    if (i != 0) {
      fputc(',', file);
    }
    write_json_string(file, options.mesh_sample_bones[i]);
  }
  fprintf(file,
          "],\"interval\":%d,\"max_per_bone\":%d}},"
          "\"mesh_sampling\":{\"enabled\":%s,\"mesh_object_count\":%d,"
          "\"eligible_mesh_object_count\":%d,\"mesh_objects_with_vertex_groups\":%d,"
          "\"evaluated_mesh_object_count\":%d,\"bind_pose_mesh_object_count\":%d,"
          "\"evaluated_mesh_failure_count\":%d,\"topology_mismatch_count\":%d,"
          "\"requested_bone_count\":%d,\"matched_bone_count\":%d,"
          "\"sampled_bone_count\":%d,\"selected_vertex_count\":%d,\"sampled_step_count\":%d,"
          "\"vertex_test_count\":%llu,\"nonpenetrating_count\":%llu,"
          "\"penetration_sample_count\":%llu,\"emitted_record_count\":%llu,\"status\":",
          options.mesh_sample_interval,
          options.mesh_max_vertices_per_bone,
          mesh_stats.enabled ? "true" : "false",
          mesh_stats.mesh_object_count,
          mesh_stats.eligible_mesh_object_count,
          mesh_stats.mesh_objects_with_vertex_groups,
          mesh_stats.evaluated_mesh_object_count,
          mesh_stats.bind_pose_mesh_object_count,
          mesh_stats.evaluated_mesh_failure_count,
          mesh_stats.topology_mismatch_count,
          mesh_stats.requested_bone_count,
          mesh_stats.matched_bone_count,
          mesh_stats.sampled_bone_count,
          mesh_stats.selected_vertex_count,
          mesh_stats.sampled_step_count,
          static_cast<unsigned long long>(mesh_stats.vertex_test_count),
          static_cast<unsigned long long>(mesh_stats.nonpenetrating_count),
          static_cast<unsigned long long>(mesh_stats.penetration_sample_count),
          static_cast<unsigned long long>(mesh_stats.emitted_record_count));
  write_json_string(file, mesh_stats.status);
  fputs(",\"reason\":", file);
  write_json_string(file, mesh_stats.reason);
  fputs(",\"pose_mode\":", file);
  write_json_string(file, mesh_stats.pose_mode);
  fputs(",\"penetration_metric\":\"aabb_nearest_face_containment\","
        "\"sampled_mesh_objects\":[", file);
  for (size_t i = 0; i < mesh_stats.sampled_mesh_objects.size(); i++) {
    if (i != 0) {
      fputc(',', file);
    }
    write_json_string(file, mesh_stats.sampled_mesh_objects[i]);
  }
  fputs("]}}\n", file);

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
  std::string mesh_object_name;
  int vertex_index = -1;
  std::array<float, 3> blender_position{};
};

struct MeshSampleSet {
  bool valid = false;
  std::vector<MeshSampleVertex> vertices;
  MeshSamplingStats stats;
};

bool mesh_is_bound_to_armature(const Object &mesh_obj, const Object &armature)
{
  if (mesh_obj.parent == &armature) {
    return true;
  }
  for (const ModifierData *md = static_cast<const ModifierData *>(mesh_obj.modifiers.first());
       md != nullptr;
       md = md->next)
  {
    if (md->type != eModifierType_Armature) {
      continue;
    }
    const ArmatureModifierData *armature_md = reinterpret_cast<const ArmatureModifierData *>(md);
    if (armature_md->object == &armature) {
      return true;
    }
  }
  return false;
}

/* A vertex-index identity proof is required before pairing source deform
 * weights with evaluated positions. A missing CD_ORIGINDEX is safe only when
 * every enabled modifier is known to preserve mesh topology. Unknown or
 * constructive modifiers are rejected rather than silently pairing by index. */
bool evaluated_mesh_preserves_vertex_identity(const Object &mesh_obj,
                                               const Depsgraph *depsgraph,
                                               const Mesh &evaluated_mesh)
{
  const int *origindex = static_cast<const int *>(
      CustomData_get_layer(&evaluated_mesh.vert_data, CD_ORIGINDEX));
  if (origindex != nullptr) {
    for (const int vi : evaluated_mesh.vert_positions().index_range()) {
      if (origindex[vi] != vi) {
        return false;
      }
    }
    return true;
  }
  if (depsgraph == nullptr || (mesh_obj.mode & OB_MODE_EDIT) != 0) {
    return false;
  }
  const int required_mode = DEG_get_mode(depsgraph) == DAG_EVAL_RENDER ? eModifierMode_Render :
                                                                      eModifierMode_Realtime;
  for (const ModifierData *md = static_cast<const ModifierData *>(mesh_obj.modifiers.first());
       md != nullptr;
       md = md->next)
  {
    if ((md->mode & required_mode) == 0) {
      continue;
    }
    const ModifierTypeInfo *modifier_type = BKE_modifier_get_info(md->type);
    if (modifier_type == nullptr || modifier_type->type != ModifierTypeType::OnlyDeform) {
      return false;
    }
  }
  return true;
}

/* Build the pre-selected vertex set. Output volume is strictly bounded:
 *   - Only meshes in the selected model collection and bound to the armature are considered.
 *   - Only vertices whose top-weight bone is in `sample_bones` are considered.
 *   - Per bone, at most `max_per_bone` vertices are selected across the whole model.
 *   - Positions are precomputed in Blender space so per-step sampling is cheap.
 *   - A depsgraph capture rejects evaluated meshes whose topology/deformation
 *     data cannot be matched to the source vertex identity; it does not silently
 *     remap changed topology. */
MeshSampleSet build_mesh_sample_set(Collection *model_collection,
                                    Depsgraph *depsgraph,
                                    Object *armature,
                                    const std::vector<std::string> &sample_bones,
                                    int max_per_bone)
{
  MeshSampleSet set;
  set.stats.enabled = !sample_bones.empty();
  set.stats.requested_bone_count = int(std::set<std::string>(sample_bones.begin(),
                                                             sample_bones.end())
                                           .size());
  if (!set.stats.enabled) {
    return set;
  }
  set.stats.pose_mode = "not_applicable";
  if (armature == nullptr || model_collection == nullptr) {
    set.stats.status = "not_run";
    set.stats.reason = "no_model_collection_or_armature";
    return set;
  }
  if (max_per_bone < 1) {
    max_per_bone = 1;
  }

  std::vector<Object *> mesh_objects;
  std::set<Object *> seen_objects;
  FOREACH_COLLECTION_OBJECT_RECURSIVE_BEGIN (model_collection, object) {
    if (object == nullptr || object->type != OB_MESH || !seen_objects.insert(object).second) {
      continue;
    }
    mesh_objects.push_back(object);
  }
  FOREACH_COLLECTION_OBJECT_RECURSIVE_END;
  std::sort(mesh_objects.begin(), mesh_objects.end(), [](const Object *a, const Object *b) {
    const char *a_name = a->id.name + 2;
    const char *b_name = b->id.name + 2;
    return std::strcmp(a_name, b_name) < 0;
  });
  set.stats.mesh_object_count = int(mesh_objects.size());

  const std::set<std::string> requested_bones(sample_bones.begin(), sample_bones.end());
  std::set<std::string> matched_bones;
  std::map<std::string, std::vector<MeshSampleVertex>> candidates_by_bone;

  for (Object *mesh_obj : mesh_objects) {
    if (!mesh_is_bound_to_armature(*mesh_obj, *armature) ||
        !BKE_collection_has_object_recursive(model_collection, mesh_obj))
    {
      continue;
    }

    const Object *eval_obj = mesh_obj;
    const Mesh *source_mesh = reinterpret_cast<const Mesh *>(mesh_obj->data);
    const Mesh *mesh = source_mesh;
    if (source_mesh == nullptr) {
      set.stats.evaluated_mesh_failure_count++;
      continue;
    }
    if (depsgraph != nullptr) {
      const Object *evaluated = DEG_get_evaluated(depsgraph, mesh_obj);
      const bool evaluated_ready = evaluated != nullptr && evaluated != mesh_obj &&
                                   DEG_is_evaluated(evaluated) &&
                                   DEG_object_geometry_is_evaluated(*evaluated) &&
                                   DEG_object_transform_is_evaluated(*evaluated);
      const Mesh *evaluated_mesh = evaluated_ready ? BKE_object_get_evaluated_mesh(evaluated) :
                                                    nullptr;
      if (evaluated_mesh == nullptr) {
        set.stats.evaluated_mesh_failure_count++;
        continue;
      }
      eval_obj = evaluated;
      mesh = evaluated_mesh;
      set.stats.evaluated_mesh_object_count++;
    }
    else {
      set.stats.bind_pose_mesh_object_count++;
    }
    set.stats.eligible_mesh_object_count++;

    std::map<int, std::string> vg_to_bone;
    /* Preserve source vertex identity: source deform weights and group names are
     * paired with evaluated positions only after the vertex count check below. */
    const ListBaseT<bDeformGroup> *defbase = BKE_object_defgroup_list(mesh_obj);
    if (defbase != nullptr) {
      int group_index = 0;
      for (const bDeformGroup &group_ref : *defbase) {
        const bDeformGroup *group = &group_ref;
        if (requested_bones.find(group->name) != requested_bones.end()) {
          vg_to_bone.emplace(group_index, group->name);
          matched_bones.insert(group->name);
        }
        group_index++;
      }
    }
    if (vg_to_bone.empty()) {
      continue;
    }
    set.stats.mesh_objects_with_vertex_groups++;

    const Span<float3> positions = mesh->vert_positions();
    const Span<MDeformVert> dverts = source_mesh->deform_verts();
    if (positions.is_empty() || dverts.is_empty()) {
      continue;
    }
    if (int(positions.size()) != source_mesh->verts_num || dverts.size() != positions.size()) {
      /* Level A retains source vertex identity; changed topology is not silently remapped. */
      set.stats.topology_mismatch_count++;
      continue;
    }
    if (depsgraph != nullptr &&
        !evaluated_mesh_preserves_vertex_identity(*eval_obj, depsgraph, *mesh)) {
      set.stats.topology_mismatch_count++;
      continue;
    }
    const float(*obj_mat)[4] = eval_obj->object_to_world().ptr();
    const std::string mesh_name = mesh_obj->id.name + 2;
    for (const int vi : dverts.index_range()) {
      if (vi < 0 || vi >= int(positions.size())) {
        continue;
      }
      const MDeformVert &dv = dverts[vi];
      if (dv.dw == nullptr || dv.totweight == 0) {
        continue;
      }
      int top_index = 0;
      float top_weight = dv.dw[0].weight;
      for (int weight_index = 1; weight_index < dv.totweight; weight_index++) {
        if (dv.dw[weight_index].weight > top_weight) {
          top_weight = dv.dw[weight_index].weight;
          top_index = weight_index;
        }
      }
      const auto bone_it = vg_to_bone.find(dv.dw[top_index].def_nr);
      if (bone_it == vg_to_bone.end()) {
        continue;
      }
      const float3 &co = positions[vi];
      float world[3];
      mul_v3_m4v3(world, obj_mat, co);
      MeshSampleVertex sample;
      sample.bone_name = bone_it->second;
      sample.mesh_object_name = mesh_name;
      sample.vertex_index = vi;
      sample.blender_position = {world[0], world[1], world[2]};
      candidates_by_bone[sample.bone_name].push_back(std::move(sample));
    }
  }

  set.stats.matched_bone_count = int(matched_bones.size());
  if (depsgraph == nullptr) {
    set.stats.pose_mode = "single_bind_pose";
  }
  else if (set.stats.evaluated_mesh_failure_count > 0 || set.stats.topology_mismatch_count > 0) {
    set.stats.pose_mode = set.stats.evaluated_mesh_object_count > 0 ?
                              "single_evaluated_pose_with_rejected_meshes" :
                              "evaluated_pose_unavailable";
  }
  else {
    set.stats.pose_mode = "single_evaluated_pose";
  }
  for (const std::string &bone : requested_bones) {
    auto candidates_it = candidates_by_bone.find(bone);
    if (candidates_it == candidates_by_bone.end() || candidates_it->second.empty()) {
      continue;
    }
    set.stats.sampled_bone_count++;
    std::vector<MeshSampleVertex> &candidates = candidates_it->second;
    std::sort(candidates.begin(), candidates.end(), [](const MeshSampleVertex &a,
                                                       const MeshSampleVertex &b) {
      return std::tie(a.bone_name, a.mesh_object_name, a.vertex_index) <
             std::tie(b.bone_name, b.mesh_object_name, b.vertex_index);
    });
    if (int(candidates.size()) <= max_per_bone) {
      set.vertices.insert(set.vertices.end(), candidates.begin(), candidates.end());
    }
    else if (max_per_bone == 1) {
      set.vertices.push_back(candidates.front());
    }
    else {
      const int total = int(candidates.size());
      for (int i = 0; i < max_per_bone; i++) {
        const int index = int((int64_t(i) * (total - 1)) / (max_per_bone - 1));
        set.vertices.push_back(candidates[index]);
      }
    }
  }

  std::sort(set.vertices.begin(), set.vertices.end(), [](const MeshSampleVertex &a,
                                                         const MeshSampleVertex &b) {
    return std::tie(a.bone_name, a.mesh_object_name, a.vertex_index) <
           std::tie(b.bone_name, b.mesh_object_name, b.vertex_index);
  });
  std::set<std::string> sampled_mesh_objects;
  for (const MeshSampleVertex &vertex : set.vertices) {
    sampled_mesh_objects.insert(vertex.mesh_object_name);
  }
  set.stats.sampled_mesh_objects.assign(sampled_mesh_objects.begin(), sampled_mesh_objects.end());
  set.stats.selected_vertex_count = int(set.vertices.size());
  set.valid = !set.vertices.empty();
  if (!set.valid) {
    set.stats.status = "not_run";
    set.stats.reason = set.stats.eligible_mesh_object_count == 0 ||
                               set.stats.mesh_objects_with_vertex_groups == 0 ?
                           "no_eligible_mesh_or_vertex_group" :
                           "no_selected_vertices_for_requested_bones";
  }
  else {
    set.stats.status = "sampling_ready";
    set.stats.reason = "selected_vertices_ready";
  }
  return set;
}

/* Run penetration tests for all pre-selected vertices and append only
 * the penetrating ones to `r_frame.mesh_samples`. `r_frame.bodies` must
 * already be populated (capture_diagnostic_frame fills it before this is
 * called) so we can look up the body's bound bone name by runtime index. */
void capture_mesh_samples(const MMDPhysicsWorld &world,
                          const MeshSampleSet &sample_set,
                          MMDDiagnosticFrame &r_frame,
                          MeshSamplingStats &r_stats)
{
  if (!sample_set.valid) {
    return;
  }
  for (const MeshSampleVertex &sv : sample_set.vertices) {
    r_stats.vertex_test_count++;
    int body_index = -1;
    float depth = 0.0f;
    if (!world.find_body_containing_point(sv.blender_position, nullptr, body_index, depth)) {
      r_stats.nonpenetrating_count++;
      continue;
    }
    MMDDiagnosticMeshSample sample;
    sample.bone_name = sv.bone_name;
    sample.mesh_object_name = sv.mesh_object_name;
    sample.vertex_index = sv.vertex_index;
    sample.position_world = sv.blender_position;
    sample.inside_body_index = body_index;
    sample.penetration_depth = depth;
    if (body_index >= 0 && body_index < int(r_frame.bodies.size())) {
      sample.inside_body_bone = r_frame.bodies[body_index].bone_name;
    }
    r_frame.mesh_samples.append(std::move(sample));
    r_stats.penetration_sample_count++;
    r_stats.emitted_record_count++;
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
  const bool write_ok = ferror(file) == 0;
  const bool close_ok = fclose(file) == 0;
  return write_ok && close_ok;
}

MMDDiagnosticCaptureResult capture_mmd_physics_diagnostics(
    const MMDPhysicsDefinition &definition,
    Object *armature,
    Collection *model_collection,
    Main *bmain,
    Depsgraph *depsgraph,
    const MMDDiagnosticCaptureOptions &options)
{
  MMDDiagnosticCaptureResult result;
  if (armature == nullptr || bmain == nullptr) {
    result.error = "MMD Physics diagnostics: invalid Blender context";
    return result;
  }
  if (options.fixed_step_hz < 30 || options.fixed_step_hz > 480 || options.steps < 1 ||
      options.steps > 100000 || options.solver_iterations <= 0 ||
      !float3_is_finite(options.gravity) || !std::isfinite(options.joint_spring_damping) ||
      options.joint_spring_damping < 0.0f || options.joint_spring_damping > 1.0f ||
      options.startup_prewarm_steps < 0 || options.startup_prewarm_steps > 16 ||
      options.startup_sync_steps < 0 || options.startup_sync_steps > 300 ||
      options.mesh_sample_interval < 1 || options.mesh_sample_interval > 240 ||
      options.mesh_max_vertices_per_bone < 1 || options.mesh_max_vertices_per_bone > 256 ||
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
  MeshSampleSet mesh_set = build_mesh_sample_set(model_collection,
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
     * produce no output, so the JSONL size stays bounded. The summary keeps
     * the independent accounting for selected and sampled vertices. */
    if (mesh_set.valid && step % options.mesh_sample_interval == 0) {
      mesh_set.stats.sampled_step_count++;
      capture_mesh_samples(world, mesh_set, frame, mesh_set.stats);
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

  if (mesh_set.valid) {
    mesh_set.stats.status = mesh_set.stats.penetration_sample_count == 0 ?
                                "completed_no_penetration_observed" :
                                "completed_penetration_observed";
    mesh_set.stats.reason = mesh_set.stats.penetration_sample_count == 0 ?
                                "no_penetration_samples_emitted" :
                                "penetration_samples_emitted";
  }

  /* Write both temporary files before touching the last successful capture.
   * Publishing two files cannot be atomic, so move the old pair to backups
   * and restore it on every ordinary failure path. The summary remains the
   * publication gate consumers use to discover a matching JSONL file. */
  const int first_step = options.capture_initial_state ? -1 :
                         (result.captured_steps > 0 ? 0 : -1);
  const int last_step = result.captured_steps > 0 ? result.captured_steps - 1 : -1;
  if (!write_summary(summary_temp_path,
                     options,
                     result.captured_steps,
                     result.body_count,
                     result.joint_count,
                     first_step,
                     last_step,
                     mesh_set.stats,
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
