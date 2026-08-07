/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_vector.hh"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct PMXModel;
struct bArmature;

namespace blender {

struct Collection;
struct ReportList;

}  // namespace blender

namespace blender::mmd_physics {

struct MMDBoneMapping {
  int pmx_index = -1;
  std::string pmx_name_local;
  std::string pmx_name_universal;
  std::string blender_bone_name;
  bool resolved = false;
};

struct MMDRigidBodyDefinition {
  int pmx_index = -1;
  std::string name_local;
  std::string name_universal;
  int pmx_bone_index = -1;
  std::string blender_bone_name;
  bool bone_resolved = false;
  uint8_t collision_group = 0;
  uint16_t no_collision_group = 0;
  uint8_t shape_type = 0;
  std::array<float, 3> shape_size{};
  std::array<float, 3> position{};
  std::array<float, 3> rotation{};
  float mass = 0.0f;
  float linear_damping = 0.0f;
  float angular_damping = 0.0f;
  float restitution = 0.0f;
  float friction = 0.0f;
  uint8_t physics_type = 0;
};

enum class MMDJointAxisLimitMode : uint8_t {
  Limited = 0,
  Free = 1,
};

struct MMDJointDefinition {
  int pmx_index = -1;
  std::string name_local;
  std::string name_universal;
  uint8_t type = 0;
  int rigid_a_index = -1;
  int rigid_b_index = -1;
  std::array<float, 3> position{};
  std::array<float, 3> rotation{};
  std::array<float, 3> translation_min{};
  std::array<float, 3> translation_max{};
  std::array<float, 3> rotation_min{};
  std::array<float, 3> rotation_max{};
  std::array<MMDJointAxisLimitMode, 3> rotation_limit_mode{};
  std::array<float, 3> spring_translation{};
  std::array<float, 3> spring_rotation{};
};

struct MMDPhysicsValidationSummary {
  bool valid = true;
  int unresolved_bones = 0;
  int invalid_rigid_bodies = 0;
  int invalid_joints = 0;
  int total_errors = 0;
};

struct MMDPhysicsDefinition {
  int schema_version = 2;
  std::string source_model_name;
  float source_pmx_version = 2.0f;
  float coordinate_scale = 1.0f;
  std::string coordinate_space = "blender_import_space";
  std::string rotation_order = "YXZ";
  /* Joint positions, limits, and springs use the same Blender Z-up axis order
   * as rigid-body transforms. Linear values include `coordinate_scale`. */
  std::string joint_limits_space = "blender_import_space";
  size_t source_file_size = 0;
  size_t source_parse_end_offset = 0;
  std::vector<MMDBoneMapping> bone_mapping;
  std::vector<MMDRigidBodyDefinition> rigid_bodies;
  std::vector<MMDJointDefinition> joints;
  MMDPhysicsValidationSummary validation;
};

struct MMDPhysicsBuildResult {
  MMDPhysicsDefinition definition;
  std::vector<std::string> errors;

  bool success() const
  {
    return definition.validation.valid && errors.empty();
  }
};

enum class MMDPhysicsMappingIssueSeverity : uint8_t {
  Info = 0,
  Warning = 1,
  Error = 2,
};

struct MMDPhysicsMappingIssue {
  MMDPhysicsMappingIssueSeverity severity = MMDPhysicsMappingIssueSeverity::Error;
  std::string path;
  std::string message;
};

struct MMDPhysicsMappingReport {
  bool definition_valid = false;
  bool mapping_valid = true;
  int resolved_bones = 0;
  int unresolved_bones = 0;
  int resolved_rigid_bones = 0;
  int unresolved_rigid_bones = 0;
  int resolved_joint_endpoints = 0;
  int invalid_joint_endpoints = 0;
  int invalid_rigid_bodies = 0;
  int invalid_joints = 0;
  int total_issues = 0;
  std::vector<MMDPhysicsMappingIssue> issues;
};

struct MMDPhysicsDebugReport {
  bool definition_valid = false;
  bool mapping_valid = false;
  bool static_ready = false;

  int bone_count = 0;
  int resolved_bone_count = 0;
  int rigid_body_count = 0;
  int bound_rigid_body_count = 0;
  int unbound_rigid_body_count = 0;
  int invalid_rigid_body_binding_count = 0;
  int joint_count = 0;
  int resolved_joint_count = 0;

  std::array<int, 16> collision_group_counts{};
  std::array<int, 16> collision_group_mask_counts{};
  std::array<int, 3> rigid_shape_counts{};
  std::array<int, 3> rigid_type_counts{};
  std::array<int, 3> joint_rotation_limited_axes{};
  std::array<int, 3> joint_rotation_free_axes{};

  int joints_with_same_endpoint = 0;
  int negative_spring_count = 0;
  int warning_count = 0;
  int error_count = 0;
  std::vector<std::string> diagnostics;
};

/** Build a pure in-memory physics definition. Does not touch Blender data. */
MMDPhysicsBuildResult build_physics_definition(const PMXModel &model,
                                               const Vector<std::string> &bone_names,
                                               const char *model_name,
                                               float coordinate_scale);

/** Persist/read a validated definition on the PMX model root Collection. */
bool serialize_physics_definition(Collection &model_root,
                                  const MMDPhysicsDefinition &definition,
                                  ReportList *reports);

bool deserialize_physics_definition(const Collection &model_root,
                                    MMDPhysicsDefinition &definition,
                                    ReportList *reports);

/** Write a stable, definition-only JSON artifact for cross-implementation audits.
 * This does not initialize Bullet or evaluate animation. */
bool write_physics_definition_json(const MMDPhysicsDefinition &definition,
                                   const char *filepath,
                                   ReportList *reports);

/** Validate a persisted definition against the current Armature data. */
MMDPhysicsMappingReport validate_physics_mapping(const MMDPhysicsDefinition &definition,
                                                 const bArmature *armature);

/** Build static diagnostics from a definition and its mapping report. */
MMDPhysicsDebugReport build_physics_debug_report(const MMDPhysicsDefinition &definition,
                                                 const MMDPhysicsMappingReport &mapping_report);

}  // namespace blender::mmd_physics
