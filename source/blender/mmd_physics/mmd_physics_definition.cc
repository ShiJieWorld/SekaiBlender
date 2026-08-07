/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "mmd_physics_definition.hh"

#include "BKE_armature.hh"
#include "BKE_idprop.hh"
#include "BKE_report.hh"

#include "BLI_fileops.hh"
#include "BLI_vector.hh"
#include "MEM_guardedalloc.h"
#include "DNA_collection_types.h"

#include "../io/pmx/intern/pmx_types.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <system_error>

#include "BLI_string_ref.hh"

namespace blender::mmd_physics {
namespace {

bool finite_vec3(const float value[3])
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

void add_error(MMDPhysicsBuildResult &result,
               const char *section,
               const int index,
               const char *field,
               const std::string &detail)
{
  std::ostringstream message;
  message << section << "[" << index << "]." << field << ": " << detail;
  result.errors.push_back(message.str());
  result.definition.validation.valid = false;
  result.definition.validation.total_errors++;
}

bool check_finite_vec3(MMDPhysicsBuildResult &result,
                        const char *section,
                        const int index,
                        const char *field,
                        const float value[3])
{
  const bool valid = finite_vec3(value);
  if (!valid) {
    add_error(result, section, index, field, "contains NaN or Inf");
  }
  return valid;
}

void transform_position(std::array<float, 3> &dst, const float src[3], const float scale)
{
  /* PMX -> Blender uses the same axis conversion as the importer.
   * mmd_tools: loc = Vector(pmx.pos).xzy * scale
   * PMX (X, Y, Z) -> Blender (X, Z, Y) * scale */
  dst[0] = src[0] * scale;
  dst[1] = src[2] * scale;
  dst[2] = src[1] * scale;
}

void transform_rotation(std::array<float, 3> &dst, const float src[3])
{
  /* mmd_tools PMX importer: rot = Vector(pmx.rot).xzy * -1
   * PMX (X, Y, Z) -> Blender (-X, -Z, -Y)
   * This is NOT a standard coordinate-system rotation; it is mmd_tools'
   * convention. We replicate it to align Bullet world body transforms with
   * MikuMikuPhysics reference. */
  dst[0] = -src[0];
  dst[1] = -src[2];
  dst[2] = -src[1];
}

void transform_vec3_yz_swap(std::array<float, 3> &dst, const float src[3])
{
  /* mmd_tools: vec.xzy (swap Y/Z, no negate, no scale).
   * Used for joint spring_linear/angular. */
  dst[0] = src[0];
  dst[1] = src[2];
  dst[2] = src[1];
}

void transform_vec3_yz_swap_negate(std::array<float, 3> &dst, const float src[3])
{
  /* mmd_tools: vec.xzy * -1 (swap Y/Z + negate).
   * Used for joint rotation limits. */
  dst[0] = -src[0];
  dst[1] = -src[2];
  dst[2] = -src[1];
}

bool positive_shape_value(const float value)
{
  return std::isfinite(value) && value > 0.0f;
}

}  // namespace

namespace {

constexpr char kDefinitionProperty[] = "mmd_physics_definition";
constexpr int kMaxPersistedItems = 100000;

IDProperty *new_int_array(const char *name, const int *values, const int length)
{
  blender::Vector<int32_t> copied(length);
  for (int i = 0; i < length; i++) {
    copied[i] = int32_t(values[i]);
  }
  return blender::bke::idprop::create(name, copied.as_span()).release();
}

IDProperty *new_float_array(const char *name, const std::array<float, 3> &values)
{
  return blender::bke::idprop::create(name, blender::Span<float>(values.data(), 3)).release();
}

void add_property(IDProperty *group, IDProperty *property)
{
  if (!property || !IDP_AddToGroup(group, property)) {
    if (property) {
      IDP_FreeProperty(property);
    }
  }
}

void add_int(IDProperty *group, const char *name, const int value)
{
  add_property(group, IDP_NewInt(value, name));
}

void add_float(IDProperty *group, const char *name, const float value)
{
  add_property(group, blender::bke::idprop::create(name, value).release());
}

void add_bool(IDProperty *group, const char *name, const bool value)
{
  add_property(group, blender::bke::idprop::create_bool(name, value).release());
}

void add_string(IDProperty *group, const char *name, const std::string &value)
{
  add_property(group, IDP_NewString(value.c_str(), name));
}

void add_vec3(IDProperty *group, const char *name, const std::array<float, 3> &values)
{
  add_property(group, new_float_array(name, values));
}

void report_error(ReportList *reports, const std::string &message)
{
  if (reports) {
    BKE_report(reports, RPT_WARNING, message.c_str());
  }
}

IDProperty *get_group(IDProperty *group, const char *name)
{
  return IDP_GetPropertyTypeFromGroup(group, name, IDP_GROUP);
}

IDProperty *get_array(IDProperty *group, const char *name, const int length, const char subtype)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_ARRAY);
  return property && property->len == length && property->subtype == subtype ? property : nullptr;
}

bool read_int(IDProperty *group, const char *name, int &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_INT);
  if (!property) {
    return false;
  }
  value = IDP_int_get(property);
  return true;
}

bool read_float(IDProperty *group, const char *name, float &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_FLOAT);
  if (!property || !std::isfinite(IDP_float_get(property))) {
    return false;
  }
  value = IDP_float_get(property);
  return true;
}

bool read_bool(IDProperty *group, const char *name, bool &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_BOOLEAN);
  if (!property) {
    return false;
  }
  value = IDP_bool_get(property);
  return true;
}

bool read_string(IDProperty *group, const char *name, std::string &value)
{
  IDProperty *property = IDP_GetPropertyTypeFromGroup(group, name, IDP_STRING);
  if (!property) {
    return false;
  }
  value = IDP_string_get(property);
  return true;
}

bool read_vec3(IDProperty *group, const char *name, std::array<float, 3> &value)
{
  IDProperty *property = get_array(group, name, 3, IDP_FLOAT);
  if (!property) {
    return false;
  }
  const float *values = IDP_array_float_get(property);
  if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
    return false;
  }
  std::copy(values, values + 3, value.begin());
  return true;
}

void append_group(IDProperty *array, IDProperty *item)
{
  IDP_ResizeIDPArray(array, array->len + 1);
  IDP_SetIndexArray(array, array->len - 1, item);
  /* IDP_SetIndexArray makes a shallow copy of the item. */
  MEM_delete(item);
}

void write_json_string(FILE *file, const std::string &value)
{
  static constexpr char hex[] = "0123456789abcdef";
  fputc('"', file);
  for (const unsigned char byte : value) {
    switch (byte) {
      case '"': fputs("\\\"", file); break;
      case '\\': fputs("\\\\", file); break;
      case '\b': fputs("\\b", file); break;
      case '\f': fputs("\\f", file); break;
      case '\n': fputs("\\n", file); break;
      case '\r': fputs("\\r", file); break;
      case '\t': fputs("\\t", file); break;
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
  for (int axis = 0; axis < 3; axis++) {
    if (axis != 0) {
      fputc(',', file);
    }
    write_json_float(file, value[axis]);
  }
  fputc(']', file);
}

void write_limit_modes(FILE *file, const std::array<MMDJointAxisLimitMode, 3> &modes)
{
  fputc('[', file);
  for (int axis = 0; axis < 3; axis++) {
    if (axis != 0) {
      fputc(',', file);
    }
    write_json_string(file, modes[axis] == MMDJointAxisLimitMode::Free ? "free" : "limited");
  }
  fputc(']', file);
}

}  // namespace

MMDPhysicsBuildResult build_physics_definition(const PMXModel &model,
                                               const Vector<std::string> &bone_names,
                                               const char *model_name,
                                               const float coordinate_scale)
{
  MMDPhysicsBuildResult result;
  MMDPhysicsDefinition &definition = result.definition;
  definition.source_model_name = model_name ? std::string(model_name) : model.name_local;
  definition.source_pmx_version = model.header.version;
  definition.coordinate_scale = coordinate_scale;
  definition.source_file_size = model.file_size;
  definition.source_parse_end_offset = model.parse_end_offset;

  definition.bone_mapping.reserve(model.bones.size());
  for (int i = 0; i < int(model.bones.size()); i++) {
    const PMXBone &pmx_bone = model.bones[i];
    MMDBoneMapping mapping;
    mapping.pmx_index = i;
    mapping.pmx_name_local = pmx_bone.name_local;
    mapping.pmx_name_universal = pmx_bone.name_universal;
    if (i < bone_names.size()) {
      mapping.blender_bone_name = bone_names[i];
      mapping.resolved = !mapping.blender_bone_name.empty();
    }
    if (!mapping.resolved) {
      definition.validation.unresolved_bones++;
      add_error(result, "bones", i, "blender_bone_name", "no Blender bone mapping");
    }
    definition.bone_mapping.push_back(std::move(mapping));
  }

  definition.rigid_bodies.reserve(model.rigid_bodies.size());
  for (int i = 0; i < int(model.rigid_bodies.size()); i++) {
    const PMXRigidBody &pmx_rigid = model.rigid_bodies[i];
    MMDRigidBodyDefinition rigid;
    rigid.pmx_index = i;
    rigid.name_local = pmx_rigid.name_local;
    rigid.name_universal = pmx_rigid.name_universal;
    rigid.pmx_bone_index = pmx_rigid.bone_index;
    rigid.collision_group = pmx_rigid.collision_group;
    rigid.no_collision_group = pmx_rigid.no_collision_group;
    rigid.shape_type = pmx_rigid.shape_type;
    rigid.physics_type = pmx_rigid.physics_type;
    rigid.mass = pmx_rigid.mass;
    rigid.linear_damping = pmx_rigid.linear_damping;
    rigid.angular_damping = pmx_rigid.angular_damping;
    rigid.restitution = pmx_rigid.restitution;
    rigid.friction = pmx_rigid.friction;
    transform_position(rigid.position, pmx_rigid.pos, coordinate_scale);
    transform_rotation(rigid.rotation, pmx_rigid.rot);
    rigid.shape_size[0] = pmx_rigid.shape_size[0] * coordinate_scale;
    rigid.shape_size[1] = pmx_rigid.shape_size[rigid.shape_type == 1 ? 2 : 1] * coordinate_scale;
    rigid.shape_size[2] = pmx_rigid.shape_size[rigid.shape_type == 1 ? 1 : 2] * coordinate_scale;

    bool rigid_valid = true;
    if (pmx_rigid.bone_index == -1) {
      rigid.bone_resolved = true;
    }
    else if (pmx_rigid.bone_index >= 0 &&
             pmx_rigid.bone_index < int(definition.bone_mapping.size()) &&
             definition.bone_mapping[pmx_rigid.bone_index].resolved)
    {
      rigid.blender_bone_name = definition.bone_mapping[pmx_rigid.bone_index].blender_bone_name;
      rigid.bone_resolved = true;
    }
    else {
      add_error(result, "rigid_bodies", i, "bone_index", "does not resolve to a Blender bone");
      rigid_valid = false;
    }

    if (rigid.collision_group > 15) {
      add_error(result, "rigid_bodies", i, "collision_group", "must be in range 0..15");
      rigid_valid = false;
    }
    if (rigid.shape_type > 2) {
      add_error(result, "rigid_bodies", i, "shape_type", "unsupported shape type");
      rigid_valid = false;
    }
    else {
      const int required_axes = rigid.shape_type == 0 ? 1 : 2;
      const int axis_count = rigid.shape_type == 1 ? 3 : required_axes;
      for (int axis = 0; axis < axis_count; axis++) {
        if (!positive_shape_value(rigid.shape_size[axis])) {
          add_error(result,
                    "rigid_bodies",
                    i,
                    "shape_size",
                    rigid.shape_type == 1 ? "box dimensions must be positive and finite" :
                                            "required dimensions must be positive and finite");
          rigid_valid = false;
          break;
        }
      }
    }
    if (!check_finite_vec3(result, "rigid_bodies", i, "pos", pmx_rigid.pos)) {
      rigid_valid = false;
    }
    if (!check_finite_vec3(result, "rigid_bodies", i, "rot", pmx_rigid.rot)) {
      rigid_valid = false;
    }
    if (!std::isfinite(rigid.mass) || !std::isfinite(rigid.linear_damping) ||
        !std::isfinite(rigid.angular_damping) || !std::isfinite(rigid.restitution) ||
        !std::isfinite(rigid.friction))
    {
      add_error(result, "rigid_bodies", i, "physical_parameters", "contains NaN or Inf");
      rigid_valid = false;
    }
    if (pmx_rigid.physics_type > 2) {
      add_error(result, "rigid_bodies", i, "physics_type", "unsupported physics type");
      rigid_valid = false;
    }
    if (!rigid_valid) {
      definition.validation.invalid_rigid_bodies++;
    }
    definition.rigid_bodies.push_back(std::move(rigid));
  }

  definition.joints.reserve(model.joints.size());
  for (int i = 0; i < int(model.joints.size()); i++) {
    const PMXJoint &pmx_joint = model.joints[i];
    MMDJointDefinition joint;
    joint.pmx_index = i;
    joint.name_local = pmx_joint.name_local;
    joint.name_universal = pmx_joint.name_universal;
    joint.type = pmx_joint.type;
    joint.rigid_a_index = pmx_joint.rigid_a_index;
    joint.rigid_b_index = pmx_joint.rigid_b_index;
    transform_position(joint.position, pmx_joint.pos, coordinate_scale);
    transform_rotation(joint.rotation, pmx_joint.rot);
    /* mmd_tools: maximum_location = pmx.max.xzy * scale,
     *            minimum_location = pmx.min.xzy * scale.
     * Both min and max are Y/Z swapped and scaled (no min/max exchange). */
    transform_position(joint.translation_min, pmx_joint.translation_limit_min, coordinate_scale);
    transform_position(joint.translation_max, pmx_joint.translation_limit_max, coordinate_scale);
    /* mmd_tools: maximum_rotation = pmx.min_rotation.xzy * -1,
     *            minimum_rotation = pmx.max_rotation.xzy * -1.
     * Note the min/max exchange! After exchange, our rotation_min holds
     * pmx.max_rotation transformed, and rotation_max holds pmx.min_rotation
     * transformed. This mirrors mmd_tools' createJoint argument order. */
    transform_vec3_yz_swap_negate(joint.rotation_min, pmx_joint.rotation_limit_max);
    transform_vec3_yz_swap_negate(joint.rotation_max, pmx_joint.rotation_limit_min);
    /* mmd_tools: spring_linear = pmx.spring_constant.xzy,
     *            spring_angular = pmx.spring_rotation_constant.xzy.
     * Y/Z swap only, no negate, no scale. */
    transform_vec3_yz_swap(joint.spring_translation, pmx_joint.spring_translation);
    transform_vec3_yz_swap(joint.spring_rotation, pmx_joint.spring_rotation);

    bool joint_valid = true;
    if (joint.type != 0) {
      add_error(result, "joints", i, "type", "unsupported joint type");
      joint_valid = false;
    }
    if (joint.rigid_a_index < 0 || joint.rigid_a_index >= int(model.rigid_bodies.size())) {
      add_error(result, "joints", i, "rigid_a_index", "out of range");
      joint_valid = false;
    }
    if (joint.rigid_b_index < 0 || joint.rigid_b_index >= int(model.rigid_bodies.size())) {
      add_error(result, "joints", i, "rigid_b_index", "out of range");
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "pos", pmx_joint.pos)) {
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "rot", pmx_joint.rot)) {
      joint_valid = false;
    }
    if (!check_finite_vec3(
            result, "joints", i, "translation_limit_min", pmx_joint.translation_limit_min))
    {
      joint_valid = false;
    }
    if (!check_finite_vec3(
            result, "joints", i, "translation_limit_max", pmx_joint.translation_limit_max))
    {
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "rotation_limit_min", pmx_joint.rotation_limit_min))
    {
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "rotation_limit_max", pmx_joint.rotation_limit_max))
    {
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "spring_translation", pmx_joint.spring_translation))
    {
      joint_valid = false;
    }
    if (!check_finite_vec3(result, "joints", i, "spring_rotation", pmx_joint.spring_rotation))
    {
      joint_valid = false;
    }
    for (int axis = 0; axis < 3; axis++) {
      if (joint.translation_min[axis] > joint.translation_max[axis]) {
        add_error(result, "joints", i, "translation_limits", "minimum exceeds maximum");
        joint_valid = false;
      }
      /* PMX/MMD uses an inverted angular interval to represent a free axis. */
      joint.rotation_limit_mode[axis] = joint.rotation_min[axis] > joint.rotation_max[axis] ?
                                            MMDJointAxisLimitMode::Free :
                                            MMDJointAxisLimitMode::Limited;
    }
    if (!joint_valid) {
      definition.validation.invalid_joints++;
    }
    definition.joints.push_back(std::move(joint));
  }

  return result;
}

bool serialize_physics_definition(Collection &model_root,
                                  const MMDPhysicsDefinition &definition,
                                  ReportList *reports)
{
  if (!definition.validation.valid) {
    report_error(reports, "MMD physics definition: refusing to persist invalid definition");
    return false;
  }
  if (definition.bone_mapping.size() > kMaxPersistedItems ||
      definition.rigid_bodies.size() > kMaxPersistedItems ||
      definition.joints.size() > kMaxPersistedItems)
  {
    report_error(reports, "MMD physics definition: item count exceeds persistence limit");
    return false;
  }

  IDProperty *root = blender::bke::idprop::create_group(kDefinitionProperty).release();
  if (!root) {
    report_error(reports, "MMD physics definition: failed to allocate root property");
    return false;
  }
  add_int(root, "schema_version", definition.schema_version);
  add_string(root, "source_model_name", definition.source_model_name);
  add_float(root, "source_pmx_version", definition.source_pmx_version);
  add_float(root, "coordinate_scale", definition.coordinate_scale);
  add_string(root, "coordinate_space", definition.coordinate_space);
  add_string(root, "rotation_order", definition.rotation_order);
  add_string(root, "joint_limits_space", definition.joint_limits_space);
  if (definition.source_file_size > size_t(INT32_MAX) ||
      definition.source_parse_end_offset > size_t(INT32_MAX))
  {
    IDP_FreeProperty(root);
    report_error(reports, "MMD physics definition: source diagnostics exceed IDProperty range");
    return false;
  }
  add_int(root, "source_file_size", int(definition.source_file_size));
  add_int(root, "source_parse_end_offset", int(definition.source_parse_end_offset));
  add_int(root, "bone_count", int(definition.bone_mapping.size()));
  add_int(root, "rigid_body_count", int(definition.rigid_bodies.size()));
  add_int(root, "joint_count", int(definition.joints.size()));
  add_bool(root, "validation_valid", definition.validation.valid);
  add_int(root, "validation_total_errors", definition.validation.total_errors);

  IDProperty *bones = IDP_NewIDPArray("bones");
  for (const MMDBoneMapping &bone : definition.bone_mapping) {
    IDProperty *item = blender::bke::idprop::create_group("bone").release();
    add_int(item, "pmx_index", bone.pmx_index);
    add_string(item, "pmx_name_local", bone.pmx_name_local);
    add_string(item, "pmx_name_universal", bone.pmx_name_universal);
    add_string(item, "blender_bone_name", bone.blender_bone_name);
    add_bool(item, "resolved", bone.resolved);
    append_group(bones, item);
  }
  add_property(root, bones);

  IDProperty *rigids = IDP_NewIDPArray("rigid_bodies");
  for (const MMDRigidBodyDefinition &rigid : definition.rigid_bodies) {
    IDProperty *item = blender::bke::idprop::create_group("rigid_body").release();
    add_int(item, "pmx_index", rigid.pmx_index);
    add_string(item, "name_local", rigid.name_local);
    add_string(item, "name_universal", rigid.name_universal);
    add_int(item, "pmx_bone_index", rigid.pmx_bone_index);
    add_string(item, "blender_bone_name", rigid.blender_bone_name);
    add_bool(item, "bone_resolved", rigid.bone_resolved);
    add_int(item, "collision_group", rigid.collision_group);
    add_int(item, "no_collision_group", rigid.no_collision_group);
    add_int(item, "shape_type", rigid.shape_type);
    add_vec3(item, "shape_size", rigid.shape_size);
    add_vec3(item, "position", rigid.position);
    add_vec3(item, "rotation", rigid.rotation);
    add_float(item, "mass", rigid.mass);
    add_float(item, "linear_damping", rigid.linear_damping);
    add_float(item, "angular_damping", rigid.angular_damping);
    add_float(item, "restitution", rigid.restitution);
    add_float(item, "friction", rigid.friction);
    add_int(item, "physics_type", rigid.physics_type);
    append_group(rigids, item);
  }
  add_property(root, rigids);

  IDProperty *joints = IDP_NewIDPArray("joints");
  for (const MMDJointDefinition &joint : definition.joints) {
    IDProperty *item = blender::bke::idprop::create_group("joint").release();
    add_int(item, "pmx_index", joint.pmx_index);
    add_string(item, "name_local", joint.name_local);
    add_string(item, "name_universal", joint.name_universal);
    add_int(item, "type", joint.type);
    add_int(item, "rigid_a_index", joint.rigid_a_index);
    add_int(item, "rigid_b_index", joint.rigid_b_index);
    add_vec3(item, "position", joint.position);
    add_vec3(item, "rotation", joint.rotation);
    add_vec3(item, "translation_min", joint.translation_min);
    add_vec3(item, "translation_max", joint.translation_max);
    add_vec3(item, "rotation_min", joint.rotation_min);
    add_vec3(item, "rotation_max", joint.rotation_max);
    int modes[3] = {int(joint.rotation_limit_mode[0]),
                    int(joint.rotation_limit_mode[1]),
                    int(joint.rotation_limit_mode[2])};
    add_property(item, new_int_array("rotation_limit_mode", modes, 3));
    add_vec3(item, "spring_translation", joint.spring_translation);
    add_vec3(item, "spring_rotation", joint.spring_rotation);
    append_group(joints, item);
  }
  add_property(root, joints);

  IDProperty *system = IDP_ID_system_properties_ensure(&model_root.id);
  IDP_ReplaceInGroup(system, root);
  return true;
}

bool deserialize_physics_definition(const Collection &model_root,
                                    MMDPhysicsDefinition &definition,
                                    ReportList *reports)
{
  IDProperty *system = model_root.id.system_properties;
  IDProperty *root = system ? IDP_GetPropertyTypeFromGroup(system, kDefinitionProperty, IDP_GROUP) : nullptr;
  if (!root) {
    report_error(reports, "MMD physics definition: property is missing or not a group");
    return false;
  }
  int schema = 0;
  int bone_count = 0;
  int rigid_count = 0;
  int joint_count = 0;
  if (!read_int(root, "schema_version", schema) || schema != 2 ||
      !read_int(root, "bone_count", bone_count) || !read_int(root, "rigid_body_count", rigid_count) ||
      !read_int(root, "joint_count", joint_count) || bone_count < 0 || rigid_count < 0 ||
      joint_count < 0 || bone_count > kMaxPersistedItems || rigid_count > kMaxPersistedItems ||
      joint_count > kMaxPersistedItems)
  {
    report_error(reports, "MMD physics definition: invalid schema or item counts");
    return false;
  }
  IDProperty *bones = IDP_GetPropertyTypeFromGroup(root, "bones", IDP_IDPARRAY);
  IDProperty *rigids = IDP_GetPropertyTypeFromGroup(root, "rigid_bodies", IDP_IDPARRAY);
  IDProperty *joints = IDP_GetPropertyTypeFromGroup(root, "joints", IDP_IDPARRAY);
  if (!bones || !rigids || !joints || bones->len != bone_count || rigids->len != rigid_count ||
      joints->len != joint_count)
  {
    report_error(reports, "MMD physics definition: array lengths do not match metadata");
    return false;
  }
  definition = MMDPhysicsDefinition{};
  definition.schema_version = schema;
  if (!read_string(root, "source_model_name", definition.source_model_name) ||
      !read_float(root, "source_pmx_version", definition.source_pmx_version) ||
      !read_float(root, "coordinate_scale", definition.coordinate_scale) ||
      !read_string(root, "coordinate_space", definition.coordinate_space) ||
      !read_string(root, "rotation_order", definition.rotation_order) ||
      definition.rotation_order != "YXZ")
  {
    report_error(reports, "MMD physics definition: invalid metadata types or values");
    return false;
  }
  if (!read_string(root, "joint_limits_space", definition.joint_limits_space) ||
      definition.joint_limits_space != "blender_import_space")
  {
    report_error(reports, "MMD physics definition: unsupported joint coordinate space");
    return false;
  }
  int source_size = 0;
  int parse_end = 0;
  if (!read_int(root, "source_file_size", source_size) || !read_int(root, "source_parse_end_offset", parse_end) ||
      source_size < 0 || parse_end < 0)
  {
    report_error(reports, "MMD physics definition: invalid source file diagnostics");
    return false;
  }
  definition.source_file_size = size_t(source_size);
  definition.source_parse_end_offset = size_t(parse_end);
  definition.bone_mapping.reserve(bone_count);
  for (int i = 0; i < bone_count; i++) {
    IDProperty *item = IDP_GetIndexArray(bones, i);
    int index = -1;
    MMDBoneMapping bone;
    if (!item || item->type != IDP_GROUP || !read_int(item, "pmx_index", index) || index != i ||
        !read_string(item, "pmx_name_local", bone.pmx_name_local) ||
        !read_string(item, "pmx_name_universal", bone.pmx_name_universal) ||
        !read_string(item, "blender_bone_name", bone.blender_bone_name) ||
        !read_bool(item, "resolved", bone.resolved))
    {
      report_error(reports, "MMD physics definition: invalid bone entry");
      return false;
    }
    bone.pmx_index = index;
    definition.bone_mapping.push_back(std::move(bone));
  }
  definition.rigid_bodies.reserve(rigid_count);
  for (int i = 0; i < rigid_count; i++) {
    IDProperty *item = IDP_GetIndexArray(rigids, i);
    MMDRigidBodyDefinition rigid;
    int value = 0;
    if (!item || item->type != IDP_GROUP || !read_int(item, "pmx_index", value) || value != i ||
        !read_string(item, "name_local", rigid.name_local) ||
        !read_string(item, "name_universal", rigid.name_universal) ||
        !read_int(item, "pmx_bone_index", rigid.pmx_bone_index) ||
        !read_string(item, "blender_bone_name", rigid.blender_bone_name) ||
        !read_bool(item, "bone_resolved", rigid.bone_resolved) ||
        !read_int(item, "collision_group", value))
    {
      report_error(reports, "MMD physics definition: invalid rigid body entry");
      return false;
    }
    rigid.pmx_index = i;
    rigid.collision_group = uint8_t(value);
    if (!read_int(item, "no_collision_group", value)) return false;
    rigid.no_collision_group = uint16_t(value);
    if (!read_int(item, "shape_type", value)) return false;
    rigid.shape_type = uint8_t(value);
    if (!read_vec3(item, "shape_size", rigid.shape_size) || !read_vec3(item, "position", rigid.position) ||
        !read_vec3(item, "rotation", rigid.rotation) || !read_float(item, "mass", rigid.mass) ||
        !read_float(item, "linear_damping", rigid.linear_damping) ||
        !read_float(item, "angular_damping", rigid.angular_damping) ||
        !read_float(item, "restitution", rigid.restitution) || !read_float(item, "friction", rigid.friction) ||
        !read_int(item, "physics_type", value))
    {
      report_error(reports, "MMD physics definition: invalid rigid body fields");
      return false;
    }
    rigid.physics_type = uint8_t(value);
    definition.rigid_bodies.push_back(std::move(rigid));
  }
  definition.joints.reserve(joint_count);
  for (int i = 0; i < joint_count; i++) {
    IDProperty *item = IDP_GetIndexArray(joints, i);
    MMDJointDefinition joint;
    int value = 0;
    if (!item || item->type != IDP_GROUP || !read_int(item, "pmx_index", value) || value != i ||
        !read_string(item, "name_local", joint.name_local) ||
        !read_string(item, "name_universal", joint.name_universal) || !read_int(item, "type", value))
    {
      report_error(reports, "MMD physics definition: invalid joint entry");
      return false;
    }
    joint.pmx_index = i;
    joint.type = uint8_t(value);
    if (!read_int(item, "rigid_a_index", joint.rigid_a_index) ||
        !read_int(item, "rigid_b_index", joint.rigid_b_index) ||
        !read_vec3(item, "position", joint.position) || !read_vec3(item, "rotation", joint.rotation) ||
        !read_vec3(item, "translation_min", joint.translation_min) ||
        !read_vec3(item, "translation_max", joint.translation_max) ||
        !read_vec3(item, "rotation_min", joint.rotation_min) ||
        !read_vec3(item, "rotation_max", joint.rotation_max) ||
        !read_vec3(item, "spring_translation", joint.spring_translation) ||
        !read_vec3(item, "spring_rotation", joint.spring_rotation))
    {
      report_error(reports, "MMD physics definition: invalid joint fields");
      return false;
    }
    IDProperty *modes = get_array(item, "rotation_limit_mode", 3, IDP_INT);
    if (!modes) {
      report_error(reports, "MMD physics definition: invalid joint rotation limit modes");
      return false;
    }
    const int *mode_values = IDP_array_int_get(modes);
    for (int axis = 0; axis < 3; axis++) {
      if (mode_values[axis] < 0 || mode_values[axis] > 1) {
        report_error(reports, "MMD physics definition: unsupported joint rotation limit mode");
        return false;
      }
      joint.rotation_limit_mode[axis] = MMDJointAxisLimitMode(mode_values[axis]);
      if (joint.translation_min[axis] > joint.translation_max[axis] ||
          (joint.rotation_limit_mode[axis] == MMDJointAxisLimitMode::Limited &&
           joint.rotation_min[axis] > joint.rotation_max[axis]))
      {
        report_error(reports, "MMD physics definition: invalid persisted joint limits");
        return false;
      }
    }
    definition.joints.push_back(std::move(joint));
  }
  bool valid = true;
  if (!read_bool(root, "validation_valid", valid) || !valid) {
    report_error(reports, "MMD physics definition: persisted definition is not valid");
    return false;
  }
  definition.validation.valid = true;
  return true;
}

bool write_physics_definition_json(const MMDPhysicsDefinition &definition,
                                   const char *filepath,
                                   ReportList *reports)
{
  if (filepath == nullptr || filepath[0] == '\0') {
    report_error(reports, "MMD physics definition export: filepath is empty");
    return false;
  }
  if (!definition.validation.valid) {
    report_error(reports, "MMD physics definition export: refusing to export invalid definition");
    return false;
  }

  const std::string output_path(filepath);
  const std::string temporary_path = output_path + ".tmp";
  if (!BLI_file_ensure_parent_dir_exists(output_path.c_str())) {
    report_error(reports, "MMD physics definition export: failed to create output directory");
    return false;
  }
  FILE *file = BLI_fopen(temporary_path.c_str(), "wb");
  if (file == nullptr) {
    report_error(reports, "MMD physics definition export: failed to open temporary JSON output");
    return false;
  }

  fprintf(file,
          "{\"schema_version\":%d,\"export_kind\":\"mmd_physics_definition\","
          "\"producer\":\"blender_mmdworld\",\"source\":{\"model_name\":",
          definition.schema_version);
  write_json_string(file, definition.source_model_name);
  fputs(",\"pmx_version\":", file);
  write_json_float(file, definition.source_pmx_version);
  fputs(",\"coordinate_scale\":", file);
  write_json_float(file, definition.coordinate_scale);
  fputs(",\"coordinate_space\":", file);
  write_json_string(file, definition.coordinate_space);
  fputs(",\"rotation_order\":", file);
  write_json_string(file, definition.rotation_order);
  fputs(",\"joint_limits_space\":", file);
  write_json_string(file, definition.joint_limits_space);
  fprintf(file,
          ",\"source_file_size\":%zu,\"source_parse_end_offset\":%zu},\"validation\":{\"valid\":true,\"total_errors\":%d},\"bones\":[",
          definition.source_file_size,
          definition.source_parse_end_offset,
          definition.validation.total_errors);
  for (size_t index = 0; index < definition.bone_mapping.size(); index++) {
    const MMDBoneMapping &bone = definition.bone_mapping[index];
    if (index != 0) {
      fputc(',', file);
    }
    fprintf(file, "{\"pmx_index\":%d,\"pmx_name_local\":", bone.pmx_index);
    write_json_string(file, bone.pmx_name_local);
    fputs(",\"pmx_name_universal\":", file);
    write_json_string(file, bone.pmx_name_universal);
    fputs(",\"blender_bone_name\":", file);
    write_json_string(file, bone.blender_bone_name);
    fputs(bone.resolved ? ",\"resolved\":true}" : ",\"resolved\":false}", file);
  }
  fputs("],\"rigid_bodies\":[", file);
  for (size_t index = 0; index < definition.rigid_bodies.size(); index++) {
    const MMDRigidBodyDefinition &rigid = definition.rigid_bodies[index];
    if (index != 0) {
      fputc(',', file);
    }
    fprintf(file, "{\"pmx_index\":%d,\"name_local\":", rigid.pmx_index);
    write_json_string(file, rigid.name_local);
    fputs(",\"name_universal\":", file);
    write_json_string(file, rigid.name_universal);
    fprintf(file, ",\"pmx_bone_index\":%d,\"blender_bone_name\":", rigid.pmx_bone_index);
    write_json_string(file, rigid.blender_bone_name);
    fputs(rigid.bone_resolved ? ",\"bone_resolved\":true" : ",\"bone_resolved\":false", file);
    fprintf(file,
            ",\"collision_group\":%u,\"no_collision_group\":%u,\"shape_type\":%u,\"shape_size\":",
            unsigned(rigid.collision_group),
            unsigned(rigid.no_collision_group),
            unsigned(rigid.shape_type));
    write_float3(file, rigid.shape_size);
    fputs(",\"position\":", file);
    write_float3(file, rigid.position);
    fputs(",\"rotation\":", file);
    write_float3(file, rigid.rotation);
    fputs(",\"mass\":", file);
    write_json_float(file, rigid.mass);
    fputs(",\"linear_damping\":", file);
    write_json_float(file, rigid.linear_damping);
    fputs(",\"angular_damping\":", file);
    write_json_float(file, rigid.angular_damping);
    fputs(",\"restitution\":", file);
    write_json_float(file, rigid.restitution);
    fputs(",\"friction\":", file);
    write_json_float(file, rigid.friction);
    fprintf(file, ",\"physics_type\":%u}", unsigned(rigid.physics_type));
  }
  fputs("],\"joints\":[", file);
  for (size_t index = 0; index < definition.joints.size(); index++) {
    const MMDJointDefinition &joint = definition.joints[index];
    if (index != 0) {
      fputc(',', file);
    }
    fprintf(file, "{\"pmx_index\":%d,\"name_local\":", joint.pmx_index);
    write_json_string(file, joint.name_local);
    fputs(",\"name_universal\":", file);
    write_json_string(file, joint.name_universal);
    fprintf(file,
            ",\"type\":%u,\"rigid_a_index\":%d,\"rigid_b_index\":%d,\"position\":",
            unsigned(joint.type),
            joint.rigid_a_index,
            joint.rigid_b_index);
    write_float3(file, joint.position);
    fputs(",\"rotation\":", file);
    write_float3(file, joint.rotation);
    fputs(",\"translation_min\":", file);
    write_float3(file, joint.translation_min);
    fputs(",\"translation_max\":", file);
    write_float3(file, joint.translation_max);
    fputs(",\"rotation_min\":", file);
    write_float3(file, joint.rotation_min);
    fputs(",\"rotation_max\":", file);
    write_float3(file, joint.rotation_max);
    fputs(",\"rotation_limit_mode\":", file);
    write_limit_modes(file, joint.rotation_limit_mode);
    fputs(",\"spring_translation\":", file);
    write_float3(file, joint.spring_translation);
    fputs(",\"spring_rotation\":", file);
    write_float3(file, joint.spring_rotation);
    fputc('}', file);
  }
  fputs("]}\n", file);
  const bool write_ok = ferror(file) == 0;
  const bool close_ok = fclose(file) == 0;
  if (!write_ok || !close_ok) {
    BLI_delete(temporary_path.c_str(), false, false);
    report_error(reports, "MMD physics definition export: failed while writing JSON");
    return false;
  }
  if (BLI_rename_overwrite(temporary_path.c_str(), output_path.c_str()) != 0) {
    BLI_delete(temporary_path.c_str(), false, false);
    report_error(reports, "MMD physics definition export: failed to publish JSON");
    return false;
  }
  return true;
}

namespace {

void add_mapping_issue(MMDPhysicsMappingReport &report,
                       const MMDPhysicsMappingIssueSeverity severity,
                       const std::string &path,
                       const std::string &message)
{
  report.issues.push_back({severity, path, message});
  report.total_issues++;
  if (severity == MMDPhysicsMappingIssueSeverity::Error) {
    report.mapping_valid = false;
  }
}

std::string indexed_path(const char *section, const int index, const char *field)
{
  std::ostringstream path;
  path << section << "[" << index << "]";
  if (field[0] != '\0') {
    path << "." << field;
  }
  return path.str();
}

bool finite_array(const std::array<float, 3> &values)
{
  return std::all_of(values.begin(), values.end(), [](const float value) {
    return std::isfinite(value);
  });
}

bool validate_rigid_body(const MMDRigidBodyDefinition &rigid,
                         const int index,
                         const MMDPhysicsDefinition &definition,
                         MMDPhysicsMappingReport &report)
{
  bool valid = true;
  const std::string prefix = indexed_path("rigid_bodies", index, "") + ".";
  if (rigid.pmx_index != index) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "pmx_index",
                      "does not match array index");
    valid = false;
  }
  if (rigid.collision_group > 15) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "collision_group",
                      "must be in range 0..15");
    valid = false;
  }
  if (rigid.shape_type > 2) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "shape_type",
                      "unsupported shape type");
    valid = false;
  }
  else {
    const int required_axes = rigid.shape_type == 0 ? 1 : 2;
    const int axis_count = rigid.shape_type == 1 ? 3 : required_axes;
    for (int axis = 0; axis < axis_count; axis++) {
      if (!std::isfinite(rigid.shape_size[axis]) || rigid.shape_size[axis] <= 0.0f) {
        add_mapping_issue(report,
                          MMDPhysicsMappingIssueSeverity::Error,
                          prefix + "shape_size",
                          "required dimensions must be positive and finite");
        valid = false;
        break;
      }
    }
  }
  for (const char *field : {"shape_size", "position", "rotation"}) {
    const std::array<float, 3> *values = std::string(field) == "shape_size" ?
                                              &rigid.shape_size :
                                              std::string(field) == "position" ? &rigid.position :
                                                                                   &rigid.rotation;
    if (!finite_array(*values)) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        prefix + field,
                        "contains NaN or Inf");
      valid = false;
    }
  }
  if (!std::isfinite(rigid.mass) || rigid.mass < 0.0f) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "mass",
                      "must be finite and non-negative");
    valid = false;
  }
  if (!std::isfinite(rigid.linear_damping) || !std::isfinite(rigid.angular_damping)) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "damping",
                      "must be finite");
    valid = false;
  }
  else if (rigid.linear_damping < 0.0f || rigid.angular_damping < 0.0f) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Warning,
                      prefix + "damping",
                      "contains a negative value");
  }
  if (!std::isfinite(rigid.restitution) || !std::isfinite(rigid.friction) ||
      rigid.restitution < 0.0f || rigid.friction < 0.0f)
  {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "surface_parameters",
                      "must be finite and non-negative");
    valid = false;
  }
  else if (rigid.restitution > 1.0f || rigid.friction > 1.0f) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Warning,
                      prefix + "surface_parameters",
                      "contains a value above 1");
  }
  if (rigid.physics_type > 2) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "physics_type",
                      "unsupported physics type");
    valid = false;
  }
  if (rigid.pmx_bone_index < -1 || rigid.pmx_bone_index >= int(definition.bone_mapping.size())) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "pmx_bone_index",
                      "out of range");
    valid = false;
  }
  return valid;
}

bool validate_joint(const MMDJointDefinition &joint,
                    const int index,
                    const MMDPhysicsDefinition &definition,
                    MMDPhysicsMappingReport &report)
{
  bool valid = true;
  const std::string prefix = indexed_path("joints", index, "") + ".";
  if (joint.pmx_index != index) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "pmx_index",
                      "does not match array index");
    valid = false;
  }
  if (joint.type != 0) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "type",
                      "unsupported joint type");
    valid = false;
  }
  const bool rigid_a_valid = joint.rigid_a_index >= 0 &&
                             joint.rigid_a_index < int(definition.rigid_bodies.size());
  const bool rigid_b_valid = joint.rigid_b_index >= 0 &&
                             joint.rigid_b_index < int(definition.rigid_bodies.size());
  if (!rigid_a_valid) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "rigid_a_index",
                      "out of range");
    valid = false;
  }
  if (!rigid_b_valid) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      prefix + "rigid_b_index",
                      "out of range");
    valid = false;
  }
  if (rigid_a_valid && rigid_b_valid && joint.rigid_a_index == joint.rigid_b_index) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Warning,
                      prefix + "rigid_endpoints",
                      "both endpoints reference the same rigid body");
  }
  for (const char *field : {"position", "rotation", "translation_min", "translation_max", "rotation_min", "rotation_max", "spring_translation", "spring_rotation"}) {
    const std::array<float, 3> *values = nullptr;
    if (std::string(field) == "position") values = &joint.position;
    else if (std::string(field) == "rotation") values = &joint.rotation;
    else if (std::string(field) == "translation_min") values = &joint.translation_min;
    else if (std::string(field) == "translation_max") values = &joint.translation_max;
    else if (std::string(field) == "rotation_min") values = &joint.rotation_min;
    else if (std::string(field) == "rotation_max") values = &joint.rotation_max;
    else if (std::string(field) == "spring_translation") values = &joint.spring_translation;
    else values = &joint.spring_rotation;
    if (!finite_array(*values)) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        prefix + field,
                        "contains NaN or Inf");
      valid = false;
    }
  }
  for (int axis = 0; axis < 3; axis++) {
    if (joint.translation_min[axis] > joint.translation_max[axis]) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        prefix + "translation_limits",
                        "minimum exceeds maximum");
      valid = false;
    }
    if (joint.rotation_limit_mode[axis] != MMDJointAxisLimitMode::Limited &&
        joint.rotation_limit_mode[axis] != MMDJointAxisLimitMode::Free)
    {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        prefix + "rotation_limit_mode",
                        "unsupported axis mode");
      valid = false;
    }
    else if (joint.rotation_limit_mode[axis] == MMDJointAxisLimitMode::Limited &&
             joint.rotation_min[axis] > joint.rotation_max[axis])
    {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        prefix + "rotation_limits",
                        "limited axis minimum exceeds maximum");
      valid = false;
    }
    if (joint.spring_translation[axis] < 0.0f || joint.spring_rotation[axis] < 0.0f) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Warning,
                        prefix + "spring",
                        "contains a negative value");
    }
  }
  return valid;
}

}  // namespace

MMDPhysicsMappingReport validate_physics_mapping(const MMDPhysicsDefinition &definition,
                                                 const bArmature *armature)
{
  MMDPhysicsMappingReport report;
  report.definition_valid = definition.validation.valid;
  if (!definition.validation.valid) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      "definition",
                      "persisted definition is not valid");
  }

  if (!armature) {
    add_mapping_issue(report,
                      MMDPhysicsMappingIssueSeverity::Error,
                      "armature",
                      "current Armature data is missing");
  }
  else {
    bArmature *mutable_armature = const_cast<bArmature *>(armature);
    BKE_armature_bone_hash_make(mutable_armature);
  }

  for (int i = 0; i < int(definition.bone_mapping.size()); i++) {
    const MMDBoneMapping &bone = definition.bone_mapping[i];
    bool valid = bone.pmx_index == i;
    if (!valid) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        indexed_path("bones", i, "pmx_index"),
                        "does not match array index");
    }
    if (bone.blender_bone_name.empty()) {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        indexed_path("bones", i, "blender_bone_name"),
                        "is empty");
      valid = false;
    }
    const bool found = armature && !bone.blender_bone_name.empty() &&
                       BKE_armature_find_bone_name(const_cast<bArmature *>(armature),
                                                   bone.blender_bone_name.c_str()) != nullptr;
    if (found) {
      report.resolved_bones++;
    }
    else {
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        indexed_path("bones", i, "blender_bone_name"),
                        "not found in current Armature");
      valid = false;
    }
    if (valid) {
      /* The mapping is valid for this entry. */
    }
    else {
      report.unresolved_bones++;
    }
  }

  for (int i = 0; i < int(definition.rigid_bodies.size()); i++) {
    const MMDRigidBodyDefinition &rigid = definition.rigid_bodies[i];
    const bool valid = validate_rigid_body(rigid, i, definition, report);
    if (!valid) {
      report.invalid_rigid_bodies++;
    }
    if (rigid.pmx_bone_index == -1) {
      report.resolved_rigid_bones++;
    }
    else if (rigid.pmx_bone_index >= 0 && rigid.pmx_bone_index < int(definition.bone_mapping.size()) &&
             armature)
    {
      const MMDBoneMapping &mapping = definition.bone_mapping[rigid.pmx_bone_index];
      const bool snapshot_matches = rigid.blender_bone_name == mapping.blender_bone_name;
      const bool current_bone_exists =
          BKE_armature_find_bone_name(const_cast<bArmature *>(armature),
                                      mapping.blender_bone_name.c_str()) != nullptr;
      if (snapshot_matches && current_bone_exists) {
        report.resolved_rigid_bones++;
      }
      else {
        report.unresolved_rigid_bones++;
        add_mapping_issue(report,
                          MMDPhysicsMappingIssueSeverity::Error,
                          indexed_path("rigid_bodies", i, "blender_bone_name"),
                          snapshot_matches ? "does not resolve to current Armature" :
                                              "does not match referenced bone mapping");
      }
    }
    else {
      report.unresolved_rigid_bones++;
      add_mapping_issue(report,
                        MMDPhysicsMappingIssueSeverity::Error,
                        indexed_path("rigid_bodies", i, "blender_bone_name"),
                        "does not resolve to current Armature");
    }
  }

  for (int i = 0; i < int(definition.joints.size()); i++) {
    const MMDJointDefinition &joint = definition.joints[i];
    const bool valid = validate_joint(joint, i, definition, report);
    if (!valid) {
      report.invalid_joints++;
    }
    const bool endpoints_valid = joint.rigid_a_index >= 0 &&
                                  joint.rigid_a_index < int(definition.rigid_bodies.size()) &&
                                  joint.rigid_b_index >= 0 &&
                                  joint.rigid_b_index < int(definition.rigid_bodies.size());
    if (endpoints_valid) {
      report.resolved_joint_endpoints++;
    }
    else {
      report.invalid_joint_endpoints++;
    }
  }
  return report;
}

MMDPhysicsDebugReport build_physics_debug_report(
    const MMDPhysicsDefinition &definition, const MMDPhysicsMappingReport &mapping_report)
{
  MMDPhysicsDebugReport report;
  report.definition_valid = definition.validation.valid;
  report.mapping_valid = mapping_report.mapping_valid;
  report.bone_count = int(definition.bone_mapping.size());
  report.resolved_bone_count = mapping_report.resolved_bones;
  report.rigid_body_count = int(definition.rigid_bodies.size());
  report.joint_count = int(definition.joints.size());
  report.resolved_joint_count = mapping_report.resolved_joint_endpoints;

  for (const MMDPhysicsMappingIssue &issue : mapping_report.issues) {
    if (issue.severity == MMDPhysicsMappingIssueSeverity::Error) {
      report.error_count++;
    }
    else if (issue.severity == MMDPhysicsMappingIssueSeverity::Warning) {
      report.warning_count++;
    }
    if (issue.severity != MMDPhysicsMappingIssueSeverity::Info) {
      const char *severity = issue.severity == MMDPhysicsMappingIssueSeverity::Error ? "ERROR" :
                                                                                         "WARNING";
      report.diagnostics.push_back(std::string(severity) + " " + issue.path + ": " +
                                   issue.message);
    }
  }

  for (const MMDRigidBodyDefinition &rigid : definition.rigid_bodies) {
    if (rigid.collision_group < report.collision_group_counts.size()) {
      report.collision_group_counts[rigid.collision_group]++;
    }
    for (int group = 0; group < 16; group++) {
      if ((rigid.no_collision_group & (uint16_t(1) << group)) != 0) {
        report.collision_group_mask_counts[group]++;
      }
    }
    if (rigid.shape_type < report.rigid_shape_counts.size()) {
      report.rigid_shape_counts[rigid.shape_type]++;
    }
    if (rigid.physics_type < report.rigid_type_counts.size()) {
      report.rigid_type_counts[rigid.physics_type]++;
    }

    if (rigid.pmx_bone_index == -1) {
      report.unbound_rigid_body_count++;
    }
    else if (rigid.pmx_bone_index >= 0 &&
             rigid.pmx_bone_index < int(definition.bone_mapping.size()) && rigid.bone_resolved &&
             definition.bone_mapping[rigid.pmx_bone_index].resolved)
    {
      report.bound_rigid_body_count++;
    }
    else {
      report.invalid_rigid_body_binding_count++;
    }
  }

  for (const MMDJointDefinition &joint : definition.joints) {
    if (joint.rigid_a_index == joint.rigid_b_index && joint.rigid_a_index >= 0 &&
        joint.rigid_a_index < int(definition.rigid_bodies.size()))
    {
      report.joints_with_same_endpoint++;
    }
    for (int axis = 0; axis < 3; axis++) {
      if (joint.rotation_limit_mode[axis] == MMDJointAxisLimitMode::Limited) {
        report.joint_rotation_limited_axes[axis]++;
      }
      else if (joint.rotation_limit_mode[axis] == MMDJointAxisLimitMode::Free) {
        report.joint_rotation_free_axes[axis]++;
      }
      if (joint.spring_translation[axis] < 0.0f) {
        report.negative_spring_count++;
      }
      if (joint.spring_rotation[axis] < 0.0f) {
        report.negative_spring_count++;
      }
    }
  }

  report.static_ready = report.definition_valid && report.mapping_valid &&
                        report.error_count == 0 && mapping_report.invalid_rigid_bodies == 0 &&
                        mapping_report.invalid_joints == 0 &&
                        report.invalid_rigid_body_binding_count == 0 &&
                        report.resolved_joint_count == report.joint_count;
  return report;
}

}  // namespace blender::mmd_physics
