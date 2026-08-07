/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_camera_action.hh"

#include "ANIM_action.hh"
#include "ANIM_fcurve.hh"
#include "BKE_anim_data.hh"
#include "BKE_camera.h"
#include "BKE_collection.hh"
#include "BKE_fcurve.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "BLI_math_matrix_c.hh"

#include "DNA_camera_types.h"
#include "DNA_object_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace blender::io::vmd {
namespace {

constexpr int64_t kMinActionFrame = std::numeric_limits<int>::min();
constexpr int64_t kMaxActionFrame = std::numeric_limits<int>::max();
constexpr float kDefaultSensorHeight = 24.0f;
constexpr float kDefaultCameraAngleDegrees = 30.0f;
constexpr float kMinCameraAngleDegrees = 1.0f;
constexpr float kMaxCameraAngleDegrees = 172.847f;
constexpr float kMinimumOrthoScale = 0.001f;
constexpr float kOrthoScalePerDistance = 25.0f / 45.0f;

void add_error(VMDCameraActionReport &result, ReportList *reports, const std::string &message)
{
  result.errors.push_back(message);
  BKE_report(reports, RPT_ERROR, message.c_str());
}

void add_warning(VMDCameraActionReport &result, ReportList *reports, const std::string &message)
{
  result.warnings.push_back(message);
  BKE_report(reports, RPT_WARNING, message.c_str());
}

bool checked_frame(const VMDCameraKeyframe &keyframe,
                  const int frame_offset,
                  int &r_frame,
                  VMDCameraActionReport &result,
                  ReportList *reports)
{
  const int64_t frame = int64_t(keyframe.frame) + int64_t(frame_offset);
  if (frame < kMinActionFrame || frame > kMaxActionFrame) {
    add_error(result,
              reports,
              "VMD camera frame plus frame offset is outside Blender's signed frame range");
    return false;
  }
  r_frame = int(frame);
  return true;
}

bool write_keyframe(FCurve &fcurve,
                    const int frame,
                    const float value,
                    const animrig::KeyframeSettings &settings)
{
  if (!std::isfinite(value)) {
    return false;
  }
  return animrig::insert_vert_fcurve(&fcurve, {float(frame), value}, settings, INSERTKEY_FAST) ==
         animrig::SingleKeyingResult::SUCCESS;
}

void free_action(Main *bmain, animrig::Action *action)
{
  if (action != nullptr) {
    BKE_id_free(bmain, &action->id);
  }
}

struct ActionBuildContext {
  animrig::Action *action = nullptr;
  animrig::Slot *slot = nullptr;
  animrig::Channelbag *channelbag = nullptr;
};

bool initialize_action(Main *bmain,
                       ID &target_id,
                       const std::string &action_name,
                       ActionBuildContext &r_context,
                       VMDCameraActionReport &result,
                       ReportList *reports)
{
  animrig::Action &action = animrig::action_add(*bmain, action_name);
  animrig::Slot &slot = action.slot_add_for_id(target_id);
  action.layer_keystrip_ensure();
  if (action.layers().is_empty() || action.layer(0)->strips().is_empty()) {
    add_error(result, reports, "Failed to initialize a VMD camera Action keyframe strip");
    free_action(bmain, &action);
    return false;
  }

  animrig::Strip &strip = *action.layer(0)->strip(0);
  r_context.action = &action;
  r_context.slot = &slot;
  r_context.channelbag = &strip.data<animrig::StripKeyframeData>(action)
                              .channelbag_for_slot_add(slot);
  return true;
}

FCurve &ensure_curve(animrig::Channelbag &channelbag,
                     const char *rna_path,
                     const int array_index,
                     const PropertyType prop_type)
{
  animrig::FCurveDescriptor descriptor;
  descriptor.rna_path = rna_path;
  descriptor.array_index = array_index;
  descriptor.prop_type = prop_type;
  descriptor.prop_subtype = PROP_NONE;
  return channelbag.fcurve_ensure(nullptr, descriptor);
}

float camera_lens_from_angle(const Camera &camera, const uint32_t view_angle, int &r_clamped_count)
{
  float angle_degrees = float(view_angle);
  const float clamped_angle = std::clamp(
      angle_degrees, kMinCameraAngleDegrees, kMaxCameraAngleDegrees);
  if (angle_degrees != clamped_angle) {
    r_clamped_count++;
  }
  const float sensor_height = camera.sensor_y > 0.0f ? camera.sensor_y : kDefaultSensorHeight;
  const float angle_radians = clamped_angle * float(M_PI) / 180.0f;
  return std::max(1.0f, sensor_height / (2.0f * std::tan(angle_radians * 0.5f)));
}

float camera_ortho_scale(const float distance)
{
  return std::max(kMinimumOrthoScale, std::abs(distance) * kOrthoScalePerDistance);
}

bool has_existing_action(const ID &id)
{
  const AnimData *anim_data = BKE_animdata_from_id(const_cast<ID *>(&id));
  return anim_data != nullptr && anim_data->action != nullptr;
}

}  // namespace

bool create_vmd_camera_rig(Main *bmain,
                           Collection &collection,
                           const std::string &base_name,
                           const float coordinate_scale,
                           Object *&r_target_empty,
                           Object *&r_camera,
                           ReportList *reports,
                           VMDCameraActionReport &r_result,
                           Object *existing_camera)
{
  r_target_empty = nullptr;
  r_camera = nullptr;
  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD camera rig creation requires a valid Main database");
    return false;
  }
  if (!std::isfinite(coordinate_scale) || coordinate_scale <= 0.0f) {
    add_error(r_result, reports, "VMD camera coordinate scale must be finite and positive");
    return false;
  }

  const std::string empty_name = base_name.empty() ? "VMD Camera" : base_name;
  const std::string camera_name = empty_name + " Camera";
  Object *target_empty = BKE_object_add_only_object(bmain, OB_EMPTY, empty_name.c_str());
  Object *camera = existing_camera;
  Camera *camera_data = nullptr;
  const bool created_camera = existing_camera == nullptr;
  if (created_camera) {
    camera_data = BKE_camera_add(bmain, (camera_name + " Data").c_str());
    camera = BKE_object_add_only_object(bmain, OB_CAMERA, camera_name.c_str());
    camera->data = id_cast<ID *>(&camera_data->id);
    id_us_plus(&camera_data->id);
  }
  else {
    if (existing_camera->type != OB_CAMERA || existing_camera->data == nullptr) {
      BKE_id_free(bmain, &target_empty->id);
      add_error(r_result, reports, "The selected VMD camera target is not a valid Camera object");
      return false;
    }
    camera_data = id_cast<Camera *>(existing_camera->data);
    if (camera_data == nullptr) {
      BKE_id_free(bmain, &target_empty->id);
      add_error(r_result, reports, "The selected VMD camera target has invalid Camera data");
      return false;
    }
  }

  const bool empty_linked = BKE_collection_object_add(bmain, &collection, target_empty);
  const bool camera_linked = BKE_collection_has_object(&collection, camera) ||
                              BKE_collection_object_add(bmain, &collection, camera);
  if (!empty_linked || !camera_linked)
  {
    if (created_camera) {
      BKE_id_free(bmain, &camera->id);
    }
    BKE_id_free(bmain, &target_empty->id);
    add_error(r_result, reports, "Failed to link the imported VMD camera rig to the collection");
    return false;
  }

  target_empty->empty_drawsize = 5.0f * coordinate_scale;
  target_empty->rotmode = ROT_MODE_YXZ;
  target_empty->loc[0] = target_empty->loc[1] = target_empty->loc[2] = 0.0f;
  target_empty->rot[0] = target_empty->rot[1] = target_empty->rot[2] = 0.0f;
  target_empty->scale[0] = target_empty->scale[1] = target_empty->scale[2] = 1.0f;

  camera->parent = target_empty;
  camera->partype = PAROBJECT;
  unit_m4(camera->parentinv);
  camera->rotmode = ROT_MODE_XYZ;
  camera->loc[0] = camera->loc[1] = camera->loc[2] = 0.0f;
  camera->rot[0] = float(M_PI_2);
  camera->rot[1] = camera->rot[2] = 0.0f;
  camera->scale[0] = camera->scale[1] = camera->scale[2] = 1.0f;

  camera_data->sensor_fit = CAMERA_SENSOR_FIT_VERT;
  camera_data->clip_end = 500.0f * coordinate_scale;
  camera_data->ortho_scale = 25.0f * coordinate_scale;
  camera_data->type = CAM_PERSP;
  camera_data->lens = camera_lens_from_angle(
      *camera_data, uint32_t(kDefaultCameraAngleDegrees), r_result.clamped_angle_count);

  r_target_empty = target_empty;
  r_camera = camera;
  r_result.target_empty_name = target_empty->id.name + 2;
  r_result.target_camera_name = camera->id.name + 2;
  return true;
}

bool build_vmd_camera_action(Main *bmain,
                             Object &target_empty,
                             Object &target_camera,
                             const VMDModel &model,
                             const std::string &action_name,
                             const VMDCameraActionOptions &options,
                             ReportList *reports,
                             VMDCameraActionReport &r_result)
{
  r_result = {};
  r_result.action_name = action_name;
  r_result.target_empty_name = target_empty.id.name + 2;
  r_result.target_camera_name = target_camera.id.name + 2;

  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD camera Action build requires a valid Main database");
    return false;
  }
  if (target_empty.type != OB_EMPTY || target_camera.type != OB_CAMERA ||
      target_camera.data == nullptr)
  {
    add_error(r_result, reports, "VMD camera Action targets must be an Empty and a Camera");
    return false;
  }
  if (action_name.empty()) {
    add_error(r_result, reports, "VMD camera Action name must not be empty");
    return false;
  }
  if (model.camera_keyframes.empty()) {
    add_error(r_result, reports, "VMD file contains no camera keyframes");
    return false;
  }
  if (!std::isfinite(options.coordinate_scale) || options.coordinate_scale <= 0.0f) {
    add_error(r_result, reports, "VMD camera coordinate scale must be finite and positive");
    return false;
  }
  if (!options.replace_existing_action &&
      (has_existing_action(target_empty.id) || has_existing_action(target_camera.id) ||
       has_existing_action(*target_camera.data)))
  {
    add_error(r_result,
              reports,
              "A VMD camera target already has an Action; refusing to replace it");
    return false;
  }

  Camera *camera_data = id_cast<Camera *>(target_camera.data);
  if (camera_data == nullptr) {
    add_error(r_result, reports, "VMD camera target has invalid Camera data");
    return false;
  }

  std::vector<const VMDCameraKeyframe *> keyframes;
  keyframes.reserve(model.camera_keyframes.size());
  for (const VMDCameraKeyframe &keyframe : model.camera_keyframes) {
    keyframes.push_back(&keyframe);
  }
  std::sort(keyframes.begin(), keyframes.end(), [](const VMDCameraKeyframe *a,
                                                   const VMDCameraKeyframe *b) {
    if (a->frame != b->frame) {
      return a->frame < b->frame;
    }
    return a->source_offset < b->source_offset;
  });

  std::vector<const VMDCameraKeyframe *> unique_keyframes;
  unique_keyframes.reserve(keyframes.size());
  for (const VMDCameraKeyframe *keyframe : keyframes) {
    if (!unique_keyframes.empty() && unique_keyframes.back()->frame == keyframe->frame) {
      unique_keyframes.back() = keyframe;
      r_result.duplicate_frame_count++;
    }
    else {
      unique_keyframes.push_back(keyframe);
    }
  }
  if (r_result.duplicate_frame_count > 0) {
    add_warning(r_result,
                reports,
                "Duplicate VMD camera frames were collapsed; the last record at each frame was "
                "kept");
  }

  for (const VMDCameraKeyframe *keyframe : unique_keyframes) {
    if (!std::isfinite(keyframe->distance)) {
      add_error(r_result, reports, "VMD camera distance contains a non-finite value");
      return false;
    }
    for (const float value : keyframe->position) {
      if (!std::isfinite(value)) {
        add_error(r_result, reports, "VMD camera position contains a non-finite value");
        return false;
      }
    }
    for (const float value : keyframe->rotation) {
      if (!std::isfinite(value)) {
        add_error(r_result, reports, "VMD camera rotation contains a non-finite value");
        return false;
      }
    }
    int frame = 0;
    if (!checked_frame(*keyframe, options.frame_offset, frame, r_result, reports)) {
      return false;
    }
    if (r_result.first_frame < 0 || frame < r_result.first_frame) {
      r_result.first_frame = frame;
    }
    if (r_result.last_frame < 0 || frame > r_result.last_frame) {
      r_result.last_frame = frame;
    }
  }

  ActionBuildContext parent_action;
  ActionBuildContext camera_action;
  ActionBuildContext camera_data_action;
  if (!initialize_action(bmain,
                         target_empty.id,
                         action_name + " | VMD Camera Target",
                         parent_action,
                         r_result,
                         reports))
  {
    return false;
  }
  if (!initialize_action(bmain,
                         target_camera.id,
                         action_name + " | VMD Camera Distance",
                         camera_action,
                         r_result,
                         reports))
  {
    free_action(bmain, parent_action.action);
    return false;
  }
  if (!initialize_action(bmain,
                         camera_data->id,
                         action_name + " | VMD Camera Data",
                         camera_data_action,
                         r_result,
                         reports))
  {
    free_action(bmain, camera_action.action);
    free_action(bmain, parent_action.action);
    return false;
  }

  std::array<FCurve *, 3> location_curves;
  std::array<FCurve *, 3> rotation_curves;
  for (int index = 0; index < 3; index++) {
    location_curves[index] = &ensure_curve(
        *parent_action.channelbag, "location", index, PROP_FLOAT);
    rotation_curves[index] = &ensure_curve(
        *parent_action.channelbag, "rotation_euler", index, PROP_FLOAT);
  }
  FCurve *distance_curve = &ensure_curve(*camera_action.channelbag, "location", 1, PROP_FLOAT);
  FCurve *lens_curve = &ensure_curve(*camera_data_action.channelbag, "lens", 0, PROP_FLOAT);
  FCurve *ortho_scale_curve = &ensure_curve(
      *camera_data_action.channelbag, "ortho_scale", 0, PROP_FLOAT);
  FCurve *type_curve = &ensure_curve(*camera_data_action.channelbag, "type", 0, PROP_ENUM);
  r_result.fcurve_count = 3 + 3 + 1 + 3;

  const animrig::KeyframeSettings key_settings = {
      BEZT_KEYTYPE_KEYFRAME,
      HD_AUTO_ANIM,
      options.use_linear_interpolation ? BEZT_IPO_LIN : BEZT_IPO_BEZ};
  const animrig::KeyframeSettings type_settings = {
      BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, BEZT_IPO_CONST};

  for (const VMDCameraKeyframe *keyframe : unique_keyframes) {
    int frame = 0;
    if (!checked_frame(*keyframe, options.frame_offset, frame, r_result, reports)) {
      free_action(bmain, camera_data_action.action);
      free_action(bmain, camera_action.action);
      free_action(bmain, parent_action.action);
      return false;
    }

    const float scale = options.coordinate_scale;
    const std::array<float, 3> location = {
        keyframe->position[0] * scale, keyframe->position[2] * scale, keyframe->position[1] * scale};
    const std::array<float, 3> rotation = {
        keyframe->rotation[0], keyframe->rotation[2], keyframe->rotation[1]};
    const float distance = keyframe->distance * scale;
    const float lens = camera_lens_from_angle(
        *camera_data, keyframe->view_angle, r_result.clamped_angle_count);
    const float ortho_scale = camera_ortho_scale(distance);
    const float camera_type = keyframe->perspective ? float(CAM_PERSP) : float(CAM_ORTHO);

    for (int index = 0; index < 3; index++) {
      if (!write_keyframe(*location_curves[index], frame, location[index], key_settings) ||
          !write_keyframe(*rotation_curves[index], frame, rotation[index], key_settings))
      {
        add_error(r_result, reports, "Failed to insert VMD camera transform keyframe");
        free_action(bmain, camera_data_action.action);
        free_action(bmain, camera_action.action);
        free_action(bmain, parent_action.action);
        return false;
      }
    }
    if (!write_keyframe(*distance_curve, frame, distance, key_settings) ||
        !write_keyframe(*lens_curve, frame, lens, key_settings) ||
        !write_keyframe(*ortho_scale_curve, frame, ortho_scale, key_settings) ||
        !write_keyframe(*type_curve, frame, camera_type, type_settings))
    {
      add_error(r_result, reports, "Failed to insert VMD camera property keyframe");
      free_action(bmain, camera_data_action.action);
      free_action(bmain, camera_action.action);
      free_action(bmain, parent_action.action);
      return false;
    }
    r_result.keyframe_count += 10;
  }

  std::vector<const uint8_t *> interpolation_data;
  interpolation_data.reserve(unique_keyframes.size());
  for (const VMDCameraKeyframe *keyframe : unique_keyframes) {
    interpolation_data.push_back(keyframe->interpolation.data());
  }
  const std::array<int, 4> camera_control_offsets = {0, 2, 1, 3};
  const VMDBezierWarningCallback warning_callback = [&](const std::string &message) {
    add_warning(r_result, reports, message);
  };

  for (FCurve *curve : location_curves) {
    BKE_fcurve_handles_recalc(*curve);
  }
  for (FCurve *curve : rotation_curves) {
    BKE_fcurve_handles_recalc(*curve);
  }
  BKE_fcurve_handles_recalc(*distance_curve);
  BKE_fcurve_handles_recalc(*lens_curve);
  BKE_fcurve_handles_recalc(*ortho_scale_curve);
  BKE_fcurve_handles_recalc(*type_curve);

  if (options.use_vmd_bezier_interpolation && !options.use_linear_interpolation) {
    apply_vmd_bezier_to_curve(*location_curves[0],
                              interpolation_data,
                              VMD_CAMERA_INTERP_X,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    apply_vmd_bezier_to_curve(*location_curves[1],
                              interpolation_data,
                              VMD_CAMERA_INTERP_Z,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    apply_vmd_bezier_to_curve(*location_curves[2],
                              interpolation_data,
                              VMD_CAMERA_INTERP_Y,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    for (FCurve *curve : rotation_curves) {
      apply_vmd_bezier_to_curve(*curve,
                                interpolation_data,
                                VMD_CAMERA_INTERP_ROTATION,
                                VMD_CAMERA_INTERP_CHANNEL_BYTES,
                                camera_control_offsets,
                                warning_callback);
    }
    apply_vmd_bezier_to_curve(*distance_curve,
                              interpolation_data,
                              VMD_CAMERA_INTERP_DISTANCE,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    apply_vmd_bezier_to_curve(*lens_curve,
                              interpolation_data,
                              VMD_CAMERA_INTERP_ANGLE,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    apply_vmd_bezier_to_curve(*ortho_scale_curve,
                              interpolation_data,
                              VMD_CAMERA_INTERP_DISTANCE,
                              VMD_CAMERA_INTERP_CHANNEL_BYTES,
                              camera_control_offsets,
                              warning_callback);
    r_result.bezier_curve_count = 3 + 3 + 1 + 1 + 1;
  }

  if (animrig::assign_action_and_slot(
          parent_action.action, parent_action.slot, target_empty.id) !=
      animrig::ActionSlotAssignmentResult::OK)
  {
    add_error(r_result, reports, "Failed to bind the VMD camera target Action");
    free_action(bmain, camera_data_action.action);
    free_action(bmain, camera_action.action);
    free_action(bmain, parent_action.action);
    return false;
  }
  r_result.parent_action_bound = true;
  if (animrig::assign_action_and_slot(
          camera_action.action, camera_action.slot, target_camera.id) !=
      animrig::ActionSlotAssignmentResult::OK)
  {
    add_error(r_result, reports, "Failed to bind the VMD camera distance Action");
    free_action(bmain, camera_data_action.action);
    free_action(bmain, camera_action.action);
    return false;
  }
  r_result.camera_action_bound = true;
  if (animrig::assign_action_and_slot(
          camera_data_action.action, camera_data_action.slot, camera_data->id) !=
      animrig::ActionSlotAssignmentResult::OK)
  {
    add_error(r_result, reports, "Failed to bind the VMD camera data Action");
    free_action(bmain, camera_data_action.action);
    return false;
  }
  r_result.camera_data_action_bound = true;

  if (r_result.clamped_angle_count > 0) {
    add_warning(r_result,
                reports,
                "VMD camera view angles outside Blender's supported range were clamped on " +
                    std::to_string(r_result.clamped_angle_count) + " keyframe(s)");
  }
  r_result.success = true;
  return true;
}

}  // namespace blender::io::vmd
