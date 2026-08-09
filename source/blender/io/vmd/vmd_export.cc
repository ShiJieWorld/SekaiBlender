/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "vmd_export.hh"

#include "IO_vmd.hh"
#include "vmd_action.hh"

#include "ANIM_action.hh"
#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_fcurve.hh"
#include "BKE_key.hh"
#include "BKE_pose.hh"
#include "BKE_report.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_string.hh"
#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_camera_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>

namespace blender::io::vmd {
namespace {

struct ExportBoneTrack {
  std::string name;
  std::array<const FCurve *, 3> location{};
  std::array<const FCurve *, 4> rotation{};
  std::set<int> frames;
  BoneConverter converter;
};

struct ExportMorphTrack {
  std::string name;
  const FCurve *curve = nullptr;
  std::set<int> frames;
};

/* Shared by the bone/morph and camera report types; both expose `errors` and `warnings`. */
template<typename Report>
void add_error(Report &report, ReportList *reports, const std::string &message)
{
  report.errors.push_back(message);
  BKE_report(reports, RPT_ERROR, message.c_str());
}

template<typename Report>
void add_warning(Report &report, ReportList *reports, const std::string &message)
{
  report.warnings.push_back(message);
  BKE_report(reports, RPT_WARNING, message.c_str());
}

bool collect_curve_frames(const FCurve &curve,
                          const int frame_start,
                          const int frame_end,
                          std::set<int> &frames,
                          std::string &r_error)
{
  auto add_frame = [&](const float source_frame) {
    if (!std::isfinite(source_frame)) {
      r_error = "VMD export encountered a non-finite Action keyframe";
      return false;
    }
    const float rounded = std::round(source_frame);
    if (std::fabs(source_frame - rounded) > 1.0e-4f) {
      r_error = "VMD export requires integer Action keyframes";
      return false;
    }
    if (rounded < float(std::numeric_limits<int>::min()) ||
        rounded > float(std::numeric_limits<int>::max()))
    {
      r_error = "VMD export Action keyframe is outside the supported frame range";
      return false;
    }
    const int frame = int(rounded);
    if (source_frame >= float(frame_start) && source_frame <= float(frame_end)) {
      frames.insert(frame);
    }
    return true;
  };

  if (curve.bezt != nullptr) {
    for (int i = 0; i < curve.totvert; i++) {
      if (!add_frame(curve.bezt[i].vec[1][0])) {
        return false;
      }
    }
  }
  else if (curve.fpt != nullptr) {
    for (int i = 0; i < curve.totvert; i++) {
      if (!add_frame(curve.fpt[i].vec[0])) {
        return false;
      }
    }
  }
  return true;
}

std::array<int8_t, 64> linear_interpolation()
{
  std::array<int8_t, 64> result{};
  std::array<int8_t, 16> block{};
  for (int channel = 0; channel < 4; channel++) {
    block[channel] = 20;
    block[channel + 4] = 20;
    block[channel + 8] = 107;
    block[channel + 12] = 107;
  }
  for (int block_index = 0; block_index < 4; block_index++) {
    std::copy(block.begin(), block.end(), result.begin() + block_index * 16);
  }
  return result;
}

struct VMDBezierControl {
  std::array<int8_t, 4> bytes{};
};

bool quantize_vmd_control(const float value, int8_t &r_byte)
{
  if (!std::isfinite(value) || value < -1.0e-5f || value > 1.0f + 1.0e-5f) {
    return false;
  }
  const float scaled = std::clamp(value, 0.0f, 1.0f) * 127.0f;
  r_byte = int8_t(std::round(scaled));
  return true;
}

bool find_bezier_segment(const FCurve &curve,
                         const int frame,
                         const int next_frame,
                         const BezTriple *&r_start,
                         const BezTriple *&r_end)
{
  if (curve.bezt == nullptr || next_frame <= frame) {
    return false;
  }
  for (int i = 0; i + 1 < curve.totvert; i++) {
    const BezTriple &start = curve.bezt[i];
    const BezTriple &end = curve.bezt[i + 1];
    if (std::fabs(start.vec[1][0] - float(frame)) <= 1.0e-4f &&
        std::fabs(end.vec[1][0] - float(next_frame)) <= 1.0e-4f)
    {
      if (start.ipo != BEZT_IPO_BEZ || start.h2 != HD_FREE || end.h1 != HD_FREE) {
        return false;
      }
      r_start = &start;
      r_end = &end;
      return true;
    }
  }
  return false;
}

bool bezier_control_from_curve(const FCurve &curve,
                               const int frame,
                               const int next_frame,
                               VMDBezierControl &r_control)
{
  const BezTriple *start = nullptr;
  const BezTriple *end = nullptr;
  if (!find_bezier_segment(curve, frame, next_frame, start, end)) {
    return false;
  }
  const float frame_delta = end->vec[1][0] - start->vec[1][0];
  const float value_delta = end->vec[1][1] - start->vec[1][1];
  const float value_scale = std::max({1.0f, std::fabs(start->vec[1][1]), std::fabs(end->vec[1][1])});
  if (!(frame_delta > 0.0f) || std::fabs(value_delta) <= 1.0e-7f * value_scale) {
    return false;
  }
  const float normalized[4] = {
      (start->vec[2][0] - start->vec[1][0]) / frame_delta,
      (start->vec[2][1] - start->vec[1][1]) / value_delta,
      (end->vec[0][0] - start->vec[1][0]) / frame_delta,
      (end->vec[0][1] - start->vec[1][1]) / value_delta,
  };
  for (int i = 0; i < 4; i++) {
    if (!quantize_vmd_control(normalized[i], r_control.bytes[i])) {
      return false;
    }
  }
  /* Deliberately no `ax <= bx` requirement. MikuMikuDance's own interpolation editor writes
   * segments whose two control points cross in frame space (real motions contain e.g.
   * {ax=125, bx=124}), producing a curve that is non-monotonic in time. Rejecting those would
   * silently downgrade an author's easing to linear on export. */
  return true;
}

bool bezier_timing_from_curve(const FCurve &curve,
                              const int frame,
                              const int next_frame,
                              std::array<int8_t, 2> &r_timing)
{
  const BezTriple *start = nullptr;
  const BezTriple *end = nullptr;
  if (!find_bezier_segment(curve, frame, next_frame, start, end)) {
    return false;
  }
  const float frame_delta = end->vec[1][0] - start->vec[1][0];
  if (!(frame_delta > 0.0f) ||
      !quantize_vmd_control((start->vec[2][0] - start->vec[1][0]) / frame_delta,
                            r_timing[0]) ||
      !quantize_vmd_control((end->vec[0][0] - start->vec[1][0]) / frame_delta, r_timing[1]))
  {
    return false;
  }
  /* Crossed control points are legal in VMD; see `bezier_control_from_curve()`. */
  return true;
}

void set_interpolation_channel(std::array<int8_t, 64> &interpolation,
                               const int channel,
                               const VMDBezierControl &control)
{
  const int base = channel * 16;
  interpolation[base] = control.bytes[0];
  interpolation[base + 4] = control.bytes[1];
  interpolation[base + 8] = control.bytes[2];
  interpolation[base + 12] = control.bytes[3];
}

bool same_control(const VMDBezierControl &a, const VMDBezierControl &b)
{
  return a.bytes == b.bytes;
}

std::array<int8_t, 64> interpolation_for_segment(const ExportBoneTrack &track,
                                                 const int frame,
                                                 const int next_frame)
{
  std::array<int8_t, 64> result = linear_interpolation();
  for (int blender_axis = 0; blender_axis < 3; blender_axis++) {
    VMDBezierControl control;
    const FCurve *curve = track.location[blender_axis];
    if (curve != nullptr && bezier_control_from_curve(*curve, frame, next_frame, control)) {
      set_interpolation_channel(
          result, track.converter.interpolation_indices[blender_axis], control);
    }
  }

  bool have_rotation_control = false;
  bool rotation_is_consistent = true;
  VMDBezierControl rotation_control;
  std::array<int8_t, 2> rotation_timing{};
  bool have_rotation_timing = false;
  for (const FCurve *curve : track.rotation) {
    if (curve == nullptr) {
      rotation_is_consistent = false;
      break;
    }
    std::array<int8_t, 2> candidate_timing;
    if (!bezier_timing_from_curve(*curve, frame, next_frame, candidate_timing)) {
      rotation_is_consistent = false;
      break;
    }
    if (!have_rotation_timing) {
      rotation_timing = candidate_timing;
      have_rotation_timing = true;
    }
    else if (rotation_timing != candidate_timing) {
      rotation_is_consistent = false;
      break;
    }
    VMDBezierControl candidate;
    if (!bezier_control_from_curve(*curve, frame, next_frame, candidate)) {
      continue;
    }
    if (!have_rotation_control) {
      rotation_control = candidate;
      have_rotation_control = true;
    }
    else if (!same_control(rotation_control, candidate)) {
      rotation_is_consistent = false;
      break;
    }
  }
  if (rotation_is_consistent && have_rotation_control &&
      rotation_control.bytes[0] == rotation_timing[0] &&
      rotation_control.bytes[2] == rotation_timing[1])
  {
    set_interpolation_channel(result, 3, rotation_control);
  }
  return result;
}

bool collect_morph_tracks(const Object &controller,
                          const VMDExportOptions &options,
                          VMDExportReport &report,
                          ReportList *reports,
                          VMDModel &model)
{
  if (controller.type != OB_MESH) {
    add_warning(report, reports, "Skipping VMD Morph export: controller is not a Mesh");
    return true;
  }
  const Mesh *mesh = reinterpret_cast<const Mesh *>(controller.data);
  const Key *key = mesh ? mesh->key : nullptr;
  const AnimData *anim_data = key ? BKE_animdata_from_id(&key->id) : nullptr;
  if (key == nullptr || anim_data == nullptr || anim_data->action == nullptr ||
      anim_data->slot_handle == animrig::Slot::unassigned)
  {
    return true;
  }

  std::map<std::string, ExportMorphTrack> tracks;
  std::string frame_error;
  for (const FCurve *curve :
       animrig::fcurves_for_action_slot(anim_data->action->wrap(), anim_data->slot_handle))
  {
    if (curve == nullptr || curve->rna_path().is_empty() || curve->array_index != 0) {
      continue;
    }
    char morph_name[MAX_NAME] = {};
    if (!BLI_str_quoted_substr(
            curve->rna_path().c_str(), "key_blocks[", morph_name, sizeof(morph_name)) ||
        !curve->rna_path().endswith("].value"))
    {
      continue;
    }
    const KeyBlock *block = BKE_keyblock_find_name(const_cast<Key *>(key), morph_name);
    if (block == nullptr || block == key->refkey) {
      continue;
    }
    ExportMorphTrack &track = tracks[morph_name];
    track.name = morph_name;
    track.curve = curve;
    if (!collect_curve_frames(
            *curve, options.frame_start, options.frame_end, track.frames, frame_error))
    {
      add_error(report, reports, frame_error);
      return false;
    }
  }

  for (auto &[name, track] : tracks) {
    track.frames.insert(options.frame_start);
    track.frames.insert(options.frame_end);
    for (const int frame : track.frames) {
      const float weight = evaluate_fcurve(track.curve, float(frame));
      if (!std::isfinite(weight)) {
        add_error(report, reports, "VMD export encountered a non-finite Morph value: " + name);
        return false;
      }
      model.morph_keyframes.push_back(
          {name, uint32_t(frame - options.frame_start), weight, 0});
    }
    report.morph_count++;
  }
  report.morph_frame_count = int(model.morph_keyframes.size());
  return true;
}

/* -------------------------------------------------------------------- */
/** \name Camera Export
 *
 * The exact inverse of the native VMD camera importer (`vmd_camera_action.cc`). The importer
 * builds a parent Empty holding the MMD look-at center and YXZ rotation, plus a child Camera
 * whose local Y carries the MMD distance and whose Camera data carries the view angle and the
 * perspective flag. Exporting reads those same F-Curves back and re-encodes them, so an
 * import → export round trip reproduces the source records rather than approximating them.
 * \{ */

/* MikuMikuDance identifies a camera/lighting motion by this fixed header name. Written as
 * explicit UTF-8 bytes because C++20 `u8""` literals are `char8_t` and the source encoding of
 * a raw literal is not guaranteed. Decodes to "カメラ・照明". */
constexpr char kVMDCameraModelName[] =
    "\xE3\x82\xAB\xE3\x83\xA1\xE3\x83\xA9\xE3\x83\xBB\xE7\x85\xA7\xE6\x98\x8E";

/* The importer's fixed camera child pose: +90 degrees about local X, no lateral offset. */
constexpr float kCameraChildPitch = float(M_PI_2);
constexpr float kPoseEpsilon = 1.0e-4f;

/* The importer always writes vertical sensor fit and derives the lens from `sensor_y`. */
float camera_sensor_height(const Camera &camera)
{
  return camera.sensor_y > 0.0f ? camera.sensor_y : 24.0f;
}

/* Inverse of `camera_lens_from_angle()`: lens = sensor_h / (2 * tan(angle / 2)). */
bool view_angle_from_lens(const Camera &camera,
                          const float lens,
                          uint32_t &r_view_angle,
                          bool &r_clamped)
{
  r_clamped = false;
  if (!std::isfinite(lens) || lens <= 0.0f) {
    return false;
  }
  const float degrees = 2.0f *
                        std::atan(camera_sensor_height(camera) / (2.0f * lens)) * 180.0f /
                        float(M_PI);
  if (!std::isfinite(degrees)) {
    return false;
  }
  const float rounded = std::round(degrees);
  const float clamped = std::clamp(rounded, 1.0f, 180.0f);
  /* Only report a real clamp, not the sub-degree rounding the integer VMD field always costs. */
  if (std::fabs(degrees - clamped) > 0.5f) {
    r_clamped = true;
  }
  r_view_angle = uint32_t(clamped);
  return true;
}

const FCurve *find_id_fcurve(const ID &id, const char *rna_path, const int array_index)
{
  const AnimData *anim_data = BKE_animdata_from_id(const_cast<ID *>(&id));
  if (anim_data == nullptr || anim_data->action == nullptr ||
      anim_data->slot_handle == animrig::Slot::unassigned)
  {
    return nullptr;
  }
  for (const FCurve *curve :
       animrig::fcurves_for_action_slot(anim_data->action->wrap(), anim_data->slot_handle))
  {
    if (curve == nullptr || curve->rna_path().is_empty() || curve->array_index != array_index) {
      continue;
    }
    if (curve->rna_path() == rna_path) {
      return curve;
    }
  }
  return nullptr;
}

float sample_curve(const FCurve *curve, const int frame, const float fallback)
{
  return curve != nullptr ? evaluate_fcurve(curve, float(frame)) : fallback;
}

/**
 * Reads an animated object's rotation as MMD-facing YXZ Euler angles.
 *
 * The importer writes `rotation_euler` on a YXZ Empty, which is the only layout whose channels
 * map one-to-one onto the VMD rotation channel. Other rotation modes still export correctly,
 * but they must be recomposed through a matrix per frame, so their per-channel Bezier handles
 * no longer describe the exported curve and interpolation degrades to linear.
 */
struct CameraRotationSource {
  std::array<const FCurve *, 3> euler{};
  std::array<const FCurve *, 4> quaternion{};
  bool is_quaternion = false;
  bool is_native_yxz = false;
  short rotmode = ROT_MODE_YXZ;
  const Object *object = nullptr;

  bool resolve(const Object &ob, std::string &r_error)
  {
    object = &ob;
    rotmode = ob.rotmode;
    if (rotmode == ROT_MODE_AXISANGLE) {
      r_error =
          "VMD camera export does not support Axis Angle rotation; use Quaternion or an Euler "
          "order";
      return false;
    }
    if (rotmode == ROT_MODE_QUAT) {
      is_quaternion = true;
      for (int i = 0; i < 4; i++) {
        quaternion[i] = find_id_fcurve(ob.id, "rotation_quaternion", i);
      }
      return true;
    }
    is_native_yxz = rotmode == ROT_MODE_YXZ;
    for (int i = 0; i < 3; i++) {
      euler[i] = find_id_fcurve(ob.id, "rotation_euler", i);
    }
    return true;
  }

  /** Evaluate at `frame` and return YXZ Euler angles in Blender axes. */
  bool evaluate_yxz(const int frame, float r_yxz[3]) const
  {
    if (is_quaternion) {
      float quat[4];
      for (int i = 0; i < 4; i++) {
        quat[i] = sample_curve(quaternion[i], frame, object->quat[i]);
      }
      if (normalize_qt(quat) == 0.0f) {
        return false;
      }
      quat_to_eulO(r_yxz, ROT_MODE_YXZ, quat);
      return true;
    }
    float euler_values[3];
    for (int i = 0; i < 3; i++) {
      euler_values[i] = sample_curve(euler[i], frame, object->rot[i]);
      if (!std::isfinite(euler_values[i])) {
        return false;
      }
    }
    if (is_native_yxz) {
      std::copy(std::begin(euler_values), std::end(euler_values), r_yxz);
      return true;
    }
    float mat[3][3];
    eulO_to_mat3(mat, euler_values, rotmode);
    mat3_to_eulO(r_yxz, ROT_MODE_YXZ, mat);
    return true;
  }

  /** Collect keyframe times from whichever channel set backs this rotation. */
  bool collect_frames(const int frame_start,
                      const int frame_end,
                      std::set<int> &frames,
                      std::string &r_error) const
  {
    const int count = is_quaternion ? 4 : 3;
    for (int i = 0; i < count; i++) {
      const FCurve *curve = is_quaternion ? quaternion[i] : euler[i];
      if (curve != nullptr && !collect_curve_frames(*curve, frame_start, frame_end, frames, r_error))
      {
        return false;
      }
    }
    return true;
  }
};

/**
 * The resolved camera animation source.
 *
 * `rig` mode consumes an importer-shaped Empty + Camera pair and is lossless. `bare` mode takes
 * an unparented Camera, treats its own transform as world space, and re-derives the MMD center
 * and rotation with distance pinned to zero. MMD's camera model can represent any pose that way,
 * so the exported motion still matches, but the rotation channel is a matrix recomposition of the
 * camera's own Euler decomposition and therefore cannot carry Bezier handles.
 */
struct ExportCameraTrack {
  const Object *target_empty = nullptr;
  const Object *camera_object = nullptr;
  const Camera *camera_data = nullptr;
  std::array<const FCurve *, 3> location{};
  const FCurve *distance = nullptr;
  const FCurve *lens = nullptr;
  const FCurve *camera_type = nullptr;
  CameraRotationSource rotation;
  std::set<int> frames;
  bool rig_mode = false;
  bool rotation_bezier_allowed = false;
};

/* MMD packs a camera channel as {ax, bx, ay, by}, so the normalized {ax, ay, bx, by} order the
 * shared Bezier extraction produces maps onto byte offsets {0, 2, 1, 3}. */
constexpr std::array<int, 4> kCameraControlOffsets = {0, 2, 1, 3};

std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> linear_camera_interpolation()
{
  std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> result{};
  for (int channel = 0; channel < VMD_CAMERA_INTERP_CHANNELS; channel++) {
    const int base = channel * VMD_CAMERA_INTERP_CHANNEL_BYTES;
    result[base + kCameraControlOffsets[0]] = 20;
    result[base + kCameraControlOffsets[1]] = 20;
    result[base + kCameraControlOffsets[2]] = 107;
    result[base + kCameraControlOffsets[3]] = 107;
  }
  return result;
}

void set_camera_interpolation_channel(std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> &interpolation,
                                      const int channel,
                                      const VMDBezierControl &control)
{
  const int base = channel * VMD_CAMERA_INTERP_CHANNEL_BYTES;
  for (int i = 0; i < 4; i++) {
    interpolation[base + kCameraControlOffsets[i]] = uint8_t(control.bytes[i]);
  }
}

/** Copy one curve's Bezier handles into `channel` when the segment is representable in VMD. */
void camera_channel_from_curve(std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> &interpolation,
                               const FCurve *curve,
                               const int channel,
                               const int frame,
                               const int next_frame)
{
  VMDBezierControl control;
  if (curve != nullptr && bezier_control_from_curve(*curve, frame, next_frame, control)) {
    set_camera_interpolation_channel(interpolation, channel, control);
  }
}

std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> camera_interpolation_for_segment(
    const ExportCameraTrack &track, const int frame, const int next_frame)
{
  std::array<uint8_t, VMD_CAMERA_INTERP_BYTES> result = linear_camera_interpolation();
  /* The importer maps MMD (x, y, z) onto Blender location (x, z, y). */
  camera_channel_from_curve(result, track.location[0], VMD_CAMERA_INTERP_X, frame, next_frame);
  camera_channel_from_curve(result, track.location[2], VMD_CAMERA_INTERP_Y, frame, next_frame);
  camera_channel_from_curve(result, track.location[1], VMD_CAMERA_INTERP_Z, frame, next_frame);
  camera_channel_from_curve(
      result, track.distance, VMD_CAMERA_INTERP_DISTANCE, frame, next_frame);
  camera_channel_from_curve(result, track.lens, VMD_CAMERA_INTERP_ANGLE, frame, next_frame);

  /* MMD stores a single rotation channel for all three angles, so only emit it when every Euler
   * curve agrees on timing and shape. Mirrors the bone exporter's rotation consistency rule. */
  if (!track.rotation_bezier_allowed) {
    return result;
  }
  bool consistent = true;
  bool have_control = false;
  bool have_timing = false;
  VMDBezierControl rotation_control;
  std::array<int8_t, 2> rotation_timing{};
  for (const FCurve *curve : track.rotation.euler) {
    if (curve == nullptr) {
      consistent = false;
      break;
    }
    std::array<int8_t, 2> candidate_timing;
    if (!bezier_timing_from_curve(*curve, frame, next_frame, candidate_timing)) {
      consistent = false;
      break;
    }
    if (!have_timing) {
      rotation_timing = candidate_timing;
      have_timing = true;
    }
    else if (rotation_timing != candidate_timing) {
      consistent = false;
      break;
    }
    VMDBezierControl candidate;
    if (!bezier_control_from_curve(*curve, frame, next_frame, candidate)) {
      continue; /* Constant channel: timing still applies, no value control to compare. */
    }
    if (!have_control) {
      rotation_control = candidate;
      have_control = true;
    }
    else if (!same_control(rotation_control, candidate)) {
      consistent = false;
      break;
    }
  }
  if (consistent && have_control && rotation_control.bytes[0] == rotation_timing[0] &&
      rotation_control.bytes[2] == rotation_timing[1])
  {
    set_camera_interpolation_channel(result, VMD_CAMERA_INTERP_ROTATION, rotation_control);
  }
  return result;
}

/** Report deviations from the importer's rig contract that change the exported motion. */
void check_camera_rig_pose(const ExportCameraTrack &track,
                           VMDCameraExportReport &report,
                           ReportList *reports)
{
  const Object &camera = *track.camera_object;
  if (std::fabs(camera.loc[0]) > kPoseEpsilon || std::fabs(camera.loc[2]) > kPoseEpsilon) {
    add_warning(report,
                reports,
                "The camera has a lateral offset from its VMD target; MMD only stores a distance "
                "along the view axis, so the offset is not exported");
  }
  if (camera.rotmode != ROT_MODE_XYZ || std::fabs(camera.rot[0] - kCameraChildPitch) >
                                            kPoseEpsilon ||
      std::fabs(camera.rot[1]) > kPoseEpsilon || std::fabs(camera.rot[2]) > kPoseEpsilon)
  {
    add_warning(report,
                reports,
                "The camera's local rotation differs from the VMD import pose (90 degrees about "
                "X); the exported orientation will not match the viewport");
  }
  for (const Object *object : {track.target_empty, track.camera_object}) {
    for (int axis = 0; axis < 3; axis++) {
      if (std::fabs(object->scale[axis] - 1.0f) > kPoseEpsilon ||
          std::fabs(object->dloc[axis]) > kPoseEpsilon ||
          std::fabs(object->dscale[axis] - 1.0f) > kPoseEpsilon)
      {
        add_warning(report,
                    reports,
                    std::string("Object '") + (object->id.name + 2) +
                        "' has a non-default scale or delta transform; VMD stores no scale, so "
                        "it is not exported");
        break;
      }
    }
  }
}

/** \} */

}  // namespace

bool export_vmd_action(const Object &armature,
                       const std::string &filepath,
                       const VMDExportOptions &options,
                       ReportList *reports,
                       VMDExportReport &r_report)
{
  r_report = {};
  if (armature.type != OB_ARMATURE || armature.pose == nullptr) {
    add_error(r_report, reports, "VMD export requires an Armature with a Pose");
    return false;
  }
  if (options.frame_end < options.frame_start || options.frame_start < 0) {
    add_error(r_report, reports, "VMD export frame range must be non-negative and ordered");
    return false;
  }
  if (!std::isfinite(options.coordinate_scale) || options.coordinate_scale <= 0.0f) {
    add_error(r_report, reports, "VMD export coordinate scale must be finite and positive");
    return false;
  }
  const AnimData *anim_data = BKE_animdata_from_id(&armature.id);
  if (anim_data == nullptr || anim_data->action == nullptr ||
      anim_data->slot_handle == animrig::Slot::unassigned)
  {
    add_error(r_report, reports, "VMD export requires an active Action and assigned Action slot");
    return false;
  }

  std::map<std::string, ExportBoneTrack> tracks;
  std::string frame_error;
  bke::BKE_action_find_fcurves_with_bones(
      anim_data->action, anim_data->slot_handle, [&](const FCurve *curve, const char *bone_name) {
        if (curve == nullptr || curve->rna_path().is_empty() || bone_name == nullptr) {
          return;
        }
        ExportBoneTrack &track = tracks[bone_name];
        track.name = bone_name;
        const std::string path = curve->rna_path().c_str();
        if (path.ends_with("].location") && curve->array_index >= 0 && curve->array_index < 3) {
          track.location[curve->array_index] = curve;
        }
        else if (path.ends_with("].rotation_quaternion") && curve->array_index >= 0 &&
                 curve->array_index < 4)
        {
          track.rotation[curve->array_index] = curve;
        }
        else {
          r_report.skipped_curve_count++;
          return;
        }
        if (!collect_curve_frames(
                *curve, options.frame_start, options.frame_end, track.frames, frame_error))
        {
          return;
        }
      });
  if (!frame_error.empty()) {
    add_error(r_report, reports, frame_error);
    return false;
  }

  VMDModel model;
  model.header.signature = "Vocaloid Motion Data 0002";
  model.header.model_name = options.model_name.empty() ? armature.id.name + 2 : options.model_name;
  model.header.compatible = true;
  for (auto &[name, track] : tracks) {
    const bool has_location = std::any_of(
        track.location.begin(), track.location.end(), [](const FCurve *curve) { return curve; });
    const bool has_rotation = std::any_of(
        track.rotation.begin(), track.rotation.end(), [](const FCurve *curve) { return curve; });
    if (!has_location && !has_rotation) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, name.c_str());
    if (pchan == nullptr) {
      add_warning(r_report, reports, "Skipping Action curves for missing Pose bone: " + name);
      continue;
    }
    /* Preserve the cropped range boundaries even when all source keys are outside it. */
    track.frames.insert(options.frame_start);
    track.frames.insert(options.frame_end);
    track.converter.compute_from_pose_bone(*pchan, armature);
    const std::vector<int> frames(track.frames.begin(), track.frames.end());
    float previous_vmd_rotation[4] = {};
    bool has_previous_rotation = false;
    for (int frame_index = 0; frame_index < int(frames.size()); frame_index++) {
      const int frame = frames[frame_index];
      float bl_location[3];
      float bl_rotation[4];
      std::copy(std::begin(pchan->loc), std::end(pchan->loc), bl_location);
      copy_qt_qt(bl_rotation, pchan->quat);
      for (int axis = 0; axis < 3; axis++) {
        if (track.location[axis] != nullptr) {
          bl_location[axis] = evaluate_fcurve(track.location[axis], float(frame));
        }
      }
      for (int component = 0; component < 4; component++) {
        if (track.rotation[component] != nullptr) {
          bl_rotation[component] = evaluate_fcurve(track.rotation[component], float(frame));
        }
      }
      if (normalize_qt(bl_rotation) == 0.0f) {
        add_error(r_report, reports, "VMD export encountered a zero quaternion on bone: " + name);
        return false;
      }
      VMDBoneKeyframe key;
      key.bone_name = name;
      key.frame = uint32_t(frame - options.frame_start);
      track.converter.inverse_location(bl_location, key.translation.data(), options.coordinate_scale);
      track.converter.inverse_rotation(bl_rotation, key.rotation.data());
      if (has_previous_rotation) {
        float dot = 0.0f;
        for (int i = 0; i < 4; i++) {
          dot += previous_vmd_rotation[i] * key.rotation[i];
        }
        if (dot < 0.0f) {
          for (float &value : key.rotation) {
            value = -value;
          }
        }
      }
      std::copy(key.rotation.begin(), key.rotation.end(), previous_vmd_rotation);
      has_previous_rotation = true;
      key.interpolation = frame_index > 0 ?
                              interpolation_for_segment(track, frames[frame_index - 1], frame) :
                              linear_interpolation();
      model.bone_keyframes.push_back(std::move(key));
    }
    r_report.bone_count++;
  }
  if (model.bone_keyframes.empty()) {
    add_error(r_report, reports, "VMD export found no supported bone Action keyframes in range");
    return false;
  }
  if (options.morph_controller != nullptr &&
      !collect_morph_tracks(*options.morph_controller, options, r_report, reports, model))
  {
    return false;
  }

  VMDWriteReport write_report;
  if (!write_vmd(filepath, model, &write_report)) {
    for (const std::string &error : write_report.errors) {
      add_error(r_report, reports, error);
    }
    return false;
  }
  for (const std::string &warning : write_report.warnings) {
    add_warning(r_report, reports, warning);
  }
  r_report.success = true;
  r_report.bone_frame_count = int(model.bone_keyframes.size());
  return true;
}

const char *const VMD_CAMERA_MODEL_NAME = kVMDCameraModelName;

bool export_vmd_camera(const Object &camera,
                       const std::string &filepath,
                       const VMDCameraExportOptions &options,
                       ReportList *reports,
                       VMDCameraExportReport &r_report)
{
  r_report = {};
  if (camera.type != OB_CAMERA || camera.data == nullptr) {
    add_error(r_report, reports, "VMD camera export requires a Camera object");
    return false;
  }
  if (options.frame_end < options.frame_start || options.frame_start < 0) {
    add_error(
        r_report, reports, "VMD camera export frame range must be non-negative and ordered");
    return false;
  }
  if (!std::isfinite(options.coordinate_scale) || options.coordinate_scale <= 0.0f) {
    add_error(
        r_report, reports, "VMD camera export coordinate scale must be finite and positive");
    return false;
  }

  ExportCameraTrack track;
  track.camera_object = &camera;
  track.camera_data = reinterpret_cast<const Camera *>(camera.data);

  /* Resolve which of the two supported layouts this Camera is in. Anything else is rejected
   * rather than guessed, because a foreign parent makes the local F-Curves meaningless as the
   * world pose MMD needs. */
  if (camera.parent == nullptr) {
    track.rig_mode = false;
    track.target_empty = nullptr;
  }
  else if (camera.parent->type == OB_EMPTY && camera.parent->parent == nullptr &&
           camera.partype == PAROBJECT)
  {
    track.rig_mode = true;
    track.target_empty = camera.parent;
  }
  else {
    add_error(r_report,
              reports,
              "VMD camera export needs either an unparented Camera or the Empty + Camera rig "
              "created by VMD Camera import; re-parent the Camera or clear its parent");
    return false;
  }
  r_report.used_camera_rig = track.rig_mode;

  if (track.rig_mode) {
    float identity[4][4];
    unit_m4(identity);
    if (!equals_m4m4(camera.parentinv, identity)) {
      add_warning(r_report,
                  reports,
                  "The camera's parent inverse is not identity; VMD stores no such offset, so "
                  "the exported pose will differ from the viewport");
    }
  }
  if (camera.constraints.first != nullptr ||
      (track.target_empty != nullptr && track.target_empty->constraints.first != nullptr))
  {
    add_warning(r_report,
                reports,
                "VMD camera export samples F-Curves, not evaluated constraints; constrained "
                "motion is not included");
  }
  if (track.camera_data->sensor_fit != CAMERA_SENSOR_FIT_VERT) {
    add_warning(r_report,
                reports,
                "The camera sensor fit is not Vertical; the MMD view angle is derived from the "
                "vertical sensor size and may not match the render");
  }

  /* The Empty owns the MMD look-at center and rotation in rig mode; a standalone Camera carries
   * its own world pose instead. */
  const Object &transform_source = track.rig_mode ? *track.target_empty : camera;
  for (int axis = 0; axis < 3; axis++) {
    track.location[axis] = find_id_fcurve(transform_source.id, "location", axis);
  }
  std::string rotation_error;
  if (!track.rotation.resolve(transform_source, rotation_error)) {
    add_error(r_report, reports, rotation_error);
    return false;
  }
  /* Bezier for the single MMD rotation channel is only faithful when the source is the importer's
   * native YXZ Empty; any other layout is recomposed per frame through a matrix. */
  track.rotation_bezier_allowed = track.rig_mode && track.rotation.is_native_yxz;
  if (track.rig_mode) {
    track.distance = find_id_fcurve(camera.id, "location", 1);
    check_camera_rig_pose(track, r_report, reports);
  }
  else if (!track.rotation.is_native_yxz) {
    add_warning(r_report,
                reports,
                "Standalone camera rotation is recomposed into MMD's YXZ channel, so its Bezier "
                "handles are exported as linear; use a VMD camera rig to keep them");
  }
  track.lens = find_id_fcurve(track.camera_data->id, "lens", 0);
  track.camera_type = find_id_fcurve(track.camera_data->id, "type", 0);

  std::string frame_error;
  for (const FCurve *curve : track.location) {
    if (curve != nullptr && !collect_curve_frames(*curve,
                                                  options.frame_start,
                                                  options.frame_end,
                                                  track.frames,
                                                  frame_error))
    {
      add_error(r_report, reports, frame_error);
      return false;
    }
  }
  if (!track.rotation.collect_frames(
          options.frame_start, options.frame_end, track.frames, frame_error))
  {
    add_error(r_report, reports, frame_error);
    return false;
  }
  for (const FCurve *curve : {track.distance, track.lens, track.camera_type}) {
    if (curve != nullptr && !collect_curve_frames(*curve,
                                                  options.frame_start,
                                                  options.frame_end,
                                                  track.frames,
                                                  frame_error))
    {
      add_error(r_report, reports, frame_error);
      return false;
    }
  }
  /* Keep the requested range boundaries so MMD always receives a frame-zero record, matching the
   * bone exporter. A camera with no keys at all still exports its static pose. */
  track.frames.insert(options.frame_start);
  track.frames.insert(options.frame_end);

  /* Undo the importer's +90 degree pitch on the camera child to recover the Empty-equivalent
   * rotation of a standalone camera. Built through Blender's own Euler helper so the matrix
   * convention matches `mul_m3_m3m3()`. */
  float inverse_pitch[3][3];
  const float inverse_pitch_euler[3] = {-kCameraChildPitch, 0.0f, 0.0f};
  eulO_to_mat3(inverse_pitch, inverse_pitch_euler, ROT_MODE_XYZ);

  VMDModel model;
  model.header.signature = "Vocaloid Motion Data 0002";
  model.header.model_name = options.model_name.empty() ? kVMDCameraModelName : options.model_name;
  model.header.compatible = true;

  const std::vector<int> frames(track.frames.begin(), track.frames.end());
  for (int frame_index = 0; frame_index < int(frames.size()); frame_index++) {
    const int frame = frames[frame_index];
    float location[3];
    for (int axis = 0; axis < 3; axis++) {
      location[axis] = sample_curve(
          track.location[axis], frame, transform_source.loc[axis]);
      if (!std::isfinite(location[axis])) {
        add_error(r_report, reports, "VMD camera export sampled a non-finite location");
        return false;
      }
    }
    float yxz[3];
    if (!track.rotation.evaluate_yxz(frame, yxz)) {
      add_error(r_report, reports, "VMD camera export sampled an invalid rotation");
      return false;
    }
    if (!track.rig_mode) {
      /* R_empty = R_camera_world * Rx(-90) recovers the look-at rotation MMD stores. */
      float camera_rotation[3][3];
      float empty_rotation[3][3];
      eulO_to_mat3(camera_rotation, yxz, ROT_MODE_YXZ);
      mul_m3_m3m3(empty_rotation, camera_rotation, inverse_pitch);
      mat3_to_eulO(yxz, ROT_MODE_YXZ, empty_rotation);
    }
    const float distance = track.rig_mode ?
                               sample_curve(track.distance, frame, camera.loc[1]) :
                               0.0f;
    if (!std::isfinite(distance)) {
      add_error(r_report, reports, "VMD camera export sampled a non-finite distance");
      return false;
    }
    const float lens = sample_curve(track.lens, frame, track.camera_data->lens);
    uint32_t view_angle = 0;
    bool clamped = false;
    if (!view_angle_from_lens(*track.camera_data, lens, view_angle, clamped)) {
      add_error(r_report, reports, "VMD camera export sampled an invalid focal length");
      return false;
    }
    if (clamped) {
      r_report.clamped_angle_count++;
    }
    const float type_value = sample_curve(
        track.camera_type, frame, float(track.camera_data->type));

    VMDCameraKeyframe key;
    key.frame = uint32_t(frame - options.frame_start);
    /* Inverse of the importer's MMD (x, y, z) -> Blender (x, z, y) mapping. */
    key.position = {location[0] / options.coordinate_scale,
                    location[2] / options.coordinate_scale,
                    location[1] / options.coordinate_scale};
    key.rotation = {yxz[0], yxz[2], yxz[1]};
    key.distance = distance / options.coordinate_scale;
    key.view_angle = view_angle;
    key.perspective = int(std::round(type_value)) != CAM_ORTHO;
    key.interpolation = frame_index > 0 ? camera_interpolation_for_segment(
                                              track, frames[frame_index - 1], frame) :
                                          linear_camera_interpolation();
    if (frame_index > 0 && key.interpolation != linear_camera_interpolation()) {
      r_report.bezier_segment_count++;
    }
    model.camera_keyframes.push_back(std::move(key));
  }

  if (!track.rig_mode && int(std::round(sample_curve(track.camera_type,
                                                     options.frame_start,
                                                     float(track.camera_data->type)))) == CAM_ORTHO)
  {
    add_warning(r_report,
                reports,
                "An orthographic standalone camera exports with distance zero; MMD derives its "
                "orthographic framing from distance, so use a VMD camera rig for that");
  }

  VMDWriteReport write_report;
  if (!write_vmd(filepath, model, &write_report)) {
    for (const std::string &error : write_report.errors) {
      add_error(r_report, reports, error);
    }
    return false;
  }
  for (const std::string &warning : write_report.warnings) {
    add_warning(r_report, reports, warning);
  }
  if (r_report.clamped_angle_count > 0) {
    add_warning(r_report,
                reports,
                "Focal lengths outside MMD's 1-180 degree view angle were clamped on " +
                    std::to_string(r_report.clamped_angle_count) + " keyframe(s)");
  }
  r_report.success = true;
  r_report.camera_frame_count = int(model.camera_keyframes.size());
  return true;
}

}  // namespace blender::io::vmd
