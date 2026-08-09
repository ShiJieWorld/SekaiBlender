/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors/io
 *
 * F2/F3/F4: real-time MMD physics operators. Provides Start / Step / Stop /
 * Reset operators that drive `mmd_physics::MMDPhysicsWorld` from the
 * Blender UI, plus an N-panel ("MMD Physics" tab) in the View3D sidebar.
 *
 * Start is a modal operator: it runs the F3 startup sequence (bone
 * disconnection → temporal kinematic init → prewarm), arms a `TIMER` that
 * drives elapsed-time steps while paused and frame steps during playback, and
 * stays modal until Stop (or cancel) tears down the world. Stop removes the
 * timer, destroys the world, and pushes a mouse-move event so the modal loop
 * wakes up and exits.
 */

#include "io_mmd_physics_ops.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "DNA_ID.h"
#include "DNA_action_types.h"
#include "DNA_anim_types.h"

#include "mmd_physics_definition.hh"
#include "mmd_physics_diagnostics.hh"
#include "mmd_physics_world.hh"
#include "../../io/pmx/importer/pmx_import_pose_snapshot.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"
#include "ANIM_fcurve.hh"

#include "BKE_action.hh"
#include "BKE_anim_data.hh"
#include "BKE_blender.hh"
#include "BKE_armature.hh"
#include "BKE_collection.hh"
#include "BKE_context.hh"
#include "BKE_fcurve.hh"
#include "BKE_idprop.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_path_utils.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "DEG_depsgraph.hh"

#include "ED_screen.hh"

#include "BLT_translation.hh"

#include "DNA_armature_types.h"
#include "DNA_collection_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "MEM_guardedalloc.h"

#include "RNA_define.hh"
#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_interface_types.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

namespace blender {

namespace {

using mmd_physics::MMDPhysicsDefinition;
using mmd_physics::MMDPhysicsWorld;

bool physics_action_is_baked(const bAction &action);

struct MutedActionCurve {
  uint64_t action_session_uid = 0;
  animrig::slot_handle_t slot_handle = animrig::Slot::unassigned;
  std::string rna_path;
  int array_index = 0;
  bool was_muted = false;
};

struct PhysicsPoseDebugBone {
  std::string bone_name;
  float local_location[3];
  float pose_location[3];
};

struct PhysicsInputPoseBone {
  std::string bone_name;
  float location[3];
  float quaternion[4];
  float scale[3];
  float pose_matrix[4][4];
};

struct MMDPhysicsPlaybackEdgeTrace {
  int remaining_ticks = 0;
  bool has_kinematic_positions = false;
  Vector<std::array<float, 3>> previous_kinematic_positions;
};

struct MMDPhysicsRuntimeProfile {
  int samples = 0;
  int fixed_steps = 0;
  double step_full_ms = 0.0;
  double bullet_ms = 0.0;
  double post_step_ms = 0.0;
  double pose_write_ms = 0.0;
  double depsgraph_ms = 0.0;
  double total_ms = 0.0;
  double max_step_full_ms = 0.0;
  double max_total_ms = 0.0;
};

struct MMDPhysicsModelBinding {
  uint64_t armature_session_uid = 0;
  uint64_t model_collection_session_uid = 0;
  MMDPhysicsDefinition definition;
  Set<std::string> input_bone_names;
  Set<std::string> physics_bone_names;
  Set<std::string> dynamic_bone_names;
  Set<std::string> dynamic_merge_bone_names;
};

struct MMDPhysicsRuntimeSession {
  MMDPhysicsModelBinding binding;
  std::unique_ptr<MMDPhysicsWorld> world;
  Object *armature = nullptr;
  Collection *model_collection = nullptr;
  Main *bmain = nullptr;

  Vector<MutedActionCurve> muted_action_curves;
  Vector<PhysicsPoseDebugBone> start_pose;
  Vector<PhysicsInputPoseBone> input_pose;
  int input_pose_scene_frame = 0;
  MMDPhysicsPlaybackEdgeTrace playback_edge_trace;
  MMDPhysicsRuntimeProfile runtime_profile;

  uint64_t debug_session = 0;
  bool debug_first_writeback_pending = false;
  uint64_t total_fixed_steps = 0;
  double last_timer_elapsed = 0.0;
  int last_fixed_steps = 0;
  bool last_writeback = false;
};

struct MMDPhysicsSceneScheduler {
  Main *bmain = nullptr;
  Scene *scene = nullptr;
  wmWindow *window = nullptr;
  uint64_t scene_session_uid = 0;
  wmTimer *timer = nullptr;
  std::vector<std::unique_ptr<MMDPhysicsRuntimeSession>> sessions;

  int realtime_substeps_per_frame = 5;
  float realtime_fixed_timestep = 1.0f / 60.0f;
  double time_accumulator = 0.0;
  double last_timer_elapsed = 0.0;
  int last_fixed_steps = 0;
  bool last_writeback = false;
  bool have_last_playback_frame = false;
  int last_playback_frame = 0;
  bool have_observed_scene_frame = false;
  int last_observed_scene_frame = 0;
};

std::unique_ptr<MMDPhysicsSceneScheduler> g_physics_scheduler;

struct PhysicsBakeModalData;
PhysicsBakeModalData *g_physics_bake = nullptr;

void cleanup_physics_global_state(void * /*user_data*/)
{
  g_physics_scheduler.reset();
  g_physics_bake = nullptr;
}

struct PhysicsAtExitRegistrar {
  PhysicsAtExitRegistrar()
  {
    BKE_blender_atexit_register(cleanup_physics_global_state, nullptr);
  }
};
PhysicsAtExitRegistrar g_physics_atexit_registrar;

/* Log absolute Z statistics for physics-driven bones at a labeled stage.
 * Unlike log_physics_pose_debug_delta (which compares against a baseline and
 * therefore masks any drop that happened BEFORE baseline capture), this
 * outputs the absolute average/min Z so drops can be located across the
 * Start flow (ensure_world / prepare_world_for_simulation / etc.). */
void log_physics_pose_absolute(const char *stage,
                               Object &armature,
                               const Set<std::string> &physics_bones,
                               const uint64_t debug_session)
{
  if (armature.pose == nullptr) {
    return;
  }
  double total_z = 0.0;
  float min_z = std::numeric_limits<float>::max();
  const char *min_z_bone = "";
  int measured = 0;
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(armature.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    if (!physics_bones.contains(pchan->name)) {
      continue;
    }
    const float z = pchan->pose_mat[3][2];
    total_z += double(z);
    if (z < min_z) {
      min_z = z;
      min_z_bone = pchan->name;
    }
    measured++;
  }
  fprintf(stderr,
          "[MMD Physics AbsZ] session=%llu stage=%s bones=%d avg_z=%.6f "
          "min_z=%.6f min_z_bone='%s'\n",
           static_cast<unsigned long long>(debug_session),
          stage,
          measured,
          measured > 0 ? float(total_z / double(measured)) : 0.0f,
          min_z,
          min_z_bone);
  fflush(stderr);
}

/* Per-tick Z tracker for a single named bone. Unlike the session-level
 * absolute-Z probe, this fires on EVERY Timer writeback so we can see the
 * continuous Z trajectory of one bone (e.g. a known dropping skirt bone)
 * across Start/Stop/Start cycles. The bone name is hardcoded per user
 * request ("左侧发B__18_1"); leave empty to disable. */
static const char *kPhysicsTrackBoneName = "左侧发B__18_1";

void log_tracked_bone_z(const char *stage,
                        Object &armature,
                        const uint64_t debug_session,
                        int tick = -1)
{
  if (armature.pose == nullptr) {
    return;
  }
  bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, kPhysicsTrackBoneName);
  if (pchan == nullptr) {
    return;
  }
  fprintf(stderr,
          "[MMD Physics TrackZ] session=%llu stage=%s tick=%d bone='%s' "
          "pose_z=%.6f local_z=%.6f\n",
           static_cast<unsigned long long>(debug_session),
          stage,
          tick,
          kPhysicsTrackBoneName,
          pchan->pose_mat[3][2],
          pchan->loc[2]);
  fflush(stderr);
}

void capture_physics_pose_debug_baseline(MMDPhysicsRuntimeSession &session, Object &armature)
{
  session.start_pose.clear();
  if (armature.pose == nullptr) {
    return;
  }
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(armature.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    if (!session.binding.physics_bone_names.contains(pchan->name)) {
      continue;
    }
    PhysicsPoseDebugBone state;
    state.bone_name = pchan->name;
    copy_v3_v3(state.local_location, pchan->loc);
    copy_v3_v3(state.pose_location, pchan->pose_mat[3]);
    session.start_pose.append(std::move(state));
  }
}

void log_physics_pose_debug_delta(const char *stage,
                                  const MMDPhysicsRuntimeSession &session,
                                  Object &armature)
{
  if (armature.pose == nullptr || session.start_pose.is_empty()) {
    return;
  }
  float max_pose_distance = 0.0f;
  float max_local_distance = 0.0f;
  float minimum_dz = 0.0f;
  double total_dz = 0.0;
  int measured = 0;
  const char *max_pose_bone = "";
  const char *max_local_bone = "";
  const char *minimum_dz_bone = "";
  for (const PhysicsPoseDebugBone &before : session.start_pose) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, before.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    const float pose_distance = len_v3v3(before.pose_location, pchan->pose_mat[3]);
    const float local_distance = len_v3v3(before.local_location, pchan->loc);
    const float dz = pchan->pose_mat[3][2] - before.pose_location[2];
    if (pose_distance > max_pose_distance) {
      max_pose_distance = pose_distance;
      max_pose_bone = pchan->name;
    }
    if (local_distance > max_local_distance) {
      max_local_distance = local_distance;
      max_local_bone = pchan->name;
    }
    if (dz < minimum_dz) {
      minimum_dz = dz;
      minimum_dz_bone = pchan->name;
    }
    total_dz += dz;
    measured++;
  }
  fprintf(stderr,
          "[MMD Physics Debug] session=%llu stage=%s bones=%d max_pose_delta=%.6f "
          "pose_bone='%s' max_local_delta=%.6f local_bone='%s' avg_dz=%.6f "
          "min_dz=%.6f min_dz_bone='%s'\n",
           static_cast<unsigned long long>(session.debug_session),
          stage,
          measured,
          max_pose_distance,
          max_pose_bone,
          max_local_distance,
          max_local_bone,
          measured > 0 ? float(total_dz / double(measured)) : 0.0f,
          minimum_dz,
          minimum_dz_bone);
  fflush(stderr);
}

int mute_physics_bone_action_curves(MMDPhysicsRuntimeSession &session, Object &armature)
{
  session.muted_action_curves.clear();
  AnimData *anim_data = BKE_animdata_from_id(&armature.id);
  if (anim_data == nullptr || anim_data->action == nullptr) {
    return 0;
  }

  animrig::foreach_fcurve_in_action(anim_data->action->wrap(), [&](FCurve &fcurve) {
    constexpr const char *prefix = "pose.bones[\"";
    if (fcurve.rna_path().is_empty() || !STRPREFIX(fcurve.rna_path().c_str(), prefix)) {
      return;
    }
    const char *name_begin = fcurve.rna_path().c_str() + std::strlen(prefix);
    const char *name_end = std::strstr(name_begin, "\"].");
    if (name_end == nullptr) {
      return;
    }
    const std::string bone_name(name_begin, name_end);
    const char *property = name_end + 3;
    const bool transform_property = STREQ(property, "location") ||
                                    STREQ(property, "rotation_euler") ||
                                    STREQ(property, "rotation_quaternion") ||
                                    STREQ(property, "rotation_axis_angle") ||
                                    STREQ(property, "scale");
    const bool merge_property = STREQ(property, "rotation_euler") ||
                                STREQ(property, "rotation_quaternion") ||
                                STREQ(property, "rotation_axis_angle") ||
                                STREQ(property, "scale");
    if ((!session.binding.dynamic_bone_names.contains(bone_name) || !transform_property) &&
        (!session.binding.dynamic_merge_bone_names.contains(bone_name) || !merge_property))
    {
      return;
    }
    session.muted_action_curves.append({anim_data->action->id.session_uid,
                                        anim_data->slot_handle,
                                        fcurve.rna_path().c_str(),
                                        fcurve.array_index,
                                        (fcurve.flag & FCURVE_MUTED) != 0});
    fcurve.flag |= FCURVE_MUTED;
  });
  return int(session.muted_action_curves.size());
}

void restore_physics_bone_action_curves(MMDPhysicsRuntimeSession &session, Main *bmain)
{
  if (bmain == nullptr) {
    session.muted_action_curves.clear();
    return;
  }
  for (const MutedActionCurve &muted : session.muted_action_curves) {
    bAction *action = reinterpret_cast<bAction *>(
        BKE_libblock_find_session_uid(bmain, ID_AC, muted.action_session_uid));
    if (action == nullptr) {
      continue;
    }
    animrig::Channelbag *channelbag = animrig::channelbag_for_action_slot(action->wrap(),
                                                                          muted.slot_handle);
    FCurve *fcurve = channelbag != nullptr ?
                         channelbag->fcurve_find({muted.rna_path, muted.array_index}) :
                         nullptr;
    if (fcurve == nullptr) {
      continue;
    }
    if (muted.was_muted) {
      fcurve->flag |= FCURVE_MUTED;
    }
    else {
      fcurve->flag &= ~FCURVE_MUTED;
    }
  }
  session.muted_action_curves.clear();
}

int restore_unanimated_physics_bones(Object &armature, const Set<std::string> &physics_bones)
{
  io::pmx::PMXPoseSnapshot snapshot;
  if (armature.pose == nullptr || !io::pmx::read_pose_snapshot(armature.id, snapshot)) {
    return 0;
  }

  Set<std::string> animated_bones;
  AnimData *anim_data = BKE_animdata_from_id(&armature.id);
  if (anim_data != nullptr && anim_data->action != nullptr) {
    animrig::foreach_fcurve_in_action(anim_data->action->wrap(), [&](FCurve &fcurve) {
      constexpr const char *prefix = "pose.bones[\"";
      if (fcurve.rna_path().is_empty() || !STRPREFIX(fcurve.rna_path().c_str(), prefix)) {
        return;
      }
      const char *name_begin = fcurve.rna_path().c_str() + std::strlen(prefix);
      const char *name_end = std::strstr(name_begin, "\"]");
      if (name_end != nullptr) {
        animated_bones.add(std::string(name_begin, name_end));
      }
    });
  }

  int restored = 0;
  for (const io::pmx::PMXPoseBoneSnapshot &bone : snapshot.bones) {
    if (!physics_bones.contains(bone.name) || animated_bones.contains(bone.name)) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, bone.name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    copy_v3_v3(pchan->loc, bone.loc);
    copy_qt_qt(pchan->quat, bone.quat);
    if (pchan->rotmode == ROT_MODE_XYZ || pchan->rotmode == ROT_MODE_XZY ||
        pchan->rotmode == ROT_MODE_YXZ || pchan->rotmode == ROT_MODE_YZX ||
        pchan->rotmode == ROT_MODE_ZXY || pchan->rotmode == ROT_MODE_ZYX)
    {
      quat_to_eulO(pchan->eul, pchan->rotmode, pchan->quat);
    }
    else if (pchan->rotmode == ROT_MODE_AXISANGLE) {
      quat_to_axis_angle(pchan->rotAxis, &pchan->rotAngle, pchan->quat);
    }
    restored++;
  }
  return restored;
}

void capture_physics_input_pose(MMDPhysicsRuntimeSession &session,
                                Object &armature,
                                const int scene_frame)
{
  session.input_pose.clear();
  session.input_pose_scene_frame = scene_frame;
  if (armature.pose == nullptr) {
    return;
  }
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(armature.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    if (!session.binding.input_bone_names.contains(pchan->name)) {
      continue;
    }
    PhysicsInputPoseBone state;
    state.bone_name = pchan->name;
    copy_v3_v3(state.location, pchan->loc);
    copy_qt_qt(state.quaternion, pchan->quat);
    copy_v3_v3(state.scale, pchan->scale);
    copy_m4_m4(state.pose_matrix, pchan->pose_mat);
    session.input_pose.append(std::move(state));
  }
}

int restore_physics_input_pose(const Vector<PhysicsInputPoseBone> &input_pose, Object &armature)
{
  if (armature.pose == nullptr) {
    return 0;
  }
  int restored = 0;
  for (const PhysicsInputPoseBone &state : input_pose) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, state.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    copy_v3_v3(pchan->loc, state.location);
    copy_qt_qt(pchan->quat, state.quaternion);
    copy_v3_v3(pchan->scale, state.scale);
    copy_m4_m4(pchan->pose_mat, state.pose_matrix);
    if (ELEM(pchan->rotmode,
             ROT_MODE_XYZ,
             ROT_MODE_XZY,
             ROT_MODE_YXZ,
             ROT_MODE_YZX,
             ROT_MODE_ZXY,
             ROT_MODE_ZYX))
    {
      quat_to_eulO(pchan->eul, pchan->rotmode, pchan->quat);
    }
    else if (pchan->rotmode == ROT_MODE_AXISANGLE) {
      quat_to_axis_angle(pchan->rotAxis, &pchan->rotAngle, pchan->quat);
    }
    restored++;
  }
  return restored;
}

void append_compare_trace(MMDPhysicsRuntimeSession &session,
                          const char *path_env,
                          const int scene_frame,
                          const double timer_elapsed,
                          const double accumulator,
                          const int fixed_steps,
                          const uint64_t total_fixed_steps)
{
  const char *path = std::getenv(path_env);
  if (path == nullptr || path[0] == '\0') {
    return;
  }

  MMDPhysicsWorld &world = *session.world;
  mmd_physics::MMDDiagnosticFrame frame;
  if (!world.capture_diagnostic_frame(scene_frame, frame)) {
    return;
  }
  frame.runtime_timer_elapsed = timer_elapsed;
  frame.runtime_accumulator = accumulator;
  frame.runtime_fixed_steps = fixed_steps;
  frame.runtime_total_fixed_steps = total_fixed_steps;
  frame.runtime_writeback = true;
  if (!mmd_physics::append_mmd_physics_diagnostic_frame(path, frame)) {
    fprintf(stderr, "[MMD Physics] Failed to append compare trace: %s\n", path);
    fflush(stderr);
  }
}

void trace_playback_edge(MMDPhysicsRuntimeSession &session,
                         const char *phase,
                         const int scene_frame,
                         const int consumed_steps,
                         const double accumulator)
{
  if (std::getenv("MMD_RT_TRACE_PLAYBACK_EDGE") == nullptr) {
    return;
  }

  MMDPhysicsWorld &world = *session.world;
  mmd_physics::MMDDiagnosticFrame frame;
  if (!world.capture_diagnostic_frame(int(session.total_fixed_steps), frame)) {
    return;
  }

  float max_linear_speed = 0.0f;
  float max_angular_speed = 0.0f;
  float max_kinematic_delta = 0.0f;
  int max_angular_body_index = -1;
  const char *max_linear_body = "";
  const char *max_angular_body = "";
  const char *max_kinematic_body = "";
  for (const mmd_physics::MMDDiagnosticBodySample &body : frame.bodies) {
    const float linear_speed = std::sqrt(body.linear_velocity[0] * body.linear_velocity[0] +
                                         body.linear_velocity[1] * body.linear_velocity[1] +
                                         body.linear_velocity[2] * body.linear_velocity[2]);
    const float angular_speed = std::sqrt(body.angular_velocity[0] * body.angular_velocity[0] +
                                          body.angular_velocity[1] * body.angular_velocity[1] +
                                          body.angular_velocity[2] * body.angular_velocity[2]);
    if (body.physics_type != 0) {
      if (linear_speed > max_linear_speed) {
        max_linear_speed = linear_speed;
        max_linear_body = body.bone_name.c_str();
      }
      if (angular_speed > max_angular_speed) {
        max_angular_speed = angular_speed;
        max_angular_body = body.bone_name.c_str();
        max_angular_body_index = body.runtime_index;
      }
    }
    else if (body.runtime_index >= 0) {
      const int index = body.runtime_index;
      if (index >= int(session.playback_edge_trace.previous_kinematic_positions.size())) {
        session.playback_edge_trace.previous_kinematic_positions.resize(index + 1);
      }
      auto &previous = session.playback_edge_trace.previous_kinematic_positions[index];
      const float dx = body.position[0] - previous[0];
      const float dy = body.position[1] - previous[1];
      const float dz = body.position[2] - previous[2];
      if (session.playback_edge_trace.has_kinematic_positions) {
        const float delta = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (delta > max_kinematic_delta) {
          max_kinematic_delta = delta;
          max_kinematic_body = body.bone_name.c_str();
        }
      }
      previous = body.position;
    }
  }
  session.playback_edge_trace.has_kinematic_positions = true;

  float max_spring_target_velocity = 0.0f;
  float max_limit_error = 0.0f;
  float max_joint_impulse = 0.0f;
  float max_contact_impulse = 0.0f;
  float max_contact_distance = 0.0f;
  int max_contact_body_a = -1;
  int max_contact_body_b = -1;
  const char *max_spring_joint = "";
  const char *max_limit_joint = "";
  const char *max_impulse_joint = "";
  for (const mmd_physics::MMDDiagnosticJointSample &joint : frame.joints) {
    if (joint.rigid_a != max_angular_body_index && joint.rigid_b != max_angular_body_index) {
      continue;
    }
    for (int axis = 0; axis < 3; axis++) {
      const float target_velocity = std::fabs(joint.angular_target_velocity[axis]);
      const float limit_error = std::fabs(joint.angular_current_limit_error[axis]);
      if (target_velocity > max_spring_target_velocity) {
        max_spring_target_velocity = target_velocity;
        max_spring_joint = joint.name_local.c_str();
      }
      if (limit_error > max_limit_error) {
        max_limit_error = limit_error;
        max_limit_joint = joint.name_local.c_str();
      }
    }
    const float impulse = std::fabs(joint.applied_impulse);
    if (impulse > max_joint_impulse) {
      max_joint_impulse = impulse;
      max_impulse_joint = joint.name_local.c_str();
    }
  }
  for (const mmd_physics::MMDDiagnosticContactSample &contact : frame.contacts) {
    if (contact.body_a == max_angular_body_index || contact.body_b == max_angular_body_index) {
      const float impulse = std::fabs(contact.applied_impulse);
      if (impulse > max_contact_impulse) {
        max_contact_impulse = impulse;
        max_contact_distance = contact.distance;
        max_contact_body_a = contact.body_a;
        max_contact_body_b = contact.body_b;
      }
    }
  }
  const auto body_name_from_index = [&frame](const int runtime_index) {
    for (const mmd_physics::MMDDiagnosticBodySample &body : frame.bodies) {
      if (body.runtime_index == runtime_index) {
        return body.bone_name.c_str();
      }
    }
    return "";
  };
  const auto body_from_index = [&frame](const int runtime_index) {
    for (const mmd_physics::MMDDiagnosticBodySample &body : frame.bodies) {
      if (body.runtime_index == runtime_index) {
        return &body;
      }
    }
    return static_cast<const mmd_physics::MMDDiagnosticBodySample *>(nullptr);
  };
  const auto body_is_static = [&body_from_index](const int runtime_index) {
    const mmd_physics::MMDDiagnosticBodySample *body = body_from_index(runtime_index);
    return body != nullptr && body->physics_type == 0;
  };
  const auto body_is_dynamic = [&body_from_index](const int runtime_index) {
    const mmd_physics::MMDDiagnosticBodySample *body = body_from_index(runtime_index);
    return body != nullptr && body->physics_type != 0;
  };
  const auto dynamic_jointed_to_static = [&](const int dynamic_body) {
    for (const mmd_physics::MMDDiagnosticJointSample &joint : frame.joints) {
      if (joint.rigid_a == dynamic_body && body_is_static(joint.rigid_b)) {
        return true;
      }
      if (joint.rigid_b == dynamic_body && body_is_static(joint.rigid_a)) {
        return true;
      }
    }
    return false;
  };
  const auto max_limit_error_for_body = [&frame](const int body_index) {
    float max_error = 0.0f;
    for (const mmd_physics::MMDDiagnosticJointSample &joint : frame.joints) {
      if (joint.rigid_a != body_index && joint.rigid_b != body_index) {
        continue;
      }
      for (int axis = 0; axis < 3; axis++) {
        max_error = std::max(max_error, std::fabs(joint.angular_current_limit_error[axis]));
      }
    }
    return max_error;
  };
  struct StaticDynamicContactTop {
    float impulse = 0.0f;
    float distance = 0.0f;
    float dynamic_limit_error = 0.0f;
    int static_body = -1;
    int dynamic_body = -1;
    bool dynamic_root = false;
  };
  std::array<StaticDynamicContactTop, 5> top_static_dynamic_contacts;
  for (const mmd_physics::MMDDiagnosticContactSample &contact : frame.contacts) {
    int static_body = -1;
    int dynamic_body = -1;
    if (body_is_static(contact.body_a) && body_is_dynamic(contact.body_b)) {
      static_body = contact.body_a;
      dynamic_body = contact.body_b;
    }
    else if (body_is_static(contact.body_b) && body_is_dynamic(contact.body_a)) {
      static_body = contact.body_b;
      dynamic_body = contact.body_a;
    }
    else {
      continue;
    }

    StaticDynamicContactTop sample;
    sample.impulse = std::fabs(contact.applied_impulse);
    sample.distance = contact.distance;
    sample.static_body = static_body;
    sample.dynamic_body = dynamic_body;
    sample.dynamic_root = dynamic_jointed_to_static(dynamic_body);
    sample.dynamic_limit_error = max_limit_error_for_body(dynamic_body);
    for (StaticDynamicContactTop &slot : top_static_dynamic_contacts) {
      if (sample.impulse <= slot.impulse) {
        continue;
      }
      std::swap(sample, slot);
    }
  }
  fprintf(stderr,
          "[MMD Playback Edge] phase=%s scene_frame=%d fixed_total=%llu "
          "consumed_steps=%d accumulator=%.6f max_dynamic_linear=%.6f body='%s' "
          "max_dynamic_angular=%.6f body='%s' max_kinematic_delta=%.6f body='%s' "
          "spring_target=%.6f joint='%s' limit_error=%.6f joint='%s' "
          "joint_impulse=%.6f joint='%s' contact_impulse=%.6f pair='%s'<->'%s' "
          "contact_distance=%.6f\n",
          phase,
          scene_frame,
           static_cast<unsigned long long>(session.total_fixed_steps),
          consumed_steps,
          accumulator,
          max_linear_speed,
          max_linear_body,
          max_angular_speed,
          max_angular_body,
          max_kinematic_delta,
          max_kinematic_body,
          max_spring_target_velocity,
          max_spring_joint,
          max_limit_error,
          max_limit_joint,
          max_joint_impulse,
          max_impulse_joint,
          max_contact_impulse,
          body_name_from_index(max_contact_body_a),
          body_name_from_index(max_contact_body_b),
          max_contact_distance);
  for (int i = 0; i < int(top_static_dynamic_contacts.size()); i++) {
    const StaticDynamicContactTop &contact = top_static_dynamic_contacts[i];
    if (contact.impulse <= 0.0f) {
      continue;
    }
    fprintf(stderr,
            "[MMD Playback ContactClass] phase=%s rank=%d impulse=%.6f "
            "distance=%.6f static='%s' dynamic='%s' dynamic_root=%s "
            "dynamic_limit_error=%.6f\n",
            phase,
            i + 1,
            contact.impulse,
            contact.distance,
            body_name_from_index(contact.static_body),
            body_name_from_index(contact.dynamic_body),
            contact.dynamic_root ? "true" : "false",
            contact.dynamic_limit_error);
  }
  fflush(stderr);
}

constexpr float kDefaultGravity[3] = {0.0f, 0.0f, -9.81f};
constexpr int kDefaultSolverIterations = 20;
constexpr int kDefaultDynamicConstraintIterations = 80;
constexpr int kMaxRealtimeDynamicConstraintIterations = 128;
constexpr int kDefaultFixedStepHz = 60;
constexpr int kDefaultMaxSubsteps = 8;
constexpr float kDefaultTimerInterval = 1.0f / 30.0f;
constexpr float kDefaultFixedTimestep = 1.0f / float(kDefaultFixedStepHz);
constexpr int kNativeBridgeFixedSteps = 3;
constexpr int kDefaultRealtimeSubstepsPerFrame = 5;
constexpr int kRealtimeSeekStopThresholdFrames = 12;
constexpr const char *kRealtimeSubstepsProperty = "mmd_physics_realtime_substeps_per_frame";
constexpr const char *kDynamicConstraintIterationsProperty =
    "mmd_physics_dynamic_constraint_iterations";
constexpr const char *kLegacyRealtimeFixedStepHzProperty =
    "mmd_physics_realtime_fixed_step_hz";
constexpr int kDefaultBakeSubstepsPerFrame = 5;
constexpr const char *kBakeSubstepsProperty = "mmd_physics_bake_substeps_per_frame";
constexpr const char *kBakeProvenanceProperty = "mmd_physics_bake";
constexpr const char *kPanelLanguageProperty = "mmd_physics_panel_language";

enum class MMDPhysicsPanelLanguage : int {
  Chinese = 0,
  English = 1,
  Japanese = 2,
};

static const EnumPropertyItem mmd_physics_panel_language_items[] = {
    {int(MMDPhysicsPanelLanguage::Chinese), "CHINESE", 0, "中文", "使用中文界面"},
    {int(MMDPhysicsPanelLanguage::English), "ENGLISH", 0, "English", "Use the English interface"},
    {int(MMDPhysicsPanelLanguage::Japanese), "JAPANESE", 0, "日本語", "日本語のインターフェースを使用"},
    {0, nullptr, 0, nullptr, nullptr},
};

struct MMDPhysicsPanelText {
  const char *language;
  const char *realtime_preview;
  const char *models;
  const char *status_running;
  const char *status_stopped;
  const char *status_baked;
  const char *fixed_step_format;
  const char *dynamic_constraint_iterations;
  const char *start;
  const char *stop;
  const char *use_source;
  const char *step;
  const char *reset;
  const char *action_bake;
  const char *simulating_frames;
  const char *writing_action;
  const char *cancel_bake;
  const char *range_start;
  const char *range_end;
  const char *simulation_quality;
  const char *steps_format;
  const char *quality_hint_format;
  const char *bake_action;
};

const MMDPhysicsPanelText &mmd_physics_panel_text(const MMDPhysicsPanelLanguage language)
{
  static const MMDPhysicsPanelText chinese = {
      "语言", "实时预览", "模型", "状态：运行中", "状态：已停止", "状态：已烘焙", "场景帧率：%.2f FPS",
      "动态约束迭代",
      "开始", "停止", "切回源动作", "单步", "重置", "动作烘焙", "正在模拟帧", "正在写入动作",
      "取消烘焙", "起始帧", "结束帧", "模拟质量", "%d 子步",
      "当前：%d 子步/帧（%.2f Hz）", "烘焙物理动作"};
  static const MMDPhysicsPanelText english = {
      "Language", "Realtime Preview", "Models", "Status: Running", "Status: Stopped", "Status: Baked",
      "Scene Rate: %.2f FPS", "Dynamic Constraint Iterations", "Start", "Stop", "Use Source Action", "Step", "Reset", "Action Bake",
      "Simulating Frames", "Writing Action", "Cancel Bake", "Start Frame", "End Frame",
      "Simulation Quality", "%d Steps",
      "Current: %d steps/frame (%.2f Hz)",
      "Bake Physics Action"};
  static const MMDPhysicsPanelText japanese = {
      "言語", "リアルタイムプレビュー", "モデル", "状態：実行中", "状態：停止", "状態：ベイク済み", "シーン：%.2f FPS",
      "動的制約反復",
      "開始", "停止", "ソースアクションに戻す", "1ステップ", "リセット", "アクションベイク", "フレームをシミュレーション中",
      "アクションを書き込み中", "ベイクをキャンセル", "開始フレーム", "終了フレーム", "シミュレーション品質",
      "%d サブステップ", "現在：%d サブステップ/フレーム（%.2f Hz）",
      "物理アクションをベイク"};
  switch (language) {
    case MMDPhysicsPanelLanguage::English:
      return english;
    case MMDPhysicsPanelLanguage::Japanese:
      return japanese;
    case MMDPhysicsPanelLanguage::Chinese:
    default:
      return chinese;
  }
}

int clamp_realtime_substeps_per_frame(const int substeps)
{
  return std::clamp(substeps, 1, 10);
}

double scene_seconds_per_frame(const Scene *scene)
{
  if (scene == nullptr) {
    return 1.0 / 24.0;
  }
  return double(scene->r.frs_sec_base) / double(std::max(1, int(scene->r.frs_sec)));
}

int scene_realtime_substeps_per_frame(Scene *scene)
{
  if (scene == nullptr) {
    return kDefaultRealtimeSubstepsPerFrame;
  }
  IDProperty *props = IDP_GetProperties(&scene->id);
  if (props == nullptr) {
    return kDefaultRealtimeSubstepsPerFrame;
  }
  if (IDProperty *prop = IDP_GetPropertyTypeFromGroup(
          props, kRealtimeSubstepsProperty, IDP_INT))
  {
    return clamp_realtime_substeps_per_frame(IDP_int_get(prop));
  }
  if (IDProperty *legacy_prop = IDP_GetPropertyTypeFromGroup(
          props, kLegacyRealtimeFixedStepHzProperty, IDP_INT))
  {
    return clamp_realtime_substeps_per_frame(
        int(std::round(double(IDP_int_get(legacy_prop)) / 24.0)));
  }
  return kDefaultRealtimeSubstepsPerFrame;
}

void scene_realtime_substeps_per_frame_set(Scene *scene, const int substeps)
{
  if (scene == nullptr) {
    return;
  }
  IDProperty *props = IDP_EnsureProperties(&scene->id);
  const int clamped_substeps = clamp_realtime_substeps_per_frame(substeps);
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(
      props, kRealtimeSubstepsProperty, IDP_INT);
  if (prop != nullptr) {
    IDP_int_set(prop, clamped_substeps);
  }
  else {
    IDP_AddToGroup(props, IDP_NewInt(clamped_substeps, kRealtimeSubstepsProperty));
  }
}

int clamp_realtime_dynamic_constraint_iterations(const int iterations)
{
  return std::clamp(iterations, 1, kMaxRealtimeDynamicConstraintIterations);
}

int scene_realtime_dynamic_constraint_iterations(Scene *scene)
{
  if (scene != nullptr) {
    IDProperty *props = IDP_GetProperties(&scene->id);
    if (props != nullptr) {
      if (IDProperty *prop = IDP_GetPropertyTypeFromGroup(
              props, kDynamicConstraintIterationsProperty, IDP_INT))
      {
        return clamp_realtime_dynamic_constraint_iterations(IDP_int_get(prop));
      }
    }
  }
  return kDefaultDynamicConstraintIterations;
}

void scene_realtime_dynamic_constraint_iterations_set(Scene *scene, const int iterations)
{
  if (scene == nullptr) {
    return;
  }
  IDProperty *props = IDP_EnsureProperties(&scene->id);
  const int clamped_iterations = clamp_realtime_dynamic_constraint_iterations(iterations);
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(
      props, kDynamicConstraintIterationsProperty, IDP_INT);
  if (prop != nullptr) {
    IDP_int_set(prop, clamped_iterations);
  }
  else {
    IDP_AddToGroup(props,
                   IDP_NewInt(clamped_iterations, kDynamicConstraintIterationsProperty));
  }
}

int scene_bake_substeps_per_frame(Scene *scene)
{
  if (scene == nullptr) {
    return kDefaultBakeSubstepsPerFrame;
  }
  IDProperty *props = IDP_GetProperties(&scene->id);
  if (props != nullptr) {
    if (IDProperty *prop = IDP_GetPropertyTypeFromGroup(props, kBakeSubstepsProperty, IDP_INT)) {
      return clamp_realtime_substeps_per_frame(IDP_int_get(prop));
    }
  }
  return kDefaultBakeSubstepsPerFrame;
}

void scene_bake_substeps_per_frame_set(Scene *scene, const int substeps)
{
  if (scene == nullptr) {
    return;
  }
  IDProperty *props = IDP_EnsureProperties(&scene->id);
  const int clamped_substeps = clamp_realtime_substeps_per_frame(substeps);
  IDProperty *prop = IDP_GetPropertyTypeFromGroup(props, kBakeSubstepsProperty, IDP_INT);
  if (prop != nullptr) {
    IDP_int_set(prop, clamped_substeps);
  }
  else {
    IDP_AddToGroup(props, IDP_NewInt(clamped_substeps, kBakeSubstepsProperty));
  }
}

MMDPhysicsPanelLanguage scene_mmd_physics_panel_language(Scene *scene)
{
  if (scene != nullptr) {
    IDProperty *props = IDP_GetProperties(&scene->id);
    if (props != nullptr) {
      if (IDProperty *prop = IDP_GetPropertyTypeFromGroup(props, kPanelLanguageProperty, IDP_INT)) {
        return MMDPhysicsPanelLanguage(std::clamp(IDP_int_get(prop), 0, 2));
      }
    }
  }
  return MMDPhysicsPanelLanguage::Chinese;
}

void scene_mmd_physics_panel_language_set(Scene *scene, const MMDPhysicsPanelLanguage language)
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

wmOperatorStatus physics_set_panel_language_exec(bContext *C, wmOperator *op)
{
  scene_mmd_physics_panel_language_set(
      CTX_data_scene(C), MMDPhysicsPanelLanguage(RNA_enum_get(op->ptr, "language")));
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_set_bake_quality_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  Scene *scene = CTX_data_scene(C);
  scene_bake_substeps_per_frame_set(scene, RNA_int_get(op->ptr, "substeps_per_frame"));
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: bake quality set to %d substeps per frame (%.2f Hz)",
              scene_bake_substeps_per_frame(scene),
              1.0 / (scene_seconds_per_frame(scene) / double(scene_bake_substeps_per_frame(scene))));
  return OPERATOR_FINISHED;
}

void configure_realtime_timing(MMDPhysicsSceneScheduler &scheduler, Scene *scene)
{
  scheduler.realtime_substeps_per_frame = scene_realtime_substeps_per_frame(scene);
  scheduler.realtime_fixed_timestep = kDefaultFixedTimestep;
}

wmOperatorStatus physics_set_realtime_hz_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: stop real-time physics before changing Hz");
    return OPERATOR_CANCELLED;
  }
  Scene *scene = CTX_data_scene(C);
  const int substeps = RNA_int_get(op->ptr, "substeps_per_frame");
  scene_realtime_substeps_per_frame_set(scene, substeps);
  const int configured_substeps = scene_realtime_substeps_per_frame(scene);
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: realtime preview set to %d substeps per frame (%.2f Hz)",
              configured_substeps,
              1.0 / (scene_seconds_per_frame(scene) / double(configured_substeps)));
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_set_dynamic_constraint_iterations_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics: stop real-time physics before changing constraint iterations");
    return OPERATOR_CANCELLED;
  }

  Scene *scene = CTX_data_scene(C);
  scene_realtime_dynamic_constraint_iterations_set(
      scene, RNA_int_get(op->ptr, "iterations"));
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: realtime dynamic constraint iterations set to %d",
              scene_realtime_dynamic_constraint_iterations(scene));
  return OPERATOR_FINISHED;
}

int runtime_fixed_step_hz(const MMDPhysicsSceneScheduler *scheduler = nullptr)
{
  if (const char *env = std::getenv("MMD_RT_FIXED_STEP_HZ")) {
    return std::clamp(std::atoi(env), 30, 480);
  }
  if (scheduler != nullptr && scheduler->realtime_fixed_timestep > 0.0f) {
    return int(std::round(1.0f / scheduler->realtime_fixed_timestep));
  }
  return kDefaultFixedStepHz;
}

float runtime_fixed_timestep(const MMDPhysicsSceneScheduler *scheduler = nullptr)
{
  if (const char *env = std::getenv("MMD_RT_FIXED_STEP_HZ")) {
    return 1.0f / float(std::clamp(std::atoi(env), 30, 480));
  }
  return scheduler != nullptr ? scheduler->realtime_fixed_timestep : kDefaultFixedTimestep;
}

int runtime_fixed_steps_per_frame(const Scene *scene,
                                  const MMDPhysicsSceneScheduler *scheduler = nullptr)
{
  if (std::getenv("MMD_RT_FIXED_STEP_HZ") != nullptr) {
    return std::max(
        1,
        int(std::round(scene_seconds_per_frame(scene) /
                       double(runtime_fixed_timestep(scheduler)))));
  }
  return scheduler != nullptr ? scheduler->realtime_substeps_per_frame :
                                 scene_realtime_substeps_per_frame(const_cast<Scene *>(scene));
}

int runtime_dynamic_constraint_iterations(Scene *scene)
{
  if (const char *env = std::getenv("MMD_RT_DYNAMIC_CONSTRAINT_ITERATIONS")) {
    return std::clamp(std::atoi(env), 1, 500);
  }
  return scene_realtime_dynamic_constraint_iterations(scene);
}
constexpr int kDefaultPrewarmSteps = 2;

/* Find the model-root Collection that owns `armature` and carries a
 * persisted `mmd_physics_definition` IDProperty. Returns nullptr if none.
 *
 * Note: the physics definition is persisted via
 * `IDP_ID_system_properties_ensure` (i.e. into `id.system_properties`,
 * the "system" sub-group of the ID's root properties), so we must read
 * it from there too -- `IDP_GetProperties` would miss it. */
Collection *find_physics_collection_for_armature(Main *bmain,
                                                  Object *armature,
                                                  bool &r_ambiguous)
{
  r_ambiguous = false;
  if (bmain == nullptr || armature == nullptr) {
    return nullptr;
  }

  Collection *direct_match = nullptr;
  Collection *recursive_match = nullptr;
  bool recursive_ambiguous = false;
  for (Collection &collection_ref : bmain->collections) {
    Collection *collection = &collection_ref;
    IDProperty *system = collection->id.system_properties;
    if (system == nullptr ||
        IDP_GetPropertyTypeFromGroup(system, "mmd_physics_definition", IDP_GROUP) == nullptr ||
        !BKE_collection_has_object_recursive(collection, armature))
    {
      continue;
    }

    if (BKE_collection_has_object(collection, armature)) {
      if (direct_match != nullptr) {
        r_ambiguous = true;
        return nullptr;
      }
      direct_match = collection;
    }
    if (recursive_match != nullptr) {
      recursive_ambiguous = true;
    }
    else {
      recursive_match = collection;
    }
  }
  if (direct_match != nullptr) {
    return direct_match;
  }
  if (recursive_ambiguous) {
    r_ambiguous = true;
    return nullptr;
  }
  return recursive_match;
}

MMDPhysicsRuntimeSession *find_runtime_session(MMDPhysicsSceneScheduler &scheduler,
                                               const uint64_t armature_session_uid)
{
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    if (session->binding.armature_session_uid == armature_session_uid) {
      return session.get();
    }
  }
  return nullptr;
}

MMDPhysicsRuntimeSession *runtime_session_for_armature(bContext *C, Object *armature)
{
  if (g_physics_scheduler == nullptr || C == nullptr || armature == nullptr ||
      armature->type != OB_ARMATURE)
  {
    return nullptr;
  }
  Scene *scene = CTX_data_scene(C);
  if (scene == nullptr || scene->id.session_uid != g_physics_scheduler->scene_session_uid) {
    return nullptr;
  }
  return find_runtime_session(*g_physics_scheduler, armature->id.session_uid);
}

MMDPhysicsRuntimeSession *active_runtime_session(bContext *C)
{
  return runtime_session_for_armature(C, CTX_data_active_object(C));
}

Object *resolve_operator_armature(bContext *C, wmOperator *op, ReportList *reports)
{
  char armature_name[MAX_ID_NAME] = {};
  RNA_string_get(op->ptr, "armature_name", armature_name);
  if (armature_name[0] == '\0') {
    return CTX_data_active_object(C);
  }

  Object *armature = reinterpret_cast<Object *>(
      BKE_libblock_find_name(CTX_data_main(C), ID_OB, armature_name));
  if (armature == nullptr || armature->type != OB_ARMATURE) {
    BKE_reportf(reports, RPT_ERROR, "MMD Physics: armature '%s' was not found", armature_name);
    return nullptr;
  }
  if (BKE_scene_object_find_by_name(*CTX_data_main(C), CTX_data_scene(C), armature_name) !=
      armature)
  {
    BKE_reportf(reports,
                RPT_ERROR,
                "MMD Physics: armature '%s' is not part of the active Scene",
                armature_name);
    return nullptr;
  }
  return armature;
}

bool refresh_runtime_session_binding(bContext *C, MMDPhysicsRuntimeSession &session)
{
  Main *bmain = CTX_data_main(C);
  if (bmain == nullptr || session.world == nullptr || session.bmain != bmain) {
    return false;
  }
  Object *armature = reinterpret_cast<Object *>(BKE_libblock_find_session_uid(
      bmain, ID_OB, session.binding.armature_session_uid));
  if (armature == nullptr || armature->type != OB_ARMATURE ||
      !session.world->is_binding_valid(bmain))
  {
    return false;
  }
  session.armature = armature;
  return true;
}

bool scheduler_bindings_are_valid(bContext *C)
{
  if (g_physics_scheduler == nullptr) {
    return false;
  }
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session :
       g_physics_scheduler->sessions)
  {
    if (!refresh_runtime_session_binding(C, *session)) {
      return false;
    }
  }
  return true;
}

bool running_world_binding_is_valid(bContext *C)
{
  MMDPhysicsRuntimeSession *session = active_runtime_session(C);
  return session != nullptr && refresh_runtime_session_binding(C, *session);
}

void remove_timer(bContext *C);
void destroy_world(Main *current_bmain);

bool load_model_binding(Main *bmain,
                        Object &armature,
                        MMDPhysicsModelBinding &r_binding,
                        Collection *&r_model_collection,
                        ReportList *reports)
{
  bool ambiguous_collection = false;
  Collection *model_collection = find_physics_collection_for_armature(
      bmain, &armature, ambiguous_collection);
  if (model_collection == nullptr) {
    BKE_report(reports,
               RPT_ERROR,
               ambiguous_collection ?
                   "MMD Physics: armature belongs to multiple collections with physics data" :
                   "MMD Physics: no PMX physics definition found for this armature's collection");
    return false;
  }

  MMDPhysicsDefinition definition;
  if (!mmd_physics::deserialize_physics_definition(*model_collection, definition, reports)) {
    BKE_report(reports, RPT_ERROR, "MMD Physics: failed to deserialize physics definition");
    return false;
  }

  r_binding = {};
  r_binding.armature_session_uid = armature.id.session_uid;
  r_binding.model_collection_session_uid = model_collection->id.session_uid;
  r_binding.definition = std::move(definition);
  for (const mmd_physics::MMDRigidBodyDefinition &body : r_binding.definition.rigid_bodies) {
    if (body.blender_bone_name.empty()) {
      continue;
    }
    r_binding.input_bone_names.add(body.blender_bone_name);
    if (body.physics_type == 0) {
      continue;
    }
    r_binding.physics_bone_names.add(body.blender_bone_name);
    if (body.physics_type == 1) {
      r_binding.dynamic_bone_names.add(body.blender_bone_name);
    }
    else if (body.physics_type == 2) {
      r_binding.dynamic_merge_bone_names.add(body.blender_bone_name);
    }
  }
  r_model_collection = model_collection;
  return true;
}

std::unique_ptr<MMDPhysicsRuntimeSession> create_runtime_session(
    bContext *C,
    Object &armature,
    const int fixed_step_hz,
    ReportList *reports,
    const Vector<PhysicsInputPoseBone> *input_pose_override = nullptr,
    const int dynamic_constraint_iterations = -1)
{
  Main *bmain = CTX_data_main(C);
  auto session = std::make_unique<MMDPhysicsRuntimeSession>();
  session->armature = &armature;
  session->bmain = bmain;
  if (!load_model_binding(
          bmain, armature, session->binding, session->model_collection, reports))
  {
    return nullptr;
  }

  /* Pose Mode edits are intentional user input even when they are not keyed.
   * Do not replace them with the import-time PMX snapshot before building the
   * new world. Object-mode restarts keep the legacy stale-pose cleanup. */
  const int restored_physics_bones = (armature.mode & OB_MODE_POSE) == 0 ?
                                         restore_unanimated_physics_bones(
                                             armature, session->binding.physics_bone_names) :
                                         0;
  if (restored_physics_bones > 0) {
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
    if (depsgraph != nullptr) {
      DEG_id_tag_update(&armature.id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      BKE_scene_graph_update_for_newframe(depsgraph);
    }
  }

  /* Frame evaluation can reconstruct constrained physics bones from a
   * physics-written dependency state. At a playback edge, restore the exact
   * evaluated matrices that initialized this session before the new world
   * reads pose_mat. */
  if (input_pose_override != nullptr) {
    restore_physics_input_pose(*input_pose_override, armature);
  }

  auto world = std::make_unique<MMDPhysicsWorld>();
  /* joint_spring_damping = 0.15 aligns with the Capture Diagnostics RNA
   * default (0.15) and the damping sweep conclusion (2026-07-30): damping
   * must stay 0.15. Higher values (0.5, 0.85) amplify the spring motor's
   * targetVelocity (Bullet 2.82 internalUpdateSprings:
   *   velFactor = fps * damping / numIterations;
   *   targetVelocity = velFactor * (-delta * stiffness);
   * ), causing the spring to yank the body back toward equilibrium=0
   * (PMX rest pose) against the VMD pose's non-zero angle. This sustained
   * spring-vs-pose conflict drives the persistent ~87 rad/s angular
   * velocity observed on Luoxi's ear_base_L_a_02_jnt (stiffness 125/150,
   * equilibrium≈0, body angle 0.23 rad). The previous 0.85 value was a
   * misreading of Bullet's damping semantics (it is a spring response
   * coefficient, NOT a damping ratio; larger = stronger spring pull, not
   * faster settle). Capture path already used 0.15 via RNA default, so
   * this also makes real-time and diagnostic paths consistent. */
  const bool diagnostic_disable_contacts = std::getenv("MMD_RT_DISABLE_CONTACTS") != nullptr;
  const bool diagnostic_disable_springs = std::getenv("MMD_RT_DISABLE_SPRINGS") != nullptr;
  const bool diagnostic_frame_driven_steps =
      std::getenv("MMD_RT_FRAME_DRIVEN_STEPS") != nullptr;
  const bool diagnostic_finalize_constraints =
      std::getenv("MMD_RT_FINALIZE_CONSTRAINTS") != nullptr;
  const bool diagnostic_kinematic_velocity =
      std::getenv("MMD_RT_KINEMATIC_VELOCITY") != nullptr;
  const bool diagnostic_fast_timer = std::getenv("MMD_RT_FAST_TIMER") != nullptr;
  const bool diagnostic_fixed_step_hz = std::getenv("MMD_RT_FIXED_STEP_HZ") != nullptr;
  const char *solver_iterations_env = std::getenv("MMD_RT_SOLVER_ITERATIONS");
  const int runtime_solver_iterations = solver_iterations_env != nullptr ?
                                            std::clamp(std::atoi(solver_iterations_env), 1, 200) :
                                            kDefaultSolverIterations;
  if (diagnostic_disable_contacts || diagnostic_disable_springs ||
      diagnostic_frame_driven_steps || diagnostic_finalize_constraints ||
      diagnostic_kinematic_velocity || diagnostic_fast_timer || diagnostic_fixed_step_hz ||
      solver_iterations_env != nullptr)
  {
    fprintf(stderr,
            "[MMD Physics] Runtime diagnostic overrides: contacts=%s springs=%s "
            "frame_driven_steps=%s finalize_constraints=%s kinematic_velocity=%s "
            "fast_timer=%s fixed_step_hz=%d solver_iterations=%d\n",
            diagnostic_disable_contacts ? "off" : "on",
            diagnostic_disable_springs ? "off" : "on",
            diagnostic_frame_driven_steps ? "on" : "off",
            diagnostic_finalize_constraints ? "on" : "off",
            diagnostic_kinematic_velocity ? "on" : "off",
            diagnostic_fast_timer ? "on" : "off",
            runtime_fixed_step_hz(),
            runtime_solver_iterations);
    fflush(stderr);
  }

  if (!world->initialize(session->binding.definition,
                         &armature,
                         bmain,
                         kDefaultGravity,
                         runtime_solver_iterations,
                         fixed_step_hz,
                         kDefaultMaxSubsteps,
                         diagnostic_disable_contacts,
                         diagnostic_disable_springs,
                         /*joint_spring_damping=*/0.15f))
  {
    BKE_report(reports, RPT_ERROR, "MMD Physics: world initialization failed");
    return nullptr;
  }

  world->set_dynamic_constraint_iterations(
      dynamic_constraint_iterations > 0 ?
          dynamic_constraint_iterations : runtime_dynamic_constraint_iterations(CTX_data_scene(C)));
  session->world = std::move(world);
  BKE_reportf(reports,
              RPT_INFO,
              "MMD Physics: initialized (bodies=%d, joints=%d)",
              session->world->body_count(),
              session->world->joint_count());
  return session;
}

MMDPhysicsSceneScheduler *ensure_scene_scheduler(bContext *C, ReportList *reports)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  wmWindow *window = CTX_wm_window(C);
  if (bmain == nullptr || scene == nullptr || window == nullptr) {
    BKE_report(reports, RPT_ERROR, "MMD Physics: missing Scene, Main, or Window context");
    return nullptr;
  }
  if (g_physics_scheduler != nullptr) {
    if (g_physics_scheduler->bmain != bmain ||
        g_physics_scheduler->scene_session_uid != scene->id.session_uid)
    {
      BKE_report(reports,
                 RPT_ERROR,
                 "MMD Physics: another Scene already owns the active physics scheduler");
      return nullptr;
    }
    return g_physics_scheduler.get();
  }

  auto scheduler = std::make_unique<MMDPhysicsSceneScheduler>();
  scheduler->bmain = bmain;
  scheduler->scene = scene;
  scheduler->window = window;
  scheduler->scene_session_uid = scene->id.session_uid;
  configure_realtime_timing(*scheduler, scene);
  g_physics_scheduler = std::move(scheduler);
  return g_physics_scheduler.get();
}

bool refresh_current_physics_frame(bContext *C, Object &armature);

MMDPhysicsRuntimeSession *add_runtime_session(bContext *C,
                                               ReportList *reports,
                                               Object *requested_armature = nullptr)
{
  MMDPhysicsSceneScheduler *scheduler = ensure_scene_scheduler(C, reports);
  Object *armature = requested_armature != nullptr ? requested_armature :
                                                     CTX_data_active_object(C);
  if (scheduler == nullptr || armature == nullptr || armature->type != OB_ARMATURE) {
    BKE_report(reports, RPT_ERROR, "MMD Physics: select an armature object first");
    return nullptr;
  }
  if (MMDPhysicsRuntimeSession *existing =
          find_runtime_session(*scheduler, armature->id.session_uid))
  {
    if (!refresh_runtime_session_binding(C, *existing)) {
      BKE_report(reports, RPT_ERROR, "MMD Physics: active model session is invalid");
      return nullptr;
    }
    return existing;
  }
  AnimData *anim_data = BKE_animdata_from_id(&armature->id);
  if (anim_data != nullptr && anim_data->action != nullptr) {
    if (physics_action_is_baked(*anim_data->action)) {
      BKE_report(reports,
                 RPT_ERROR,
                 "MMD Physics: active Action is baked; switch to its source Action first");
      if (scheduler->sessions.empty() && scheduler->timer == nullptr) {
        g_physics_scheduler.reset();
      }
      return nullptr;
    }
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &existing : scheduler->sessions) {
      if (existing->armature == nullptr) {
        continue;
      }
      AnimData *existing_anim_data = BKE_animdata_from_id(&existing->armature->id);
      if (existing_anim_data != nullptr && existing_anim_data->action == anim_data->action) {
        BKE_report(reports,
                   RPT_ERROR,
                   "MMD Physics: cannot run models that share an Action; "
                   "FCurve mute is Action-scoped");
        return nullptr;
      }
    }
  }
  if (!refresh_current_physics_frame(C, *armature)) {
    BKE_report(reports, RPT_ERROR, "MMD Physics: failed to evaluate the current frame");
    return nullptr;
  }
  std::unique_ptr<MMDPhysicsRuntimeSession> session = create_runtime_session(
      C, *armature, runtime_fixed_step_hz(scheduler), reports);
  if (session == nullptr) {
    return nullptr;
  }
  MMDPhysicsRuntimeSession *result = session.get();
  scheduler->sessions.push_back(std::move(session));
  return result;
}

MMDPhysicsWorld *ensure_world(bContext *C, ReportList *reports)
{
  MMDPhysicsRuntimeSession *session = active_runtime_session(C);
  if (session == nullptr) {
    session = add_runtime_session(C, reports);
  }
  if (session == nullptr || !refresh_runtime_session_binding(C, *session)) {
    BKE_report(reports, RPT_ERROR, "MMD Physics: active model session is unavailable");
    return nullptr;
  }
  return session->world.get();
}

bool refresh_current_physics_frame(bContext *C, Object &armature)
{
  Scene *scene = CTX_data_scene(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  if (scene == nullptr || depsgraph == nullptr) {
    return false;
  }

  /* Pose Mode edits are already the user's evaluated input. Re-evaluating the
   * active Action here would replace unkeyed manual changes before the physics
   * world gets a chance to read them. Object Mode has no such edit buffer, so
   * force the current frame to be evaluated before initializing bodies. */
  if ((armature.mode & OB_MODE_POSE) != 0) {
    return true;
  }
  DEG_id_tag_update(&armature.id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
  BKE_scene_graph_update_for_newframe(depsgraph);
  return true;
}

bool runtime_session_owner_is_live(const MMDPhysicsRuntimeSession &session, Main *current_bmain)
{
  if (current_bmain == nullptr || session.armature == nullptr ||
      session.binding.armature_session_uid == 0)
  {
    return false;
  }
  ID *current_id = BKE_libblock_find_session_uid(
      current_bmain, ID_OB, session.binding.armature_session_uid);
  return current_id == reinterpret_cast<ID *>(session.armature) &&
         session.armature->type == OB_ARMATURE;
}

void destroy_session(MMDPhysicsRuntimeSession &session, Main *current_bmain)
{
  const bool restore_blender_state = session.world != nullptr &&
                                     runtime_session_owner_is_live(session, current_bmain);
  if (restore_blender_state) {
    restore_physics_bone_action_curves(session, current_bmain);
    session.world->restore_physics_bone_connections();
    session.world->restore_physics_bone_constraints();
  }
  else {
    session.muted_action_curves.clear();
  }
  session.world.reset();
}

constexpr float kInteractiveFixedTimestep = 1.0f / 60.0f;
constexpr int kMaxInteractiveStepsPerTick = 2;

struct PhysicsStartupResult {
  int muted_constraints = 0;
  int muted_action_curves = 0;
  int disconnected_bones = 0;
  int prewarm_substeps = 0;
};
PhysicsStartupResult prepare_world_for_simulation(MMDPhysicsRuntimeSession &session);

int run_interactive_physics_tick(bContext *C,
                                 MMDPhysicsSceneScheduler &scheduler,
                                 Depsgraph *depsgraph,
                                 const int scene_frame,
                                 const double timer_elapsed)
{
  /* Paused interaction prioritizes input latency over wall-clock catch-up.
   * Never run faster than MMD's 60 Hz fixed step, even when playback/bake is
   * configured for denser substeps. */
  const float fixed_timestep = std::max(runtime_fixed_timestep(&scheduler),
                                        kInteractiveFixedTimestep);
  if (fixed_timestep <= 0.0f) {
    return 0;
  }

  const double elapsed = std::clamp(timer_elapsed, 0.0, 0.25);
  scheduler.time_accumulator = std::min(
      scheduler.time_accumulator + elapsed,
      double(fixed_timestep) * double(kMaxInteractiveStepsPerTick));

  int fixed_steps = 0;
  while (scheduler.time_accumulator + 1.0e-9 >= double(fixed_timestep) &&
         fixed_steps < kMaxInteractiveStepsPerTick)
  {
    fixed_steps++;
    scheduler.time_accumulator -= double(fixed_timestep);
  }

  if (fixed_steps > 0) {
    const float batch_timestep = fixed_timestep * float(fixed_steps);
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      if (session->world == nullptr) {
        continue;
      }
      /* The locked-joint pullback and broadphase refresh are batch-safe. Doing
       * them once per Timer event avoids repeating the full joint pass for
       * every fixed sub-step while preserving the simulated elapsed time. */
      session->world->set_defer_constraint_correction(true);
      if (std::getenv("MMD_RT_NATIVE_BRIDGE_DYNAMIC_SYNC") != nullptr) {
        session->world->step_full(batch_timestep, 1, false, fixed_steps);
        for (int frame_step = 0; frame_step < fixed_steps; frame_step++) {
          for (int bridge_step = 0; bridge_step < kNativeBridgeFixedSteps; bridge_step++) {
            session->world->sync_dynamic_from_pose_and_clear_velocities();
            session->world->step_full(fixed_timestep, 1, false, 1);
          }
        }
      }
      else {
        session->world->step_full(batch_timestep, 1, false, fixed_steps);
      }
      session->world->finalize_deferred_constraint_correction();
      session->world->set_defer_constraint_correction(false);
      session->world->apply_dynamic_to_pose();
      session->total_fixed_steps += uint64_t(fixed_steps);
    }
  }

  scheduler.last_timer_elapsed = elapsed;
  scheduler.last_fixed_steps = fixed_steps;
  scheduler.last_writeback = fixed_steps > 0;
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    session->last_timer_elapsed = elapsed;
    session->last_fixed_steps = fixed_steps;
    session->last_writeback = fixed_steps > 0;
    if (fixed_steps > 0 && session->world != nullptr) {
      session->world->flush_depsgraph(depsgraph);
      append_compare_trace(*session,
                           "MMD_RT_COMPARE_TRACE",
                           scene_frame,
                           elapsed,
                           scheduler.time_accumulator,
                           fixed_steps,
                           session->total_fixed_steps);
      WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
    }
  }
  return fixed_steps;
}

bool rebuild_runtime_sessions_at_current_frame(bContext *C,
                                                MMDPhysicsSceneScheduler &scheduler,
                                                ReportList *reports,
                                                Depsgraph *depsgraph,
                                                const int scene_frame,
                                                const bool restore_session_input = false)
{
  Main *bmain = CTX_data_main(C);
  if (bmain == nullptr || depsgraph == nullptr || scheduler.sessions.empty()) {
    return false;
  }

  struct RebuildEntry {
    Object *armature = nullptr;
    uint64_t debug_session = 0;
    Vector<PhysicsInputPoseBone> input_pose;
    int input_pose_scene_frame = 0;
  };
  std::vector<RebuildEntry> entries;
  entries.reserve(scheduler.sessions.size());
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    if (session == nullptr || session->armature == nullptr) {
      return false;
    }
    entries.push_back(
        {session->armature,
         session->debug_session + 1,
         session->input_pose,
         session->input_pose_scene_frame});
  }

  /* Restore every Blender-side owner before evaluating the destination frame.
   * This prevents a physics-written pose from becoming the input to the next
   * world when the timeline is scrubbed or seeks across a large gap. */
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    destroy_session(*session, bmain);
  }
  for (const RebuildEntry &entry : entries) {
    const bool use_saved_input = restore_session_input &&
                                  entry.input_pose_scene_frame == scene_frame;
    if (use_saved_input) {
      restore_physics_input_pose(entry.input_pose, *entry.armature);
    }
    DEG_id_tag_update(&entry.armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
  }
  BKE_scene_graph_update_for_newframe(depsgraph);

  std::vector<std::unique_ptr<MMDPhysicsRuntimeSession>> rebuilt_sessions;
  rebuilt_sessions.reserve(entries.size());
  for (const RebuildEntry &entry : entries) {
    const bool use_saved_input = restore_session_input &&
                                 entry.input_pose_scene_frame == scene_frame;
    std::unique_ptr<MMDPhysicsRuntimeSession> rebuilt = create_runtime_session(
        C,
        *entry.armature,
        runtime_fixed_step_hz(&scheduler),
        reports,
        use_saved_input ? &entry.input_pose : nullptr);
    if (rebuilt == nullptr) {
      for (std::unique_ptr<MMDPhysicsRuntimeSession> &partial : rebuilt_sessions) {
        destroy_session(*partial, bmain);
      }
      scheduler.sessions.clear();
      return false;
    }
    rebuilt->debug_session = entry.debug_session;
    if (use_saved_input) {
      rebuilt->input_pose = entry.input_pose;
      rebuilt->input_pose_scene_frame = entry.input_pose_scene_frame;
    }
    else {
      capture_physics_input_pose(*rebuilt, *entry.armature, scene_frame);
    }
    prepare_world_for_simulation(*rebuilt);
    rebuilt->world->apply_dynamic_to_pose();
    rebuilt->world->flush_depsgraph(depsgraph);
    append_compare_trace(*rebuilt, "MMD_RT_COMPARE_TRACE", scene_frame, 0.0, 0.0, 0, 0);
    rebuilt_sessions.push_back(std::move(rebuilt));
  }

  scheduler.sessions = std::move(rebuilt_sessions);
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    session->total_fixed_steps = 0;
    session->last_fixed_steps = 0;
    session->last_writeback = true;
    session->last_timer_elapsed = 0.0;
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
  }
  return true;
}

void destroy_world(Main *current_bmain)
{
  if (g_physics_scheduler == nullptr) {
    return;
  }
  for (std::unique_ptr<MMDPhysicsRuntimeSession> &session : g_physics_scheduler->sessions) {
    destroy_session(*session, current_bmain);
  }
  g_physics_scheduler->sessions.clear();
}

bool poll_armature(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE;
}

bool poll_running_physics(bContext * /*C*/)
{
  return g_physics_scheduler != nullptr &&
         (!g_physics_scheduler->sessions.empty() || g_physics_scheduler->timer != nullptr);
}

/* Tear down the real-time timer. Safe no-op when already removed. */
void remove_timer(bContext *C)
{
  if (g_physics_scheduler == nullptr || g_physics_scheduler->timer == nullptr) {
    return;
  }
  wmWindowManager *wm = CTX_wm_manager(C);
  if (wm != nullptr && g_physics_scheduler->window != nullptr) {
    WM_event_timer_remove(wm, g_physics_scheduler->window, g_physics_scheduler->timer);
  }
  g_physics_scheduler->timer = nullptr;
}

/* ----------------------------------------------------------------- */
/* Operators.                                                         */
/* ----------------------------------------------------------------- */

PhysicsStartupResult prepare_world_for_simulation(MMDPhysicsRuntimeSession &session)
{
  BLI_assert(session.world != nullptr);
  BLI_assert(session.armature != nullptr);
  MMDPhysicsWorld &world = *session.world;
  Object &armature = *session.armature;
  PhysicsStartupResult result;
  /* Snapshot the evaluated VMD pose before muting its physics-bone curves. */
  result.muted_constraints = world.mute_physics_bone_constraints();
  result.muted_action_curves = mute_physics_bone_action_curves(session, armature);
  result.disconnected_bones = world.disconnect_physics_bones();
  world.temporal_kinematic_init();
  /* Gradually interpolate kinematic (STATIC) bodies from PMX rest pose to the
   * current VMD pose over 30 frames, letting dynamic bodies free-simulate
   * while the spring delta builds up smoothly. Without this, the one-tick
   * jump from rest to VMD pose creates a sustained spring oscillation
   * ("颤动") on cape/hair chains. Mirrors MikuMikuPhysics's
   * `_sync_to_start_pose` (physics_world.py:1328-1353). */
  constexpr int kStartupSyncSteps = 30;
  world.startup_sync(kStartupSyncSteps);
  /* Prewarm lets springs settle toward equilibrium before the first real
   * Timer tick. Without it, dynamic bodies start at the VMD pose but with
   * zero velocity, and the first few ticks produce a transient drop as
   * gravity + spring find equilibrium. 2 coarse steps at 1/33 s mirrors
   * MikuMikuPhysics's default prewarm.
   *
   * Execute prewarm here (not in physics_start_invoke) so both the Start
   * operator and the Step operator benefit. Previously prewarm_substeps was
   * only returned as a hint, and physics_start_invoke mistook
   * `apply_dynamic_to_pose()` for prewarm — that writes current body
   * transforms to pose but does NOT advance the simulation, so prewarm was
   * effectively a no-op. */
  int prewarm_substeps = 2;
  if (const char *env = std::getenv("MMD_RT_PREWARM_STEPS")) {
    prewarm_substeps = std::clamp(std::atoi(env), 0, 60);
  }
  world.prewarm(prewarm_substeps, kDefaultTimerInterval, kDefaultMaxSubsteps);
  result.prewarm_substeps = prewarm_substeps;
  return result;
}

void remove_runtime_session(bContext *C, const uint64_t armature_session_uid)
{
  if (g_physics_scheduler == nullptr) {
    return;
  }
  Main *bmain = CTX_data_main(C);
  auto &sessions = g_physics_scheduler->sessions;
  for (auto it = sessions.begin(); it != sessions.end(); ++it) {
    if ((*it)->binding.armature_session_uid != armature_session_uid) {
      continue;
    }
    destroy_session(**it, bmain);
    sessions.erase(it);
    break;
  }
  if (sessions.empty()) {
    remove_timer(C);
    g_physics_scheduler.reset();
  }
}

wmOperatorStatus physics_start_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  MMDPhysicsSceneScheduler *scheduler = ensure_scene_scheduler(C, op->reports);
  Object *target_armature = resolve_operator_armature(C, op, op->reports);
  if (scheduler == nullptr || target_armature == nullptr || target_armature->type != OB_ARMATURE) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: select a target armature object first");
    return OPERATOR_CANCELLED;
  }
  if (MMDPhysicsRuntimeSession *existing =
          find_runtime_session(*scheduler, target_armature->id.session_uid))
  {
    remove_runtime_session(C, existing->binding.armature_session_uid);
    scheduler = ensure_scene_scheduler(C, op->reports);
  }
  const bool install_modal = scheduler != nullptr && scheduler->timer == nullptr;

  fprintf(stderr, "[MMD Physics] Start: ensure_world...\n");
  fflush(stderr);
  MMDPhysicsRuntimeSession *session = add_runtime_session(C, op->reports, target_armature);
  if (session == nullptr) {
    fprintf(stderr, "[MMD Physics] Start: ensure_world failed\n");
    fflush(stderr);
    return OPERATOR_CANCELLED;
  }
  MMDPhysicsWorld *world = session->world.get();
  capture_physics_input_pose(
      *session, *session->armature, scheduler->scene != nullptr ? scheduler->scene->r.cfra : 0);
  fprintf(stderr,
          "[MMD Physics] Start: world ready (bodies=%d, joints=%d)\n",
          world->body_count(),
          world->joint_count());
  fflush(stderr);

  /* F3 startup sequence (mirrors MikuMikuPhysics physics_world.py::start).
   * Keep this shared with first-time Step so both entry points snapshot the
   * initial pose and establish the same simulation lifecycle. */
  fprintf(stderr, "[MMD Physics] Start: preparing simulation state...\n");
  fflush(stderr);
  Object &armature = *session->armature;
  session->debug_session++;
  /* Absolute-Z probe AFTER ensure_world (initialize snapped bodies to
   * pchan->pose_mat) but BEFORE prepare_world_for_simulation. If the drop
   * happens here, the root cause is in ensure_world/initialize reading a
   * stale pose. */
  log_physics_pose_absolute("post_ensure_world",
                            armature,
                            session->binding.physics_bone_names,
                            session->debug_session);
  log_tracked_bone_z("post_ensure_world", armature, session->debug_session);
  const PhysicsStartupResult startup = prepare_world_for_simulation(*session);
  /* Absolute-Z probe AFTER prepare_world_for_simulation (mute constraints +
   * mute FCurves + disconnect bones + temporal kinematic init). If the drop
   * appears here but not at post_ensure_world, the root cause is in
   * prepare_world_for_simulation — most likely disconnect_physics_bones
   * rebuilding the armature with muted FCurves so pose_mat is recomputed
   * from stale pchan->loc instead of VMD animation. */
  log_physics_pose_absolute("post_prepare",
                            armature,
                            session->binding.physics_bone_names,
                            session->debug_session);
  log_tracked_bone_z("post_prepare", armature, session->debug_session);
  capture_physics_pose_debug_baseline(*session, armature);
  session->debug_first_writeback_pending = true;
  log_physics_pose_debug_delta("start_baseline", *session, armature);
  log_tracked_bone_z("start_baseline", armature, session->debug_session);
  fprintf(stderr,
          "[MMD Physics] Start: prepared (constraints=%d, action_curves=%d, "
          "disconnected=%d, prewarm_substeps=%d)\n",
          startup.muted_constraints,
          startup.muted_action_curves,
          startup.disconnected_bones,
          startup.prewarm_substeps);
  fflush(stderr);

  /* Never write the freshly initialized rigid transforms back as if they were
   * simulated output. Even a tiny conversion difference accumulates across
   * rebuilt worlds and appears as a drop on the second Start. */
  if (startup.prewarm_substeps > 0) {
    const int written = world->apply_dynamic_to_pose();
    fprintf(stderr,
            "[MMD Physics] Start: prewarm state applied (bones_written=%d)\n",
            written);
  }
  else {
    fprintf(stderr, "[MMD Physics] Start: prewarm writeback skipped (no simulated steps)\n");
  }
  fflush(stderr);

  BKE_reportf(op->reports,
              RPT_INFO,
               "MMD Physics: started (action_curves=%d, bones_disconnected=%d, "
               "prewarm_substeps=%d, initial_writeback=%s)",
               startup.muted_action_curves,
               startup.disconnected_bones,
               startup.prewarm_substeps,
               startup.prewarm_substeps > 0 ? "yes" : "no");

  /* F4: arm a real-time Timer that drives one physics step per tick.
   * The modal handler below consumes `TIMER` events whose `customdata`
   * matches the Scene scheduler's timer. */
  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *win = CTX_wm_window(C);
  scheduler = g_physics_scheduler.get();
  if (install_modal) {
    scheduler->time_accumulator = 0.0;
    scheduler->last_timer_elapsed = 0.0;
    scheduler->last_fixed_steps = 0;
    scheduler->last_writeback = false;
    scheduler->have_last_playback_frame = false;
    scheduler->last_playback_frame = 0;
    scheduler->have_observed_scene_frame = CTX_data_scene(C) != nullptr;
    scheduler->last_observed_scene_frame = CTX_data_scene(C) != nullptr ?
                                               CTX_data_scene(C)->r.cfra :
                                               0;
  }
  session->total_fixed_steps = 0;
  session->last_fixed_steps = 0;
  session->last_timer_elapsed = 0.0;
  session->last_writeback = false;
  session->runtime_profile = {};
  const double timer_interval = std::getenv("MMD_RT_FAST_TIMER") != nullptr ?
                                     double(runtime_fixed_timestep(scheduler)) :
                                     double(kDefaultTimerInterval);
  if (install_modal) {
    scheduler->timer = WM_event_timer_add(wm, win, TIMER, timer_interval);
  }

  WM_event_add_notifier(C, NC_OBJECT | ND_POSE, &armature);
  if (install_modal) {
    WM_event_add_modal_handler(C, op);
    fprintf(stderr, "[MMD Physics] Start: modal handler installed, returning\n");
  }
  else {
    fprintf(stderr, "[MMD Physics] Start: added model to running Scene scheduler\n");
  }
  fflush(stderr);
  return install_modal ? OPERATOR_RUNNING_MODAL : OPERATOR_FINISHED;
}

wmOperatorStatus physics_start_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (g_physics_scheduler == nullptr || g_physics_scheduler->sessions.empty()) {
    fprintf(stderr, "[MMD Physics] Modal: world gone, exiting\n");
    fflush(stderr);
    return OPERATOR_FINISHED;
  }
  if (!scheduler_bindings_are_valid(C)) {
    remove_timer(C);
    destroy_world(CTX_data_main(C));
    g_physics_scheduler.reset();
    fprintf(stderr, "[MMD Physics] Modal: Blender data changed, world discarded\n");
    fflush(stderr);
    return OPERATOR_CANCELLED;
  }
  /* Only consume our own TIMER events; pass everything else (mouse,
   * keyboard, redraw, etc.) through so the UI stays responsive while
   * the physics loop runs in the background. Without PASS_THROUGH the
   * modal handler swallows all events and Blender appears frozen. */
  if (event->type != TIMER || event->customdata != g_physics_scheduler->timer) {
    return OPERATOR_PASS_THROUGH;
  }

  MMDPhysicsSceneScheduler &scheduler = *g_physics_scheduler;
  Scene *scene = CTX_data_scene(C);
  const bScreen *screen = CTX_wm_screen(C);
  const bool animation_playing = screen != nullptr && screen->animtimer != nullptr;
  const bool playback_started = animation_playing && !scheduler.have_last_playback_frame;
  const int scene_frame = scene != nullptr ? scene->r.cfra : 0;
  Main *bmain = CTX_data_main(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);

  if (!animation_playing) {
    /* Pose editing is a live physics input even when the timeline is paused.
     * Keep the timer-driven fixed-step loop active so unkeyed Pose Mode edits
     * update kinematic bodies instead of producing only the startup prewarm. */
    const bool scene_frame_changed = scheduler.have_observed_scene_frame &&
                                     scene_frame != scheduler.last_observed_scene_frame;
    scheduler.have_observed_scene_frame = true;
    scheduler.last_observed_scene_frame = scene_frame;
    if (scene_frame_changed) {
      if (!rebuild_runtime_sessions_at_current_frame(
              C, scheduler, op->reports, depsgraph, scene_frame))
      {
        remove_timer(C);
        destroy_world(bmain);
        g_physics_scheduler.reset();
        return OPERATOR_CANCELLED;
      }
      scheduler.time_accumulator = 0.0;
      scheduler.last_fixed_steps = 0;
      scheduler.last_writeback = true;
      scheduler.have_last_playback_frame = false;
      scheduler.last_playback_frame = scene_frame;
      return OPERATOR_RUNNING_MODAL;
    }
    scheduler.have_last_playback_frame = false;
    scheduler.last_playback_frame = scene_frame;
    const wmTimer *timer = static_cast<const wmTimer *>(event->customdata);
    run_interactive_physics_tick(C,
                                 scheduler,
                                 depsgraph,
                                 scene_frame,
                                 timer != nullptr ? timer->time_delta : 0.0);
    return OPERATOR_RUNNING_MODAL;
  }

  if (playback_started) {
    /* Paused pose editing advances a live Bullet world whose contact and
     * spring history is not part of VMD playback. Rebuild from the evaluated
     * current frame at the playback edge, matching the history-independent
     * startup used by Action bake. */
    if (!rebuild_runtime_sessions_at_current_frame(
            C, scheduler, op->reports, depsgraph, scene_frame, true))
    {
      remove_timer(C);
      destroy_world(bmain);
      g_physics_scheduler.reset();
      return OPERATOR_CANCELLED;
    }
    scheduler.time_accumulator = 0.0;
    scheduler.last_fixed_steps = 0;
    scheduler.last_writeback = false;
    scheduler.have_last_playback_frame = true;
    scheduler.last_playback_frame = scene_frame;
    scheduler.have_observed_scene_frame = true;
    scheduler.last_observed_scene_frame = scene_frame;
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      session->last_fixed_steps = 0;
      session->last_writeback = false;
      session->playback_edge_trace.remaining_ticks = 0;
    }
    return OPERATOR_RUNNING_MODAL;
  }

  if (scheduler.have_last_playback_frame && scene_frame < scheduler.last_playback_frame)
  {
    if (scene == nullptr || bmain == nullptr || depsgraph == nullptr) {
      remove_timer(C);
      return OPERATOR_CANCELLED;
    }
    fprintf(stderr,
            "[MMD Physics V2] Recreate preview world at playback frame=%d\n",
            scene_frame);
    fflush(stderr);

    if (!rebuild_runtime_sessions_at_current_frame(
            C, scheduler, op->reports, depsgraph, scene_frame))
    {
      remove_timer(C);
      destroy_world(bmain);
      g_physics_scheduler.reset();
      return OPERATOR_CANCELLED;
    }
    scheduler.time_accumulator = 0.0;
    scheduler.last_fixed_steps = 0;
    scheduler.last_writeback = true;
    scheduler.have_last_playback_frame = true;
    scheduler.last_playback_frame = scene_frame;
    scheduler.have_observed_scene_frame = true;
    scheduler.last_observed_scene_frame = scene_frame;
    return OPERATOR_RUNNING_MODAL;
  }

  if (scheduler.have_last_playback_frame && scene_frame == scheduler.last_playback_frame) {
    return OPERATOR_RUNNING_MODAL;
  }

  if (scene == nullptr || depsgraph == nullptr) {
    return OPERATOR_RUNNING_MODAL;
  }

  const int playback_frame_delta = scene_frame - scheduler.last_playback_frame;
  if (scheduler.have_last_playback_frame &&
      playback_frame_delta > kRealtimeSeekStopThresholdFrames)
  {
    const int playback_frame_before_seek = scheduler.last_playback_frame;
    /* A large forward jump while playback is active is an interactive timeline
     * seek, not ordinary dropped-frame catch-up. Simulating every skipped frame
     * blocks the UI for seconds on long seeks (for example 100 -> 1000). Stop
     * playback and rebuild the world immediately at the requested frame so the
     * next Play cannot continue from the old frame's physics state. */
    ED_screen_animation_play(C, 0, 0);
    if (scene->r.cfra != scene_frame) {
      BKE_scene_frame_set(scene, float(scene_frame));
      for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
        DEG_id_tag_update(&session->armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      }
      BKE_scene_graph_update_for_newframe(depsgraph);
    }
    if (!rebuild_runtime_sessions_at_current_frame(
            C, scheduler, op->reports, depsgraph, scene_frame))
    {
      remove_timer(C);
      destroy_world(bmain);
      g_physics_scheduler.reset();
      return OPERATOR_CANCELLED;
    }
    scheduler.time_accumulator = 0.0;
    scheduler.last_fixed_steps = 0;
    scheduler.last_writeback = true;
    scheduler.have_last_playback_frame = false;
    scheduler.last_playback_frame = scene_frame;
    scheduler.have_observed_scene_frame = true;
    scheduler.last_observed_scene_frame = scene_frame;
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      session->last_fixed_steps = 0;
      session->last_writeback = true;
      session->playback_edge_trace.remaining_ticks = 0;
    }
    fprintf(stderr,
            "[MMD Physics V2] Stop playback after timeline seek: from=%d to=%d delta=%d\n",
            playback_frame_before_seek,
            scene_frame,
            playback_frame_delta);
    fflush(stderr);
    WM_event_add_notifier(C, NC_SCENE | ND_FRAME, scene);
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
    }
    return OPERATOR_RUNNING_MODAL;
  }

  const float fixed_timestep = runtime_fixed_timestep(&scheduler);
  const double seconds_per_frame = double(scene->r.frs_sec_base) /
                                   double(std::max(1, int(scene->r.frs_sec)));
  const int fixed_steps_per_frame = runtime_fixed_steps_per_frame(scene, &scheduler);
  const bool diagnostic_native_bridge_timing =
      std::getenv("MMD_RT_NATIVE_BRIDGE_TIMING") != nullptr;
  const bool diagnostic_native_bridge_dynamic_sync =
      std::getenv("MMD_RT_NATIVE_BRIDGE_DYNAMIC_SYNC") != nullptr;
  const bool use_native_bridge_timing = diagnostic_native_bridge_timing ||
                                        std::getenv("MMD_RT_FIXED_STEP_HZ") == nullptr;
  const int base_fixed_steps_per_frame = use_native_bridge_timing ?
                                             (diagnostic_native_bridge_timing ?
                                                  fixed_steps_per_frame :
                                                  std::max(1,
                                                           fixed_steps_per_frame -
                                                               kNativeBridgeFixedSteps)) :
                                             fixed_steps_per_frame;
  const int executed_fixed_steps_per_frame = base_fixed_steps_per_frame +
                                             (use_native_bridge_timing ?
                                                  kNativeBridgeFixedSteps :
                                                  0);
  const int first_frame = scheduler.last_playback_frame + 1;
  for (int evaluated_frame = first_frame; evaluated_frame <= scene_frame; evaluated_frame++) {
    const bool last_frame = evaluated_frame == scene_frame;
    if (scene->r.cfra != evaluated_frame) {
      BKE_scene_frame_set(scene, float(evaluated_frame));
      for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
        DEG_id_tag_update(&session->armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      }
      BKE_scene_graph_update_for_newframe(depsgraph);
    }
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      session->world->set_defer_constraint_correction(false);
      session->world->step_full(fixed_timestep * float(base_fixed_steps_per_frame),
                                1,
                                false,
                                base_fixed_steps_per_frame);
      if (use_native_bridge_timing) {
        /* MMD's first bridge step consumes the frame-time remainder after
         * the integer 1/60 steps. At 24 FPS this is 1/120; at 30 FPS it is
         * zero. Omitting it makes the 24 FPS path use a different Bullet
         * local-time phase from the native bridge. */
        const float remainder = std::max(
            0.0f,
            float(seconds_per_frame) - fixed_timestep * float(base_fixed_steps_per_frame));
        if (remainder > 1.0e-7f) {
          session->world->step_full(remainder, 1, false, 1);
        }
        /* MMD's physical bridge performs three additional fixed 1/60 steps
         * after the frame-budget step. Keep the pose writeback boundary after
         * the whole bridge so dynamic bodies see the same frame input. */
        for (int bridge_step = 0; bridge_step < kNativeBridgeFixedSteps; bridge_step++) {
          if (diagnostic_native_bridge_dynamic_sync) {
            session->world->sync_dynamic_from_pose_and_clear_velocities();
          }
          session->world->step_full(fixed_timestep, 1, false, 1);
        }
      }
      session->total_fixed_steps += uint64_t(executed_fixed_steps_per_frame);
    }
    if (last_frame || std::getenv("MMD_RT_COMPARE_TRACE") != nullptr) {
      for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
        session->world->apply_dynamic_to_pose();
        if (last_frame) {
          session->world->flush_depsgraph(depsgraph);
        }
      }
    }
    for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
      append_compare_trace(*session,
                           "MMD_RT_COMPARE_TRACE",
                           evaluated_frame,
                           seconds_per_frame,
                           0.0,
                           executed_fixed_steps_per_frame,
                           session->total_fixed_steps);
    }
  }
  scheduler.last_timer_elapsed = seconds_per_frame;
  scheduler.last_fixed_steps = fixed_steps_per_frame;
  scheduler.last_writeback = true;
  scheduler.have_last_playback_frame = true;
  scheduler.last_playback_frame = scene_frame;
  scheduler.have_observed_scene_frame = true;
  scheduler.last_observed_scene_frame = scene_frame;
  for (const std::unique_ptr<MMDPhysicsRuntimeSession> &session : scheduler.sessions) {
    session->last_timer_elapsed = seconds_per_frame;
    session->last_fixed_steps = fixed_steps_per_frame;
    session->last_writeback = true;
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
  }
  return OPERATOR_RUNNING_MODAL;
}

void physics_start_cancel(bContext *C, wmOperator * /*op*/)
{
  remove_timer(C);
  Object *armature = CTX_data_active_object(C);
  destroy_world(CTX_data_main(C));
  g_physics_scheduler.reset();
  if (armature != nullptr) {
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
    Main *bmain = CTX_data_main(C);
    if (depsgraph != nullptr && bmain != nullptr) {
      /* Action curves were muted during physics. Re-evaluate the current VMD
       * frame after restoring them so a later Start cannot inherit physics or
       * the previous startup frame through stale pose channels. */
      DEG_id_tag_update(&armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      BKE_scene_graph_update_for_newframe(depsgraph);
    }
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, armature);
  }
}

wmOperatorStatus physics_step_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  const bool needs_startup = active_runtime_session(C) == nullptr;
  MMDPhysicsWorld *world = ensure_world(C, op->reports);
  if (world == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (needs_startup) {
    MMDPhysicsRuntimeSession *session = active_runtime_session(C);
    if (session == nullptr) {
      return OPERATOR_CANCELLED;
    }
    prepare_world_for_simulation(*session);
  }
  const int written = world->step_full(kDefaultTimerInterval, kDefaultMaxSubsteps, true);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  world->flush_depsgraph(depsgraph);
  const auto &perf = world->performance();
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: stepped (substeps=%d, %.2fms, bones_written=%d)",
              perf.last_substeps,
              perf.last_step_time_ms,
              written);
  if (MMDPhysicsRuntimeSession *session = active_runtime_session(C)) {
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
  }
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_reset_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: cannot reset during Action bake");
    return OPERATOR_CANCELLED;
  }
  MMDPhysicsRuntimeSession *session = active_runtime_session(C);
  if (session == nullptr || session->world == nullptr) {
    BKE_report(op->reports, RPT_WARNING, "MMD Physics: no active world to reset");
    return OPERATOR_CANCELLED;
  }
  session->world->reset();
  BKE_report(op->reports, RPT_INFO, "MMD Physics: reset to initial transforms");
  WM_event_add_notifier(C, NC_OBJECT | ND_POSE, session->armature);
  return OPERATOR_FINISHED;
}

struct PhysicsBakeTrack {
  std::string bone_name;
  std::string location_path;
  std::string rotation_path;
  std::string rotation_euler_path;
  std::string rotation_axis_angle_path;
  std::string scale_path;
  bool full_transform = false;
  std::vector<std::array<float, 3>> locations;
  std::vector<std::array<float, 4>> rotations;
  std::vector<std::array<float, 3>> scales;
};

struct PhysicsBakePoseBone {
  std::string bone_name;
  float location[3];
  float quaternion[4];
  float euler[3];
  float rotation_axis[3];
  float rotation_angle;
  float scale[3];
  eRotationModes rotation_mode;
  float channel_matrix[4][4];
  float pose_matrix[4][4];
  float constraint_inverse[4][4];
  float pose_head[3];
  float pose_tail[3];
};

void physics_bake_capture_pose(const Object &armature, std::vector<PhysicsBakePoseBone> &pose)
{
  pose.clear();
  if (armature.pose == nullptr) {
    return;
  }
  for (const bPoseChannel *pchan =
           static_cast<const bPoseChannel *>(armature.pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    PhysicsBakePoseBone bone;
    bone.bone_name = pchan->name;
    copy_v3_v3(bone.location, pchan->loc);
    copy_qt_qt(bone.quaternion, pchan->quat);
    copy_v3_v3(bone.euler, pchan->eul);
    copy_v3_v3(bone.rotation_axis, pchan->rotAxis);
    bone.rotation_angle = pchan->rotAngle;
    copy_v3_v3(bone.scale, pchan->scale);
    bone.rotation_mode = pchan->rotmode;
    copy_m4_m4(bone.channel_matrix, pchan->chan_mat);
    copy_m4_m4(bone.pose_matrix, pchan->pose_mat);
    copy_m4_m4(bone.constraint_inverse, pchan->constinv);
    copy_v3_v3(bone.pose_head, pchan->pose_head);
    copy_v3_v3(bone.pose_tail, pchan->pose_tail);
    pose.push_back(std::move(bone));
  }
}

void physics_bake_restore_pose(Object &armature,
                               const std::vector<PhysicsBakePoseBone> &pose,
                               const bool update_depsgraph = true)
{
  if (armature.pose == nullptr) {
    return;
  }
  for (const PhysicsBakePoseBone &bone : pose) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, bone.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    copy_v3_v3(pchan->loc, bone.location);
    copy_qt_qt(pchan->quat, bone.quaternion);
    copy_v3_v3(pchan->eul, bone.euler);
    copy_v3_v3(pchan->rotAxis, bone.rotation_axis);
    pchan->rotAngle = bone.rotation_angle;
    copy_v3_v3(pchan->scale, bone.scale);
    pchan->rotmode = bone.rotation_mode;
    copy_m4_m4(pchan->chan_mat, bone.channel_matrix);
    copy_m4_m4(pchan->pose_mat, bone.pose_matrix);
    copy_m4_m4(pchan->constinv, bone.constraint_inverse);
    copy_v3_v3(pchan->pose_head, bone.pose_head);
    copy_v3_v3(pchan->pose_tail, bone.pose_tail);
  }
  if (update_depsgraph) {
    DEG_id_tag_update(&armature.id, ID_RECALC_GEOMETRY);
  }
}

IDProperty *physics_bake_provenance(const bAction &action)
{
  IDProperty *properties = IDP_GetProperties(const_cast<ID *>(&action.id));
  return properties != nullptr ?
             IDP_GetPropertyTypeFromGroup(properties, kBakeProvenanceProperty, IDP_GROUP) :
             nullptr;
}

bool physics_action_is_baked(const bAction &action)
{
  IDProperty *provenance = physics_bake_provenance(action);
  if (provenance == nullptr) {
    return false;
  }
  IDProperty *schema = IDP_GetPropertyTypeFromGroup(provenance, "schema_version", IDP_INT);
  IDProperty *generator = IDP_GetPropertyTypeFromGroup(provenance, "generator", IDP_STRING);
  return schema != nullptr && IDP_int_get(schema) == 1 && generator != nullptr &&
         STREQ(IDP_string_get(generator), "MMD Physics Bake");
}

wmOperatorStatus physics_bake_use_source_action_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: stop real-time physics first");
    return OPERATOR_CANCELLED;
  }

  Object *armature = resolve_operator_armature(C, op, op->reports);
  Main *bmain = CTX_data_main(C);
  if (armature == nullptr || bmain == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: invalid target Armature or scene");
    return OPERATOR_CANCELLED;
  }
  AnimData *anim_data = BKE_animdata_from_id(&armature->id);
  bAction *baked_action = anim_data != nullptr ? anim_data->action : nullptr;
  IDProperty *provenance = baked_action != nullptr ? physics_bake_provenance(*baked_action) : nullptr;
  IDProperty *source_action_property = provenance != nullptr ?
                                           IDP_GetPropertyTypeFromGroup(
                                               provenance, "source_action", IDP_STRING) :
                                           nullptr;
  IDProperty *source_slot_property = provenance != nullptr ?
                                         IDP_GetPropertyTypeFromGroup(
                                             provenance, "source_slot", IDP_STRING) :
                                         nullptr;
  if (baked_action == nullptr || !physics_action_is_baked(*baked_action) ||
      source_action_property == nullptr || source_slot_property == nullptr)
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: the target has no baked Action to restore");
    return OPERATOR_CANCELLED;
  }
  if (!BKE_animdata_action_editable(anim_data)) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics: cannot switch Actions while NLA tweak mode is active");
    return OPERATOR_CANCELLED;
  }

  const char *source_action_name = IDP_string_get(source_action_property);
  const char *source_slot_identifier = IDP_string_get(source_slot_property);
  bAction *source_action = reinterpret_cast<bAction *>(
      BKE_libblock_find_name(bmain, ID_AC, source_action_name));
  if (source_action == nullptr) {
    BKE_reportf(op->reports,
                RPT_ERROR,
                "MMD Physics: source Action '%s' was not found",
                source_action_name);
    return OPERATOR_CANCELLED;
  }
  animrig::Slot *source_slot = source_action->wrap().slot_find_by_identifier(
      source_slot_identifier);
  if (source_slot == nullptr || !source_slot->is_suitable_for(armature->id)) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: source Action slot is not suitable");
    return OPERATOR_CANCELLED;
  }

  /* Keep the baked Action in the file as a selectable result after switching
   * back to the pristine source Action. */
  id_fake_user_set(&baked_action->id);
  const animrig::ActionSlotAssignmentResult assignment_result =
      animrig::assign_action_and_slot(&source_action->wrap(), source_slot, armature->id);
  if (assignment_result != animrig::ActionSlotAssignmentResult::OK) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: failed to restore the source Action");
    return OPERATOR_CANCELLED;
  }

  DEG_id_tag_update(&armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
  if (Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C)) {
    BKE_scene_graph_update_for_newframe(depsgraph);
  }
  WM_event_add_notifier(C, NC_ANIMATION | ND_NLA_ACTCHANGE, armature);
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: restored source Action '%s'; baked Action '%s' was kept",
              source_action->id.name + 2,
              baked_action->id.name + 2);
  return OPERATOR_FINISHED;
}

void physics_bake_mark_action(bAction &baked_action,
                              const bAction &source_action,
                              const animrig::slot_handle_t source_slot_handle,
                              const Object &armature,
                              const int frame_start,
                              const int frame_end,
                              const int substeps_per_frame)
{
  IDProperty *provenance = IDP_New(IDP_GROUP, nullptr, kBakeProvenanceProperty);
  IDP_AddToGroup(provenance, IDP_NewInt(1, "schema_version"));
  IDP_AddToGroup(provenance, IDP_NewString("MMD Physics Bake", "generator"));
  IDP_AddToGroup(provenance, IDP_NewString(source_action.id.name + 2, "source_action"));
  const animrig::Slot *source_slot = source_action.wrap().slot_for_handle(source_slot_handle);
  IDP_AddToGroup(
      provenance,
      IDP_NewString(source_slot != nullptr ? source_slot->identifier : "", "source_slot"));
  IDP_AddToGroup(provenance, IDP_NewString(armature.id.name + 2, "source_armature"));
  IDP_AddToGroup(provenance, IDP_NewInt(frame_start, "frame_start"));
  IDP_AddToGroup(provenance, IDP_NewInt(frame_end, "frame_end"));
  IDP_AddToGroup(provenance, IDP_NewInt(substeps_per_frame, "substeps_per_frame"));
  IDP_ReplaceInGroup(IDP_EnsureProperties(&baked_action.id), provenance);
}

std::string physics_bone_path(const std::string &bone_name, const char *property)
{
  char escaped_name[128] = {};
  BLI_str_escape(escaped_name, bone_name.c_str(), sizeof(escaped_name));
  return std::string("pose.bones[\"") + escaped_name + "\"]." + property;
}

void capture_physics_bake_frame(Object &armature, std::vector<PhysicsBakeTrack> &tracks)
{
  for (PhysicsBakeTrack &track : tracks) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, track.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    track.locations.push_back({pchan->loc[0], pchan->loc[1], pchan->loc[2]});
    track.rotations.push_back({pchan->quat[0], pchan->quat[1], pchan->quat[2], pchan->quat[3]});
    track.scales.push_back({pchan->scale[0], pchan->scale[1], pchan->scale[2]});
  }
}

std::vector<std::string> physics_bake_bone_names(const MMDPhysicsRuntimeSession &session)
{
  Set<std::string> bake_bones = session.binding.dynamic_bone_names;
  for (const std::string &bone_name : session.binding.dynamic_merge_bone_names) {
    bake_bones.add(bone_name);
  }
  std::vector<std::string> names;
  names.reserve(bake_bones.size());
  for (const std::string &bone_name : bake_bones) {
    names.push_back(bone_name);
  }
  std::sort(names.begin(), names.end());
  return names;
}

struct PhysicsBakeActionWriter {
  bAction *baked_action = nullptr;
  uint64_t baked_action_session_uid = 0;
  animrig::slot_handle_t source_slot_handle = animrig::Slot::unassigned;
  int inserted_keys = 0;
};

bool physics_bake_action_begin(Main &bmain,
                               bAction &source_action,
                               const animrig::slot_handle_t source_slot_handle,
                               const std::vector<PhysicsBakeTrack> &tracks,
                               ReportList *reports,
                               PhysicsBakeActionWriter &writer)
{
  writer = {};
  writer.baked_action = id_cast<bAction *>(BKE_id_copy(&bmain, &source_action.id));
  if (writer.baked_action == nullptr) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: failed to duplicate source Action");
    return false;
  }
  writer.baked_action_session_uid = writer.baked_action->id.session_uid;
  writer.source_slot_handle = source_slot_handle;
  /* Keep the copied Action owned while a modal bake yields between timer events. */
  BKE_id_rename(bmain,
                writer.baked_action->id,
                std::string(source_action.id.name + 2) + "_MMD_Physics_Bake");

  animrig::Action &action = writer.baked_action->wrap();
  animrig::Slot *slot = action.slot_for_handle(source_slot_handle);
  animrig::Channelbag *channelbag = animrig::channelbag_for_action_slot(action,
                                                                       source_slot_handle);
  if (slot == nullptr || channelbag == nullptr) {
    BKE_report(reports,
               RPT_ERROR,
               "MMD Physics Bake: source Action has no editable keyframe slot");
    id_us_min(&writer.baked_action->id);
    if (writer.baked_action->id.us == 0) {
      BKE_id_free(&bmain, &writer.baked_action->id);
    }
    writer.baked_action = nullptr;
    writer.baked_action_session_uid = 0;
    return false;
  }

  const auto is_replaced_curve = [&](const FCurve &fcurve) {
    if (fcurve.rna_path().is_empty()) {
      return false;
    }
    for (const PhysicsBakeTrack &track : tracks) {
      if (STREQ(fcurve.rna_path().c_str(), track.rotation_path.c_str()) ||
          STREQ(fcurve.rna_path().c_str(), track.rotation_euler_path.c_str()) ||
          STREQ(fcurve.rna_path().c_str(), track.rotation_axis_angle_path.c_str()))
      {
        return true;
      }
      if (track.full_transform &&
          (STREQ(fcurve.rna_path().c_str(), track.location_path.c_str()) ||
           STREQ(fcurve.rna_path().c_str(), track.scale_path.c_str())))
      {
        return true;
      }
    }
    return false;
  };
  for (int64_t i = channelbag->fcurves().size() - 1; i >= 0; i--) {
    if (is_replaced_curve(*channelbag->fcurve(i))) {
      channelbag->fcurve_remove_by_index(i);
    }
  }
  return true;
}

bool physics_bake_action_write_track(Main &bmain,
                                     const PhysicsBakeTrack &track,
                                     const int frame_start,
                                     ReportList *reports,
                                     PhysicsBakeActionWriter &writer)
{
  if (writer.baked_action == nullptr) {
    return false;
  }
  animrig::Action &action = writer.baked_action->wrap();
  animrig::Channelbag *channelbag = animrig::channelbag_for_action_slot(
      action, writer.source_slot_handle);
  if (channelbag == nullptr) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: lost editable keyframe slot");
    return false;
  }
  const animrig::KeyframeSettings key_settings = {
      BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, BEZT_IPO_LIN};
  const int sample_count = int(track.rotations.size());
  if (sample_count == 0 || int(track.locations.size()) != sample_count ||
      int(track.scales.size()) != sample_count)
  {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: incomplete sampled bone track");
    return false;
  }

  std::array<FCurve *, 3> location_curves = {};
  std::array<FCurve *, 4> rotation_curves = {};
  std::array<FCurve *, 3> scale_curves = {};
  auto ensure_curve = [&](const std::string &path, const int array_index) -> FCurve * {
    animrig::FCurveDescriptor descriptor;
    descriptor.rna_path = path;
    descriptor.array_index = array_index;
    descriptor.prop_type = PROP_FLOAT;
    descriptor.prop_subtype = PROP_NONE;
    return &channelbag->fcurve_ensure(&bmain, descriptor);
  };
  for (int i = 0; i < 4; i++) {
    rotation_curves[i] = ensure_curve(track.rotation_path, i);
  }
  if (track.full_transform) {
    for (int i = 0; i < 3; i++) {
      location_curves[i] = ensure_curve(track.location_path, i);
      scale_curves[i] = ensure_curve(track.scale_path, i);
    }
  }

  std::array<float, 4> previous_quaternion = track.rotations[0];
  for (int sample = 0; sample < sample_count; sample++) {
    std::array<float, 4> quaternion = track.rotations[sample];
    if (sample > 0) {
      float dot = 0.0f;
      for (int i = 0; i < 4; i++) {
        dot += previous_quaternion[i] * quaternion[i];
      }
      if (dot < 0.0f) {
        for (float &value : quaternion) {
          value = -value;
        }
      }
    }
    previous_quaternion = quaternion;
    const float frame = float(frame_start + sample);
    for (int i = 0; i < 4; i++) {
      if (animrig::insert_vert_fcurve(
              rotation_curves[i], {frame, quaternion[i]}, key_settings, INSERTKEY_FAST) !=
          animrig::SingleKeyingResult::SUCCESS)
      {
        BKE_report(reports, RPT_ERROR, "MMD Physics Bake: failed to insert rotation key");
        return false;
      }
      writer.inserted_keys++;
    }
    if (track.full_transform) {
      for (int i = 0; i < 3; i++) {
        if (animrig::insert_vert_fcurve(location_curves[i],
                                        {frame, track.locations[sample][i]},
                                        key_settings,
                                        INSERTKEY_FAST) !=
                animrig::SingleKeyingResult::SUCCESS ||
            animrig::insert_vert_fcurve(scale_curves[i],
                                        {frame, track.scales[sample][i]},
                                        key_settings,
                                        INSERTKEY_FAST) !=
                animrig::SingleKeyingResult::SUCCESS)
        {
          BKE_report(reports, RPT_ERROR, "MMD Physics Bake: failed to insert transform key");
          return false;
        }
        writer.inserted_keys += 2;
      }
    }
  }

  for (FCurve *curve : rotation_curves) {
    BKE_fcurve_handles_recalc(*curve);
  }
  if (track.full_transform) {
    for (FCurve *curve : location_curves) {
      BKE_fcurve_handles_recalc(*curve);
    }
    for (FCurve *curve : scale_curves) {
      BKE_fcurve_handles_recalc(*curve);
    }
  }
  return true;
}

bool physics_bake_action_finish(Object &armature,
                                bAction &source_action,
                                const int frame_start,
                                const int frame_end,
                                const int substeps_per_frame,
                                const std::vector<PhysicsBakeTrack> &tracks,
                                ReportList *reports,
                                PhysicsBakeActionWriter &writer,
                                bAction *&r_baked_action)
{
  r_baked_action = nullptr;
  if (writer.baked_action == nullptr) {
    return false;
  }
  animrig::Action &action = writer.baked_action->wrap();
  animrig::Slot *slot = action.slot_for_handle(writer.source_slot_handle);
  if (slot == nullptr) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: lost Action slot before assignment");
    return false;
  }
  AnimData *anim_data = BKE_animdata_from_id(&armature.id);
  if (anim_data == nullptr) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: Armature lost animation data");
    return false;
  }
  if (!BKE_animdata_action_editable(anim_data)) {
    BKE_report(reports,
               RPT_ERROR,
               "MMD Physics Bake: cannot replace the Action while NLA tweak mode is active");
    return false;
  }
  if (!slot->is_suitable_for(armature.id)) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: baked Action slot is not suitable");
    return false;
  }
  bAction *original_action = anim_data->action;
  const animrig::slot_handle_t original_slot_handle = anim_data->slot_handle;
  animrig::Slot *original_slot = original_action != nullptr ?
                                     original_action->wrap().slot_for_handle(original_slot_handle) :
                                     nullptr;
  if (original_action == nullptr || original_slot == nullptr ||
      !original_slot->is_suitable_for(armature.id))
  {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: original Action slot is no longer valid");
    return false;
  }

  physics_bake_mark_action(*writer.baked_action,
                           source_action,
                           writer.source_slot_handle,
                           armature,
                           frame_start,
                           frame_end,
                           substeps_per_frame);

  /* Keep the source alive while assign_action_and_slot() temporarily removes its user. */
  id_us_plus(&original_action->id);
  const animrig::ActionSlotAssignmentResult assignment_result = animrig::assign_action_and_slot(
      &action, slot, armature.id);
  if (assignment_result != animrig::ActionSlotAssignmentResult::OK) {
    bool rollback_ok = anim_data->action == original_action &&
                       anim_data->slot_handle == original_slot_handle;
    if (!rollback_ok) {
      rollback_ok = animrig::assign_action_and_slot(
                        &original_action->wrap(), original_slot, armature.id) ==
                    animrig::ActionSlotAssignmentResult::OK;
    }
    id_us_min(&original_action->id);
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: failed to assign baked Action");
    if (!rollback_ok) {
      BKE_report(reports,
                 RPT_ERROR,
                 "MMD Physics Bake: failed to roll back the original Action and slot");
    }
    return false;
  }
  id_us_min(&original_action->id);
  id_fake_user_set(&source_action.id);
  for (const PhysicsBakeTrack &track : tracks) {
    if (bPoseChannel *pchan = BKE_pose_channel_find_name(armature.pose, track.bone_name.c_str())) {
      pchan->rotmode = ROT_MODE_QUAT;
    }
  }
  DEG_id_tag_update(&writer.baked_action->id, ID_RECALC_ANIMATION_NO_FLUSH);
  DEG_id_tag_update(&armature.id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
  r_baked_action = writer.baked_action;
  /* Transfer the temporary copy user to the Action assignment. */
  id_us_min(&writer.baked_action->id);
  writer.baked_action = nullptr;
  writer.baked_action_session_uid = 0;
  BKE_reportf(reports, RPT_INFO, "MMD Physics Bake: inserted %d keys", writer.inserted_keys);
  return true;
}

void physics_bake_action_discard(Main &bmain, PhysicsBakeActionWriter &writer)
{
  if (writer.baked_action != nullptr) {
    bAction *action = writer.baked_action;
    /* Drop only the writer's ownership. An Action adopted by another user must survive. */
    id_us_min(&action->id);
    if (action->id.us == 0) {
      BKE_id_free(&bmain, &action->id);
    }
    writer.baked_action = nullptr;
    writer.baked_action_session_uid = 0;
  }
}

bool write_physics_bake_action(Main &bmain,
                               Object &armature,
                               bAction &source_action,
                               const animrig::slot_handle_t source_slot_handle,
                               const int frame_start,
                               const int frame_end,
                               const int substeps_per_frame,
                               const std::vector<PhysicsBakeTrack> &tracks,
                               ReportList *reports,
                               bAction *&r_baked_action)
{
  PhysicsBakeActionWriter writer;
  if (!physics_bake_action_begin(
          bmain, source_action, source_slot_handle, tracks, reports, writer))
  {
    return false;
  }
  for (const PhysicsBakeTrack &track : tracks) {
    if (!physics_bake_action_write_track(bmain, track, frame_start, reports, writer)) {
      physics_bake_action_discard(bmain, writer);
      return false;
    }
  }
  if (!physics_bake_action_finish(armature,
                                  source_action,
                                  frame_start,
                                  frame_end,
                                  substeps_per_frame,
                                  tracks,
                                  reports,
                                  writer,
                                  r_baked_action))
  {
    physics_bake_action_discard(bmain, writer);
    return false;
  }
  return true;
}

wmOperatorStatus physics_bake_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: another bake is already running");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: stop real-time physics first");
    return OPERATOR_CANCELLED;
  }
  Object *armature = resolve_operator_armature(C, op, op->reports);
  Scene *scene = CTX_data_scene(C);
  Main *bmain = CTX_data_main(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  if (armature == nullptr || armature->type != OB_ARMATURE || armature->pose == nullptr ||
      scene == nullptr || bmain == nullptr || depsgraph == nullptr)
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: invalid target Armature or scene");
    return OPERATOR_CANCELLED;
  }
  AnimData *anim_data = BKE_animdata_from_id(&armature->id);
  if (anim_data == nullptr || anim_data->action == nullptr ||
      anim_data->slot_handle == animrig::Slot::unassigned)
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: Armature requires an active Action");
    return OPERATOR_CANCELLED;
  }
  bAction *source_action = anim_data->action;
  const animrig::slot_handle_t source_slot_handle = anim_data->slot_handle;
  if (physics_action_is_baked(*source_action)) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics Bake: the active Action is already an MMD physics bake; "
               "select the pristine source Action first");
    return OPERATOR_CANCELLED;
  }
  const int bake_substeps_per_frame = scene_bake_substeps_per_frame(scene);
  int frame_start = RNA_int_get(op->ptr, "frame_start");
  int frame_end = RNA_int_get(op->ptr, "frame_end");
  if (frame_start == 0 && frame_end == 0) {
    frame_start = scene->r.sfra;
    frame_end = scene->r.efra;
  }
  if (frame_end < frame_start) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: end frame is before start frame");
    return OPERATOR_CANCELLED;
  }
  const float original_frame = float(scene->r.cfra) + scene->r.subframe;
  std::vector<PhysicsBakePoseBone> original_pose;
  physics_bake_capture_pose(*armature, original_pose);
  auto evaluate_frame = [&](const float frame) {
    BKE_scene_frame_set(scene, frame);
    DEG_id_tag_update(&armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    BKE_scene_graph_update_for_newframe(depsgraph);
  };
  evaluate_frame(frame_start);

  const float fixed_timestep = float(scene_seconds_per_frame(scene) /
                                     double(bake_substeps_per_frame));
  std::unique_ptr<MMDPhysicsRuntimeSession> session = create_runtime_session(
      C,
      *armature,
      int(std::round(1.0f / fixed_timestep)),
      op->reports,
      nullptr,
      kDefaultDynamicConstraintIterations);
  if (session == nullptr) {
    evaluate_frame(original_frame);
    physics_bake_restore_pose(*armature, original_pose);
    return OPERATOR_CANCELLED;
  }
  MMDPhysicsWorld *world = session->world.get();
  prepare_world_for_simulation(*session);
  world->apply_dynamic_to_pose();
  world->flush_depsgraph(depsgraph);

  std::vector<PhysicsBakeTrack> tracks;
  tracks.reserve(session->binding.dynamic_bone_names.size() +
                 session->binding.dynamic_merge_bone_names.size());
  for (const std::string &bone_name : physics_bake_bone_names(*session)) {
    if (BKE_pose_channel_find_name(armature->pose, bone_name.c_str()) == nullptr) {
      continue;
    }
    PhysicsBakeTrack track;
    track.bone_name = bone_name;
    track.full_transform = session->binding.dynamic_bone_names.contains(bone_name);
    track.location_path = physics_bone_path(bone_name, "location");
    track.rotation_path = physics_bone_path(bone_name, "rotation_quaternion");
    track.rotation_euler_path = physics_bone_path(bone_name, "rotation_euler");
    track.rotation_axis_angle_path = physics_bone_path(bone_name, "rotation_axis_angle");
    track.scale_path = physics_bone_path(bone_name, "scale");
    const int frame_count = frame_end - frame_start + 1;
    track.locations.reserve(frame_count);
    track.rotations.reserve(frame_count);
    track.scales.reserve(frame_count);
    tracks.push_back(std::move(track));
  }
  if (tracks.empty()) {
    destroy_session(*session, bmain);
    evaluate_frame(original_frame);
    physics_bake_restore_pose(*armature, original_pose);
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: no physics-driven bones found");
    return OPERATOR_CANCELLED;
  }

  capture_physics_bake_frame(*armature, tracks);
  const float seconds_per_frame = float(scene->r.frs_sec_base) /
                                  float(std::max(1, int(scene->r.frs_sec)));
  const int fixed_steps_per_frame = bake_substeps_per_frame;
  append_compare_trace(*session,
                       "MMD_BAKE_COMPARE_TRACE",
                       frame_start,
                       0.0,
                       0.0,
                       0,
                       0);
  uint64_t bake_total_fixed_steps = 0;
  for (int frame = frame_start + 1; frame <= frame_end; frame++) {
    evaluate_frame(frame);
    world->step_full(fixed_timestep * float(fixed_steps_per_frame),
                     1,
                     false,
                     fixed_steps_per_frame);
    world->apply_dynamic_to_pose();
    world->flush_depsgraph(depsgraph);
    capture_physics_bake_frame(*armature, tracks);
    bake_total_fixed_steps += uint64_t(fixed_steps_per_frame);
    append_compare_trace(*session,
                         "MMD_BAKE_COMPARE_TRACE",
                         frame,
                         seconds_per_frame,
                         0.0,
                         fixed_steps_per_frame,
                         bake_total_fixed_steps);
  }

  destroy_session(*session, bmain);
  evaluate_frame(original_frame);
  bAction *baked_action = nullptr;
  if (!write_physics_bake_action(*bmain,
                                 *armature,
                                 *source_action,
                                 source_slot_handle,
                                 frame_start,
                                 frame_end,
                                 bake_substeps_per_frame,
                                 tracks,
                                 op->reports,
                                 baked_action))
  {
    physics_bake_restore_pose(*armature, original_pose);
    return OPERATOR_CANCELLED;
  }
  BKE_scene_graph_update_for_newframe(depsgraph);
  WM_event_add_notifier(C, NC_ANIMATION | ND_NLA_ACTCHANGE, armature);
  WM_event_add_notifier(C, NC_OBJECT | ND_POSE, armature);
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: baked %d bones over frames %d-%d to Action '%s'",
              int(tracks.size()),
              frame_start,
              frame_end,
              baked_action->id.name + 2);
  return OPERATOR_FINISHED;
}

enum class PhysicsBakeModalPhase : uint8_t {
  Sampling,
  Writing,
};

struct PhysicsBakeModalData {
  wmTimer *timer = nullptr;
  wmWindow *window = nullptr;
  std::unique_ptr<MMDPhysicsRuntimeSession> session;
  Object *armature = nullptr;
  Scene *scene = nullptr;
  Main *bmain = nullptr;
  Depsgraph *depsgraph = nullptr;
  bAction *source_action = nullptr;
  uint64_t armature_session_uid = 0;
  uint64_t scene_session_uid = 0;
  uint64_t source_action_session_uid = 0;
  animrig::slot_handle_t source_slot_handle = animrig::Slot::unassigned;
  float original_frame = 0.0f;
  bool scene_evaluated = false;
  bool pose_captured = false;
  int frame_start = 0;
  int frame_end = 0;
  int next_frame = 0;
  int write_track_index = 0;
  uint64_t bake_total_fixed_steps = 0;
  int fixed_steps_per_frame = 1;
  float fixed_timestep = kDefaultFixedTimestep;
  double seconds_per_frame = kDefaultTimerInterval;
  PhysicsBakeModalPhase phase = PhysicsBakeModalPhase::Sampling;
  std::vector<PhysicsBakeTrack> tracks;
  std::vector<PhysicsBakePoseBone> original_pose;
  std::vector<PhysicsBakePoseBone> sampling_resume_pose;
  PhysicsBakeActionWriter writer;
  bool cancel_requested = false;
};

void physics_bake_evaluate_frame(PhysicsBakeModalData &data, const float frame)
{
  BKE_scene_frame_set(data.scene, frame);
  DEG_id_tag_update(&data.armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
  BKE_scene_graph_update_for_newframe(data.depsgraph);
}

float physics_bake_progress(const PhysicsBakeModalData &data)
{
  const int frame_count = std::max(1, data.frame_end - data.frame_start + 1);
  if (data.phase == PhysicsBakeModalPhase::Sampling) {
    const int sampled = std::clamp(data.next_frame - data.frame_start, 0, frame_count);
    return 0.7f * float(sampled) / float(frame_count);
  }
  const int track_count = std::max(1, int(data.tracks.size()));
  return 0.7f + 0.3f * float(std::clamp(data.write_track_index, 0, track_count)) /
                         float(track_count);
}

void physics_bake_update_progress(const PhysicsBakeModalData &data)
{
  if (data.window == nullptr) {
    return;
  }
  const float progress = physics_bake_progress(data);
  WM_progress_set(data.window, progress);
}

bool physics_bake_refresh_context(bContext *C, PhysicsBakeModalData &data)
{
  Main *bmain = CTX_data_main(C);
  if (bmain == nullptr) {
    return false;
  }
  Object *armature = reinterpret_cast<Object *>(
      BKE_libblock_find_session_uid(bmain, ID_OB, data.armature_session_uid));
  Scene *scene = reinterpret_cast<Scene *>(
      BKE_libblock_find_session_uid(bmain, ID_SCE, data.scene_session_uid));
  bAction *source_action = reinterpret_cast<bAction *>(
      BKE_libblock_find_session_uid(bmain, ID_AC, data.source_action_session_uid));
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  data.armature = armature;
  data.scene = scene;
  data.source_action = source_action;
  data.depsgraph = depsgraph;
  data.bmain = bmain;
  if (data.session != nullptr) {
    data.session->armature = armature;
    data.session->bmain = bmain;
  }
  if (data.writer.baked_action_session_uid != 0) {
    data.writer.baked_action = reinterpret_cast<bAction *>(BKE_libblock_find_session_uid(
        bmain, ID_AC, data.writer.baked_action_session_uid));
    if (data.writer.baked_action == nullptr) {
      data.writer.baked_action_session_uid = 0;
    }
  }
  if (armature == nullptr || scene == nullptr || source_action == nullptr || depsgraph == nullptr ||
      CTX_data_scene(C) != scene)
  {
    data.armature = armature;
    data.scene = scene;
    data.source_action = source_action;
    data.depsgraph = depsgraph;
    return false;
  }
  AnimData *anim_data = BKE_animdata_from_id(&armature->id);
  if (anim_data == nullptr || anim_data->action != source_action ||
      anim_data->slot_handle != data.source_slot_handle)
  {
    return false;
  }
  if (data.writer.baked_action_session_uid == 0 && data.phase == PhysicsBakeModalPhase::Writing) {
    return false;
  }
  return true;
}

void physics_bake_restore_state(bContext *C, PhysicsBakeModalData &data, const bool restore_pose)
{
  Main *bmain = CTX_data_main(C);
  if (data.session != nullptr && bmain != nullptr) {
    destroy_session(*data.session, bmain);
    data.session.reset();
  }
  /* Initialization can fail before Bake evaluates any frame (for example when
   * the requested range is invalid). Re-evaluating the scene in that path is
   * not a restore and can perturb constrained pose channels. */
  if (!data.scene_evaluated) {
    if (restore_pose && data.pose_captured && data.armature != nullptr) {
      physics_bake_restore_pose(*data.armature, data.original_pose, false);
    }
    return;
  }
  physics_bake_refresh_context(C, data);
  if (data.scene != nullptr) {
    if (data.armature != nullptr && data.depsgraph != nullptr && CTX_data_scene(C) == data.scene) {
      physics_bake_evaluate_frame(data, data.original_frame);
    }
    else {
      /* The original Scene still owns its frame even when the window switched Scenes. */
      BKE_scene_frame_set(data.scene, data.original_frame);
    }
  }
  if (restore_pose && data.armature != nullptr) {
    /* The cleanup notifier handles redraw; another depsgraph evaluation here
     * can overwrite the exact snapshot for constrained pose channels. */
    physics_bake_restore_pose(*data.armature, data.original_pose, false);
  }
}

void physics_bake_cleanup(bContext *C, wmOperator *op, PhysicsBakeModalData *data, const bool success)
{
  if (data == nullptr) {
    return;
  }
  if (data->timer != nullptr) {
    WM_event_timer_remove(CTX_wm_manager(C), data->window, data->timer);
    data->timer = nullptr;
  }
  physics_bake_restore_state(C, *data, !success);
  if (!success && data->bmain != nullptr) {
    physics_bake_action_discard(*data->bmain, data->writer);
  }
  if (data->window != nullptr) {
    WM_progress_clear(data->window);
  }
  if (data->armature != nullptr) {
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, data->armature);
  }
  if (g_physics_bake == data) {
    g_physics_bake = nullptr;
  }
  MEM_delete(data);
  op->customdata = nullptr;
}

bool physics_bake_modal_initialize(bContext *C, wmOperator *op, PhysicsBakeModalData &data)
{
  data.armature = resolve_operator_armature(C, op, op->reports);
  data.scene = CTX_data_scene(C);
  data.bmain = CTX_data_main(C);
  data.depsgraph = CTX_data_depsgraph_pointer(C);
  data.window = CTX_wm_window(C);
  if (data.armature == nullptr || data.armature->type != OB_ARMATURE ||
      data.armature->pose == nullptr || data.scene == nullptr || data.bmain == nullptr ||
      data.depsgraph == nullptr || data.window == nullptr)
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: invalid Armature or scene");
    return false;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: stop real-time physics first");
    return false;
  }
  AnimData *anim_data = BKE_animdata_from_id(&data.armature->id);
  if (anim_data == nullptr || anim_data->action == nullptr ||
      anim_data->slot_handle == animrig::Slot::unassigned)
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: Armature requires an active Action");
    return false;
  }
  data.source_action = anim_data->action;
  data.source_slot_handle = anim_data->slot_handle;
  if (physics_action_is_baked(*data.source_action)) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics Bake: the active Action is already an MMD physics bake; "
               "select the pristine source Action first");
    return false;
  }
  data.original_frame = float(data.scene->r.cfra) + data.scene->r.subframe;
  physics_bake_capture_pose(*data.armature, data.original_pose);
  data.pose_captured = true;
  data.armature_session_uid = data.armature->id.session_uid;
  data.scene_session_uid = data.scene->id.session_uid;
  data.source_action_session_uid = data.source_action->id.session_uid;
  data.frame_start = RNA_int_get(op->ptr, "frame_start");
  data.frame_end = RNA_int_get(op->ptr, "frame_end");
  if (data.frame_start == 0 && data.frame_end == 0) {
    data.frame_start = data.scene->r.sfra;
    data.frame_end = data.scene->r.efra;
  }
  if (data.frame_end < data.frame_start) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: end frame is before start frame");
    return false;
  }
  data.seconds_per_frame = double(data.scene->r.frs_sec_base) /
                            double(std::max(1, int(data.scene->r.frs_sec)));
  data.fixed_steps_per_frame = scene_bake_substeps_per_frame(data.scene);
  data.fixed_timestep = float(data.seconds_per_frame / double(data.fixed_steps_per_frame));

  data.scene_evaluated = true;
  physics_bake_evaluate_frame(data, data.frame_start);
  data.session = create_runtime_session(
      C,
      *data.armature,
      int(std::round(1.0f / data.fixed_timestep)),
      op->reports,
      nullptr,
      kDefaultDynamicConstraintIterations);
  if (data.session == nullptr) {
    physics_bake_evaluate_frame(data, data.original_frame);
    physics_bake_restore_pose(*data.armature, data.original_pose);
    return false;
  }
  MMDPhysicsWorld *world = data.session->world.get();
  prepare_world_for_simulation(*data.session);
  world->apply_dynamic_to_pose();
  world->flush_depsgraph(data.depsgraph);

  data.tracks.reserve(data.session->binding.dynamic_bone_names.size() +
                      data.session->binding.dynamic_merge_bone_names.size());
  for (const std::string &bone_name : physics_bake_bone_names(*data.session)) {
    if (BKE_pose_channel_find_name(data.armature->pose, bone_name.c_str()) == nullptr) {
      continue;
    }
    PhysicsBakeTrack track;
    track.bone_name = bone_name;
    track.full_transform = data.session->binding.dynamic_bone_names.contains(bone_name);
    track.location_path = physics_bone_path(bone_name, "location");
    track.rotation_path = physics_bone_path(bone_name, "rotation_quaternion");
    track.rotation_euler_path = physics_bone_path(bone_name, "rotation_euler");
    track.rotation_axis_angle_path = physics_bone_path(bone_name, "rotation_axis_angle");
    track.scale_path = physics_bone_path(bone_name, "scale");
    const int frame_count = data.frame_end - data.frame_start + 1;
    track.locations.reserve(frame_count);
    track.rotations.reserve(frame_count);
    track.scales.reserve(frame_count);
    data.tracks.push_back(std::move(track));
  }
  if (data.tracks.empty()) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: no physics-driven bones found");
    return false;
  }
  capture_physics_bake_frame(*data.armature, data.tracks);
  physics_bake_capture_pose(*data.armature, data.sampling_resume_pose);
  data.next_frame = data.frame_start + 1;
  append_compare_trace(*data.session,
                       "MMD_BAKE_COMPARE_TRACE",
                       data.frame_start,
                       0.0,
                       0.0,
                       0,
                       0);
  return true;
}

bool physics_bake_begin_writing(bContext *C, PhysicsBakeModalData &data, ReportList *reports)
{
  physics_bake_restore_state(C, data, false);
  if (!physics_bake_refresh_context(C, data)) {
    BKE_report(reports, RPT_ERROR, "MMD Physics Bake: source data changed during bake");
    return false;
  }
  if (!physics_bake_action_begin(*data.bmain,
                                 *data.source_action,
                                 data.source_slot_handle,
                                 data.tracks,
                                 reports,
                                 data.writer))
  {
    return false;
  }
  data.phase = PhysicsBakeModalPhase::Writing;
  data.write_track_index = 0;
  return true;
}

wmOperatorStatus physics_bake_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: another bake is already running");
    return OPERATOR_CANCELLED;
  }
  PhysicsBakeModalData *data = MEM_new<PhysicsBakeModalData>("MMD Physics Bake Modal");
  if (!physics_bake_modal_initialize(C, op, *data)) {
    physics_bake_restore_state(C, *data, true);
    MEM_delete(data);
    return OPERATOR_CANCELLED;
  }
  wmWindowManager *wm = CTX_wm_manager(C);
  data->timer = WM_event_timer_add(wm, data->window, TIMER, 0.01f);
  if (data->timer == nullptr) {
    physics_bake_restore_state(C, *data, true);
    MEM_delete(data);
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: failed to create progress timer");
    return OPERATOR_CANCELLED;
  }
  g_physics_bake = data;
  op->customdata = data;
  WM_progress_set(data->window, physics_bake_progress(*data));
  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

wmOperatorStatus physics_bake_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  PhysicsBakeModalData *data = static_cast<PhysicsBakeModalData *>(op->customdata);
  if (data == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (data->cancel_requested ||
      (event->type == EVT_ESCKEY && event->val == KM_PRESS))
  {
    physics_bake_cleanup(C, op, data, false);
    BKE_report(op->reports, RPT_INFO, "MMD Physics Bake: cancelled");
    return OPERATOR_CANCELLED;
  }
  if (event->type != TIMER || event->customdata != data->timer) {
    return OPERATOR_PASS_THROUGH;
  }
  if (!physics_bake_refresh_context(C, *data) ||
      (data->phase == PhysicsBakeModalPhase::Sampling &&
       (data->session == nullptr || data->session->world == nullptr ||
        !data->session->world->is_binding_valid(data->bmain))))
  {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics Bake: source data changed; bake cancelled");
    physics_bake_cleanup(C, op, data, false);
    return OPERATOR_CANCELLED;
  }

  if (data->phase == PhysicsBakeModalPhase::Sampling) {
    if (data->next_frame <= data->frame_end) {
      physics_bake_restore_pose(*data->armature, data->sampling_resume_pose);
      physics_bake_evaluate_frame(*data, data->next_frame);
      MMDPhysicsWorld *world = data->session->world.get();
      world->step_full(data->fixed_timestep * float(data->fixed_steps_per_frame),
                       1,
                       false,
                       data->fixed_steps_per_frame);
      world->apply_dynamic_to_pose();
      world->flush_depsgraph(data->depsgraph);
      capture_physics_bake_frame(*data->armature, data->tracks);
      physics_bake_capture_pose(*data->armature, data->sampling_resume_pose);
      data->bake_total_fixed_steps += uint64_t(data->fixed_steps_per_frame);
       append_compare_trace(*data->session,
                           "MMD_BAKE_COMPARE_TRACE",
                           data->next_frame,
                           data->seconds_per_frame,
                           0.0,
                           data->fixed_steps_per_frame,
                           data->bake_total_fixed_steps);
      data->next_frame++;
      physics_bake_update_progress(*data);
      return OPERATOR_RUNNING_MODAL;
    }
    if (!physics_bake_begin_writing(C, *data, op->reports)) {
      physics_bake_cleanup(C, op, data, false);
      return OPERATOR_CANCELLED;
    }
    physics_bake_update_progress(*data);
    return OPERATOR_RUNNING_MODAL;
  }

  if (data->write_track_index < int(data->tracks.size())) {
    if (!physics_bake_action_write_track(*data->bmain,
                                         data->tracks[data->write_track_index],
                                         data->frame_start,
                                         op->reports,
                                         data->writer))
    {
      physics_bake_cleanup(C, op, data, false);
      return OPERATOR_CANCELLED;
    }
    data->write_track_index++;
    physics_bake_update_progress(*data);
    return OPERATOR_RUNNING_MODAL;
  }

  bAction *baked_action = nullptr;
  if (!physics_bake_action_finish(*data->armature,
                                  *data->source_action,
                                  data->frame_start,
                                  data->frame_end,
                                  data->fixed_steps_per_frame,
                                  data->tracks,
                                  op->reports,
                                  data->writer,
                                  baked_action))
  {
    physics_bake_cleanup(C, op, data, false);
    return OPERATOR_CANCELLED;
  }
  BKE_scene_graph_update_for_newframe(data->depsgraph);
  WM_event_add_notifier(C, NC_ANIMATION | ND_NLA_ACTCHANGE, data->armature);
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: baked %d bones over frames %d-%d to Action '%s'",
              int(data->tracks.size()),
              data->frame_start,
              data->frame_end,
              baked_action->id.name + 2);
  physics_bake_cleanup(C, op, data, true);
  return OPERATOR_FINISHED;
}

void physics_bake_cancel(bContext *C, wmOperator *op)
{
  PhysicsBakeModalData *data = static_cast<PhysicsBakeModalData *>(op->customdata);
  if (data != nullptr) {
    physics_bake_cleanup(C, op, data, false);
  }
}

wmOperatorStatus physics_bake_cancel_exec(bContext * /*C*/, wmOperator * /*op*/)
{
  if (g_physics_bake == nullptr) {
    return OPERATOR_CANCELLED;
  }
  g_physics_bake->cancel_requested = true;
  return OPERATOR_FINISHED;
}

bool physics_bake_cancel_poll(bContext * /*C*/)
{
  return g_physics_bake != nullptr;
}

wmOperatorStatus physics_capture_diagnostics_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: wait for the active bake to finish");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler != nullptr && !g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics: real-time simulation is running; click Stop before capture");
    return OPERATOR_CANCELLED;
  }

  Object *armature = CTX_data_active_object(C);
  Main *bmain = CTX_data_main(C);
  bool ambiguous_collection = false;
  Collection *model_collection = find_physics_collection_for_armature(
      bmain, armature, ambiguous_collection);
  if (model_collection == nullptr) {
    BKE_report(op->reports,
               RPT_ERROR,
               ambiguous_collection ?
                   "MMD Physics: armature belongs to multiple collections with physics data" :
                   "MMD Physics: no PMX physics definition found for this armature's collection");
    return OPERATOR_CANCELLED;
  }

  MMDPhysicsDefinition definition;
  if (!mmd_physics::deserialize_physics_definition(*model_collection, definition, op->reports)) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: failed to deserialize physics definition");
    return OPERATOR_CANCELLED;
  }

  mmd_physics::MMDDiagnosticCaptureOptions options;
  options.startup_prewarm_steps = RNA_int_get(op->ptr, "startup_prewarm_steps");
  options.startup_sync_steps = RNA_int_get(op->ptr, "startup_sync_steps");
  options.capture_initial_state = RNA_boolean_get(op->ptr, "capture_initial_state");
  options.disable_rigid_body_contacts = RNA_boolean_get(op->ptr,
                                                        "disable_rigid_body_contacts");
  options.disable_joint_springs = RNA_boolean_get(op->ptr, "disable_joint_springs");
  options.joint_spring_damping = RNA_float_get(op->ptr, "joint_spring_damping");
  options.joint_collision_exclusion_depth = RNA_int_get(
      op->ptr, "joint_collision_exclusion_depth");

  /* Mesh vertex penetration sampling: parse comma-separated bone names.
   * Empty string disables mesh sampling entirely. */
  char bones_buffer[1024];
  RNA_string_get(op->ptr, "mesh_sample_bones", bones_buffer);
  if (bones_buffer[0] != '\0') {
    std::string s(bones_buffer);
    size_t start = 0;
    while (start <= s.size()) {
      size_t end = s.find(',', start);
      if (end == std::string::npos) {
        end = s.size();
      }
      std::string bone = s.substr(start, end - start);
      /* Trim whitespace. */
      const size_t b = bone.find_first_not_of(" \t");
      if (b != std::string::npos) {
        const size_t e = bone.find_last_not_of(" \t");
        options.mesh_sample_bones.push_back(bone.substr(b, e - b + 1));
      }
      start = end + 1;
    }
  }
  options.mesh_sample_interval = RNA_int_get(op->ptr, "mesh_sample_interval");
  options.mesh_max_vertices_per_bone = RNA_int_get(op->ptr, "mesh_max_vertices_per_bone");

  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  const mmd_physics::MMDDiagnosticCaptureResult result =
      mmd_physics::capture_mmd_physics_diagnostics(
          definition, armature, bmain, depsgraph, options);
  if (!result.success) {
    BKE_report(op->reports, RPT_ERROR, result.error.c_str());
    return OPERATOR_CANCELLED;
  }

  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: captured %d steps (%d bodies, %d joints), summary: %s",
              result.captured_steps,
              result.body_count,
              result.joint_count,
              result.summary_path.c_str());
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_export_definition_exec(bContext *C, wmOperator *op)
{
  Object *armature = CTX_data_active_object(C);
  Main *bmain = CTX_data_main(C);
  if (armature == nullptr || armature->type != OB_ARMATURE || bmain == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: select an armature object first");
    return OPERATOR_CANCELLED;
  }
  bool ambiguous_collection = false;
  Collection *model_collection = find_physics_collection_for_armature(
      bmain, armature, ambiguous_collection);
  if (model_collection == nullptr) {
    BKE_report(op->reports,
               RPT_ERROR,
               ambiguous_collection ?
                   "MMD Physics: armature belongs to multiple collections with physics data" :
                   "MMD Physics: no PMX physics definition found for this armature's collection");
    return OPERATOR_CANCELLED;
  }
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (filepath[0] == '\0') {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: definition export filepath is empty");
    return OPERATOR_CANCELLED;
  }
  MMDPhysicsDefinition definition;
  if (!mmd_physics::deserialize_physics_definition(*model_collection, definition, op->reports) ||
      !mmd_physics::write_physics_definition_json(definition, filepath, op->reports))
  {
    return OPERATOR_CANCELLED;
  }
  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: exported definition (%d bodies, %d joints): %s",
              int(definition.rigid_bodies.size()),
              int(definition.joints.size()),
              filepath);
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_snapshot_diagnostics_exec(bContext *C, wmOperator *op)
{
  if (!running_world_binding_is_valid(C)) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: no valid active world to snapshot");
    return OPERATOR_CANCELLED;
  }

  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (filepath[0] == '\0') {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: snapshot filepath is empty");
    return OPERATOR_CANCELLED;
  }

  mmd_physics::MMDDiagnosticFrame frame;
  const Scene *scene = CTX_data_scene(C);
  const int step = scene != nullptr ? scene->r.cfra : 0;
  MMDPhysicsRuntimeSession *session = active_runtime_session(C);
  if (session == nullptr || session->world == nullptr ||
      !session->world->capture_diagnostic_frame(step, frame)) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: failed to capture runtime diagnostic snapshot");
    return OPERATOR_CANCELLED;
  }
  frame.runtime_timer_elapsed = session->last_timer_elapsed;
  frame.runtime_accumulator = g_physics_scheduler != nullptr ?
                                  g_physics_scheduler->time_accumulator :
                                  0.0;
  frame.runtime_fixed_steps = session->last_fixed_steps;
  frame.runtime_total_fixed_steps = session->total_fixed_steps;
  frame.runtime_writeback = session->last_writeback;
  if (!mmd_physics::append_mmd_physics_diagnostic_frame(filepath, frame)) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: failed to write runtime diagnostic snapshot");
    return OPERATOR_CANCELLED;
  }

  BKE_reportf(op->reports,
              RPT_INFO,
              "MMD Physics: recorded runtime snapshot (frame=%d, contacts=%d)",
              step,
              int(frame.contacts.size()));
  return OPERATOR_FINISHED;
}

wmOperatorStatus physics_stop_exec(bContext *C, wmOperator *op)
{
  if (g_physics_bake != nullptr) {
    BKE_report(op->reports, RPT_ERROR, "MMD Physics: use Cancel Bake for the active Action bake");
    return OPERATOR_CANCELLED;
  }
  if (g_physics_scheduler == nullptr || g_physics_scheduler->sessions.empty()) {
    BKE_report(op->reports, RPT_WARNING, "MMD Physics: no active world to stop");
    return OPERATOR_CANCELLED;
  }
  if (!scheduler_bindings_are_valid(C)) {
    remove_timer(C);
    destroy_world(CTX_data_main(C));
    g_physics_scheduler.reset();
    if (wmWindow *window = CTX_wm_window(C)) {
      WM_event_add_mousemove(window);
    }
    BKE_report(op->reports,
               RPT_WARNING,
               "MMD Physics: discarded sessions invalidated by Undo or data changes");
    return OPERATOR_FINISHED;
  }
  Object *target_armature = resolve_operator_armature(C, op, op->reports);
  MMDPhysicsRuntimeSession *session = runtime_session_for_armature(C, target_armature);
  if (session == nullptr) {
    BKE_report(op->reports,
               RPT_ERROR,
               "MMD Physics: target armature has no running physics session");
    return OPERATOR_CANCELLED;
  }
  const uint64_t armature_session_uid = session->binding.armature_session_uid;
  const uint64_t debug_session = session->debug_session;
  Object *armature = session->armature;
  /* Stop only the active model. Other sessions continue under the same
   * Scene scheduler. */
  if (armature != nullptr) {
    log_physics_pose_absolute("stop_pre_destroy",
                              *armature,
                              session->binding.physics_bone_names,
                              debug_session);
    log_tracked_bone_z("stop_pre_destroy", *armature, debug_session);
  }
  remove_runtime_session(C, armature_session_uid);
  if (armature != nullptr) {
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
    Main *bmain = CTX_data_main(C);
    if (depsgraph != nullptr && bmain != nullptr) {
      /* Force animation re-evaluation after restoring muted Action FCurves.
       * Without ID_RECALC_ANIMATION the depsgraph considers the animation
       * up-to-date (mute flags changed but the depsgraph doesn't track that
       * as an animation-affecting change), so pchan->pose_mat stays at the
       * physics-written value. The next Start then snaps rigid bodies to
       * that stale pose, producing the "drop on second Start" symptom.
       * Mirrors physics_start_cancel's restore path. */
      DEG_id_tag_update(&armature->id, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      BKE_scene_graph_update_for_newframe(depsgraph);
      /* Probe AFTER re-evaluation: shows whether Stop actually restored the
       * VMD pose. Compare this against the next session's post_ensure_world
       * to confirm whether the next Start begins from a correct pose. */
       log_tracked_bone_z("stop_post_restore", *armature, debug_session);
    }
    WM_event_add_notifier(C, NC_OBJECT | ND_POSE, armature);
  }
  if (wmWindow *window = CTX_wm_window(C)) {
    WM_event_add_mousemove(window);
  }
  BKE_report(op->reports, RPT_INFO, "MMD Physics: stopped active model physics session");
  return OPERATOR_FINISHED;
}

/* ----------------------------------------------------------------- */
/* N-panel (sidebar) UI.                                             */
/* ----------------------------------------------------------------- */

bool mmd_physics_panel_poll(const bContext *C, PanelType * /*pt*/)
{
  /* Only show the panel when the active object is an armature, since
   * physics operators are armature-bound. */
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE;
}

bool armature_active_action_is_baked(Object &armature)
{
  AnimData *anim_data = BKE_animdata_from_id(&armature.id);
  return anim_data != nullptr && anim_data->action != nullptr &&
         physics_action_is_baked(*anim_data->action);
}

void mmd_physics_panel_draw(const bContext *C, Panel *panel)
{
  ui::Layout &layout = *panel->layout;
  Scene *scene = CTX_data_scene(C);
  Main *bmain = CTX_data_main(C);
  const MMDPhysicsPanelLanguage language = scene_mmd_physics_panel_language(scene);
  const MMDPhysicsPanelText &text = mmd_physics_panel_text(language);

  const char *language_name = mmd_physics_panel_language_items[int(language)].name;
  layout.op_menu_enum(C,
                      "WM_OT_mmd_physics_set_panel_language",
                      "language",
                      std::string(text.language) + ": " + language_name,
                      ICON_WORLD);
  layout.separator();

  MMDPhysicsRuntimeSession *active_session = active_runtime_session(const_cast<bContext *>(C));
  const bool runtime_running = g_physics_scheduler != nullptr &&
                               !g_physics_scheduler->sessions.empty();
  const double scene_fps = 1.0 / scene_seconds_per_frame(scene);
  std::vector<Object *> model_armatures;
  if (bmain != nullptr) {
    for (Object *armature = static_cast<Object *>(bmain->objects.first); armature != nullptr;
         armature = reinterpret_cast<Object *>(armature->id.next))
    {
      if (armature->type != OB_ARMATURE) {
        continue;
      }
      if (BKE_scene_object_find_by_name(*bmain, scene, armature->id.name + 2) != armature) {
        continue;
      }
      bool ambiguous_collection = false;
      if (find_physics_collection_for_armature(bmain, armature, ambiguous_collection) != nullptr &&
          !ambiguous_collection)
      {
        model_armatures.push_back(armature);
      }
    }
  }

  if (ui::Layout *realtime = layout.panel(
          C, "mmd_physics_realtime", false, text.realtime_preview))
  {
    realtime->active_set(g_physics_bake == nullptr);
    realtime->label(active_session != nullptr ? text.status_running : text.status_stopped,
                    active_session != nullptr ? ICON_PLAY : ICON_PAUSE);

    char timing_label[64];
    SNPRINTF(timing_label, text.fixed_step_format, scene_fps);
    realtime->label(timing_label, ICON_NONE);

    ui::Layout &hz_row = realtime->row(true);
    hz_row.active_set(!runtime_running);
    const int current_substeps = g_physics_scheduler != nullptr ?
                                     g_physics_scheduler->realtime_substeps_per_frame :
                                     scene_realtime_substeps_per_frame(scene);
    for (const int substeps : {3, 4, 5, 6}) {
      char label[16];
      SNPRINTF(label, "%d Hz", int(std::round(scene_fps * double(substeps))));
      PointerRNA props = hz_row.op("WM_OT_mmd_physics_set_realtime_hz",
                                   label,
                                   substeps == current_substeps ? ICON_CHECKMARK : ICON_NONE);
      RNA_int_set(&props, "substeps_per_frame", substeps);
    }

    realtime->label(text.dynamic_constraint_iterations, ICON_NONE);
    const int current_dynamic_constraint_iterations =
        scene_realtime_dynamic_constraint_iterations(scene);
    ui::Layout &constraint_iterations_row = realtime->row(true);
    constraint_iterations_row.active_set(!runtime_running);
    for (const int iterations : {20, 40, 80, 100, 128}) {
      char label[16];
      SNPRINTF(label, "%d", iterations);
      PointerRNA props = constraint_iterations_row.op(
          "WM_OT_mmd_physics_set_dynamic_constraint_iterations",
          label,
          iterations == current_dynamic_constraint_iterations ? ICON_CHECKMARK : ICON_NONE);
      RNA_int_set(&props, "iterations", iterations);
    }

    ui::Layout &step_row = realtime->row(true);
    step_row.op("WM_OT_mmd_physics_step", text.step, ICON_FRAME_NEXT);
    ui::Layout &reset = step_row.row(true);
    reset.active_set(active_session != nullptr);
    reset.op("WM_OT_mmd_physics_reset", text.reset, ICON_LOOP_BACK);
  }

  if (ui::Layout *models = layout.panel(C, "mmd_physics_models", false, text.models)) {
    models->active_set(g_physics_bake == nullptr);
    for (Object *armature : model_armatures) {
      ui::Layout &model = models->column(true);
      model.label(armature->id.name + 2, ICON_ARMATURE_DATA);
      MMDPhysicsRuntimeSession *model_session = runtime_session_for_armature(
          const_cast<bContext *>(C), armature);
      if (model_session != nullptr) {
        model.label(text.status_running, ICON_PLAY);
        PointerRNA model_props = model.op("WM_OT_mmd_physics_stop", text.stop, ICON_PAUSE);
        RNA_string_set(&model_props, "armature_name", armature->id.name + 2);
      }
      else if (armature_active_action_is_baked(*armature)) {
        model.label(text.status_baked, ICON_ACTION);
        PointerRNA model_props = model.op(
            "WM_OT_mmd_physics_use_bake_source", text.use_source, ICON_LOOP_BACK);
        RNA_string_set(&model_props, "armature_name", armature->id.name + 2);
      }
      else {
        model.label(text.status_stopped, ICON_PAUSE);
        PointerRNA model_props = model.op("WM_OT_mmd_physics_start", text.start, ICON_PLAY);
        RNA_string_set(&model_props, "armature_name", armature->id.name + 2);
      }
    }
    if (model_armatures.empty()) {
      if (active_session == nullptr) {
        models->op("WM_OT_mmd_physics_start", text.start, ICON_PLAY);
      }
      else {
        models->op("WM_OT_mmd_physics_stop", text.stop, ICON_PAUSE);
      }
    }
  }

  if (ui::Layout *bake = layout.panel(C, "mmd_physics_bake", true, text.action_bake)) {
    if (g_physics_bake != nullptr) {
      const char *progress_text = g_physics_bake->phase == PhysicsBakeModalPhase::Sampling ?
                                      text.simulating_frames :
                                      text.writing_action;
      bake->progress_indicator(
          progress_text, physics_bake_progress(*g_physics_bake), ui::ButProgressType::Bar);
      bake->op("WM_OT_mmd_physics_bake_cancel", text.cancel_bake, ICON_CANCEL);
    }
    else {
      PointerRNA scene_ptr = RNA_id_pointer_create(&scene->id);
      ui::Layout &range = bake->column(true);
      range.prop(&scene_ptr, "frame_start", UI_ITEM_NONE, text.range_start, ICON_NONE);
      range.prop(&scene_ptr, "frame_end", UI_ITEM_NONE, text.range_end, ICON_NONE);

      const int bake_substeps = scene_bake_substeps_per_frame(scene);
      bake->label(text.simulation_quality, ICON_NONE);
      ui::Layout &quality_row = bake->row(true);
      for (const int substeps : {4, 5, 6}) {
        char label[20];
        SNPRINTF(label, text.steps_format, substeps);
        PointerRNA props = quality_row.op("WM_OT_mmd_physics_set_bake_quality",
                                          label,
                                          substeps == bake_substeps ? ICON_CHECKMARK : ICON_NONE);
        RNA_int_set(&props, "substeps_per_frame", substeps);
      }
      char quality_hint[192];
      SNPRINTF(quality_hint,
               text.quality_hint_format,
               bake_substeps,
               scene_fps * double(bake_substeps));
      bake->label(quality_hint, ICON_INFO);

      for (Object *armature : model_armatures) {
        ui::Layout &bake_model = bake->column(true);
        bake_model.active_set(!runtime_running);
        bake_model.label(armature->id.name + 2, ICON_ARMATURE_DATA);
        if (armature_active_action_is_baked(*armature)) {
          bake_model.label(text.status_baked, ICON_ACTION);
          PointerRNA source_props = bake_model.op(
              "WM_OT_mmd_physics_use_bake_source", text.use_source, ICON_LOOP_BACK);
          RNA_string_set(&source_props, "armature_name", armature->id.name + 2);
        }
        else {
          PointerRNA bake_props = bake_model.op(
              "WM_OT_mmd_physics_bake", text.bake_action, ICON_ACTION);
          RNA_string_set(&bake_props, "armature_name", armature->id.name + 2);
          RNA_int_set(&bake_props, "frame_start", scene->r.sfra);
          RNA_int_set(&bake_props, "frame_end", scene->r.efra);
        }
      }
      if (model_armatures.empty()) {
        ui::Layout &bake_row = bake->row(false);
        bake_row.active_set(!runtime_running);
        PointerRNA bake_props = bake_row.op(
            "WM_OT_mmd_physics_bake", text.bake_action, ICON_ACTION);
        RNA_int_set(&bake_props, "frame_start", scene->r.sfra);
        RNA_int_set(&bake_props, "frame_end", scene->r.efra);
      }
    }
  }

}

}  // namespace

void WM_OT_mmd_physics_start(wmOperatorType *ot)
{
  ot->name = "MMD Physics Start";
  ot->description = "Initialize the MMD physics world, prewarm, and run real-time simulation";
  ot->idname = "WM_OT_mmd_physics_start";
  ot->invoke = physics_start_invoke;
  ot->modal = physics_start_modal;
  ot->cancel = physics_start_cancel;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna,
                 "armature_name",
                 nullptr,
                 MAX_ID_NAME,
                 "Armature",
                 "Target Armature object; empty uses the active armature");
}

void WM_OT_mmd_physics_use_bake_source(wmOperatorType *ot)
{
  ot->name = "Use MMD Physics Source Action";
  ot->description = "Switch from a baked MMD physics Action back to its preserved source Action";
  ot->idname = "WM_OT_mmd_physics_use_bake_source";
  ot->exec = physics_bake_use_source_action_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna,
                 "armature_name",
                 nullptr,
                 MAX_ID_NAME,
                 "Armature",
                 "Target Armature object; empty uses the active armature");
}

void WM_OT_mmd_physics_set_realtime_hz(wmOperatorType *ot)
{
  ot->name = "Set MMD Physics Realtime Hz";
  ot->description = "Set fixed physics substeps per animation frame for real-time preview";
  ot->idname = "WM_OT_mmd_physics_set_realtime_hz";
  ot->exec = physics_set_realtime_hz_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna,
                 "armature_name",
                 nullptr,
                 MAX_ID_NAME,
                 "Armature",
                 "Target Armature object; empty uses the active armature");

  RNA_def_int(ot->srna,
              "substeps_per_frame",
              kDefaultRealtimeSubstepsPerFrame,
              1,
              10,
              "Substeps per Frame",
              "Fixed physics steps evaluated for each animation frame",
              1,
              10);
}

void WM_OT_mmd_physics_set_dynamic_constraint_iterations(wmOperatorType *ot)
{
  ot->name = "Set MMD Physics Dynamic Constraint Iterations";
  ot->description = "Set the solver iterations used by dynamic-dynamic MMD constraints";
  ot->idname = "WM_OT_mmd_physics_set_dynamic_constraint_iterations";
  ot->exec = physics_set_dynamic_constraint_iterations_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "iterations",
              kDefaultDynamicConstraintIterations,
              1,
              kMaxRealtimeDynamicConstraintIterations,
              "Dynamic Constraint Iterations",
              "Bullet iterations applied to dynamic-dynamic constraints in real-time preview",
              20,
              kMaxRealtimeDynamicConstraintIterations);
}

void WM_OT_mmd_physics_set_panel_language(wmOperatorType *ot)
{
  ot->name = "Set MMD Physics Panel Language";
  ot->description = "Set the display language used by the MMD Physics panel";
  ot->idname = "WM_OT_mmd_physics_set_panel_language";
  ot->exec = physics_set_panel_language_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  ot->prop = RNA_def_enum(ot->srna,
                          "language",
                          mmd_physics_panel_language_items,
                          int(MMDPhysicsPanelLanguage::Chinese),
                          "Language",
                          "MMD Physics panel display language");
}

void WM_OT_mmd_physics_set_bake_quality(wmOperatorType *ot)
{
  ot->name = "Set MMD Physics Bake Quality";
  ot->description = "Set fixed physics substeps per animation frame for Action Bake";
  ot->idname = "WM_OT_mmd_physics_set_bake_quality";
  ot->exec = physics_set_bake_quality_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna,
              "substeps_per_frame",
              kDefaultBakeSubstepsPerFrame,
              1,
              10,
              "Substeps per Frame",
              "Fixed physics steps evaluated for each baked animation frame; higher values are "
              "smoother and more stable but take longer",
              1,
              10);
}

void WM_OT_mmd_physics_step(wmOperatorType *ot)
{
  ot->name = "MMD Physics Step";
  ot->description = "Advance the MMD physics world by one frame and apply results to bones";
  ot->idname = "WM_OT_mmd_physics_step";
  ot->exec = physics_step_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_UNDO;
}

void WM_OT_mmd_physics_reset(wmOperatorType *ot)
{
  ot->name = "MMD Physics Reset";
  ot->description = "Snap all MMD rigid bodies back to their initial transforms";
  ot->idname = "WM_OT_mmd_physics_reset";
  ot->exec = physics_reset_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_UNDO;
}

void WM_OT_mmd_physics_bake(wmOperatorType *ot)
{
  ot->name = "MMD Physics Bake Action";
  ot->description = "Bake deterministic MMD physics results to a duplicate of the active Action";
  ot->idname = "WM_OT_mmd_physics_bake";
  ot->invoke = physics_bake_invoke;
  ot->modal = physics_bake_modal;
  ot->cancel = physics_bake_cancel;
  ot->exec = physics_bake_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(ot->srna,
                 "armature_name",
                 nullptr,
                 MAX_ID_NAME,
                 "Armature",
                 "Target Armature object; empty uses the active armature");

  RNA_def_int(ot->srna,
              "frame_start",
              0,
              MINAFRAME,
              MAXFRAME,
              "Start Frame",
              "First frame to bake (0 with End Frame 0 uses the scene range)",
              MINAFRAME,
              MAXFRAME);
  RNA_def_int(ot->srna,
              "frame_end",
              0,
              MINAFRAME,
              MAXFRAME,
              "End Frame",
              "Last frame to bake (0 with Start Frame 0 uses the scene range)",
              MINAFRAME,
              MAXFRAME);
}

void WM_OT_mmd_physics_bake_cancel(wmOperatorType *ot)
{
  ot->name = "Cancel MMD Physics Bake";
  ot->description = "Cancel the active MMD physics Action bake and restore the original frame";
  ot->idname = "WM_OT_mmd_physics_bake_cancel";
  ot->exec = physics_bake_cancel_exec;
  ot->poll = physics_bake_cancel_poll;
  ot->flag = OPTYPE_INTERNAL;
}

void WM_OT_mmd_physics_capture_diagnostics(wmOperatorType *ot)
{
  ot->name = "MMD Physics Capture Diagnostics";
  ot->description = "Record objective rigid-body and joint data from an isolated fixed-step simulation";
  ot->idname = "WM_OT_mmd_physics_capture_diagnostics";
  ot->exec = physics_capture_diagnostics_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER;

  RNA_def_boolean(ot->srna,
                  "disable_rigid_body_contacts",
                  false,
                  "Disable Rigid Body Contacts",
                   "Exclude all rigid-body contacts while preserving gravity, integration, and joints");
  RNA_def_int(ot->srna,
              "startup_prewarm_steps",
              2,
              0,
              16,
              "Startup Prewarm Steps",
              "Number of 1/30 second startup steps before fixed-step recording",
              0,
              16);
  RNA_def_int(ot->srna,
              "startup_sync_steps",
              0,
              0,
              300,
              "Startup Sync Steps",
              "Gradually interpolate kinematic bodies from rest pose to VMD pose over N frames before prewarm",
              0,
              300);
  RNA_def_boolean(ot->srna,
                  "capture_initial_state",
                  false,
                  "Capture Initial State",
                  "Write a step -1 record after temporal initialization and prewarm, before the first fixed step");
  RNA_def_boolean(ot->srna,
                  "disable_joint_springs",
                  false,
                  "Disable Joint Springs",
                  "Disable joint springs while preserving limits, contacts, gravity, and integration");
  RNA_def_float(ot->srna,
                "joint_spring_damping",
                0.15f,
                0.0f,
                1.0f,
                "Joint Spring Damping",
                "Bullet 2.82 spring damping parameter (1 means no damping, 0 means spring fully damped)",
                0.0f,
                1.0f);
  RNA_def_int(ot->srna,
              "joint_collision_exclusion_depth",
              0,
              0,
              8,
              "Joint Collision Exclusion Depth",
              "Ignore initially penetrating pairs along simple chains (0 scans the full chain)",
              0,
              8);
  RNA_def_string(ot->srna,
                 "mesh_sample_bones",
                 nullptr,
                 1024,
                 "Mesh Sample Bones",
                 "Comma-separated bone names whose vertices will be sampled for rigid-body "
                 "penetration depth (empty disables mesh sampling)");
  RNA_def_int(ot->srna,
              "mesh_sample_interval",
              30,
              1,
              240,
              "Mesh Sample Interval",
              "Sample mesh vertices every N steps (1 = every step)",
              1,
              240);
  RNA_def_int(ot->srna,
              "mesh_max_vertices_per_bone",
              16,
              1,
              256,
              "Max Vertices Per Bone",
              "Maximum vertices sampled per bone (uniform stride selection)",
              1,
              256);
}

void WM_OT_mmd_physics_export_definition(wmOperatorType *ot)
{
  ot->name = "MMD Physics Export Definition";
  ot->description = "Export the persisted PMX physics definition without starting simulation";
  ot->idname = "WM_OT_mmd_physics_export_definition";
  ot->exec = physics_export_definition_exec;
  ot->poll = poll_armature;
  ot->flag = OPTYPE_REGISTER;

  RNA_def_string(ot->srna,
                 "filepath",
                 nullptr,
                 FILE_MAX,
                 "File Path",
                 "JSON file to write the persisted PMX physics definition");
}

void WM_OT_mmd_physics_snapshot_diagnostics(wmOperatorType *ot)
{
  ot->name = "MMD Physics Runtime Snapshot";
  ot->description = "Append objective state from the active physics world without advancing it";
  ot->idname = "WM_OT_mmd_physics_snapshot_diagnostics";
  ot->exec = physics_snapshot_diagnostics_exec;
  ot->poll = poll_running_physics;
  ot->flag = OPTYPE_REGISTER;

  RNA_def_string(ot->srna,
                 "filepath",
                 nullptr,
                 FILE_MAX,
                 "File Path",
                 "JSONL file to append with the current active-world state");
}

void WM_OT_mmd_physics_stop(wmOperatorType *ot)
{
  ot->name = "MMD Physics Stop";
  ot->description = "Destroy the MMD physics world and release all Bullet resources";
  ot->idname = "WM_OT_mmd_physics_stop";
  ot->exec = physics_stop_exec;
  ot->poll = poll_running_physics;
  ot->flag = OPTYPE_UNDO;

  RNA_def_string(ot->srna,
                 "armature_name",
                 nullptr,
                 MAX_ID_NAME,
                 "Armature",
                 "Target Armature object; empty uses the active armature");
}

void ED_mmd_physics_panel_register(ARegionType *art)
{
  if (art == nullptr) {
    return;
  }
  PanelType *pt = MEM_new_zeroed<PanelType>("spacetype view3d panel mmd physics");
  STRNCPY_UTF8(pt->idname, "VIEW3D_PT_mmd_physics");
  STRNCPY_UTF8(pt->label, N_("MMD Physics"));
  STRNCPY_UTF8(pt->category, "MMD");
  STRNCPY_UTF8(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  pt->draw = mmd_physics_panel_draw;
  pt->poll = mmd_physics_panel_poll;
  BLI_addtail(&art->paneltypes, pt);
}

}  // namespace blender

