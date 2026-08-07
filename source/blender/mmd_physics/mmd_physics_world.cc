/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup mmd_physics
 *
 * F1 implementation: builds a Bullet `btDiscreteDynamicsWorld` from an
 * `MMDPhysicsDefinition` and exposes minimal step / reset / debug accessors.
 *
 * Coordinate-system policy (important):
 *   The definition, Bullet world, diagnostics, and bone synchronization all
 *   use Blender's Z-up coordinate frame. PMX axis and rotation conversion is
 *   performed once while building `MMDPhysicsDefinition`.
 */

#include "mmd_physics_world.hh"

#include "BKE_armature.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_scene.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"

#include "DEG_depsgraph.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_constraint_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "ED_armature.hh"

#include "btBulletDynamicsCommon.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

namespace blender::mmd_physics {

namespace {

/* PMX physics type constants (mirror `pmx_types.h`). */
constexpr uint8_t kPhysicsStatic = 0;
constexpr uint8_t kPhysicsDynamic = 1;
constexpr uint8_t kPhysicsDynamicBone = 2;

/* PMX shape-type constants. */
constexpr uint8_t kShapeSphere = 0;
constexpr uint8_t kShapeBox = 1;
constexpr uint8_t kShapeCapsule = 2;

constexpr btScalar kMinDynamicMass = btScalar(0.0001);

bool is_dynamic_type(uint8_t physics_type)
{
  return physics_type == kPhysicsDynamic || physics_type == kPhysicsDynamicBone;
}

class InitialPenetrationCallback : public btCollisionWorld::ContactResultCallback {
 public:
  btScalar addSingleResult(btManifoldPoint &point,
                           const btCollisionObjectWrapper * /*object_a*/,
                           int /*part_id_a*/,
                           int /*index_a*/,
                           const btCollisionObjectWrapper * /*object_b*/,
                           int /*part_id_b*/,
                           int /*index_b*/) override
  {
    has_penetration |= point.getDistance() < btScalar(0.0);
    return btScalar(0.0);
  }

  bool has_penetration = false;
};

/* ------------------------------------------------------------------- */
/* Blender matrix <-> Bullet transform conversion.                     */
/* ------------------------------------------------------------------- */

btTransform blender_m4_to_bullet_transform(const float matrix[4][4])
{
  float location[3];
  float quaternion[4];
  mat4_to_loc_quat(location, quaternion, matrix);
  normalize_qt(quaternion);

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(btScalar(location[0]), btScalar(location[1]), btScalar(location[2])));
  transform.setRotation(btQuaternion(btScalar(quaternion[1]),
                                     btScalar(quaternion[2]),
                                     btScalar(quaternion[3]),
                                     btScalar(quaternion[0])));
  return transform;
}

void bullet_transform_to_blender_m4(const btTransform &transform, float r_matrix[4][4])
{
  const btQuaternion rotation = transform.getRotation().normalized();
  const float quaternion[4] = {
      float(rotation.w()), float(rotation.x()), float(rotation.y()), float(rotation.z())};
  quat_to_mat4(r_matrix, quaternion);
  const btVector3 &origin = transform.getOrigin();
  r_matrix[3][0] = float(origin.x());
  r_matrix[3][1] = float(origin.y());
  r_matrix[3][2] = float(origin.z());
}

/* Look up a pose channel by bone name. Returns nullptr if not found or the
 * armature has no pose. */
bPoseChannel *find_pose_channel(Object *armature, const std::string &bone_name)
{
  if (armature == nullptr || armature->pose == nullptr || bone_name.empty()) {
    return nullptr;
  }
  return static_cast<bPoseChannel *>(
      BLI_findstring(&armature->pose->chanbase, bone_name.c_str(), offsetof(bPoseChannel, name)));
}

/* Count ancestors in the armature hierarchy (root bones have depth 0). */
int bone_depth(const bPoseChannel *pchan)
{
  int depth = 0;
  for (const bPoseChannel *p = pchan->parent; p != nullptr; p = p->parent) {
    depth++;
  }
  return depth;
}

}  // namespace

/* Blender/mmd_tools YXZ Euler (radians) -> Bullet quaternion. */
namespace {

btQuaternion euler_yxz_to_quaternion(const std::array<float, 3> &euler)
{
  const btQuaternion qx(btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0)),
                        btScalar(euler[0]));
  const btQuaternion qy(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)),
                        btScalar(euler[1]));
  const btQuaternion qz(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)),
                        btScalar(euler[2]));
  return qz * qx * qy;
}

btTransform make_blender_transform(const std::array<float, 3> &position,
                                   const std::array<float, 3> &rotation)
{
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(
      btVector3(btScalar(position[0]), btScalar(position[1]), btScalar(position[2])));
  transform.setRotation(euler_yxz_to_quaternion(rotation));
  return transform;
}

}  // namespace

MMDPhysicsWorld::~MMDPhysicsWorld()
{
  destroy(false);
}

bool MMDPhysicsWorld::initialize(const MMDPhysicsDefinition &def,
                                 Object *armature,
                                 Main *bmain,
                                 const float gravity[3],
                                 const int solver_iterations,
                                 const int fixed_step_hz,
                                 const int max_substeps,
                                 const bool disable_rigid_body_contacts,
                                 const bool disable_joint_springs,
                                 const float joint_spring_damping,
                                 const int joint_collision_exclusion_depth)
{
  if (initialized_) {
    destroy(false);
  }

  if (!def.validation.valid || gravity == nullptr || !std::isfinite(gravity[0]) ||
      !std::isfinite(gravity[1]) || !std::isfinite(gravity[2]))
  {
    return false;
  }

  std::copy(gravity, gravity + 3, gravity_);
  armature_ = armature;
  bmain_ = bmain;
  pose_at_initialize_ = armature != nullptr ? armature->pose : nullptr;
  armature_session_uid_ = armature != nullptr ? armature->id.session_uid : 0;
  solver_iterations_ = std::max(1, std::min(solver_iterations, 128));
  fixed_step_hz_ = std::max(1, fixed_step_hz);
  fixed_timestep_ = 1.0 / double(fixed_step_hz_);
  max_substeps_ = std::max(1, max_substeps);
  disable_rigid_body_contacts_ = disable_rigid_body_contacts;
  disable_joint_springs_ = disable_joint_springs;
  joint_spring_damping_ = std::clamp(joint_spring_damping, 0.0f, 1.0f);
  joint_collision_exclusion_depth_ = std::clamp(joint_collision_exclusion_depth, 0, 8);

  create_world_();
  if (!dynamics_world_) {
    return false;
  }

  /* Rigid bodies: created in PMX-index order so that `body_runtimes_[i]`
   * corresponds to PMX rigid index `i` (matches joint references). */
  body_runtimes_.reserve(def.rigid_bodies.size());
  for (const MMDRigidBodyDefinition &rigid_def : def.rigid_bodies) {
    btCollisionShape *shape = create_shape_(rigid_def);
    if (shape == nullptr) {
      destroy(false);
      return false;
    }
    btRigidBody *body = create_rigid_body_(rigid_def, shape);
    if (body == nullptr) {
      destroy(false);
      return false;
    }

    RigidBodyRuntime runtime;
    runtime.body = body;
    runtime.motion_state = static_cast<btDefaultMotionState *>(body->getMotionState());
    runtime.shape = shape;
    runtime.pmx_index = rigid_def.pmx_index;
    runtime.physics_type = rigid_def.physics_type;
    runtime.collision_group_index = rigid_def.collision_group;
    runtime.no_collision_group = rigid_def.no_collision_group;
    runtime.collision_group = uint16_t(1u << rigid_def.collision_group);
    /* MMP compatibility: mmd_tools stores an inverted BoolVector and MMP
     * inverts it again, so the PMX disable mask reaches Bullet as its enable
     * mask. This is not PMX-spec semantics, but preserves body-clothing
     * contacts relied on by existing models. */
    runtime.collision_mask = disable_rigid_body_contacts_ ?
                                 uint16_t(0) :
                                 uint16_t(rigid_def.no_collision_group);
    runtime.name_local = rigid_def.name_local;
    runtime.blender_bone_name = rigid_def.blender_bone_name;
    runtime.initial_transform = body->getWorldTransform();
    runtime.mass = rigid_def.mass;
    if (rigid_def.shape_type == kShapeSphere) {
      /* MMP measures the display object's bound-box diagonal. mmd_tools uses
       * an 8x5 UV sphere whose widest latitude is cos(pi/10), not an analytic
       * sphere reaching the radius on all three axes. */
      constexpr float uv_sphere_lateral_extent = 0.9510565163f;
      runtime.binding_range = 2.0f * rigid_def.shape_size[0] *
                              std::sqrt(1.0f + 2.0f * uv_sphere_lateral_extent *
                                                   uv_sphere_lateral_extent);
    }
    else if (rigid_def.shape_type == kShapeBox) {
      runtime.binding_range = 2.0f * std::sqrt(rigid_def.shape_size[0] * rigid_def.shape_size[0] +
                                               rigid_def.shape_size[1] * rigid_def.shape_size[1] +
                                               rigid_def.shape_size[2] * rigid_def.shape_size[2]);
    }
    else {
      const float radius = rigid_def.shape_size[0];
      const float half_extent_z = rigid_def.shape_size[1] * 0.5f + radius;
      runtime.binding_range = 2.0f * std::sqrt(2.0f * radius * radius +
                                               half_extent_z * half_extent_z);
    }
    body_runtimes_.append(std::move(runtime));
  }

  /* Joints MUST be created while bodies are still at their PMX rest transforms
   * (the `initial_transform` captured during `create_rigid_body_`). This makes
   * `setEquilibriumPoint()` inside `create_joint_` anchor the spring
   * equilibrium to the PMX rest-pose relative transform — which for locked
   * rotation joints (lo==hi==0) is ~0, matching the limits [0,0].
   *
   * Previously `snap_body_to_bone_pose_` ran BEFORE joint creation, so at a
   * non-start VMD frame the equilibrium was anchored to the VMD-pose relative
   * transform (non-zero), conflicting with the [0,0] angular limits. The limit
   * solver's constraint impulse stayed near zero while the spring pulled toward
   * the non-zero equilibrium, and gravity torque accumulated → "drop on
   * non-start frame" symptom.
   *
   * MMP alignment: MMP's `physics_world.py:321-327` uses
   * `_rest_body_matrices` (PMX rest pose) as `initial_matrices` when
   * `startup_sync_steps > 0` (DEFAULT preset has prewarm_steps=5), so joints
   * are created at rest pose there too. `snap_body_to_bone_pose_` below then
   * moves bodies to the current VMD pose WITHOUT changing the already-stored
   * equilibrium — mirroring MMP's `temporal_kinematic_init(initial_matrices)`. */
  build_bone_offset_cache_();
  joint_runtimes_.reserve(def.joints.size());
  for (const MMDJointDefinition &joint_def : def.joints) {
    btGeneric6DofSpringConstraint *constraint = create_joint_(joint_def);
    if (constraint == nullptr) {
      continue;  /* Skip malformed joints rather than abort the world. */
    }
    JointRuntime runtime;
    runtime.constraint = constraint;
    runtime.pmx_index = joint_def.pmx_index;
    runtime.rigid_a = joint_def.rigid_a_index;
    runtime.rigid_b = joint_def.rigid_b_index;
    runtime.name_local = joint_def.name_local;
    runtime.translation_min = joint_def.translation_min;
    runtime.translation_max = joint_def.translation_max;
    runtime.rotation_min = joint_def.rotation_min;
    runtime.rotation_max = joint_def.rotation_max;
    /* A joint whose linear limits are all zero is effectively a pure
     * rotational joint; used by F3's locked-translation pullback. */
    runtime.locked_translation = (joint_def.translation_min[0] == 0.0f &&
                                  joint_def.translation_min[1] == 0.0f &&
                                  joint_def.translation_min[2] == 0.0f &&
                                  joint_def.translation_max[0] == 0.0f &&
                                  joint_def.translation_max[1] == 0.0f &&
                                  joint_def.translation_max[2] == 0.0f);
    joint_runtimes_.append(std::move(runtime));
  }

  /* NOW snap bone-bound bodies to the current VMD pose. This moves bodies to
   * their evaluated pose for the upcoming prewarm/real-time steps, but does
   * NOT re-derive joint frames or equilibrium — those were fixed above. */
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    snap_body_to_bone_pose_(i);
  }

  /* Performance counters. */
  int dynamic_count = 0;
  int static_count = 0;
  for (const RigidBodyRuntime &rt : body_runtimes_) {
    if (is_dynamic_type(rt.physics_type)) {
      dynamic_count++;
    }
    else {
      static_count++;
    }
  }
  performance_.body_count = int(body_runtimes_.size());
  performance_.dynamic_body_count = dynamic_count;
  performance_.static_body_count = static_count;
  performance_.joint_count = int(joint_runtimes_.size());

  build_joint_adjacency_();
  apply_joint_collision_exclusions_();
  apply_mmd_tools_ncc_();

  initialized_ = true;
  return true;
}

void MMDPhysicsWorld::create_world_()
{
  collision_configuration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collision_configuration_);
  broadphase_ = new btDbvtBroadphase();
  solver_ = new btSequentialImpulseConstraintSolver();
  dynamics_world_ = new btDiscreteDynamicsWorld(
      dispatcher_, broadphase_, solver_, collision_configuration_);

  dynamics_world_->setGravity(
      btVector3(btScalar(gravity_[0]), btScalar(gravity_[1]), btScalar(gravity_[2])));
  btContactSolverInfo &solver_info = dynamics_world_->getSolverInfo();
  solver_info.m_numIterations = solver_iterations_;
  solver_info.m_solverMode |= SOLVER_USE_WARMSTARTING;
  if (std::getenv("MMD_RT_RANDOMIZE_SOLVER_ORDER") != nullptr) {
    solver_info.m_solverMode |= SOLVER_RANDMIZE_ORDER;
  }
  if (const char *global_cfm = std::getenv("MMD_RT_GLOBAL_CFM")) {
    solver_info.m_globalCfm = btScalar(std::strtod(global_cfm, nullptr));
  }
  /* Residual early-out terminates the whole solver loop before constraints
   * with a larger per-constraint iteration override finish converging. */
  solver_info.m_leastSquaresResidualThreshold = btScalar(0.0);
  /* F5j fix: disabled split impulse and CCD to match MikuMikuPhysics.
   *
   * Split impulse (`m_splitImpulse=true`) separates position correction
   * from the velocity solver. For JOINT CONSTRAINTS this is harmful:
   * the linear constraint's position error is corrected by the split
   * impulse pass, but the velocity is NOT synchronously damped. The
   * body retains the drift velocity, so a pendulum at rest gains
   * ~1.87 rad/s in one 1/30s step from a 1.7mm linear constraint drift
   * (gravity torque on the 4.66cm lever arm). This is the root cause
   * of the cape/pendant "加速抖动" symptom.
   *
   * Without split impulse (MMP's configuration), Bullet uses Baumgarte
   * stabilization: position correction is applied THROUGH velocity, so
   * the velocity is properly damped and the pendulum stays at rest.
   *
   * CCD (`m_useContinuous=true`) is also disabled because MMP doesn't
   * use it, and it can subdivide timesteps unpredictably for thin
   * capsule shapes, adding noise to joint chains.
   *
   * The penetration these were meant to fix ("paper-like" skirt) was
   * previously masked by the `m_maxLimitForce=0` override letting
   * bodies spin freely to escape collision forces. With that override
   * removed, the collision mask fix (F5i) handles penetration; split
   * impulse / CCD are no longer needed and actively cause jitter. */
}

btCollisionShape *MMDPhysicsWorld::create_shape_(const MMDRigidBodyDefinition &def)
{
  btCollisionShape *shape = nullptr;
  switch (def.shape_type) {
    case kShapeSphere:
      shape = new btSphereShape(btScalar(def.shape_size[0]));
      break;
    case kShapeBox:
      shape = new btBoxShape(btVector3(btScalar(def.shape_size[0]),
                                       btScalar(def.shape_size[1]),
                                       btScalar(def.shape_size[2])));
      break;
    case kShapeCapsule:
      /* mmd_tools creates capsules along local Z, matching `btCapsuleShapeZ`.
       * shape_size[0] is
       * radius, shape_size[1] is height (cylinder part, exclusive of caps). */
      shape = new btCapsuleShapeZ(btScalar(def.shape_size[0]),
                                  btScalar(def.shape_size[1]));
      break;
    default:
      return nullptr;
  }
  if (shape != nullptr) {
    owned_shapes_.append(shape);
  }
  return shape;
}

btRigidBody *MMDPhysicsWorld::create_rigid_body_(const MMDRigidBodyDefinition &def,
                                                 btCollisionShape *shape)
{
  const btTransform start_transform = make_blender_transform(def.position, def.rotation);
  const bool dynamic = is_dynamic_type(def.physics_type);
  float diagnostic_mass_floor = float(kMinDynamicMass);
  if (const char *mass_floor_env = std::getenv("MMD_RT_DYNAMIC_MASS_FLOOR")) {
    diagnostic_mass_floor = std::max(diagnostic_mass_floor,
                                     float(std::strtod(mass_floor_env, nullptr)));
  }
  const btScalar mass = dynamic ? btScalar(std::max(def.mass, diagnostic_mass_floor)) :
                                  btScalar(0.0);

  btVector3 local_inertia(0, 0, 0);
  if (dynamic) {
    shape->calculateLocalInertia(mass, local_inertia);
  }

  btDefaultMotionState *motion_state = new btDefaultMotionState(start_transform);
  btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state, shape, local_inertia);
  info.m_friction = btScalar(def.friction);
  info.m_restitution = btScalar(def.restitution);
  info.m_linearDamping = btScalar(def.linear_damping);
  info.m_angularDamping = btScalar(def.angular_damping);
  info.m_additionalDamping = true;

  btRigidBody *body = new btRigidBody(info);
  /* All bodies stay awake; kinematic bodies are driven externally in F2. */
  body->setActivationState(DISABLE_DEACTIVATION);
  if (dynamic) {
    body->setSleepingThresholds(btScalar(0.0), btScalar(0.0));
  }
  else {
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_KINEMATIC_OBJECT);
  }

  const short group = short(1u << def.collision_group);
  /* Match the mmd_tools/MMP compatibility mask stored in the runtime. */
  const short mask = disable_rigid_body_contacts_ ?
                         short(0) :
                         short(def.no_collision_group);
  dynamics_world_->addRigidBody(body, group, mask);

  /* Store PMX index for cross-referencing from Bullet back to PMX data. */
  body->setUserIndex(def.pmx_index);
  return body;
}

btGeneric6DofSpringConstraint *MMDPhysicsWorld::create_joint_(const MMDJointDefinition &def)
{
  if (std::getenv("MMD_RT_DISABLE_JOINT_515") != nullptr && def.pmx_index == 515) {
    fprintf(stderr, "[MMD Physics] Diagnostic: disabled PMX joint 515\n");
    return nullptr;
  }
  if (def.rigid_a_index < 0 || def.rigid_a_index >= int(body_runtimes_.size()) ||
      def.rigid_b_index < 0 || def.rigid_b_index >= int(body_runtimes_.size()))
  {
    return nullptr;
  }

  btRigidBody &body_a = *body_runtimes_[def.rigid_a_index].body;
  btRigidBody &body_b = *body_runtimes_[def.rigid_b_index].body;

  const btTransform joint_world = make_blender_transform(def.position, def.rotation);
  const btTransform frame_a = body_a.getWorldTransform().inverse() * joint_world;
  const btTransform frame_b = body_b.getWorldTransform().inverse() * joint_world;

  const bool use_linear_reference_frame_a = true;
  auto *constraint = new btGeneric6DofSpringConstraint(
      body_a, body_b, frame_a, frame_b, use_linear_reference_frame_a);

  constraint->setLinearLowerLimit(btVector3(btScalar(def.translation_min[0]),
                                            btScalar(def.translation_min[1]),
                                            btScalar(def.translation_min[2])));
  constraint->setLinearUpperLimit(btVector3(btScalar(def.translation_max[0]),
                                            btScalar(def.translation_max[1]),
                                            btScalar(def.translation_max[2])));
  constraint->setAngularLowerLimit(btVector3(btScalar(def.rotation_min[0]),
                                             btScalar(def.rotation_min[1]),
                                             btScalar(def.rotation_min[2])));
  constraint->setAngularUpperLimit(btVector3(btScalar(def.rotation_max[0]),
                                             btScalar(def.rotation_max[1]),
                                             btScalar(def.rotation_max[2])));

  /* F5i fix: removed the `m_maxLimitForce = 0` override for locked rotation
   * axes (lo == hi == 0). The previous F5h "fix" intended to prevent
   * sub-epsilon angular diffs from producing 300-N·m impulses, but it
   * actually disabled the limit solver entirely on locked axes — body B
   * was then free to rotate under gravity-induced torque (joint anchor
   * offset from center of mass), producing 6+ rad/s angular velocity in
   * one step and the accelerating jitter symptom.
   *
   * MikuMikuPhysics does NOT touch `m_maxLimitForce` at all (see
   * `apply_joint_quality` in pmx_bullet_api.cpp:218-247); it only adjusts
   * STOP_ERP/STOP_CFM. Bullet's default `m_maxLimitForce = 300` correctly
   * locks the axis when lo == hi == 0, because the limit solver only fires
   * when angleDiff is outside [lo, hi] — and at equilibrium angleDiff == 0
   * is inside [0, 0], so no impulse is applied. The 5+ rad/s "kick"
   * observed during F5h debugging was actually caused by the
   * `m_maxLimitForce = 0` override itself (chicken-and-egg), not by the
   * default 300 value.
   *
   * Verification on a locked accessory joint:
   *   BEFORE step=0: angleDiff=(0,0,0), velocity=0  (at equilibrium)
   *   AFTER  step=0: angleDiff[1]=0.175 rad, av=(6.19,-6.66,4.54) rad/s
   *   limit[1] (Y axis, lo=hi=0): maxForce=0  ← override was active
   * The 0.175 rad rotation in a single 1/120s step is only possible
   * because maxForce=0 disabled the Y-axis limit. */

  /* PMX does not expose spring damping. Bullet 2.82 defines this parameter
   * in [0, 1], where 1 means no damping. Keep it configurable so objective
   * diagnostics can isolate the effect without changing any other variable.
   * Bullet axis mapping: 0,1,2 = linear X/Y/Z; 3,4,5 = angular X/Y/Z. */
  if (!disable_joint_springs_) {
    for (int axis = 0; axis < 3; ++axis) {
      if (def.spring_translation[axis] > 0.0f) {
        constraint->enableSpring(axis, true);
        constraint->setStiffness(axis, btScalar(def.spring_translation[axis]));
        constraint->setDamping(axis, btScalar(joint_spring_damping_));
      }
      if (def.spring_rotation[axis] > 0.0f) {
        const int angular_axis = axis + 3;
        constraint->enableSpring(angular_axis, true);
        constraint->setStiffness(angular_axis, btScalar(def.spring_rotation[axis]));
        constraint->setDamping(angular_axis, btScalar(joint_spring_damping_));
      }
    }
  }

  int constraint_iterations = solver_iterations_;
  if (const char *override_iterations = std::getenv("MMD_RT_ALL_CONSTRAINT_ITERATIONS")) {
    constraint_iterations = std::clamp(std::atoi(override_iterations), 1, 500);
  }
  else if (const char *override_iterations = std::getenv("MMD_RT_DYNAMIC_CONSTRAINT_ITERATIONS")) {
    const bool dynamic_pair = is_dynamic_type(body_runtimes_[def.rigid_a_index].physics_type) &&
                              is_dynamic_type(body_runtimes_[def.rigid_b_index].physics_type);
    if (dynamic_pair) {
      constraint_iterations = std::clamp(std::atoi(override_iterations), 1, 500);
    }
  }
  constraint->setOverrideNumSolverIterations(constraint_iterations);

  /* Joint stop ERP/CFM must match MMP DEFAULT preset: 0.5/0.1 for ALL axes
   * (linear 0-2 and angular 3-5). Previously only locked-translation joints
   * had their LINEAR axes set (to 0.2/0.0002), and ANGULAR axes were left at
   * Bullet defaults (ERP=0.2, CFM=0). For joints with locked rotation
   * (lo==hi==0), the default ERP=0.2 was too soft: the limit solver produced
   * only ~0.005 N·s impulse, far too weak to counter gravity torque on
   * dynamic bodies — so hair/ribbon bodies gradually rotated away from
   * equilibrium and sagged, the "drop on non-start frame" symptom.
   *
   * MMP config.py: joint_stop_erp=0.5, joint_stop_cfm=0.1 (all zones).
   * Locked-translation joints override LINEAR axes to 0.2/0.0002 so the
   * pullback pass can correct anchor drift (project_memory rule). */
  for (int axis = 0; axis < 6; ++axis) {
    constraint->setParam(BT_CONSTRAINT_STOP_ERP, btScalar(0.5), axis);
    constraint->setParam(BT_CONSTRAINT_STOP_CFM, btScalar(0.1), axis);
  }

  constexpr float locked_epsilon = 1.0e-5f;
  bool locked_translation = true;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(def.translation_min[axis]) > locked_epsilon ||
        std::abs(def.translation_max[axis]) > locked_epsilon ||
        std::abs(def.translation_min[axis] - def.translation_max[axis]) > locked_epsilon)
    {
      locked_translation = false;
      break;
    }
  }
  if (locked_translation) {
    for (int axis = 0; axis < 3; ++axis) {
      constraint->setParam(BT_CONSTRAINT_STOP_ERP, btScalar(0.2), axis);
      constraint->setParam(BT_CONSTRAINT_STOP_CFM, btScalar(0.0002), axis);
    }
  }
  if (std::getenv("MMD_RT_SOFT_LOCKED_ANGULAR") != nullptr) {
    for (int axis = 0; axis < 3; ++axis) {
      if (def.rotation_min[axis] == 0.0f && def.rotation_max[axis] == 0.0f) {
        const int angular_axis = axis + 3;
        constraint->setParam(BT_CONSTRAINT_STOP_ERP, btScalar(0.2), angular_axis);
        constraint->setParam(BT_CONSTRAINT_STOP_CFM, btScalar(0.0002), angular_axis);
      }
    }
  }

  constraint->setEquilibriumPoint();
  constraint->enableFeedback(true);

  /* Match MMP's joint-pair decision after the mmd_tools mask inversion. */
  const RigidBodyRuntime &runtime_a = body_runtimes_[def.rigid_a_index];
  const RigidBodyRuntime &runtime_b = body_runtimes_[def.rigid_b_index];
  const uint16_t no_collision_mask_a = uint16_t(0xFFFFu & ~runtime_a.no_collision_group);
  const uint16_t no_collision_mask_b = uint16_t(0xFFFFu & ~runtime_b.no_collision_group);
  const bool disable_collisions =
      (no_collision_mask_a & runtime_b.collision_group) != 0 ||
      (no_collision_mask_b & runtime_a.collision_group) != 0;
  dynamics_world_->addConstraint(constraint, disable_collisions);
  return constraint;
}

void MMDPhysicsWorld::step(const float timestep,
                           int max_substeps,
                           const bool apply_results,
                           const float fixed_timestep_override)
{
  if (!initialized_ || dynamics_world_ == nullptr || !std::isfinite(timestep) ||
      timestep <= 0.0f)
  {
    return;
  }

  const int clamped_substeps = std::max(1, std::min(max_substeps, max_substeps_));
  /* 0 = use world's fixed_timestep (default). >0 = per-call override used by
   * step_full's smoothing path so sub_timestep becomes Bullet's fixedTimeStep. */
  const float fixed_timestep = (fixed_timestep_override > 0.0f) ? fixed_timestep_override :
                                                                   fixed_timestep_;
  if (!std::isfinite(fixed_timestep) || fixed_timestep <= 0.0f) {
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  const int actual_substeps = dynamics_world_->stepSimulation(
      btScalar(timestep), clamped_substeps, btScalar(fixed_timestep));

  const auto end = std::chrono::steady_clock::now();

  if (!defer_constraint_correction_) {
    /* F5g: correct anchor drift on locked-translation joints after each
     * stepSimulation. During a real-time catch-up batch this is deferred so
     * direct body corrections are not applied repeatedly within one Timer
     * event. */
    finalize_deferred_constraint_correction();
  }

  performance_.last_substeps = actual_substeps;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
  performance_.last_step_time_ms = elapsed_ms;
  performance_.accumulated_step_time_ms += elapsed_ms;

  (void)apply_results;  /* F1 has nothing to write back; F2 will use this. */
}

void MMDPhysicsWorld::set_defer_constraint_correction(const bool defer)
{
  defer_constraint_correction_ = defer;
}

void MMDPhysicsWorld::set_dynamic_constraint_iterations(const int iterations)
{
  const int clamped_iterations = std::clamp(iterations, 1, 500);
  int updated = 0;
  for (JointRuntime &joint : joint_runtimes_) {
    if (joint.constraint == nullptr || joint.rigid_a < 0 || joint.rigid_b < 0) {
      continue;
    }
    if (is_dynamic_type(body_runtimes_[joint.rigid_a].physics_type) &&
        is_dynamic_type(body_runtimes_[joint.rigid_b].physics_type))
    {
      joint.constraint->setOverrideNumSolverIterations(clamped_iterations);
      updated++;
    }
  }
  if (std::getenv("MMD_RT_TRACE_PLAYBACK_BOOST") != nullptr) {
    fprintf(stderr,
            "[MMD Physics Boost] set iterations=%d constraints=%d\n",
            clamped_iterations,
            updated);
    fflush(stderr);
  }
}

void MMDPhysicsWorld::finalize_deferred_constraint_correction()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return;
  }
  pullback_locked_joints_();
  refresh_broadphase();
}

void MMDPhysicsWorld::refresh_broadphase()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return;
  }
  /* Pullback directly moves dynamic bodies, so refresh broadphase state just
   * like MMP's `refresh_world_pairs` before the next solver step. */
  dynamics_world_->updateAabbs();
  dynamics_world_->getBroadphase()->calculateOverlappingPairs(
      dynamics_world_->getDispatcher());
}

void MMDPhysicsWorld::damp_correction_velocity_(btRigidBody &body, const btVector3 &correction)
{
  /* Project out the velocity component along the correction direction so
   * the pullback doesn't inject oscillation. Mirrors MikuMikuPhysics
   * `damp_correction_velocity` (pmx_bullet_api.cpp:599-609). */
  const btScalar length2 = correction.length2();
  if (length2 <= btScalar(1.0e-12)) {
    return;
  }
  const btVector3 normal = correction / btSqrt(length2);
  const btScalar along = body.getLinearVelocity().dot(normal);
  body.setLinearVelocity(body.getLinearVelocity() - normal * along);
}

void MMDPhysicsWorld::pullback_locked_joints_()
{
  /* Correct position drift on locked-translation joints (all 3 linear
   * limits ≈ 0). Bullet's sequential-impulse solver allows joint anchors
   * to separate under load; without this correction, skirt/hair chains
   * slowly pull apart and produce the "穿模" / jitter symptom. Runs 2
   * iterations with a 0.15 correction factor, skipping corrections above
   * 4m² (which would indicate a catastrophic state, not normal drift).
   * Mirrors MikuMikuPhysics `pullback_locked_joints`
   * (pmx_bullet_api.cpp:611-655). */
  if (!initialized_ || dynamics_world_ == nullptr) {
    return;
  }
  constexpr int kIterations = 2;
  constexpr btScalar kMaxError2 = btScalar(4.0);
  constexpr btScalar kMinError2 = btScalar(4.0e-6);
  constexpr btScalar kCorrectionFactor = btScalar(0.15);

  for (int iteration = 0; iteration < kIterations; ++iteration) {
    for (const JointRuntime &rt : joint_runtimes_) {
      if (!rt.locked_translation || rt.constraint == nullptr) {
        continue;
      }
      if (rt.rigid_a < 0 || rt.rigid_b < 0 ||
          rt.rigid_a >= int(body_runtimes_.size()) ||
          rt.rigid_b >= int(body_runtimes_.size()))
      {
        continue;
      }

      btRigidBody &body_a = *body_runtimes_[rt.rigid_a].body;
      btRigidBody &body_b = *body_runtimes_[rt.rigid_b].body;
      const btTransform anchor_a = body_a.getWorldTransform() * rt.constraint->getFrameOffsetA();
      const btTransform anchor_b = body_b.getWorldTransform() * rt.constraint->getFrameOffsetB();
      const btVector3 error = anchor_a.getOrigin() - anchor_b.getOrigin();
      const btScalar error2 = error.length2();
      if (error2 < kMinError2 || error2 > kMaxError2) {
        continue;
      }

      const btVector3 correction = error * kCorrectionFactor;
      const bool b_dynamic = is_dynamic_type(body_runtimes_[rt.rigid_b].physics_type);
      const bool a_dynamic = is_dynamic_type(body_runtimes_[rt.rigid_a].physics_type);

      if (b_dynamic) {
        btTransform transform = body_b.getWorldTransform();
        transform.setOrigin(transform.getOrigin() + correction);
        body_b.setWorldTransform(transform);
        body_b.setInterpolationWorldTransform(transform);
        if (body_b.getMotionState() != nullptr) {
          body_b.getMotionState()->setWorldTransform(transform);
        }
        damp_correction_velocity_(body_b, correction);
        body_b.activate(true);
      }
      else if (a_dynamic) {
        btTransform transform = body_a.getWorldTransform();
        transform.setOrigin(transform.getOrigin() - correction);
        body_a.setWorldTransform(transform);
        body_a.setInterpolationWorldTransform(transform);
        if (body_a.getMotionState() != nullptr) {
          body_a.getMotionState()->setWorldTransform(transform);
        }
        damp_correction_velocity_(body_a, -correction);
        body_a.activate(true);
      }
    }
  }
}

void MMDPhysicsWorld::reset()
{
  if (!initialized_) {
    return;
  }
  for (RigidBodyRuntime &rt : body_runtimes_) {
    btRigidBody &body = *rt.body;
    body.setWorldTransform(rt.initial_transform);
    body.setInterpolationWorldTransform(rt.initial_transform);
    if (body.getMotionState() != nullptr) {
      body.getMotionState()->setWorldTransform(rt.initial_transform);
    }
    body.clearForces();
    body.setLinearVelocity(btVector3(0, 0, 0));
    body.setAngularVelocity(btVector3(0, 0, 0));
    body.forceActivationState(ACTIVE_TAG);
    body.activate(true);
  }

  last_kinematic_transforms_.resize(body_runtimes_.size());
  prev_prev_kinematic_transforms_.resize(body_runtimes_.size());
  current_kinematic_targets_.resize(body_runtimes_.size());
  for (const int i : body_runtimes_.index_range()) {
    const btTransform transform = body_runtimes_[i].body->getWorldTransform();
    last_kinematic_transforms_[i] = transform;
    prev_prev_kinematic_transforms_[i] = transform;
    current_kinematic_targets_[i] = transform;
  }
  has_last_kinematic_ = true;
  has_prev_prev_kinematic_ = false;
}

void MMDPhysicsWorld::reset_to_current_pose()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return;
  }

  reset();
  for (const int i : body_runtimes_.index_range()) {
    snap_body_to_bone_pose_(i);
  }
  temporal_kinematic_init();

  for (const int i : body_runtimes_.index_range()) {
    const btTransform transform = body_runtimes_[i].body->getWorldTransform();
    last_kinematic_transforms_[i] = transform;
    prev_prev_kinematic_transforms_[i] = transform;
    current_kinematic_targets_[i] = transform;
  }
  has_last_kinematic_ = true;
  has_prev_prev_kinematic_ = false;
}

void MMDPhysicsWorld::destroy(const bool restore_initial)
{
  if (restore_initial && initialized_) {
    reset();
  }
  destroy_world_();
  initialized_ = false;
  performance_ = MMDPhysicsPerformance{};
}

void MMDPhysicsWorld::destroy_world_()
{
  if (dynamics_world_ != nullptr) {
    /* Constraints first (they reference bodies). */
    for (JointRuntime &rt : joint_runtimes_) {
      if (rt.constraint != nullptr) {
        dynamics_world_->removeConstraint(rt.constraint);
        delete rt.constraint;
        rt.constraint = nullptr;
      }
    }
    /* Then bodies (they reference shapes). */
    for (RigidBodyRuntime &rt : body_runtimes_) {
      if (rt.body != nullptr) {
        dynamics_world_->removeRigidBody(rt.body);
        delete rt.body;
        rt.body = nullptr;
      }
    }
    /* Motion states are owned by us, not by btRigidBody. */
    for (RigidBodyRuntime &rt : body_runtimes_) {
      delete rt.motion_state;
      rt.motion_state = nullptr;
    }
    /* Shapes. */
    for (btCollisionShape *shape : owned_shapes_) {
      delete shape;
    }
    owned_shapes_.clear();

    delete dynamics_world_;
    dynamics_world_ = nullptr;
    delete solver_;
    solver_ = nullptr;
    delete broadphase_;
    broadphase_ = nullptr;
    delete dispatcher_;
    dispatcher_ = nullptr;
    delete collision_configuration_;
    collision_configuration_ = nullptr;
  }

  body_runtimes_.clear();
  joint_runtimes_.clear();
  bone_offset_blender_.clear();
  is_bone_driver_.clear();
  bone_depth_.clear();
  bone_driver_index_.clear();
  joint_neighbors_.clear();
  last_kinematic_transforms_.clear();
  prev_prev_kinematic_transforms_.clear();
  current_kinematic_targets_.clear();
  has_last_kinematic_ = false;
  has_prev_prev_kinematic_ = false;
  disable_rigid_body_contacts_ = false;
  disable_joint_springs_ = false;
  /* Bullet 2.82 spring damping: 1 = no damping (spring fully active, prone
   * to oscillation), 0 = spring fully damped. 0.15 is the F5e baseline.
   * Raising to 0.85 (MMP zone preset) WITHOUT porting MMP's
   * linear/angular_damping_scale and joint_stop_erp/cfm causes severe
   * oscillation (左胸下 max_av 3.34->68.79 rad/s, 20x worse). */
  joint_spring_damping_ = 0.15f;
  joint_collision_exclusion_depth_ = 0;
  armature_ = nullptr;
  bmain_ = nullptr;
  pose_at_initialize_ = nullptr;
  armature_session_uid_ = 0;
}

int MMDPhysicsWorld::body_count() const
{
  return int(body_runtimes_.size());
}

int MMDPhysicsWorld::joint_count() const
{
  return int(joint_runtimes_.size());
}

bool MMDPhysicsWorld::is_binding_valid(Main *current_bmain) const
{
  if (!initialized_ || current_bmain == nullptr || current_bmain != bmain_ || armature_ == nullptr ||
      armature_session_uid_ == 0)
  {
    return false;
  }
  ID *current_id = BKE_libblock_find_session_uid(current_bmain, ID_OB, armature_session_uid_);
  if (current_id != reinterpret_cast<ID *>(armature_)) {
    return false;
  }
  Object *current_armature = reinterpret_cast<Object *>(current_id);
  if (current_armature->type != OB_ARMATURE || current_armature->pose == nullptr) {
    return false;
  }
  if (current_armature->pose == pose_at_initialize_) {
    return true;
  }

  /* Pose Mode and dependency-graph rebuilds can recreate the pose container
   * without changing the Armature object or its named bones. The runtime
   * resolves pose channels by name, so keep the world valid for that benign
   * rebuild instead of discarding it and leaving mute/constraint state behind. */
  for (const RigidBodyRuntime &runtime : body_runtimes_) {
    if (!runtime.blender_bone_name.empty() &&
        find_pose_channel(current_armature, runtime.blender_bone_name) == nullptr)
    {
      return false;
    }
  }
  return true;
}

bool MMDPhysicsWorld::get_body_transform(const int body_index,
                                         float r_position[3],
                                         float r_rotation[4]) const
{
  if (body_index < 0 || body_index >= int(body_runtimes_.size())) {
    return false;
  }
  const btRigidBody &body = *body_runtimes_[body_index].body;
  btTransform transform;
  if (body.getMotionState() != nullptr) {
    body.getMotionState()->getWorldTransform(transform);
  }
  else {
    transform = body.getWorldTransform();
  }
  const btVector3 &origin = transform.getOrigin();
  const btQuaternion &rot = transform.getRotation();
  r_position[0] = float(origin.x());
  r_position[1] = float(origin.y());
  r_position[2] = float(origin.z());
  r_rotation[0] = float(rot.x());
  r_rotation[1] = float(rot.y());
  r_rotation[2] = float(rot.z());
  r_rotation[3] = float(rot.w());
  return true;
}

bool MMDPhysicsWorld::set_body_transform(const int body_index,
                                         const float position[3],
                                         const float rotation[4])
{
  if (body_index < 0 || body_index >= int(body_runtimes_.size())) {
    return false;
  }
  btRigidBody &body = *body_runtimes_[body_index].body;
  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(btVector3(btScalar(position[0]),
                                btScalar(position[1]),
                                btScalar(position[2])));
  transform.setRotation(btQuaternion(btScalar(rotation[0]),
                                     btScalar(rotation[1]),
                                     btScalar(rotation[2]),
                                     btScalar(rotation[3])));
  body.setWorldTransform(transform);
  body.setInterpolationWorldTransform(transform);
  if (body.getMotionState() != nullptr) {
    body.getMotionState()->setWorldTransform(transform);
  }
  body.clearForces();
  body.setLinearVelocity(btVector3(0, 0, 0));
  body.setAngularVelocity(btVector3(0, 0, 0));
  body.activate(true);
  return true;
}

int MMDPhysicsWorld::find_body_index_by_bone_name(const std::string &bone_name) const
{
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    if (body_runtimes_[i].blender_bone_name == bone_name) {
      return i;
    }
  }
  return -1;
}

const MMDPhysicsPerformance &MMDPhysicsWorld::performance() const
{
  return performance_;
}

bool MMDPhysicsWorld::capture_diagnostic_frame(const int step,
                                               MMDDiagnosticFrame &r_frame) const
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return false;
  }

  r_frame.step = step;
  r_frame.bodies.clear();
  r_frame.joints.clear();
  r_frame.contacts.clear();
  r_frame.broadphase_pairs.clear();
  r_frame.mesh_samples.clear();
  r_frame.bodies.reserve(body_runtimes_.size());
  r_frame.joints.reserve(joint_runtimes_.size());

  for (const int runtime_index : body_runtimes_.index_range()) {
    const RigidBodyRuntime &runtime = body_runtimes_[runtime_index];
    const btRigidBody &body = *runtime.body;
    const btTransform &transform = body.getWorldTransform();
    const btVector3 &position = transform.getOrigin();
    const btQuaternion &rotation = transform.getRotation();
    const btVector3 &linear_velocity = body.getLinearVelocity();
    const btVector3 &angular_velocity = body.getAngularVelocity();

    MMDDiagnosticBodySample sample;
    sample.runtime_index = runtime_index;
    sample.pmx_index = runtime.pmx_index;
    sample.name_local = runtime.name_local;
    sample.bone_name = runtime.blender_bone_name;
    sample.physics_type = runtime.physics_type;
    sample.collision_group = runtime.collision_group_index;
    sample.no_collision_group = runtime.no_collision_group;
    sample.effective_bullet_group = runtime.collision_group;
    sample.effective_bullet_mask = runtime.collision_mask;
    sample.mass = runtime.mass;
    sample.linear_damping = float(body.getLinearDamping());
    sample.angular_damping = float(body.getAngularDamping());
    sample.position = {float(position.x()), float(position.y()), float(position.z())};
    sample.quaternion = {
        float(rotation.x()), float(rotation.y()), float(rotation.z()), float(rotation.w())};
    sample.linear_velocity = {float(linear_velocity.x()),
                              float(linear_velocity.y()),
                              float(linear_velocity.z())};
    sample.angular_velocity = {float(angular_velocity.x()),
                               float(angular_velocity.y()),
                               float(angular_velocity.z())};
    sample.activation = body.getActivationState();

    btScalar linear_energy = btScalar(0.0);
    btScalar angular_energy = btScalar(0.0);
    if (body.getInvMass() > btScalar(0.0)) {
      const btScalar actual_mass = btScalar(1.0) / body.getInvMass();
      linear_energy = btScalar(0.5) * actual_mass * linear_velocity.length2();
      const btVector3 angular_velocity_local = transform.getBasis().transpose() * angular_velocity;
      const btVector3 inv_inertia = body.getInvInertiaDiagLocal();
      btScalar angular_energy_sum = btScalar(0.0);
      for (int axis = 0; axis < 3; axis++) {
        if (inv_inertia[axis] > SIMD_EPSILON) {
          angular_energy_sum += angular_velocity_local[axis] * angular_velocity_local[axis] /
                                inv_inertia[axis];
        }
      }
      angular_energy = btScalar(0.5) * angular_energy_sum;
    }
    sample.kinetic_energy = float(linear_energy + angular_energy);
    r_frame.bodies.append(std::move(sample));
  }

  for (const int runtime_index : joint_runtimes_.index_range()) {
    const JointRuntime &runtime = joint_runtimes_[runtime_index];
    btGeneric6DofSpringConstraint &constraint = *runtime.constraint;
    constraint.calculateTransforms();

    MMDDiagnosticJointSample sample;
    sample.runtime_index = runtime_index;
    sample.pmx_index = runtime.pmx_index;
    sample.name_local = runtime.name_local;
    sample.rigid_a = runtime.rigid_a;
    sample.rigid_b = runtime.rigid_b;
    const btTransform &frame_a = constraint.getFrameOffsetA();
    const btTransform &frame_b = constraint.getFrameOffsetB();
    const btVector3 &frame_a_position = frame_a.getOrigin();
    const btVector3 &frame_b_position = frame_b.getOrigin();
    const btQuaternion frame_a_rotation = frame_a.getRotation().normalized();
    const btQuaternion frame_b_rotation = frame_b.getRotation().normalized();
    sample.frame_a_position = {float(frame_a_position.x()),
                               float(frame_a_position.y()),
                               float(frame_a_position.z())};
    sample.frame_a_quaternion = {float(frame_a_rotation.x()),
                                 float(frame_a_rotation.y()),
                                 float(frame_a_rotation.z()),
                                 float(frame_a_rotation.w())};
    sample.frame_b_position = {float(frame_b_position.x()),
                               float(frame_b_position.y()),
                               float(frame_b_position.z())};
    sample.frame_b_quaternion = {float(frame_b_rotation.x()),
                                 float(frame_b_rotation.y()),
                                 float(frame_b_rotation.z()),
                                 float(frame_b_rotation.w())};
    sample.angular_lower_limit = runtime.rotation_min;
    sample.angular_upper_limit = runtime.rotation_max;
    sample.linear_lower_limit = runtime.translation_min;
    sample.linear_upper_limit = runtime.translation_max;
    sample.use_frame_offset = constraint.getUseFrameOffset();
    sample.constraint_enabled = constraint.isEnabled();
    sample.needs_feedback = constraint.needsFeedback();
    sample.override_solver_iterations = constraint.getOverrideNumSolverIterations();
    for (int axis = 0; axis < 3; axis++) {
      sample.angle[axis] = float(constraint.getAngle(axis));
      sample.linear_diff[axis] = float(constraint.getRelativePivotPosition(axis));
      constraint.testAngularLimitMotor(axis);
      const btRotationalLimitMotor *motor = constraint.getRotationalLimitMotor(axis);
      sample.angular_current_limit[axis] = motor->m_currentLimit;
      sample.angular_current_limit_error[axis] = float(motor->m_currentLimitError);
      sample.angular_target_velocity[axis] = float(motor->m_targetVelocity);
      sample.angular_max_motor_force[axis] = float(motor->m_maxMotorForce);
      sample.angular_max_limit_force[axis] = float(motor->m_maxLimitForce);
      sample.angular_damping[axis] = float(motor->m_damping);
      sample.angular_limit_softness[axis] = float(motor->m_limitSoftness);
      sample.angular_normal_cfm[axis] = float(motor->m_normalCFM);
      sample.angular_stop_erp[axis] = float(motor->m_stopERP);
      sample.angular_stop_cfm[axis] = float(motor->m_stopCFM);
      sample.angular_bounce[axis] = float(motor->m_bounce);
      sample.angular_enable_motor[axis] = motor->m_enableMotor ? 1 : 0;
      sample.angular_current_position[axis] = float(motor->m_currentPosition);
      sample.angular_accumulated_impulse[axis] = float(motor->m_accumulatedImpulse);
    }
    const btTranslationalLimitMotor *linear_motor = constraint.getTranslationalLimitMotor();
    if (linear_motor != nullptr) {
      sample.linear_limit_softness = float(linear_motor->m_limitSoftness);
      sample.linear_damping = float(linear_motor->m_damping);
      sample.linear_restitution = float(linear_motor->m_restitution);
      for (int axis = 0; axis < 3; axis++) {
        sample.linear_current_limit[axis] = linear_motor->m_currentLimit[axis];
        sample.linear_current_limit_error[axis] = float(linear_motor->m_currentLimitError[axis]);
        sample.linear_normal_cfm[axis] = float(linear_motor->m_normalCFM[axis]);
        sample.linear_stop_erp[axis] = float(linear_motor->m_stopERP[axis]);
        sample.linear_stop_cfm[axis] = float(linear_motor->m_stopCFM[axis]);
        sample.linear_enable_motor[axis] = linear_motor->m_enableMotor[axis] ? 1 : 0;
        sample.linear_target_velocity[axis] = float(linear_motor->m_targetVelocity[axis]);
        sample.linear_max_motor_force[axis] = float(linear_motor->m_maxMotorForce[axis]);
      }
    }
    for (int axis = 0; axis < 6; axis++) {
      sample.spring_enabled[axis] = constraint.isSpringEnabled(axis) ? 1 : 0;
      sample.spring_stiffness[axis] = float(constraint.getStiffness(axis));
      sample.spring_damping[axis] = float(constraint.getDamping(axis));
      sample.spring_equilibrium[axis] = float(constraint.getEquilibriumPoint(axis));
    }
    sample.applied_impulse = float(constraint.getAppliedImpulse());
    r_frame.joints.append(std::move(sample));
  }

  const int manifold_count = dispatcher_->getNumManifolds();
  for (int manifold_index = 0; manifold_index < manifold_count; manifold_index++) {
    const btPersistentManifold *manifold = dispatcher_->getManifoldByIndexInternal(manifold_index);
    const auto *body_a = static_cast<const btCollisionObject *>(manifold->getBody0());
    const auto *body_b = static_cast<const btCollisionObject *>(manifold->getBody1());
    const int body_a_index = body_a != nullptr ? body_a->getUserIndex() : -1;
    const int body_b_index = body_b != nullptr ? body_b->getUserIndex() : -1;
    for (int point_index = 0; point_index < manifold->getNumContacts(); point_index++) {
      const btManifoldPoint &point = manifold->getContactPoint(point_index);
      const btVector3 &position_a = point.getPositionWorldOnA();
      const btVector3 &position_b = point.getPositionWorldOnB();
      const btVector3 &normal_on_b = point.m_normalWorldOnB;

      MMDDiagnosticContactSample sample;
      sample.body_a = body_a_index;
      sample.body_b = body_b_index;
      sample.position_a = {
          float(position_a.x()), float(position_a.y()), float(position_a.z())};
      sample.position_b = {
          float(position_b.x()), float(position_b.y()), float(position_b.z())};
      sample.normal_on_b = {
          float(normal_on_b.x()), float(normal_on_b.y()), float(normal_on_b.z())};
      sample.distance = float(point.getDistance());
      sample.applied_impulse = float(point.getAppliedImpulse());
      r_frame.contacts.append(std::move(sample));
    }
  }
  const btBroadphasePairArray &pairs = dynamics_world_->getPairCache()->getOverlappingPairArray();
  for (int pair_index = 0; pair_index < pairs.size(); pair_index++) {
    const btBroadphasePair &pair = pairs[pair_index];
    const auto *body_a = pair.m_pProxy0 != nullptr ?
                             static_cast<const btCollisionObject *>(pair.m_pProxy0->m_clientObject) :
                             nullptr;
    const auto *body_b = pair.m_pProxy1 != nullptr ?
                             static_cast<const btCollisionObject *>(pair.m_pProxy1->m_clientObject) :
                             nullptr;
    r_frame.broadphase_pairs.append(
        {body_a != nullptr ? body_a->getUserIndex() : -1,
         body_b != nullptr ? body_b->getUserIndex() : -1});
  }
  return true;
}

bool MMDPhysicsWorld::find_body_containing_point(
    const std::array<float, 3> &blender_point,
    const std::vector<int> *bodies_to_check,
    int &r_body_index,
    float &r_penetration_depth) const
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return false;
  }
  const btVector3 point(blender_point[0], blender_point[1], blender_point[2]);
  bool found = false;
  float best_depth = -1.0f;
  int best_index = -1;

  auto test_body = [&](int runtime_index) {
    if (runtime_index < 0 || runtime_index >= int(body_runtimes_.size())) {
      return;
    }
    const RigidBodyRuntime &rt = body_runtimes_[runtime_index];
    if (rt.body == nullptr || rt.shape == nullptr) {
      return;
    }
    btVector3 aabb_min, aabb_max;
    rt.shape->getAabb(rt.body->getWorldTransform(), aabb_min, aabb_max);
    if (point.x() < aabb_min.x() || point.x() > aabb_max.x() ||
        point.y() < aabb_min.y() || point.y() > aabb_max.y() ||
        point.z() < aabb_min.z() || point.z() > aabb_max.z())
    {
      return;
    }
    /* Inside AABB: compute penetration depth as the distance to the
     * nearest face. Positive = inside. */
    const float dx_min = float(point.x() - aabb_min.x());
    const float dx_max = float(aabb_max.x() - point.x());
    const float dy_min = float(point.y() - aabb_min.y());
    const float dy_max = float(aabb_max.y() - point.y());
    const float dz_min = float(point.z() - aabb_min.z());
    const float dz_max = float(aabb_max.z() - point.z());
    const float depth = std::min({dx_min, dx_max, dy_min, dy_max, dz_min, dz_max});
    if (depth > best_depth) {
      best_depth = depth;
      best_index = runtime_index;
      found = true;
    }
  };

  if (bodies_to_check != nullptr) {
    for (int idx : *bodies_to_check) {
      test_body(idx);
    }
  }
  else {
    for (int i = 0; i < int(body_runtimes_.size()); ++i) {
      test_body(i);
    }
  }

  if (found) {
    r_body_index = best_index;
    r_penetration_depth = best_depth;
  }
  return found;
}

/* ===================================================================== */
/* F2: bone <-> body synchronization.                                    */
/* ===================================================================== */

void MMDPhysicsWorld::build_bone_offset_cache_()
{
  const int count = int(body_runtimes_.size());
  bone_offset_blender_.reinitialize(count);
  is_bone_driver_.reinitialize(count);
  bone_depth_.reinitialize(count);
  bone_driver_index_.clear();

  for (int i = 0; i < count; ++i) {
    const RigidBodyRuntime &rt = body_runtimes_[i];
    btTransform offset;
    offset.setIdentity();
    is_bone_driver_[i] = false;
    bone_depth_[i] = 0;

    if (!rt.blender_bone_name.empty()) {
      bPoseChannel *pchan = find_pose_channel(armature_, rt.blender_bone_name);
      if (pchan != nullptr) {
        const btTransform bone_rest_blender = blender_m4_to_bullet_transform(
            pchan->bone_get(*armature_)->arm_mat);
        /* The offset converts a Blender-space bone matrix into the
         * matching rigid body matrix: `body = bone * offset`. */
        offset = bone_rest_blender.inverse() * rt.initial_transform;
        bone_depth_[i] = bone_depth(pchan);

        /* Driver selection: DYNAMIC and DYNAMIC_BONE both participate.
         * When multiple bodies bind to the same bone, the one with the
         * largest mass wins (mirrors MikuMikuPhysics `_build_bone_driver_rigid_indices`).
         * DYNAMIC bodies fully override bone transform (loc+rot);
         * DYNAMIC_BONE only overrides rotation (handled in writeback). */
        if (is_dynamic_type(rt.physics_type)) {
          const int *existing_idx = bone_driver_index_.lookup_ptr(rt.blender_bone_name);
          if (existing_idx == nullptr) {
            bone_driver_index_.add_new(rt.blender_bone_name, i);
            is_bone_driver_[i] = true;
          }
          else if (rt.mass > body_runtimes_[*existing_idx].mass) {
            /* New body has larger mass: take over the driver seat. */
            is_bone_driver_[*existing_idx] = false;
            bone_driver_index_.lookup(rt.blender_bone_name) = i;
            is_bone_driver_[i] = true;
          }
        }
      }
    }
    bone_offset_blender_[i] = offset;
  }
}

int MMDPhysicsWorld::sync_kinematic_from_pose()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return 0;
  }

  /* Ensure `current_kinematic_targets_` is sized like `body_runtimes_`. */
  if (current_kinematic_targets_.size() != body_runtimes_.size()) {
    current_kinematic_targets_.resize(body_runtimes_.size(), btTransform::getIdentity());
  }

  int updated = 0;
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    RigidBodyRuntime &rt = body_runtimes_[i];
    if (rt.physics_type != kPhysicsStatic) {
      continue;  /* Only static bodies are kinematic-driven by bones. */
    }
    if (rt.blender_bone_name.empty()) {
      continue;
    }
    bPoseChannel *pchan = find_pose_channel(armature_, rt.blender_bone_name);
    if (pchan == nullptr) {
      continue;
    }
    /* `pose_mat` is the bone's current pose matrix in armature space.
     * The actual
     * `setWorldTransform` is deferred to `step_full`'s kinematic-smoothing
     * loop so that large displacements can be split into N interpolated
     * sub-steps instead of teleporting the kinematic body. */
    const btTransform bone_current_blender = blender_m4_to_bullet_transform(pchan->pose_mat);
    const btTransform target = bone_current_blender * bone_offset_blender_[i];
    current_kinematic_targets_[i] = target;
    updated++;
  }
  return updated;
}

int MMDPhysicsWorld::sync_dynamic_from_pose_and_clear_velocities()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return 0;
  }

  int updated = 0;
  for (const int i : body_runtimes_.index_range()) {
    RigidBodyRuntime &runtime = body_runtimes_[i];
    if (!is_dynamic_type(runtime.physics_type) || runtime.body == nullptr) {
      continue;
    }

    if (snap_body_to_bone_pose_(i)) {
      updated++;
    }
    runtime.body->clearForces();
    runtime.body->setLinearVelocity(btVector3(0, 0, 0));
    runtime.body->setAngularVelocity(btVector3(0, 0, 0));
    runtime.body->forceActivationState(ACTIVE_TAG);
    runtime.body->activate(true);
  }
  return updated;
}

/* Thresholds for deciding whether a kinematic body "moved" between frames.
 * Mirrors MikuMikuPhysics `kKinematicWakeMove2 / kKinematicWakeAngle2`
 * (pmx_bullet_api.cpp:25-26). Below these, the body is considered static
 * and we skip `activate(true)` + `wake_related_dynamic_bodies_` so sleeping
 * dynamic chains (accessories, straps) aren't woken every frame. */
namespace {
constexpr btScalar kKinematicWakeMove2 = btScalar(1.0e-10);
constexpr btScalar kKinematicWakeAngle2 = btScalar(1.0e-10);

inline btScalar rotation_delta2_(const btQuaternion &a, const btQuaternion &b)
{
  const btScalar dot = btFabs(a.dot(b));
  const btScalar delta = btScalar(1.0) - btMin(dot, btScalar(1.0));
  return delta * delta;
}

inline bool kinematic_transform_changed_(const btTransform &a, const btTransform &b)
{
  if ((a.getOrigin() - b.getOrigin()).length2() > kKinematicWakeMove2) {
    return true;
  }
  return rotation_delta2_(a.getRotation(), b.getRotation()) > kKinematicWakeAngle2;
}
}  // namespace

/* Apply a kinematic target to a Bullet body (body + motion state + interp). */
void MMDPhysicsWorld::apply_kinematic_transform_(int body_index,
                                                 const btTransform &t,
                                                 const float timestep,
                                                 bool *moved_out)
{
  if (body_index < 0 || body_index >= int(body_runtimes_.size())) {
    if (moved_out != nullptr) {
      *moved_out = false;
    }
    return;
  }
  btRigidBody &body = *body_runtimes_[body_index].body;

  /* Detect whether the kinematic body actually moved. If not, skip the
   * teleport + activate + AABB refresh entirely — the body is already at
   * the right place, and forcing an update would re-inject it into Bullet's
   * solver, waking sleeping dynamic neighbors and causing accessory/strap
   * jitter. Mirrors MikuMikuPhysics `set_kinematic_transform`'s
   * `activate_body` gating (pmx_bullet_api.cpp:278-289, 1058-1067). */
  const bool moved = kinematic_transform_changed_(body.getWorldTransform(), t);
  if (moved_out != nullptr) {
    *moved_out = moved;
  }
  if (!moved) {
    return;
  }

  btVector3 diagnostic_linear_velocity(0, 0, 0);
  btVector3 diagnostic_angular_velocity(0, 0, 0);
  if (std::getenv("MMD_RT_KINEMATIC_VELOCITY") != nullptr && timestep > 0.0f) {
    btTransformUtil::calculateVelocity(body.getWorldTransform(),
                                       t,
                                       btScalar(timestep),
                                       diagnostic_linear_velocity,
                                       diagnostic_angular_velocity);
  }

  body.setWorldTransform(t);
  body.setInterpolationWorldTransform(t);
  if (body.getMotionState() != nullptr) {
    body.getMotionState()->setWorldTransform(t);
  }
  /* Clear forces and velocities after teleporting the kinematic body.
   * Without this, residual velocity from the previous frame is read by
   * Bullet's contact solver when this kinematic body touches a dynamic
   * body (e.g. leg-vs-skirt), injecting a phantom impulse that causes
   * accessory/strap chains to jitter. MMP explicitly zeroes velocity
   * after every `set_body_transform` (pmx_bullet_api.cpp:1093-1095).
   *
   * NOTE: this is NOT the same as cfa2835498e's reverted "compute velocity
   * from transform delta" fix — that one *set* velocity from displacement
   * and caused breast jitter. Zeroing is what MMP has always done. */
  body.clearForces();
  body.setLinearVelocity(diagnostic_linear_velocity);
  body.setAngularVelocity(diagnostic_angular_velocity);
  /* Refresh this body's AABB in the broadphase so that new overlap pairs
   * (e.g. leg-vs-skirt) are created during the next `calculateOverlappingPairs`.
   * Without this, the broadphase still sees the leg's old AABB and never
   * generates a contact pair for the skirt it just moved into — the
   * "leg touches skirt but skirt doesn't react" symptom. Mirrors
   * MikuMikuPhysics `refresh_world_pairs` (pmx_bullet_api.cpp). */
  if (dynamics_world_ != nullptr) {
    dynamics_world_->updateSingleAabb(&body);
  }
  body.activate(true);
}

/* Build `joint_neighbors_[i]` = list of body indices directly connected to
 * body `i` via any joint. Used by `wake_related_dynamic_bodies_` to do a
 * BFS along the joint graph. Mirrors MikuMikuPhysics
 * `collect_joint_connected_dynamics` (pmx_bullet_api.cpp). */
void MMDPhysicsWorld::build_joint_adjacency_()
{
  joint_neighbors_.clear();
  joint_neighbors_.resize(body_runtimes_.size());
  for (const JointRuntime &j : joint_runtimes_) {
    if (j.rigid_a >= 0 && j.rigid_a < int(joint_neighbors_.size()) &&
        j.rigid_b >= 0 && j.rigid_b < int(joint_neighbors_.size()))
    {
      joint_neighbors_[j.rigid_a].append(j.rigid_b);
      joint_neighbors_[j.rigid_b].append(j.rigid_a);
    }
  }
}

void MMDPhysicsWorld::apply_joint_collision_exclusions_()
{
  /* MMP alignment: this function is now a no-op.
   *
   * Previously this function performed BFS over the joint graph and used
   * `contactPairTest` to detect initial penetration between non-adjacent
   * jointed bodies (graph distance >= 2), disabling collision via
   * `setIgnoreCollisionCheck` for penetrating pairs.
   *
   * MMP does NOT perform this initial-penetration detection. MMP only
   * disables collision for jointed pairs when PMX declares them as "do NOT
   * collide" (via `joint.disable_collisions` in `_build_non_collision_pairs`),
   * which is handled by `create_joint_`'s `disable_collisions` parameter.
   *
   * Evidence: FINDING-1 differential trace (2026-07-28) showed 13 body-body
   * pairs (all jointed adjacent body parts like 左ひざ↔左足首, 上半身↔左上半身2)
   * present in ref trace but absent in self trace. Root cause was
   * `create_joint_` unconditionally passing `disable_collisions=true` (fixed
   * separately), but this function's BFS-based penetration suppression also
   * risks over-disabling legitimate contacts that MMP allows. Aligning with
   * MMP means removing this extra suppression entirely. */
  if (dynamics_world_ == nullptr || body_runtimes_.is_empty()) {
    return;
  }
  /* Intentionally empty: see comment above. */
  (void)joint_collision_exclusion_depth_;
  (void)joint_neighbors_;
}

void MMDPhysicsWorld::apply_mmd_tools_ncc_()
{
  /* Reproduce MMP's proximity NCC over the mmd_tools-inverted candidate
   * groups. This compatibility behavior is intentionally kept together with
   * the Bullet masks above; changing either side requires viewport testing. */
  if (dynamics_world_ == nullptr || body_runtimes_.is_empty()) {
    return;
  }

  auto is_joint_pair = [&](const int a, const int b) {
    for (const JointRuntime &joint : joint_runtimes_) {
      if ((joint.rigid_a == a && joint.rigid_b == b) ||
          (joint.rigid_a == b && joint.rigid_b == a))
      {
        return true;
      }
    }
    return false;
  };

  for (int a = 0; a < int(body_runtimes_.size()); ++a) {
    RigidBodyRuntime &body_a = body_runtimes_[a];
    const uint16_t no_collision_mask_a = uint16_t(0xFFFFu & ~body_a.no_collision_group);
    for (int b = a + 1; b < int(body_runtimes_.size()); ++b) {
      RigidBodyRuntime &body_b = body_runtimes_[b];
      const uint16_t no_collision_mask_b = uint16_t(0xFFFFu & ~body_b.no_collision_group);
      const bool mask_candidate =
          (no_collision_mask_a & body_b.collision_group) != 0 ||
          (no_collision_mask_b & body_a.collision_group) != 0;
      if (!mask_candidate) {
        continue;
      }

      bool ignore = is_joint_pair(a, b);
      if (!ignore) {
        const btScalar distance = (body_a.initial_transform.getOrigin() -
                                   body_b.initial_transform.getOrigin())
                                      .length();
        const btScalar max_distance =
            btScalar(0.75f * (body_a.binding_range + body_b.binding_range));
        ignore = distance < max_distance;
      }
      if (ignore) {
        body_a.body->setIgnoreCollisionCheck(body_b.body, true);
        body_b.body->setIgnoreCollisionCheck(body_a.body, true);
      }
    }
  }
}

/* BFS from each moved kinematic body along the joint graph, force-activating
 * every reachable DYNAMIC / DYNAMIC_BONE body. Without this, Bullet's island
 * solver may leave skirt chains asleep when only the leg (kinematic) moves,
 * so the contact solver never generates contact constraints between the leg
 * and the skirt it just moved into. Mirrors MikuMikuPhysics
 * `wake_related_dynamic_bodies` (pmx_bullet_api.cpp:577-597). */
void MMDPhysicsWorld::wake_related_dynamic_bodies_(const Vector<int> &moved_kinematic_indices)
{
  if (joint_neighbors_.is_empty() || dynamics_world_ == nullptr) {
    return;
  }
  std::vector<bool> visited(body_runtimes_.size(), false);
  Vector<int> queue;
  for (int idx : moved_kinematic_indices) {
    if (idx >= 0 && idx < int(visited.size()) && !visited[idx]) {
      visited[idx] = true;
      queue.append(idx);
    }
  }
  while (!queue.is_empty()) {
    const int cur = queue.pop_last();
    for (int neighbor : joint_neighbors_[cur]) {
      if (neighbor < 0 || neighbor >= int(visited.size()) || visited[neighbor]) {
        continue;
      }
      visited[neighbor] = true;
      const uint8_t pt = body_runtimes_[neighbor].physics_type;
      if (pt == kPhysicsDynamic || pt == kPhysicsDynamicBone) {
        btRigidBody *body = body_runtimes_[neighbor].body;
        if (body != nullptr) {
          body->setActivationState(ACTIVE_TAG);
          body->activate(true);
        }
      }
      /* Continue BFS through dynamic bodies so the whole skirt chain
       * gets woken up, not just the segment directly jointed to the leg. */
      queue.append(neighbor);
    }
  }
}

/* Compute segment count for kinematic smoothing based on max move/angle
 * between `last_kinematic_transforms_` and `current_kinematic_targets_`.
 * Returns 1 if no smoothing needed (or no previous data). */
int MMDPhysicsWorld::compute_kinematic_segments_() const
{
  if (!has_last_kinematic_ || last_kinematic_transforms_.size() != current_kinematic_targets_.size()) {
    return 1;
  }
  int segments = 1;
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    if (body_runtimes_[i].physics_type != kPhysicsStatic) {
      continue;
    }
    const btTransform &prev = last_kinematic_transforms_[i];
    const btTransform &curr = current_kinematic_targets_[i];
    const btVector3 move = curr.getOrigin() - prev.getOrigin();
    const float move_len = float(move.length());
    if (move_len > kinematic_smoothing_move_) {
      segments = std::max(segments,
                          int(std::ceil(move_len / kinematic_smoothing_move_)));
    }
    btQuaternion delta = curr.getRotation() * prev.getRotation().inverse();
    delta.normalize();
    float angle = float(std::abs(delta.getAngle()));
    if (angle > kinematic_smoothing_angle_) {
      segments = std::max(segments,
                          int(std::ceil(angle / kinematic_smoothing_angle_)));
    }
  }
  return std::min(segments, kinematic_smoothing_max_segments_);
}

/* Interpolate between prev and curr at factor `f` (0=prev, 1=curr).
 * Position: Catmull-Rom spline (C1) when prev_prev is available — the
 * derivative at f=0 is (curr - prev_prev)/2 (central difference), which
 * is approximately continuous across batches. This fixes the C0 velocity
 * discontinuity that excited spring oscillation ("颤动"). Falls back to
 * lerp (C0) on the first frame where prev_prev is unavailable.
 * Rotation: squad (C1) when prev_prev is available — Shoemake's spherical
 * quadrangle interpolation gives angular-velocity continuity across batch
 * boundaries, symmetric to Catmull-Rom for position. Falls back to slerp
 * (C0) on the first frame.
 * next is extrapolated as 2*curr - prev (constant-velocity assumption). */

namespace {

/* Quaternion logarithm for a unit quaternion q=(w,v):
 * log(q) = (0, axis * theta) where theta = acos(w). */
btQuaternion quat_log_(const btQuaternion &q)
{
  btScalar w = q.w();
  if (w < btScalar(-1.0)) {
    w = btScalar(-1.0);
  }
  if (w > btScalar(1.0)) {
    w = btScalar(1.0);
  }
  const btScalar theta = acos(w);
  const btScalar sin_theta = sin(theta);
  if (fabs(sin_theta) < btScalar(1e-6)) {
    return btQuaternion(q.x(), q.y(), q.z(), btScalar(0));
  }
  const btScalar scale = theta / sin_theta;
  return btQuaternion(q.x() * scale, q.y() * scale, q.z() * scale, btScalar(0));
}

/* Quaternion exponential for a pure vector v=(x,y,z,0):
 * exp(v) = (cos(|v|), v/|v| * sin(|v|)). */
btQuaternion quat_exp_(const btQuaternion &v)
{
  const btScalar x = v.x(), y = v.y(), z = v.z();
  const btScalar theta = sqrt(x * x + y * y + z * z);
  if (theta < btScalar(1e-6)) {
    return btQuaternion(btScalar(0), btScalar(0), btScalar(0), btScalar(1));
  }
  const btScalar scale = sin(theta) / theta;
  return btQuaternion(x * scale, y * scale, z * scale, cos(theta));
}

/* Shoemake squad intermediate control point at q_cur, given the previous
 * neighbor q_prev and next neighbor q_next. */
btQuaternion squad_control_(const btQuaternion &q_prev,
                            const btQuaternion &q_cur,
                            const btQuaternion &q_next)
{
  const btQuaternion q_cur_inv = q_cur.inverse();
  btQuaternion a = q_cur_inv * q_next;
  btQuaternion b = q_cur_inv * q_prev;
  /* Keep logs on the shortest arc so exp/log are stable. */
  if (a.w() < btScalar(0)) {
    a = btQuaternion(-a.x(), -a.y(), -a.z(), -a.w());
  }
  if (b.w() < btScalar(0)) {
    b = btQuaternion(-b.x(), -b.y(), -b.z(), -b.w());
  }
  btQuaternion log_sum = quat_log_(a) + quat_log_(b);
  log_sum *= btScalar(-0.25);
  return q_cur * quat_exp_(log_sum);
}

/* squad interpolation between q1 and q2, with q0 and q3 as handles. */
btQuaternion squad_interp_(const btQuaternion &q0,
                           const btQuaternion &q1,
                           const btQuaternion &q2,
                           const btQuaternion &q3,
                           btScalar t)
{
  const btQuaternion s1 = squad_control_(q0, q1, q2);
  const btQuaternion s2 = squad_control_(q3, q2, q1);
  const btQuaternion a = q1.slerp(q2, t);
  const btQuaternion b = s1.slerp(s2, t);
  return a.slerp(b, btScalar(2) * t * (btScalar(1) - t));
}

}  // namespace

btTransform MMDPhysicsWorld::interpolate_kinematic_(const btTransform &prev_prev,
                                                     const btTransform &prev,
                                                     const btTransform &curr,
                                                     float f,
                                                     bool has_prev_prev) const
{
  btTransform t;
  t.setIdentity();
  if (has_prev_prev) {
    /* Position: Catmull-Rom (C1). */
    const btVector3 &P0 = prev_prev.getOrigin();
    const btVector3 &P1 = prev.getOrigin();
    const btVector3 &P2 = curr.getOrigin();
    const btVector3 P3 = btScalar(2) * P2 - P1;
    const btScalar ff = btScalar(f);
    const btScalar f2 = ff * ff;
    const btScalar f3 = f2 * ff;
    t.setOrigin(btScalar(0.5) * (btScalar(2) * P1 + (-P0 + P2) * ff +
                                 (btScalar(2) * P0 - btScalar(5) * P1 +
                                  btScalar(4) * P2 - P3) * f2 +
                                 (-P0 + btScalar(3) * P1 -
                                  btScalar(3) * P2 + P3) * f3));
    /* Rotation: squad (C1). */
    btQuaternion q0 = prev_prev.getRotation();
    btQuaternion q1 = prev.getRotation();
    btQuaternion q2 = curr.getRotation();
    /* next = q2 * (q1^-1 * q2) — extrapolate along prev->curr. */
    btQuaternion q3 = q2 * (q1.inverse() * q2);
    /* Ensure all quaternions lie on the same hemisphere as q1 so slerp
     * and log/exp take the short arc. */
    if (q0.dot(q1) < btScalar(0)) {
      q0 = btQuaternion(-q0.x(), -q0.y(), -q0.z(), -q0.w());
    }
    if (q2.dot(q1) < btScalar(0)) {
      q2 = btQuaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
    }
    if (q3.dot(q2) < btScalar(0)) {
      q3 = btQuaternion(-q3.x(), -q3.y(), -q3.z(), -q3.w());
    }
    btQuaternion q_slerp = q1.slerp(q2, btScalar(f));
    btQuaternion q_squad = squad_interp_(q0, q1, q2, q3, btScalar(f));
    q_squad.normalize();
    /* squad can be numerically unstable for large or abruptly changing
     * rotations (log/exp diverge). Fall back to slerp when the two
     * differ by more than ~5 degrees (dot < 0.996). This preserves C1
     * continuity in the common case while preventing runaway extrapolation. */
    btQuaternion q = (q_squad.dot(q_slerp) < btScalar(0.996)) ? q_slerp : q_squad;
    t.setRotation(q);
  }
  else {
    /* Fallback: lerp + slerp (C0) when prev_prev is unavailable. */
    t.setOrigin(prev.getOrigin().lerp(curr.getOrigin(), btScalar(f)));
    t.setRotation(prev.getRotation().slerp(curr.getRotation(), btScalar(f)));
  }
  return t;
}

int MMDPhysicsWorld::apply_dynamic_to_pose()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return 0;
  }

  /* Collect driver bodies and order by bone depth (parent-first). */
  struct PendingWrite {
    int body_index;
    int depth;
  };
  Vector<PendingWrite> writes;
  writes.reserve(body_runtimes_.size());
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    if (!is_bone_driver_[i]) {
      continue;
    }
    writes.append({i, bone_depth_[i]});
  }
  std::stable_sort(writes.begin(),
                   writes.end(),
                   [](const PendingWrite &a, const PendingWrite &b) {
                     return a.depth < b.depth;
                   });

  /* Effective pose matrices of bones we have already written this frame,
   * stored in Blender space. Needed because writing
   * `chan_mat` does not immediately update `pose_mat` of the bone or its
   * children — we must propagate the effective matrix manually when
   * computing later children. */
  Map<std::string, btTransform> applied_effective_blender;

  int written = 0;
  for (const PendingWrite &w : writes) {
    const RigidBodyRuntime &rt = body_runtimes_[w.body_index];
    if (rt.blender_bone_name.empty()) {
      continue;
    }
    bPoseChannel *pchan = find_pose_channel(armature_, rt.blender_bone_name);
    if (pchan == nullptr) {
      continue;
    }

    /* Read the simulated body and recover its Blender-space bone matrix. */
    btTransform body_blender;
    if (rt.body->getMotionState() != nullptr) {
      rt.body->getMotionState()->getWorldTransform(body_blender);
    }
    else {
      body_blender = rt.body->getWorldTransform();
    }
    const btTransform bone_blender =
        body_blender * bone_offset_blender_[w.body_index].inverse();

    update_pose_bone_matrix_basis_(
        pchan, bone_blender, applied_effective_blender, rt.physics_type);

    applied_effective_blender.add(rt.blender_bone_name, bone_blender);

    written++;
  }
  return written;
}

void MMDPhysicsWorld::update_pose_bone_matrix_basis_(
    bPoseChannel *pchan,
    const btTransform &target_armature_blender,
    Map<std::string, btTransform> &applied_effective_blender,
    uint8_t physics_type)
{
  float target_blender[4][4];
  bullet_transform_to_blender_m4(target_armature_blender, target_blender);

  /* Solve `matrix_basis` so that
   *   effective_pose_mat = parent_effective * parent_rest.inverse() * rest * basis
   * equals `target_blender`. Hence:
   *   basis = rest.inverse() * parent_rest * parent_effective.inverse() * target.
   */
  float rest_inv[4][4];
  invert_m4_m4(rest_inv, pchan->bone_get(*armature_)->arm_mat);

  bPoseChannel *parent = pchan->parent;
  float basis[4][4];
  if (parent == nullptr) {
    mul_m4_m4m4(basis, rest_inv, target_blender);
  }
  else {
    float parent_rest[4][4];
    copy_m4_m4(parent_rest, parent->bone_get(*armature_)->arm_mat);

    /* Use the freshly-applied effective matrix if we updated the parent this
     * frame; otherwise fall back to the depsgraph-evaluated `pose_mat`. */
    float parent_eff[4][4];
    const std::string parent_name(parent->name);
    if (const btTransform *cached_blender = applied_effective_blender.lookup_ptr(parent_name)) {
      bullet_transform_to_blender_m4(*cached_blender, parent_eff);
    }
    else {
      copy_m4_m4(parent_eff, parent->pose_mat);
    }

    float parent_eff_inv[4][4];
    invert_m4_m4(parent_eff_inv, parent_eff);

    /* basis = rest_inv * parent_rest * parent_eff_inv * target */
    float tmp[4][4];
    mul_m4_m4m4(tmp, parent_rest, parent_eff_inv);
    float tmp2[4][4];
    mul_m4_m4m4(tmp2, rest_inv, tmp);
    mul_m4_m4m4(basis, tmp2, target_blender);
  }

  /* Write the solved basis into the pose channel.
   *
   * IMPORTANT: depsgraph pose evaluation rebuilds `chan_mat` from
   * `pchan->loc/quat/scale` (the "input" fields), NOT from `chan_mat` itself.
   * So writing `chan_mat` alone is futile — the next depsgraph eval (triggered
   * by our `WM_event_add_notifier(ND_POSE)`) overwrites it from the stale
   * `pchan->quat/loc`. We must decompose `basis` into `loc/quat/scale` and
   * write those fields; `chan_mat` is written too for immediate visibility
   * before the next depsgraph flush.
   *
   * DYNAMIC (physics_type==1): fully override bone transform (loc+rot+scale).
   * DYNAMIC_BONE (physics_type==2): preserve loc/scale and only take rotation
   * from physics. Cross-frame MMD memory sampling shows that the apparent
   * translation in its writeback matrix is exactly `p - pR`, the homogeneous
   * compensation for rotating around the PMX bone position. The sampled bones
   * are non-movable, so that matrix translation is not a pose location channel. */
  float loc[3], quat[4], rot3x3[3][3], scale[3];
  mat4_to_loc_quat(loc, quat, basis);
  mat4_to_loc_rot_size(loc, rot3x3, scale, basis);
  normalize_qt(quat);

  if (physics_type == kPhysicsDynamicBone) {
    /* Keep current loc/scale; only take rotation from physics. */
    copy_v3_v3(loc, pchan->loc);
    copy_v3_v3(scale, pchan->scale);
    /* Rebuild basis with preserved loc/scale + physics rotation so that
     * `chan_mat` and `pchan->quat` stay consistent. */
    loc_quat_size_to_mat4(basis, loc, quat, scale);
  }

  copy_m4_m4(pchan->chan_mat, basis);
  copy_v3_v3(pchan->loc, loc);
  copy_qt_qt(pchan->quat, quat);
  copy_v3_v3(pchan->scale, scale);

  /* Keep the active rotation representation synchronized with the physics
   * quaternion. `BKE_pose_where_is` reads the field selected by rotmode. */
  if (pchan->rotmode == ROT_MODE_XYZ || pchan->rotmode == ROT_MODE_XZY ||
      pchan->rotmode == ROT_MODE_YXZ || pchan->rotmode == ROT_MODE_YZX ||
      pchan->rotmode == ROT_MODE_ZXY || pchan->rotmode == ROT_MODE_ZYX)
  {
    quat_to_eulO(pchan->eul, pchan->rotmode, quat);
  }
  else if (pchan->rotmode == ROT_MODE_AXISANGLE) {
    quat_to_axis_angle(pchan->rotAxis, &pchan->rotAngle, quat);
  }

}

int MMDPhysicsWorld::step_full(const float timestep,
                               const int max_substeps,
                               const bool apply_results,
                               const int minimum_kinematic_segments)
{
  sync_kinematic_from_pose();

  /* Kinematic smoothing: split large kinematic displacements into N
   * interpolated sub-steps to avoid injecting huge impulses into joints
   * (mirrors MikuMikuPhysics `_step_with_kinematic_smoothing`). */
  const int segments = std::max(compute_kinematic_segments_(),
                                std::max(1, minimum_kinematic_segments));

  if (segments <= 1) {
    /* Fast path: no smoothing. Still must apply kinematic transforms
     * (f=1.0, i.e. snap to current target) before stepping — otherwise
     * `sync_kinematic_from_pose` updates `current_kinematic_targets_` in
     * memory but the Bullet body stays at its old world transform.
     * Spring forces then yank dynamic bodies toward the OLD kinematic
     * position, and when `compute_kinematic_segments_` eventually returns
     * >1 (because the accumulated target delta crosses the threshold), the
     * segment path teleports the kinematic body in one shot, injecting a
     * huge impulse -> accelerating jitter + "弹回" symptom.
     * Mirrors the segment path's per-segment apply loop at f=1.0. */
    Vector<int> moved_kinematic;
    for (int i = 0; i < int(body_runtimes_.size()); ++i) {
      if (body_runtimes_[i].physics_type != kPhysicsStatic) {
        continue;
      }
      bool moved = false;
      apply_kinematic_transform_(i, current_kinematic_targets_[i], timestep, &moved);
      if (moved) {
        moved_kinematic.append(i);
      }
    }
    if (!moved_kinematic.is_empty()) {
      wake_related_dynamic_bodies_(moved_kinematic);
    }
    step(timestep, max_substeps, apply_results);
  }
  else {
    const float sub_timestep = timestep / float(segments);

    for (int seg = 0; seg < segments; ++seg) {
      const float f = float(seg + 1) / float(segments);  /* 0..1 endpoint of this segment. */
      /* Apply interpolated kinematic transforms for this segment, collecting
       * which kinematic bodies actually moved beyond the wake threshold
       * (kKinematicWakeMove2 / kKinematicWakeAngle2). Only those bodies
       * trigger `wake_related_dynamic_bodies_` so a perfectly still pose
       * doesn't keep re-activating sleeping accessory/strap chains.
       * Mirrors MikuMikuPhysics `pmx_bullet_set_kinematic_transforms`
       * (pmx_bullet_api.cpp:1044-1076). */
      Vector<int> moved_kinematic;
      for (int i = 0; i < int(body_runtimes_.size()); ++i) {
        if (body_runtimes_[i].physics_type != kPhysicsStatic) {
          continue;
        }
        btTransform t;
        if (has_last_kinematic_ && i < int(last_kinematic_transforms_.size())) {
          const bool has_pp = has_prev_prev_kinematic_ &&
                              i < int(prev_prev_kinematic_transforms_.size());
          t = interpolate_kinematic_(has_pp ? prev_prev_kinematic_transforms_[i]
                                            : last_kinematic_transforms_[i],
                                      last_kinematic_transforms_[i],
                                      current_kinematic_targets_[i],
                                      f,
                                      has_pp);
        }
        else {
          t = current_kinematic_targets_[i];
        }
        bool moved = false;
        apply_kinematic_transform_(i, t, sub_timestep, &moved);
        if (moved) {
          moved_kinematic.append(i);
        }
      }
      /* Force-activate dynamic bodies connected (via joints) to moved
       * kinematic bodies so Bullet's contact solver generates contact
       * constraints between the leg and the skirt it just moved into.
       * Skip entirely when nothing moved — keeps sleeping islands asleep. */
      if (!moved_kinematic.is_empty()) {
        wake_related_dynamic_bodies_(moved_kinematic);
      }
      /* Last segment writes back to bones; intermediate segments do not. */
      const bool last_seg = (seg == segments - 1);
      /* Per-segment fixedTimeStep = sub_timestep (MMP-aligned): Bullet always
       * advances exactly 1 substep regardless of segment count. Without this
       * override, sub_timestep < fixed_timestep_ would accumulate debt in
       * m_localTime without advancing physics -> visual slow-motion. */
      step(sub_timestep, 1, apply_results && last_seg, sub_timestep);
    }
  }

  /* Rotate kinematic history: prev_prev <- last, last <- current.
   * Catmull-Rom in interpolate_kinematic_ needs prev_prev for C1 position
   * continuity across batches. */
  prev_prev_kinematic_transforms_ = last_kinematic_transforms_;
  has_prev_prev_kinematic_ = has_last_kinematic_;
  last_kinematic_transforms_ = current_kinematic_targets_;
  has_last_kinematic_ = true;

  if (!apply_results) {
    return 0;
  }
  return apply_dynamic_to_pose();
}

/* ===================================================================== */
/* F3: temporal kinematic init + bone disconnection + prewarm.           */
/* ===================================================================== */

bool MMDPhysicsWorld::snap_body_to_bone_pose_(const int body_index)
{
  if (body_index < 0 || body_index >= int(body_runtimes_.size())) {
    return false;
  }
  RigidBodyRuntime &rt = body_runtimes_[body_index];
  if (rt.blender_bone_name.empty()) {
    return false;
  }
  bPoseChannel *pchan = find_pose_channel(armature_, rt.blender_bone_name);
  if (pchan == nullptr) {
    return false;
  }
  const btTransform bone_current_blender = blender_m4_to_bullet_transform(pchan->pose_mat);
  const btTransform target = bone_current_blender * bone_offset_blender_[body_index];

  btRigidBody &body = *rt.body;
  body.setWorldTransform(target);
  body.setInterpolationWorldTransform(target);
  if (body.getMotionState() != nullptr) {
    body.getMotionState()->setWorldTransform(target);
  }
  return true;
}

bool MMDPhysicsWorld::temporal_kinematic_init()
{
  if (!initialized_ || dynamics_world_ == nullptr) {
    return false;
  }

  /* 1. Save each body's current collision flags and force every body into
   *    kinematic mode so that the upcoming zero-duration step does not
   *    integrate gravity (which would yank dynamic bodies away from their
   *    bones before they are aligned). Mirrors MikuMikuPhysics's
   *    `pmx_bullet_temporal_kinematic_init` (pmx_bullet_api.cpp:1014-1020). */
  Vector<int> old_flags(body_runtimes_.size());
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    btRigidBody &body = *body_runtimes_[i].body;
    old_flags[i] = body.getCollisionFlags();
    body.setCollisionFlags(old_flags[i] | btCollisionObject::CF_KINEMATIC_OBJECT);
  }

  /* 2. Keep the transforms established during `initialize()`. Startup may
   * disconnect dynamic bones before reaching this function, which rebuilds
   * armature data without evaluating a fresh pose. Reading `pose_mat` here
   * would overwrite correctly initialized dynamic bodies with stale matrices.
   * This also matches the reference native API, whose temporal init receives
   * the already computed initial body matrices from its caller. */

  /* 3. Zero-duration step so Bullet realigns broadphase/narrowphase
   *    proxies, motion states and constraint frames against the new body
   *    transforms without advancing the simulation. */
  dynamics_world_->stepSimulation(btScalar(0.0), 0, btScalar(fixed_timestep_));

  /* 4. Restore original collision flags (dynamic bodies become dynamic
   *    again), then clear all forces/velocities accumulated during the
   *    zero-duration step and (re)activate dynamic bodies so the next real
   *    `step()` integrates them normally. */
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    btRigidBody &body = *body_runtimes_[i].body;
    body.setCollisionFlags(old_flags[i]);
    body.clearForces();
    body.setLinearVelocity(btVector3(0, 0, 0));
    body.setAngularVelocity(btVector3(0, 0, 0));
    if (is_dynamic_type(body_runtimes_[i].physics_type)) {
      body.forceActivationState(ACTIVE_TAG);
      body.activate(true);
    }
  }
  return true;
}

int MMDPhysicsWorld::prewarm(const int steps, const float timestep, const int /*max_substeps*/)
{
  if (!initialized_ || dynamics_world_ == nullptr || steps <= 0) {
    return 0;
  }
  int total_substeps = 0;
  for (int i = 0; i < steps; ++i) {
    sync_kinematic_from_pose();
    /* Use the real-time fixed_timestep (1/fixed_step_hz_) during prewarm so
     * spring behavior matches the real-time path. Previously prewarm used
     * `timestep` (1/30 s) as Bullet's fixedTimeStep, which made the spring's
     * `velFactor = fps * damping / numIterations` differ from real-time
     * (fps=30 vs 120). This caused the body to reach a different steady-state
     * during prewarm than real-time, and the first real step produced a
     * transient angular velocity (左胸上 av=6.87 rad/s at step 0) because the
     * spring force (equilibrium=0, body at VMD pose angle) was integrated
     * differently. Using fixed_timestep_ here lets the spring settle toward
     * the same equilibrium the real-time path would produce. */
    step(timestep, max_substeps_, false, fixed_timestep_);
    total_substeps += performance_.last_substeps;
  }

  /* Correct linear drift on locked-translation joints before zeroing
   * velocities. During prewarm, Bullet's sequential-impulse solver allows
   * joint anchors to separate slightly (lin_diff ~0.1-1.7mm observed on
   * 左胸上/左胸下). Without this correction, the first real step's locked
   * linear constraint solver produces a corrective impulse that generates
   * angular velocity (左胸上 av=6.87 rad/s at step 0). Running pullback
   * here aligns body B's linear position with the joint frame so step 0
   * starts from rest without constraint correction noise. */
  pullback_locked_joints_();
  refresh_broadphase();

  /* F5i fix: after prewarm, zero out ALL velocities accumulated during the
   * warmup steps. Prewarm is meant to settle broadphase/narrowphase proxies
   * and let constraints find their equilibrium; any velocities generated
   * during prewarm (e.g., a dynamic body falling under gravity before its
   * joint constraint fully converges) are transient noise that should NOT
   * carry into the first real `step_full`. Without this, body B enters the
   * first real tick with 6+ rad/s angular velocity and the visible
   * "抖动 on enable" symptom appears.
   *
   * Mirrors MikuMikuPhysics `pmx_bullet_prewarm` (pmx_bullet_api.cpp:1144-1154):
   * after the step loop it iterates all bodies, clears forces + velocities,
   * then re-activates dynamic bodies so the next real step integrates them
   * from rest. */
  for (int i = 0; i < int(body_runtimes_.size()); ++i) {
    btRigidBody &body = *body_runtimes_[i].body;
    body.clearForces();
    body.setLinearVelocity(btVector3(0, 0, 0));
    body.setAngularVelocity(btVector3(0, 0, 0));
    if (is_dynamic_type(body_runtimes_[i].physics_type)) {
      body.forceActivationState(ACTIVE_TAG);
      body.activate(true);
    }
  }

  /* F5o fix reverted (2026-07-26): re-setting equilibrium after prewarm made
   * oscillation WORSE because the post-prewarm relative transform diff was
   * OUTSIDE the joint angular limits (e.g. pmx=472 equilibrium X=-0.0558
   * but rot_limit X=[0, 0.698]). Spring pushed body toward the out-of-limit
   * equilibrium while limit solver pushed back -> oscillation amplification
   * (Step #0 av=16, Step #3 av=65, Step #4 av=70 rad/s).
   *
   * With equilibrium=0 (pre-F5o state) oscillation was also present (av=72
   * on first step) but at least did not introduce the spring-vs-limit
   * conflict. Root cause of the av=72 oscillation is deeper — likely
   * chest body's 120° initial rotation causing Bullet Euler angle
   * decomposition ambiguity, or spring+limit force interaction with
   * maxLimitForce=300. To be investigated by comparing with MMP's handling
   * of large-rotation bodies. */
  return total_substeps;
}

void MMDPhysicsWorld::startup_sync(const int steps)
{
  if (!initialized_ || dynamics_world_ == nullptr || steps <= 0) {
    return;
  }

  const int count = int(body_runtimes_.size());

  /* Save the current VMD-pose transforms (bodies already snapped by
   * initialize's `snap_body_to_bone_pose_` loop). These are the targets the
   * kinematic bodies should reach by the end of the interpolation. */
  Array<btTransform> targets(count);
  Array<btTransform> rests(count);
  for (int i = 0; i < count; ++i) {
    targets[i] = body_runtimes_[i].body->getWorldTransform();
    rests[i] = body_runtimes_[i].initial_transform;
  }

  /* Reset ALL bodies to PMX rest pose. This recreates the spring delta = 0
   * condition that existed at joint creation, so the subsequent gradual
   * interpolation can rebuild the delta smoothly instead of having it already
   * at full VMD-pose offset. Dynamic bodies also reset to rest pose; they will
   * free-simulate back toward the kinematic targets during the interpolation. */
  for (int i = 0; i < count; ++i) {
    btRigidBody &body = *body_runtimes_[i].body;
    body.setWorldTransform(rests[i]);
    body.setInterpolationWorldTransform(rests[i]);
    if (body.getMotionState() != nullptr) {
      body.getMotionState()->setWorldTransform(rests[i]);
    }
    body.clearForces();
    body.setLinearVelocity(btVector3(0, 0, 0));
    body.setAngularVelocity(btVector3(0, 0, 0));
  }
  refresh_broadphase();

  /* Gradually interpolate STATIC (kinematic) bodies from rest pose to VMD
   * pose over `steps` frames. Dynamic bodies are NOT explicitly moved; they
   * free-simulate under spring + gravity while the kinematic targets shift
   * gradually, so the spring delta builds up smoothly instead of jumping
   * from 0 to full VMD-pose offset in one tick.
   *
   * Mirrors MikuMikuPhysics `_sync_to_start_pose` (physics_world.py:1328-1353):
   * only STATIC bodies are interpolated, each step uses 1/30 s timestep. */
  const btScalar step_t = btScalar(1.0 / 30.0);
  for (int s = 1; s <= steps; ++s) {
    const btScalar factor = btScalar(s) / btScalar(steps);
    for (int i = 0; i < count; ++i) {
      if (is_dynamic_type(body_runtimes_[i].physics_type)) {
        continue;
      }
      btTransform interp;
      interp.setOrigin(rests[i].getOrigin().lerp(targets[i].getOrigin(), factor));
      interp.setRotation(
          rests[i].getRotation().slerp(targets[i].getRotation(), factor));
      btRigidBody &body = *body_runtimes_[i].body;
      body.setWorldTransform(interp);
      body.setInterpolationWorldTransform(interp);
      if (body.getMotionState() != nullptr) {
        body.getMotionState()->setWorldTransform(interp);
      }
    }
    step(float(step_t), 1, false, float(step_t));
  }

  /* Zero velocities after sync (mirrors MMP prewarm(0,...) behavior). */
  for (int i = 0; i < count; ++i) {
    btRigidBody &body = *body_runtimes_[i].body;
    body.clearForces();
    body.setLinearVelocity(btVector3(0, 0, 0));
    body.setAngularVelocity(btVector3(0, 0, 0));
    if (is_dynamic_type(body_runtimes_[i].physics_type)) {
      body.forceActivationState(ACTIVE_TAG);
      body.activate(true);
    }
  }
}

int MMDPhysicsWorld::disconnect_physics_bones()
{
  if (!initialized_ || armature_ == nullptr || bmain_ == nullptr) {
    return 0;
  }
  bArmature *arm = id_cast<bArmature *>(armature_->data);
  if (arm == nullptr) {
    return 0;
  }

  /* Collect unique bone names bound to DYNAMIC (physics_type==1) rigid bodies.
   * Only DYNAMIC bodies need use_connect=False because they fully override
   * bone transform (loc+rot); keeping use_connect=True would let the parent
   * bone's rest matrix pull the bone back to the parent's tail.
   * DYNAMIC_BONE (physics_type==2) keeps use_connect=True and only overrides
   * rotation in writeback (mirrors MikuMikuPhysics `_disconnect_physics_bones`). */
  Vector<std::string> physics_bone_names;
  for (const RigidBodyRuntime &rt : body_runtimes_) {
    if (rt.physics_type != kPhysicsDynamic) {
      continue;
    }
    if (rt.blender_bone_name.empty()) {
      continue;
    }
    bool already_present = false;
    for (const std::string &existing : physics_bone_names) {
      if (existing == rt.blender_bone_name) {
        already_present = true;
        break;
      }
    }
    if (!already_present) {
      physics_bone_names.append(rt.blender_bone_name);
    }
  }
  if (physics_bone_names.is_empty()) {
    return 0;
  }

  /* Enter edit mode. Paired with ED_armature_from_edit + ED_armature_edit_free
   * below — see project_memory lessons: never skip the free call. */
  ED_armature_to_edit(arm);

  int disconnected = 0;
  disconnected_bone_names_.clear();
  for (const std::string &bone_name : physics_bone_names) {
    EditBone *eb = ED_armature_ebone_find_name(arm->edbo, bone_name.c_str());
    if (eb == nullptr) {
      continue;
    }
    if (eb->flag & BONE_CONNECTED) {
      eb->flag &= ~BONE_CONNECTED;
      disconnected_bone_names_.append(bone_name);
      disconnected++;
    }
  }

  ED_armature_from_edit(bmain_, arm);
  ED_armature_edit_free(arm);
  return disconnected;
}

void MMDPhysicsWorld::restore_physics_bone_connections()
{
  if (disconnected_bone_names_.is_empty() || armature_ == nullptr || bmain_ == nullptr) {
    disconnected_bone_names_.clear();
    return;
  }
  bArmature *arm = id_cast<bArmature *>(armature_->data);
  if (arm == nullptr) {
    disconnected_bone_names_.clear();
    return;
  }

  ED_armature_to_edit(arm);
  for (const std::string &bone_name : disconnected_bone_names_) {
    EditBone *eb = ED_armature_ebone_find_name(arm->edbo, bone_name.c_str());
    if (eb == nullptr) {
      continue;
    }
    eb->flag |= BONE_CONNECTED;
  }
  ED_armature_from_edit(bmain_, arm);
  ED_armature_edit_free(arm);
  disconnected_bone_names_.clear();
}

/* ===================================================================== */
/* F5b: constraint mute + depsgraph flush.                               */
/* ===================================================================== */

int MMDPhysicsWorld::mute_physics_bone_constraints()
{
  if (!initialized_ || armature_ == nullptr || armature_->pose == nullptr) {
    return 0;
  }
  muted_constraints_.clear();
  initial_pose_.clear();

  /* Walk every pose channel; for those registered as a physics bone driver
   * (i.e. bound to a DYNAMIC / DYNAMIC_BONE rigid body), mute every
   * constraint so depsgraph pose eval does not overwrite the `chan_mat` /
   * `pose_mat` that `apply_dynamic_to_pose` writes from Bullet.
   * Also snapshot loc/quat/scale so `restore_physics_bone_constraints()`
   * can return the armature to its pre-simulation shape on Stop. */
  bPose *pose = armature_->pose;
  int muted_count = 0;
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(pose->chanbase.first);
       pchan != nullptr;
       pchan = pchan->next)
  {
    if (bone_driver_index_.lookup_ptr(pchan->name) == nullptr) {
      continue;  /* Not a physics-driven bone. */
    }
    /* Snapshot the pose BEFORE physics starts overwriting it. */
    InitialPose ip;
    ip.bone_name = pchan->name;
    copy_v3_v3(ip.loc, pchan->loc);
    copy_v3_v3(ip.scale, pchan->scale);
    copy_v3_v3(ip.eul, pchan->eul);
    copy_v4_v4(ip.quat, pchan->quat);
    copy_v3_v3(ip.rot_axis, pchan->rotAxis);
    ip.rot_angle = pchan->rotAngle;
    ip.rotmode = int(pchan->rotmode);
    initial_pose_.append(ip);

    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first);
         con != nullptr;
         con = con->next)
    {
      MutedConstraint muted;
      muted.bone_name = pchan->name;
      muted.constraint_name = con->name;
      muted.was_off = (con->flag & CONSTRAINT_OFF) != 0;
      muted_constraints_.append(std::move(muted));
      con->flag |= CONSTRAINT_OFF;
      muted_count++;
    }
  }

  fprintf(stderr,
          "[MMD Physics] mute_physics_bone_constraints: muted %d constraints on "
          "%d driver bones (pose snapshot=%d)\n",
          muted_count,
          int(bone_driver_index_.size()),
          int(initial_pose_.size()));
  fflush(stderr);
  return muted_count;
}

void MMDPhysicsWorld::restore_physics_bone_constraints()
{
  if (muted_constraints_.is_empty() && initial_pose_.is_empty()) {
    return;
  }
  /* Restore constraint mute state by stable Blender names. */
  if (armature_ != nullptr && armature_->pose != nullptr) {
    bPose *pose = armature_->pose;
    for (const MutedConstraint &muted : muted_constraints_) {
      bPoseChannel *pchan = static_cast<bPoseChannel *>(
          BLI_findstring(&pose->chanbase, muted.bone_name.c_str(), offsetof(bPoseChannel, name)));
      if (pchan == nullptr) {
        continue;
      }
      bConstraint *con = static_cast<bConstraint *>(BLI_findstring(
          &pchan->constraints, muted.constraint_name.c_str(), offsetof(bConstraint, name)));
      if (con == nullptr) {
        continue;
      }
      if (muted.was_off) {
        con->flag |= CONSTRAINT_OFF;
      }
      else {
        con->flag &= ~CONSTRAINT_OFF;
      }
    }
  }
  /* Restore the initial pose so the armature returns to its pre-simulation
   * shape. `apply_dynamic_to_pose` wrote physics-driven loc/quat/scale into
   * the pose bones; without this restore, those deformed values persist
   * after the Bullet world is destroyed. */
  if (armature_ != nullptr && armature_->pose != nullptr) {
    bPose *pose = armature_->pose;
    int restored = 0;
    for (const InitialPose &ip : initial_pose_) {
      bPoseChannel *pchan = static_cast<bPoseChannel *>(
          BLI_findstring(&pose->chanbase, ip.bone_name.c_str(), offsetof(bPoseChannel, name)));
      if (pchan == nullptr) {
        continue;
      }
      copy_v3_v3(pchan->loc, ip.loc);
      copy_v3_v3(pchan->scale, ip.scale);
      copy_v3_v3(pchan->eul, ip.eul);
      copy_v4_v4(pchan->quat, ip.quat);
      copy_v3_v3(pchan->rotAxis, ip.rot_axis);
      pchan->rotAngle = ip.rot_angle;
      pchan->rotmode = eRotationModes(ip.rotmode);
      restored++;
    }
    fprintf(stderr,
            "[MMD Physics] restore_physics_bone_constraints: restored %d/%d "
            "pose bones, %d constraints\n",
            restored,
            int(initial_pose_.size()),
            int(muted_constraints_.size()));
    fflush(stderr);
  }
  muted_constraints_.clear();
  initial_pose_.clear();
}

void MMDPhysicsWorld::flush_depsgraph(Depsgraph *depsgraph)
{
  if (!initialized_ || armature_ == nullptr || bmain_ == nullptr || depsgraph == nullptr) {
    return;
  }
  /* Mark the armature's pose as dirty so the depsgraph re-evaluates
   * `chan_mat` / `pose_mat` from the `pchan->loc/quat/scale` values written
   * by `apply_dynamic_to_pose`.
   *
   * IMPORTANT: do NOT call `BKE_scene_graph_update_tagged` here. During
   * Ctrl+Z undo, Blender's undo system invalidates and rebuilds view
   * layers / depsgraph. A synchronous flush from the physics modal TIMER
   * races with that rebuild and crashes in
   * `BKE_main_view_layers_synced_ensure` (access violation on a freed
   * view layer pointer). Instead, just tag the ID and let Blender's main
   * loop flush the depsgraph at a safe time (between undo completion and
   * the next redraw). This matches how other modal operators (e.g.
   * pose transform) propagate pose changes without synchronous flush. */
  DEG_id_tag_update(&armature_->id, ID_RECALC_GEOMETRY);
}

}  // namespace blender::mmd_physics
