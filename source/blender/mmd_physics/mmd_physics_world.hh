/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup mmd_physics
 *
 * MMD real-time physics world: wraps Bullet dynamics world and binds it to
 * the PMX physics definition parsed during import. Mirrors the runtime
 * behavior of MikuMikuPhysics 2.2 (`pmx_bullet_api.cpp`) but is a pure C++
 * in-process implementation built against Blender's internal Bullet 2.82
 * (double precision).
 *
 * F1 scope (this file): world + rigid body + joint construction, step, reset,
 * destroy, plus minimal debug accessors. Kinematic sync, dynamic readback,
 * bone disconnection and temporal kinematic init land in F2/F3.
 */

#pragma once

#include "mmd_physics_definition.hh"

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>

#include <array>
#include <cstdint>
#include <string>

namespace blender {

struct Object;
struct bPose;
struct bPoseChannel;
struct Main;
struct Scene;
struct bConstraint;
struct Depsgraph;

}  // namespace blender

namespace blender::mmd_physics {

struct MMDPhysicsPerformance {
  int body_count = 0;
  int dynamic_body_count = 0;
  int static_body_count = 0;
  int joint_count = 0;
  int last_substeps = 0;
  double last_step_time_ms = 0.0;
  double accumulated_step_time_ms = 0.0;
};

struct MMDDiagnosticBodySample {
  int runtime_index = -1;
  int pmx_index = -1;
  std::string name_local;
  std::string bone_name;
  uint8_t physics_type = 0;
  uint8_t collision_group = 0;
  uint16_t no_collision_group = 0;
  uint16_t effective_bullet_group = 0;
  uint16_t effective_bullet_mask = 0;
  float mass = 0.0f;
  float linear_damping = 0.0f;
  float angular_damping = 0.0f;
  std::array<float, 3> position{};
  std::array<float, 4> quaternion{};
  std::array<float, 3> linear_velocity{};
  std::array<float, 3> angular_velocity{};
  int activation = 0;
  float kinetic_energy = 0.0f;
};

struct MMDDiagnosticJointSample {
  int runtime_index = -1;
  int pmx_index = -1;
  std::string name_local;
  int rigid_a = -1;
  int rigid_b = -1;
  std::array<float, 3> frame_a_position{};
  std::array<float, 4> frame_a_quaternion{};
  std::array<float, 3> frame_b_position{};
  std::array<float, 4> frame_b_quaternion{};
  std::array<float, 3> angle{};
  std::array<float, 3> linear_diff{};
  std::array<float, 3> angular_lower_limit{};
  std::array<float, 3> angular_upper_limit{};
  std::array<float, 3> linear_lower_limit{};
  std::array<float, 3> linear_upper_limit{};
  std::array<int, 3> angular_current_limit{};
  std::array<float, 3> angular_current_limit_error{};
  std::array<int, 3> linear_current_limit{};
  std::array<float, 3> linear_current_limit_error{};
  bool use_frame_offset = false;
  bool constraint_enabled = false;
  bool needs_feedback = false;
  int override_solver_iterations = -1;
  float linear_limit_softness = 0.0f;
  float linear_damping = 0.0f;
  float linear_restitution = 0.0f;
  std::array<float, 3> linear_normal_cfm{};
  std::array<float, 3> linear_stop_erp{};
  std::array<float, 3> linear_stop_cfm{};
  std::array<int, 3> linear_enable_motor{};
  std::array<float, 3> linear_target_velocity{};
  std::array<float, 3> linear_max_motor_force{};
  std::array<float, 3> angular_target_velocity{};
  std::array<float, 3> angular_max_motor_force{};
  std::array<float, 3> angular_max_limit_force{};
  std::array<float, 3> angular_damping{};
  std::array<float, 3> angular_limit_softness{};
  std::array<float, 3> angular_normal_cfm{};
  std::array<float, 3> angular_stop_erp{};
  std::array<float, 3> angular_stop_cfm{};
  std::array<float, 3> angular_bounce{};
  std::array<int, 3> angular_enable_motor{};
  std::array<float, 3> angular_current_position{};
  std::array<float, 3> angular_accumulated_impulse{};
  std::array<int, 6> spring_enabled{};
  std::array<float, 6> spring_stiffness{};
  std::array<float, 6> spring_damping{};
  std::array<float, 6> spring_equilibrium{};
  float applied_impulse = 0.0f;
};

struct MMDDiagnosticContactSample {
  int body_a = -1;
  int body_b = -1;
  std::array<float, 3> position_a{};
  std::array<float, 3> position_b{};
  std::array<float, 3> normal_on_b{};
  float distance = 0.0f;
  float applied_impulse = 0.0f;
};

/* Mesh vertex penetration sample (schema v1.1).
 * Emitted only when a sampled vertex is found INSIDE some rigid body's
 * world-space AABB. `penetration_depth` is the max axis-aligned depth
 * (positive = inside, distance from vertex to nearest AABB face).
 * Coordinates are in Blender Z-up space to match body samples. */
struct MMDDiagnosticMeshSample {
  std::string bone_name;
  int vertex_index = -1;
  std::array<float, 3> position_world{};
  int inside_body_index = -1;
  std::string inside_body_bone;
  float penetration_depth = 0.0f;
};

struct MMDDiagnosticFrame {
  int step = -1;
  /* Optional live-scheduler context. Isolated captures leave these at their
   * defaults; runtime snapshots use them to correlate Bullet state with the
   * Blender Timer/writeback loop. */
  double runtime_timer_elapsed = 0.0;
  double runtime_accumulator = 0.0;
  int runtime_fixed_steps = 0;
  uint64_t runtime_total_fixed_steps = 0;
  bool runtime_writeback = false;
  Vector<MMDDiagnosticBodySample> bodies;
  Vector<MMDDiagnosticJointSample> joints;
  Vector<MMDDiagnosticContactSample> contacts;
  Vector<std::array<int, 2>> broadphase_pairs;
  Vector<MMDDiagnosticMeshSample> mesh_samples;
};

/**
 * Owns a Bullet `btDiscreteDynamicsWorld` plus all rigid bodies, joints and
 * shapes derived from an `MMDPhysicsDefinition`. All Bullet resources are
 * released in `destroy()` (or the destructor); the world can be safely
 * destroyed and re-initialized multiple times.
 */
class MMDPhysicsWorld {
 public:
  MMDPhysicsWorld() = default;
  ~MMDPhysicsWorld();

  MMDPhysicsWorld(const MMDPhysicsWorld &) = delete;
  MMDPhysicsWorld &operator=(const MMDPhysicsWorld &) = delete;

  /**
   * Build the Bullet world from a validated physics definition.
   * `armature` is retained for F2/F3 bone lookup; F1 does not touch it.
   *
   * \param gravity: World-space gravity in Blender's Z-up frame
   *                 (typically `(0, 0, -9.81)`).
   * \param solver_iterations: Bullet sequential-impulse iteration count
   *                           (MMD-aligned default: 20).
   * \param fixed_step_hz: Fixed simulation rate in Hz (default: 60).
   * \param max_substeps: Maximum Bullet sub-steps per `step()` call.
   * \return true on success, false if construction failed (Bullet allocation
   *         error or empty definition).
   */
  bool initialize(const MMDPhysicsDefinition &def,
                  Object *armature,
                  Main *bmain,
                  const float gravity[3],
                  int solver_iterations,
                  int fixed_step_hz,
                  int max_substeps,
                  bool disable_rigid_body_contacts = false,
                  bool disable_joint_springs = false,
                  float joint_spring_damping = 0.15f,
                  int joint_collision_exclusion_depth = 0);

  /**
   * Release all Bullet resources. If `restore_initial` is true the world's
   * bodies are first snapped back to their initial transforms before
   * destruction (used by F2/F3 when restoring the pose after simulation).
   * Safe to call on a non-initialized world (no-op).
   */
  void destroy(bool restore_initial);

  /**
   * Advance the simulation by `timestep` seconds.
   *
   * Internally calls `btDiscreteDynamicsWorld::stepSimulation` with
   * `fixed_timestep = 1 / fixed_step_hz` and `max_substeps` clamped to the
   * configured maximum. `apply_results` is currently informational (F1 has
   * nothing to apply back to Blender); F2 will use it to gate pose writeback.
   *
   * `fixed_timestep_override > 0` overrides the world's `fixed_timestep_`
   * for this call only. Used by `step_full`'s kinematic-smoothing path so
   * that each segment's `sub_timestep` becomes Bullet's `fixedTimeStep`,
   * forcing exactly 1 substep to advance per segment regardless of segment
   * count (mirrors MikuMikuPhysics `pmx_bullet_api.cpp:1115` trick of
   * `fixedTimeStep = timeStep`). Without this override, `sub_timestep <
   * fixed_timestep_` would make Bullet accumulate debt in `m_localTime`
   * without advancing physics -> visual slow-motion.
   *
   * \return true on success, false if the world is not initialized or
   *         `timestep` is non-positive.
   */
  void step(float timestep,
            int max_substeps,
            bool apply_results,
            float fixed_timestep_override = 0.0f);

  /** Defer locked-joint correction while a real-time catch-up batch runs. */
  void set_defer_constraint_correction(bool defer);

  /** Override solver iterations for joints whose two bodies are dynamic. */
  void set_dynamic_constraint_iterations(int iterations);

  /** Apply one deferred locked-joint correction and refresh broadphase state. */
  void finalize_deferred_constraint_correction();

  /** Refresh broadphase state without applying a locked-joint correction. */
  void refresh_broadphase();

  /** Snap every body back to its initial transform and clear all velocities. */
  void reset();

  /** Snap every bone-bound body to the currently evaluated pose and clear velocities. */
  void reset_to_current_pose();

  /* --- F2: bone <-> body synchronization ----------------------------- */

  /**
   * Collect phase: read each STATIC body's bound bone current world matrix
   * from the armature pose, multiply by the cached `bone_offset_blender_`
   * and push the result as the body's kinematic target.
   *
   * F2 simplification: assumes `armature` is the model root (i.e. armature's
   * `matrix_world` is identity at the PMX origin). F3 will add proper
   * `model_root` handling so that translating/rotating the model in the
   * scene does not break simulation.
   *
   * \return number of kinematic bodies actually updated.
   */
  int sync_kinematic_from_pose();

  /** MMD c45c0 bridge: snap dynamic bodies to their bones and clear motion. */
  int sync_dynamic_from_pose_and_clear_velocities();

  /**
   * Apply phase: read each DYNAMIC_BONE body's transform back from Bullet,
   * convert to a bone matrix in armature space and write it to the
   * corresponding pose bone's `matrix_basis`. Bodies are processed in
   * ascending bone-depth order so that a parent bone is updated before its
   * children (otherwise the child's effective world matrix would shift
   * again when the parent's `matrix_basis` changes).
   *
   * Pure DYNAMIC bodies (`physics_type == 1`) are not bone-bound and are
   * skipped. If multiple DYNAMIC_BONE bodies share the same bone, only the
   * first one encountered (lowest PMX index) is the "driver" and writes
   * the bone; others are read-only participants in the physics scene.
   *
   * \return number of pose bones written.
   */
  int apply_dynamic_to_pose();

  /**
   * Convenience: `sync_kinematic_from_pose()` + `step()` + (if apply_results)
   * `apply_dynamic_to_pose()`. Used by F2's "Step Once" operator and F4's
   * real-time Timer.
   */
  int step_full(float timestep,
                int max_substeps,
                bool apply_results,
                int minimum_kinematic_segments = 1);

  /* --- F3: temporal kinematic init + bone disconnection + prewarm ---- */

  /**
   * Temporarily flag every rigid body as kinematic, snap each bone-bound
   * body to its bone's current pose matrix, run a zero-duration Bullet step
   * so internal proxies/cache realign, then restore the original collision
   * flags and clear all velocities. Mirrors MikuMikuPhysics's
   * `pmx_bullet_temporal_kinematic_init` (pmx_bullet_api.cpp:1007).
   *
   * Call this once after `initialize()` (and again whenever the pose has
   * drifted far from the physics state, e.g. after scrubbing the timeline)
   * so that the first real `step()` does not explode from a large body-vs-
   * bone position mismatch.
   *
   * \return true on success.
   */
  bool temporal_kinematic_init();

  /**
   * Run `steps` real simulation steps without writing back to pose bones.
   * Used after `temporal_kinematic_init()` to let dynamic bodies settle
   * (skirts fall, hair drape) before the user sees the result. Each step
   * uses the same kinematic sync as `step_full()` so static bodies track
   * the (currently stationary) pose.
   *
   * \return total number of sub-steps Bullet actually executed.
   */
  int prewarm(int steps, float timestep, int max_substeps);

  /**
   * Gradually interpolate kinematic (STATIC) bodies from PMX rest pose to the
   * current VMD pose over `steps` frames, letting dynamic bodies free-simulate
   * under spring + gravity while the spring delta builds up smoothly.
   *
   * Mirrors MikuMikuPhysics `_sync_to_start_pose` (physics_world.py:1328-1353):
   * only STATIC bodies are interpolated, each step uses 1/30 s timestep.
   * Without this, `snap_body_to_bone_pose_` moves bodies from rest to VMD pose
   * in one tick, creating an instantaneous spring delta that excites
   * sustained oscillation ("颤动") on spring chains.
   *
   * \param steps Number of interpolation frames (0 = no-op, bodies stay where
   *              initialize() placed them).
   */
  void startup_sync(int steps);

  /**
   * Temporarily clear `use_connect` on every bone bound to a DYNAMIC_BONE
   * rigid body so that, during simulation, the parent bone's `pose_mat`
   * does not drag the physics bone's head along. Pairs with
   * `restore_physics_bone_connections()`.
   *
   * Implementation enters edit mode on `armature`, records the original
   * `use_connect` state of each affected EditBone, clears the flag, then
   * exits edit mode. The recorded state is restored by the matching
   * `restore_physics_bone_connections()` call. Mirrors MikuMikuPhysics's
   * `_disconnect_physics_bones` (physics_world.py).
   *
   * \return number of bones whose `use_connect` was actually cleared.
   */
  int disconnect_physics_bones();

  /**
   * Restore `use_connect` on bones previously disconnected by
   * `disconnect_physics_bones()`. Safe to call on a world that was never
   * disconnected (no-op).
   */
  void restore_physics_bone_connections();

  /* --- F5b: constraint mute + depsgraph flush ----------------------- */

  /**
   * Mute every constraint on every physics-driven pose bone (i.e. bones
   * bound to DYNAMIC / DYNAMIC_BONE rigid bodies and registered as the
   * driver). Mirrors MikuMikuPhysics's `_mute_mmd_tools_physics_constraints`
   * but broader: the native PMX importer attaches `MMD_Append_Rotation`
   * (TRANSFORM) constraints to D-bones (e.g. 左足D / 左ひざD / 左足首D) so
   * that, without physics, they inherit rotation from their IK parent.
   * During simulation these constraints would otherwise overwrite the
   * `chan_mat` / `pose_mat` that `apply_dynamic_to_pose` writes from
   * Bullet, making the physics-driven rotation invisible.
   *
   * Each muted constraint's previous `CONSTRAINT_OFF` state is recorded
   * and restored by `restore_physics_bone_constraints()`. Safe to call on
   * a non-initialized world (no-op).
   *
   * \return number of constraints actually muted.
   */
  int mute_physics_bone_constraints();

  /**
   * Restore `CONSTRAINT_OFF` flags previously cleared by
   * `mute_physics_bone_constraints()`. Safe to call on a world that was
   * never muted (no-op). Also restores the initial pose (loc/quat/scale)
   * captured at mute time so the armature returns to its pre-simulation
   * shape after Stop.
   */
  void restore_physics_bone_constraints();

  /**
   * Force the dependency graph to re-evaluate the armature's pose so that
   * the `pchan->loc/quat/scale` values written by `apply_dynamic_to_pose`
   * propagate to `chan_mat` and `pose_mat` (which the viewport reads for
   * rendering). Without this call the written input fields stay stale
   * until the next unrelated depsgraph trigger, which makes the physics
   * result invisible — the "character freezes when bones are not moved"
   * symptom.
   *
   * Mirrors MikuMikuPhysics's `flush_depsgraph` (`view_layer.update()`).
   */
  void flush_depsgraph(Depsgraph *depsgraph);

  /* --- Debug / test accessors (F1) ----------------------------------- */

  int body_count() const;
  int joint_count() const;

  /** Return whether the Blender data bound at initialize time is still live. */
  bool is_binding_valid(Main *current_bmain) const;

  /** Read a body's current Blender Z-up world transform. \return false if out of range. */
  bool get_body_transform(int body_index, float r_position[3], float r_rotation[4]) const;

  /**
   * Manually override a body's Blender Z-up world transform (clears velocities).
   * Useful for tests and for F2's kinematic-sync path. \return false if out
   * of range.
   */
  bool set_body_transform(int body_index, const float position[3], const float rotation[4]);

  /** Look up a body's PMX index from its Blender bone name (0 if not bound). */
  int find_body_index_by_bone_name(const std::string &bone_name) const;

  const MMDPhysicsPerformance &performance() const;

  /** Capture one diagnostics frame without exposing Bullet runtime pointers. */
  bool capture_diagnostic_frame(int step, MMDDiagnosticFrame &r_frame) const;

  /**
   * Test whether a point in Blender Z-up world space lies inside a body's
   * current world-space AABB. If yes, returns the body index and the
   * penetration depth (distance from point to nearest AABB face, positive
   * when inside). Used by the mesh-vertex penetration sampler.
   *
   * `bodies_to_check` is an optional filter list (body indices); pass
   * nullptr to scan all bodies. Scanning all 400+ bodies per vertex is
   * expensive, so callers should pass a pre-filtered list when possible.
   * \return true if the point is inside some body's AABB.
   */
  bool find_body_containing_point(
      const std::array<float, 3> &blender_point,
      const std::vector<int> *bodies_to_check,
      int &r_body_index,
      float &r_penetration_depth) const;

 private:
  struct RigidBodyRuntime {
    btRigidBody *body = nullptr;
    btDefaultMotionState *motion_state = nullptr;
    btCollisionShape *shape = nullptr;
    int pmx_index = -1;
    uint8_t physics_type = 0;
    uint8_t collision_group_index = 0;
    uint16_t no_collision_group = 0;
    uint16_t collision_group = 0;
    uint16_t collision_mask = 0;
    std::string name_local;
    std::string blender_bone_name;
    btTransform initial_transform;
    float mass = 0.0f;  /* For driver selection: max-mass body wins. */
    float binding_range = 0.0f;
  };

  struct JointRuntime {
    btGeneric6DofSpringConstraint *constraint = nullptr;
    int pmx_index = -1;
    int rigid_a = -1;
    int rigid_b = -1;
    std::string name_local;
    std::array<float, 3> translation_min{};
    std::array<float, 3> translation_max{};
    std::array<float, 3> rotation_min{};
    std::array<float, 3> rotation_max{};
    bool locked_translation = false;
  };

  /* Internal helpers. */
  void create_world_();
  void destroy_world_();
  btCollisionShape *create_shape_(const MMDRigidBodyDefinition &def);
  btRigidBody *create_rigid_body_(const MMDRigidBodyDefinition &def, btCollisionShape *shape);
  btGeneric6DofSpringConstraint *create_joint_(const MMDJointDefinition &def);

  /* F2 helpers. */
  void build_bone_offset_cache_();
  /* Apply a kinematic target to a Bullet body. When `moved_out` is non-null,
   * it is set to true iff the new transform differs from the body's current
   * one beyond `kKinematicWakeMove2 / kKinematicWakeAngle2` thresholds
   * (mirrors MikuMikuPhysics `transform_changed`,
   * pmx_bullet_api.cpp:469-475). Only bodies with `*moved_out == true`
   * should be `activate(true)`-d and passed to `wake_related_dynamic_bodies_`,
   * otherwise sleeping dynamic bodies get woken every frame even when the
   * kinematic driver is perfectly still -> accessory/strap jitter. */
  void apply_kinematic_transform_(int body_index,
                                   const btTransform &t,
                                   float timestep,
                                   bool *moved_out = nullptr);

  /* F5g: locked-translation joint pullback (single-point stabilization).
   * Extracted from F5f (57f010cedf0) without the non_collision_pairs /
   * overlap_filter / clean_disabled_pairs / refresh_world_pairs machinery
   * (those are independent mechanisms and were part of the F5f→F5n revert
   * stack). Pullback alone corrects anchor drift on locked-translation
   * joints after each stepSimulation, which is the main source of jitter
   * when kinematic smoothing raises segment count. Mirrors MikuMikuPhysics
   * `pullback_locked_joints` (pmx_bullet_api.cpp:611-655). */
  void pullback_locked_joints_();
  void damp_correction_velocity_(btRigidBody &body, const btVector3 &correction);
  int compute_kinematic_segments_() const;
  /* Interpolate between prev and curr at factor f (0=prev, 1=curr).
   * Position uses Catmull-Rom (C1) when prev_prev is available so velocity
   * is continuous across batches; falls back to lerp (C0) otherwise.
   * Rotation uses slerp (C0) — squad deferred for single-variable control. */
  btTransform interpolate_kinematic_(const btTransform &prev_prev,
                                     const btTransform &prev,
                                     const btTransform &curr,
                                     float f,
                                     bool has_prev_prev) const;
  void update_pose_bone_matrix_basis_(bPoseChannel *pchan,
                                       const btTransform &target_armature_blender,
                                       Map<std::string, btTransform> &applied_effective_blender,
                                      uint8_t physics_type);

  /* F3 helpers. */
  /* Snap a single bone-bound body to its bone's current pose matrix.
   * Used by both `temporal_kinematic_init()` (with `force_kinematic=true`)
   * and the regular kinematic sync path. Returns true if the body was
   * actually updated (i.e. it has a valid bone binding). */
  bool snap_body_to_bone_pose_(int body_index);

  /* F5c: collision response helpers.
   * Mirrors MikuMikuPhysics `refresh_world_pairs` (updateSingleAabb on
   * moved kinematic bodies so broadphase pair cache sees the new AABB)
   * and `wake_related_dynamic_bodies` (force-activate dynamic bodies
   * connected via joints to moved kinematic bodies, otherwise Bullet's
   * island solver may keep them asleep and the contact solver won't
   * generate contact constraints between the moved leg and the skirt). */
  void build_joint_adjacency_();
  void apply_joint_collision_exclusions_();
  void wake_related_dynamic_bodies_(const Vector<int> &moved_kinematic_indices);

  /** Apply mmd_tools-style Non-Collision Constraint (NCC) filtering.
   *
   * MMP alignment (2026-07-28): this function is now a no-op.
   *
   * Previously it implemented mmd_tools `buildRigids`-style NCC: for pairs
   * whose PMX `no_collision_group` declares "do NOT collide", disable the
   * pair via `setIgnoreCollisionCheck` if jointed or AABB-overlapping.
   *
   * With the MMP broadphase alignment (Bullet `mask = no_collision_group`,
   * treating PMX DISABLE mask as ENABLE mask due to mmd_tools importer
   * double-inversion bug), broadphase already filters all pairs according
   * to MMP's semantics. MMP's own `_build_non_collision_pairs` is fully
   * covered by broadphase, so it is effectively a no-op. We mirror that
   * here. Jointed-pair initial penetration is handled separately by
   * `apply_joint_collision_exclusions_` using `contactPairTest`,
   * restricted to broadphase-allowed pairs.
   *
   * Called once at the end of `initialize()` after all bodies and joints
   * exist. */
  void apply_mmd_tools_ncc_();


  /* Bullet world (owned). */
  btDefaultCollisionConfiguration *collision_configuration_ = nullptr;
  btCollisionDispatcher *dispatcher_ = nullptr;
  btDbvtBroadphase *broadphase_ = nullptr;
  btSequentialImpulseConstraintSolver *solver_ = nullptr;
  btDiscreteDynamicsWorld *dynamics_world_ = nullptr;

  /* Runtime resources (owned). */
  Vector<RigidBodyRuntime> body_runtimes_;
  Vector<JointRuntime> joint_runtimes_;
  /* Shapes created via `create_shape_` are released here on destroy; a body's
   * `shape` pointer aliases into this vector. */
  Vector<btCollisionShape *> owned_shapes_;

  /* F2 bone <-> body binding cache.
   * `bone_offset_blender_[i]` converts a bone's Blender-space matrix into the
   * corresponding rigid body's Blender-space matrix:
   *   `body = bone * bone_offset_blender_[i]`
   * For static bodies this is computed once at initialize() from the rest
   * pose; for dynamic_bone bodies the same formula drives the inverse
   * direction during apply (`bone = body * offset.inverse()`).
   * For unbound bodies (pure dynamic, no bone) the offset is identity. */
  Vector<btTransform> bone_offset_blender_;
  /* `is_bone_driver_[i]` is true for dynamic_bone bodies that own their
   * bound bone (first-come by PMX index). Other dynamic_bone bodies bound
   * to the same bone are non-driver participants. */
  Vector<bool> is_bone_driver_;
  /* `bone_depth_[i]` = ancestor count of body i's bound bone in the armature
   * hierarchy (0 for root bones). Used to order apply writes parent-first. */
  Vector<int> bone_depth_;
  /* Bone-name → driver body index, used while building `is_bone_driver_`. */
  Map<std::string, int> bone_driver_index_;

  /* F3: bone disconnection state.
   * Names of bones whose `use_connect` was cleared by
   * `disconnect_physics_bones()`, restored in pairs by
   * `restore_physics_bone_connections()`. Empty when not disconnected. */
  Vector<std::string> disconnected_bone_names_;

  /* F5b: constraint mute state, re-located by Blender names at Stop. */
  struct MutedConstraint {
    std::string bone_name;
    std::string constraint_name;
    bool was_off = false;
  };
  Vector<MutedConstraint> muted_constraints_;

  /* F5d: initial pose snapshot for Stop-time restoration.
   * Captured per physics-driven bone at `mute_physics_bone_constraints()`
   * time (i.e. at Start). `restore_physics_bone_constraints()` writes the
   * complete transform representation back so Euler and axis-angle bones
   * return to their exact pre-simulation pose as well. */
  struct InitialPose {
    std::string bone_name;
    float loc[3];
    float scale[3];
    float eul[3];
    float quat[4];
    float rot_axis[3];
    float rot_angle;
    int rotmode;
  };
  Vector<InitialPose> initial_pose_;

  /* F5c: joint adjacency cache for `wake_related_dynamic_bodies_`.
   * `joint_neighbors_[i]` lists every body index reachable from body `i`
   * by a single joint edge (either as rigid_a or rigid_b). Built once in
   * `initialize()` after `joint_runtimes_` is populated. */
  Vector<Vector<int>> joint_neighbors_;

  /* F5: kinematic smoothing state.
   * Caches the previous-frame kinematic body transforms so that large
   * bone-driven displacements can be split into N interpolated sub-steps
   * instead of teleporting kinematic bodies (which would inject huge
   * impulse into joints and kill dynamic body inertia).
   * Mirrors MikuMikuPhysics `_step_with_kinematic_smoothing` /
   * `_last_kinematic_matrices`. */
  Vector<btTransform> last_kinematic_transforms_;
  Vector<btTransform> prev_prev_kinematic_transforms_;
  Vector<btTransform> current_kinematic_targets_;
  bool has_last_kinematic_ = false;
  bool has_prev_prev_kinematic_ = false;
  float kinematic_smoothing_move_ = 0.03f;   /* Max translation per sub-step (m). */
  float kinematic_smoothing_angle_ = 0.14f;  /* Max rotation per sub-step (rad, ~8°). */
  /* Max kinematic-smoothing segments per frame. Each segment overrides
   * Bullet's `fixedTimeStep = sub_timestep` (see `step()` override arg) so
   * Bullet always advances exactly 1 substep per segment regardless of
   * count — mirrors MikuMikuPhysics `pmx_bullet_api.cpp:1115` trick.
   *
   * Currently capped at 4 = timestep/fixed_timestep = (1/30)/(1/120) to
   * limit per-frame joint error accumulation. The active
   * `pullback_locked_joints_` pass mitigates locked-translation anchor drift,
   * but raising this cap remains a separate timing/quality experiment rather
   * than an implied safe default. */
  int kinematic_smoothing_max_segments_ = 4;
  /* Configuration. */
  Object *armature_ = nullptr;
  Main *bmain_ = nullptr;
  bPose *pose_at_initialize_ = nullptr;
  uint32_t armature_session_uid_ = 0;
  float gravity_[3] = {0.0f, 0.0f, -9.81f};
  int solver_iterations_ = 20;
  int fixed_step_hz_ = 60;
  int max_substeps_ = 10;
  bool disable_rigid_body_contacts_ = false;
  bool disable_joint_springs_ = false;
  float joint_spring_damping_ = 0.15f;
  /* 0 scans the complete simple chain; positive values cap graph depth. */
  int joint_collision_exclusion_depth_ = 0;
  double fixed_timestep_ = 1.0 / 60.0;

  MMDPhysicsPerformance performance_;
  bool initialized_ = false;
  bool defer_constraint_correction_ = false;
};

}  // namespace blender::mmd_physics
