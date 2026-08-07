/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "mmd_physics_definition.hh"

#include <array>
#include <string>
#include <vector>

namespace blender {

struct Main;
struct Object;
struct Depsgraph;

namespace mmd_physics {

struct MMDDiagnosticFrame;

struct MMDDiagnosticCaptureOptions {
  std::array<float, 3> gravity = {0.0f, 0.0f, -9.81f};
  int solver_iterations = 20;
  int fixed_step_hz = 120;
  int steps = 240;
  int startup_prewarm_steps = 2;
  int startup_sync_steps = 0;
  bool capture_initial_state = false;
  bool disable_rigid_body_contacts = false;
  bool disable_joint_springs = false;
  /* Bullet 2.82 spring damping: 1 = no damping (spring fully active, prone to
   * oscillation), 0 = spring fully damped. 0.15 is the F5e baseline that
   * keeps spring response weak enough to avoid overshoot oscillation.
   * NOTE: raising this to 0.85 (MMP zone preset value) WITHOUT also porting
   * MMP's linear/angular_damping_scale and joint_stop_erp/cfm causes severe
   * oscillation (左胸下 max_av 3.34->68.79 rad/s, 20x worse). */
  float joint_spring_damping = 0.15f;
  /* 0 scans the complete simple chain; positive values cap graph depth. */
  int joint_collision_exclusion_depth = 0;

  /* Mesh vertex penetration sampling (schema v1.1).
   *
   * Output volume control is critical: a typical MMD model has 10k+ vertices
   * and 400+ rigid bodies. Naive per-vertex-per-body AABB tests would
   * produce 4M+ checks per step and megabytes of JSONL per capture. The
   * four controls below enforce strict output limits:
   *
   *   - `mesh_sample_bones` empty → mesh sampling disabled entirely.
   *   - Only vertices whose top-weight bone is in `mesh_sample_bones` are
   *     considered; all other vertices are skipped.
   *   - Per bone, at most `mesh_max_vertices_per_bone` vertices are sampled
   *     (uniform stride selection).
   *   - Sampling runs every `mesh_sample_interval` steps, not every step.
   *   - Only vertices found INSIDE some rigid body's AABB are emitted;
   *     non-penetrating vertices produce no output. */
  std::vector<std::string> mesh_sample_bones;
  int mesh_sample_interval = 30;
  int mesh_max_vertices_per_bone = 16;
};

struct MMDDiagnosticCaptureResult {
  bool success = false;
  int captured_steps = 0;
  int body_count = 0;
  int joint_count = 0;
  std::string jsonl_path;
  std::string summary_path;
  std::string error;
};

/**
 * Capture objective physics diagnostics from an isolated temporary Bullet world.
 *
 * `depsgraph` is used only for mesh vertex penetration sampling (when
 * `options.mesh_sample_bones` is non-empty): it provides the evaluated mesh
 * with armature-deformed vertex positions. Pass nullptr to skip mesh sampling
 * (raw mesh data will be used as a fallback, which reflects bind-pose and is
 * inaccurate for posed armatures).
 */
MMDDiagnosticCaptureResult capture_mmd_physics_diagnostics(
    const MMDPhysicsDefinition &definition,
    Object *armature,
    Main *bmain,
    Depsgraph *depsgraph,
    const MMDDiagnosticCaptureOptions &options);

/**
 * Append one objective frame from an already-running physics world to JSONL.
 * The caller owns frame capture; this function only serializes the supplied
 * state and never advances Bullet or writes Blender pose channels.
 */
bool append_mmd_physics_diagnostic_frame(const std::string &path,
                                         const MMDDiagnosticFrame &frame);

}  // namespace mmd_physics
}  // namespace blender
