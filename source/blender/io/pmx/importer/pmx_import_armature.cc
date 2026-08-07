/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_pmx
 */

#include "DNA_armature_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_armature.hh"
#include "BKE_collection.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "BLI_listbase.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "ED_armature.hh"

#include "intern/pmx_types.h"
#include "pmx_import_armature.hh"
#include "pmx_import_mesh.hh"

#include <cmath>

namespace blender::io::pmx {

/* -------------------------------------------------------------------- */
/** \name Utility helpers
 * \{ */

/**
 * Check that a float3 contains only finite values; return true if safe.
 */
static bool is_finite_vec3(const float v[3])
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

/**
 * Copy and clamp to finite — replace non-finite values with 0.
 */
static void clamp_finite_v3(float out[3], const float in[3])
{
  for (int i = 0; i < 3; i++) {
    out[i] = std::isfinite(in[i]) ? in[i] : 0.0f;
  }
}

/**
 * Transform PMX bone position to Blender, with finite safety.
 * Uses the same transform_coord() that mesh creation uses.
 */
static void transform_bone_coord(float out[3], const float in[3], const float scale)
{
  float safe[3];
  clamp_finite_v3(safe, in);
  transform_coord(out, safe, scale);
}

/**
 * Compute Blender bone roll from a desired local Z-axis in Blender space.
 *
 * Blender bone convention:
 *   Y = direction (head→tail)
 *   Default X (roll=0) = normalize(cross(Y, Z_global))
 *   Roll = rotation around Y from default X to desired X
 *
 * The desired X is derived from the PMX local_z (cross(Y, local_z) in Blender space).
 */
static float compute_roll_from_axis(const float dir[3], const float axis_z[3])
{
  /* Default X axis at roll=0. */
  float def_x[3];
  const float z_up[3] = {0.0f, 0.0f, 1.0f};
  if (fabsf(dir[2]) < 1.0f - 1e-6f) {
    cross_v3_v3v3(def_x, dir, z_up);
    normalize_v3(def_x);
  }
  else {
    /* Bone direction is nearly parallel to global Z. Fallback to global X. */
    const float x_ref[3] = {1.0f, 0.0f, 0.0f};
    project_plane_normalized_v3_v3v3(def_x, x_ref, dir);
    normalize_v3(def_x);
  }

  /* Desired X axis: cross(Y, desired_Z). */
  float desired_x[3];
  cross_v3_v3v3(desired_x, dir, axis_z);
  float len = normalize_v3(desired_x);
  if (len < 1e-8f) {
    return 0.0f;
  }

  /* Roll = atan2(|def_x × desired_x| · dir, def_x · desired_x). */
  float cross[3];
  cross_v3_v3v3(cross, def_x, desired_x);
  float sin_a = dot_v3v3(cross, dir);
  float cos_a = dot_v3v3(def_x, desired_x);
  return -atan2f(sin_a, cos_a);
}

/**
 * Compute Blender bone roll from PMX local axes (local_x + local_z).
 * Mirrors mmd_tools FnBone.update_bone_roll + get_axes algorithm
 * (core/bone.py:313-325).
 *
 * 1. Orthogonalize (x, y, z) from (local_x, local_z):
 *    x = local_x, y = z × x, z = x × y (correction).
 * 2. Find axis most parallel to bone direction (idx).
 * 3. Pick perpendicular axis: (idx+1)%3 if same direction, (idx-1)%3 if opposite.
 * 4. Use ED_armature_ebone_roll_to_vector to align bone Z to that axis
 *    (equivalent to mmd_tools edit_bone.align_roll).
 *
 * When local_x is unavailable or parallel to local_z, falls back to
 * compute_roll_from_axis (z-only) for backward compatibility.
 *
 * Why: PMX local_x and local_z are not always orthogonal. Using only
 * local_z (the old approach) produces a different roll than mmd_tools
 * when they are non-orthogonal, leading to different arm_mat and thus
 * different VMD quaternion conversions — visible as arm posture errors
 * on NXDE frame 939 etc.
 */
static float compute_roll_from_local_axes(EditBone *ebone,
                                          const float dir[3],
                                          const float axis_x[3],
                                          const float axis_z[3])
{
  /* Orthogonalize (mmd_tools get_axes). */
  float x[3], z[3], y[3];
  if (normalize_v3_v3(x, axis_x) < 1e-6f) {
    return compute_roll_from_axis(dir, axis_z);
  }
  normalize_v3_v3(z, axis_z);
  cross_v3_v3v3(y, z, x);
  if (normalize_v3(y) < 1e-6f) {
    /* local_x parallel to local_z — fallback. */
    return compute_roll_from_axis(dir, axis_z);
  }
  cross_v3_v3v3(z, x, y); /* correction */
  normalize_v3(z);

  /* mmd_tools update_bone_roll axis selection:
   * idx = argmax(|dir · axes[i]|), val = dir · axes[idx]
   * perp = axes[(idx-1)%3] if val < 0 else axes[(idx+1)%3] */
  float dots[3] = {dot_v3v3(dir, x), dot_v3v3(dir, y), dot_v3v3(dir, z)};
  int idx = 0;
  if (fabsf(dots[1]) > fabsf(dots[idx])) {
    idx = 1;
  }
  if (fabsf(dots[2]) > fabsf(dots[idx])) {
    idx = 2;
  }

  float val = dots[idx];
  int perp_idx = (val < 0.0f) ? (idx - 1 + 3) % 3 : (idx + 1) % 3;

  float perp[3];
  if (perp_idx == 0) {
    copy_v3_v3(perp, x);
  }
  else if (perp_idx == 1) {
    copy_v3_v3(perp, y);
  }
  else {
    copy_v3_v3(perp, z);
  }

  /* align_roll: project perp onto plane ⊥ bone dir, set roll.
   * ED_armature_ebone_roll_to_vector aligns bone's Z axis to the vector. */
  return ED_armature_ebone_roll_to_vector(ebone, perp, false);
}

/**
 * Bone name lists for auto local-axis roll computation.
 * Mirrors mmd_tools FnBone.AUTO_LOCAL_AXIS_* (core/bone.py:44-46).
 *
 * PMX bones in this list do not carry explicit localCoordinate, but mmd_tools
 * derives a roll from the bone's geometry (see update_auto_bone_roll). Native
 * import must match, otherwise matrix_local differs and VMD-driven poses
 * diverge from mmd_tools (e.g. NXDE frame 0 left elbow).
 */
static bool has_auto_local_axis(const char *name)
{
  if (name == nullptr || name[0] == '\0') {
    return false;
  }
  /* AUTO_LOCAL_AXIS_ARMS — exact match. */
  static const char *kArms[] = {
      "左肩", "左腕", "左ひじ", "左手首", "右腕", "右肩", "右ひじ", "右手首",
  };
  for (const char *a : kArms) {
    if (strcmp(name, a) == 0) {
      return true;
    }
  }
  /* AUTO_LOCAL_AXIS_SEMI_STANDARD_ARMS — exact match. */
  static const char *kSemi[] = {
      "左腕捩", "左手捩", "左肩P", "左ダミー", "右腕捩", "右手捩", "右肩P", "右ダミー",
  };
  for (const char *a : kSemi) {
    if (strcmp(name, a) == 0) {
      return true;
    }
  }
  /* AUTO_LOCAL_AXIS_FINGERS — substring match. */
  static const char *kFingers[] = {"親指", "人指", "中指", "薬指", "小指"};
  for (const char *f : kFingers) {
    if (strstr(name, f) != nullptr) {
      return true;
    }
  }
  return false;
}

/**
 * Compute bone roll for bones without explicit PMX localCoordinate but
 * recognized by mmd_tools' auto local-axis heuristic.
 *
 * Mirrors mmd_tools FnBone.update_auto_bone_roll (core/bone.py:339-357):
 *   1. Build triangle (p1, p2, p3) where p3 = p2 rotated 90° in global XZ.
 *   2. y = bone direction, z_tmp = (p3-p1).normalized, x = y × z_tmp.
 *   3. update_bone_roll(eb, y.xzy, x.xzy) — mmd_tools passes MMD coords,
 *      get_axes converts back via .xzy, net effect: axis_x = y, axis_z = x
 *      in Blender space, which is what compute_roll_from_local_axes expects.
 *
 * For +Z fallback tails (head + (0,0,scale)) this yields roll = -π/2,
 * matching mmd_tools' default for shoulder P, arm twist, hand twist, etc.
 */
static float update_auto_bone_roll(EditBone *ebone)
{
  float p1[3], p2[3], p3[3];
  copy_v3_v3(p1, ebone->head);
  copy_v3_v3(p2, ebone->tail);
  copy_v3_v3(p3, p2);

  /* theta = atan2(dz, dx) — rotation of bone's XZ projection.
   * atan2(0,0) = 0, matching mmd_tools' behavior for vertical bones. */
  float dx = p2[0] - p1[0];
  float dz = p2[2] - p1[2];
  float theta = atan2f(dz, dx);
  float norm = len_v3v3(p2, p1);
  p3[2] += norm * cosf(theta);
  p3[0] -= norm * sinf(theta);

  /* y = (p2-p1).normalized() — bone direction in Blender space. */
  float y[3];
  sub_v3_v3v3(y, p2, p1);
  if (normalize_v3(y) < 1e-8f) {
    return 0.0f;
  }

  /* z_tmp = (p3-p1).normalized(). */
  float z_tmp[3];
  sub_v3_v3v3(z_tmp, p3, p1);
  normalize_v3(z_tmp);

  /* x = y × z_tmp — normal vector of the triangle face. */
  float x[3];
  cross_v3_v3v3(x, y, z_tmp);
  normalize_v3(x);

  /* compute_roll_from_local_axes takes Blender-space axis_x / axis_z.
   * mmd_tools passes y.xzy / x.xzy (MMD), get_axes converts back via .xzy,
   * so the net axis_x = y, axis_z = x in Blender space. */
  return compute_roll_from_local_axes(ebone, y, y, x);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Armature creation
 * \{ */

PMXArmatureResult create_armature_object(Main *bmain,
                                          const PMXModel &model,
                                          const PMXImportParams &params,
                                          const char *base_name,
                                          ViewLayer *view_layer,
                                          Object *root_obj,
                                          Collection *model_collection)
{
  PMXArmatureResult result;
  const int bone_count = int(model.bones.size());
  if (bone_count == 0) {
    return result;
  }

  const float scale = params.global_scale;

  /* ------------------------------------------------------------------ */
  /* 1. Create bArmature data block. */
  /* ------------------------------------------------------------------ */
  std::string arm_data_name = std::string(base_name) + "Armature";
  bArmature *arm = BKE_armature_add(bmain, arm_data_name.c_str());

  /* ------------------------------------------------------------------ */
  /* 2. Create Armature Object. */
  /* ------------------------------------------------------------------ */
  std::string arm_obj_name = std::string(base_name) + "Arm";
  Object *arm_obj = BKE_object_add_only_object(bmain, OB_ARMATURE, arm_obj_name.c_str());
  arm_obj->data = &arm->id;

  /* Match mmd_tools' in-front display while keeping Blender's conventional
   * octahedral bone appearance. In this Blender version the settings are
   * stored in DNA flags directly; the RNA names are show_in_front and
   * display_type. */
  arm_obj->dtx |= OB_DRAW_IN_FRONT;
  arm_obj->dt = OB_WIRE;
  arm->drawtype = ARM_DRAW_TYPE_OCTA;

  /* Keep the armature directly in the imported model collection. */
  if (model_collection) {
    BKE_collection_object_add(bmain, model_collection, arm_obj);
  }
  if (root_obj) {
    arm_obj->parent = root_obj;
    arm_obj->partype = PAROBJECT;
    unit_m4(arm_obj->parentinv);
  }

  /* ------------------------------------------------------------------ */
  /* 3. Enter edit mode. */
  /* ------------------------------------------------------------------ */
  ED_armature_to_edit(arm);

  Vector<EditBone *> edit_bones_by_index;
  Vector<std::string> bone_names_by_index;
  edit_bones_by_index.reserve(bone_count);
  bone_names_by_index.reserve(bone_count);

  /* ------------------------------------------------------------------ */
  /* 4. First pass: create all EditBones. */
  /* ------------------------------------------------------------------ */
  for (int i = 0; i < bone_count; i++) {
    const char *name = model.bones[i].name_local.empty()
                           ? "Bone"
                           : model.bones[i].name_local.c_str();
    EditBone *ebone = ED_armature_ebone_add(arm, name);
    edit_bones_by_index.append(ebone);
    bone_names_by_index.append(ebone->name);
  }

  /* ------------------------------------------------------------------ */
  /* 5. Second pass: set head positions and parent. */
  /* ------------------------------------------------------------------ */
  for (int i = 0; i < bone_count; i++) {
    const PMXBone &pmx_bone = model.bones[i];
    EditBone *ebone = edit_bones_by_index[i];

    /* Head. */
    transform_bone_coord(ebone->head, pmx_bone.pos, scale);

    /* Parent, with safety checks. */
    if (pmx_bone.parent_index >= 0 && pmx_bone.parent_index < bone_count &&
        pmx_bone.parent_index != i)
    {
      ebone->parent = edit_bones_by_index[pmx_bone.parent_index];
    }
  }

  /* ------------------------------------------------------------------ */
  /* 6. Third pass: set tail positions. */
  /* ------------------------------------------------------------------ */
  for (int i = 0; i < bone_count; i++) {
    const PMXBone &pmx_bone = model.bones[i];
    EditBone *ebone = edit_bones_by_index[i];

    if (pmx_bone.tail_pos_bone >= 0 && pmx_bone.tail_pos_bone < bone_count) {
      /* Tail points to another bone's head. */
      copy_v3_v3(ebone->tail, edit_bones_by_index[pmx_bone.tail_pos_bone]->head);
    }
    else if (pmx_bone.tail_pos_bone == -2) {
      /* Tail is an offset from head. */
      float offset[3];
      transform_bone_coord(offset, pmx_bone.tail_pos_offset, scale);
      add_v3_v3v3(ebone->tail, ebone->head, offset);
    }
    else {
      /* No valid tail — will be fixed in the zero-length pass. */
      copy_v3_v3(ebone->tail, ebone->head);
    }
  }

  /* ------------------------------------------------------------------ */
  /* 7. Fourth pass: IK chain tail fix. */
  /* ------------------------------------------------------------------ */
  for (int i = 0; i < bone_count; i++) {
    const PMXBone &pmx_bone = model.bones[i];
    if (!(pmx_bone.flag & BONE_FLAG_IK)) {
      continue;
    }
    if (pmx_bone.ik_target_index < 0 || pmx_bone.ik_target_index >= bone_count) {
      continue;
    }

    EditBone *ik_chain_end = edit_bones_by_index[pmx_bone.ik_target_index];

    for (int li = 0; li < int(pmx_bone.ik_links.size()); li++) {
      const PMXIKLink &link = pmx_bone.ik_links[li];
      if (link.bone_index < 0 || link.bone_index >= bone_count) {
        continue;
      }

      EditBone *link_bone = edit_bones_by_index[link.bone_index];
      if (link_bone->length < 1e-4f) {
        /* Pick the chain node above as the tail target. */
        EditBone *tail_target = (li == 0) ? ik_chain_end :
                                            edit_bones_by_index[pmx_bone.ik_links[li - 1].bone_index];
        if (tail_target) {
          float vec[3];
          sub_v3_v3v3(vec, tail_target->head, link_bone->head);
          float len = normalize_v3(vec);
          if (len > 1e-4f) {
            mul_v3_fl(vec, len);
            add_v3_v3v3(link_bone->tail, link_bone->head, vec);
          }
        }
      }
    }
  }

  /* ------------------------------------------------------------------ */
  /* 8. Fifth pass: zero-length bone fix. */
  /* ------------------------------------------------------------------ */
  constexpr float kMinBoneLen = 1.0e-4f;
  for (int i = 0; i < bone_count; i++) {
    EditBone *ebone = edit_bones_by_index[i];
    if (len_v3v3(ebone->head, ebone->tail) >= kMinBoneLen) {
      continue;
    }

    const PMXBone &pmx_bone = model.bones[i];
    bool fixed = false;

    /* Priority 1: PMX fixed_axis. */
    if (pmx_bone.flag & BONE_FLAG_FIXED_AXIS) {
      float axis[3];
      clamp_finite_v3(axis, pmx_bone.fixed_axis);
      float axis_len = normalize_v3(axis);
      if (axis_len > 1e-6f) {
        float dir[3];
        transform_bone_coord(dir, axis, 1.0f);
        normalize_v3(dir);
        ebone->tail[0] = ebone->head[0] + dir[0] * scale;
        ebone->tail[1] = ebone->head[1] + dir[1] * scale;
        ebone->tail[2] = ebone->head[2] + dir[2] * scale;
        fixed = true;
      }
    }

    /* Priority 2: direction toward first child, using full distance to child's head. */
    if (!fixed) {
      for (int j = 0; j < bone_count; j++) {
        if (model.bones[j].parent_index == i) {
          float dir[3];
          sub_v3_v3v3(dir, edit_bones_by_index[j]->head, ebone->head);
          float child_dist = normalize_v3(dir);
          if (child_dist > 1e-4f) {
            ebone->tail[0] = ebone->head[0] + dir[0] * child_dist;
            ebone->tail[1] = ebone->head[1] + dir[1] * child_dist;
            ebone->tail[2] = ebone->head[2] + dir[2] * child_dist;
            fixed = true;
          }
          break;
        }
      }
    }

    /* Priority 3: Z axis fallback (matches mmd_tools core/pmx/importer.py:248-256).
     *
     * The old "away from parent" heuristic is intentionally dropped: it gave
     * zero-length bones like 左肩C / 左肩P a real direction, while mmd_tools
     * uses the +Z default. That matrix_local mismatch propagated through the
     * parent chain and broke VMD-driven arm posture on NXDE frame 939.
     *
     * Priority 2 (toward first child) is kept because leg D-bones
     * (左足D / 左ひざD / 左足首D) rely on it; removing it makes the leg float.
     * Bones whose only child shares their head (e.g. 左肩C → 左腕, both at the
     * same position) fall through priority 2 and land here on +Z, matching
     * mmd_tools. */
    if (!fixed) {
      ebone->tail[0] = ebone->head[0];
      ebone->tail[1] = ebone->head[1];
      ebone->tail[2] = ebone->head[2] + scale;
    }
  }

  /* ------------------------------------------------------------------ */
  /* 9. Sixth pass: roll from local_axis / fixed_axis. */
  /* ------------------------------------------------------------------ */
  for (int i = 0; i < bone_count; i++) {
    const PMXBone &pmx_bone = model.bones[i];
    EditBone *ebone = edit_bones_by_index[i];

    float dir[3];
    sub_v3_v3v3(dir, ebone->tail, ebone->head);
    if (normalize_v3(dir) < 1e-8f) {
      continue;
    }

    float axis_z_bl[3];

    if (pmx_bone.flag & BONE_FLAG_LOCAL_AXIS) {
      /* PMX local axes in Blender coordinates.
       * Use local_x + local_z jointly (mmd_tools algorithm) when both
       * are available; fall back to local_z-only for robustness. */
      float local_x[3], local_z[3];
      clamp_finite_v3(local_x, pmx_bone.local_x);
      clamp_finite_v3(local_z, pmx_bone.local_z);
      if (is_finite_vec3(local_x) && is_finite_vec3(local_z) &&
          normalize_v3(local_x) > 1e-6f && normalize_v3(local_z) > 1e-6f)
      {
        float axis_x_bl[3], axis_z_bl2[3];
        transform_bone_coord(axis_x_bl, local_x, 1.0f);
        transform_bone_coord(axis_z_bl2, local_z, 1.0f);
        normalize_v3(axis_x_bl);
        normalize_v3(axis_z_bl2);
        ebone->roll = compute_roll_from_local_axes(ebone, dir, axis_x_bl, axis_z_bl2);
      }
      else if (is_finite_vec3(local_z) && normalize_v3(local_z) > 1e-6f) {
        transform_bone_coord(axis_z_bl, local_z, 1.0f);
        normalize_v3(axis_z_bl);
        ebone->roll = compute_roll_from_axis(dir, axis_z_bl);
      }
    }
    else if (has_auto_local_axis(pmx_bone.name_local.c_str())) {
      /* Auto local-axis roll takes priority over BONE_FLAG_FIXED_AXIS.
       *
       * mmd_tools' roll pass (core/pmx/importer.py:264-266) checks
       * `localCoordinate is not None` first, then `has_auto_local_axis`.
       * It does NOT consult fixed_axis for roll — fixed_axis is only used
       * for zero-length bone tail direction and the MMD_Fixed_Axis_Approx
       * constraint, never for bone roll.
       *
       * Bones like 左腕捩 / 左手捩 carry BONE_FLAG_FIXED_AXIS in PMX (used to
       * build MMD_Fixed_Axis_Approx), so the old `elif FIXED_AXIS` branch
       * hijacked them and computed roll from fixed_axis — diverging from
       * mmd_tools, which uses update_auto_bone_roll for these names. The
       * resulting matrix_local mismatch (~2.55 rad roll error) propagates
       * through the arm chain and twists 左ひじ / 左手首 on NXDE frame 108. */
      ebone->roll = update_auto_bone_roll(ebone);
    }
    else if (pmx_bone.flag & BONE_FLAG_FIXED_AXIS) {
      /* Use fixed_axis as the bone's local Z for roll purposes.
       * Only reached when the bone has no PMX localCoordinate AND does not
       * match mmd_tools' auto local-axis name list. mmd_tools leaves roll=0
       * for such bones, but we use fixed_axis as a sensible fallback to
       * avoid degenerate roll on fixed-axis-only bones. */
      float fixed[3];
      clamp_finite_v3(fixed, pmx_bone.fixed_axis);
      if (is_finite_vec3(fixed) && normalize_v3(fixed) > 1e-6f) {
        transform_bone_coord(axis_z_bl, fixed, 1.0f);
        normalize_v3(axis_z_bl);
        ebone->roll = compute_roll_from_axis(dir, axis_z_bl);
      }
    }
  }

  /* ------------------------------------------------------------------ */
  /* 10. Exit edit mode. */
  /* ------------------------------------------------------------------ */
  ED_armature_from_edit(bmain, arm);
  ED_armature_edit_free(arm);

  /* ------------------------------------------------------------------ */
  /* 11. Build PMX→Blender bone name mapping. */
  /* ------------------------------------------------------------------ */
  result.armature_obj = arm_obj;
  result.bone_names = std::move(bone_names_by_index);

  /* Notify dependency graph. */
  DEG_id_tag_update_ex(bmain, &arm_obj->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  DEG_relations_tag_update(bmain);

  return result;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Armature modifier binding
 * \{ */

void bind_armature_modifiers(PMXImportContext &ctx)
{
  Object *arm_obj = ctx.armature_obj;
  if (!arm_obj) {
    return;
  }

  for (Object *mesh_obj : ctx.mesh_objects) {
    if (!mesh_obj || mesh_obj->type != OB_MESH) {
      continue;
    }

    /* Reuse existing Armature modifier if found. */
    ModifierData *md = BKE_modifiers_findby_type(mesh_obj, eModifierType_Armature);
    if (!md) {
      md = BKE_modifier_new(eModifierType_Armature);
      if (!md) {
        continue;
      }
      BLI_addtail(&mesh_obj->modifiers, md);
      BKE_modifiers_persistent_uid_init(*mesh_obj, *md);
    }

    ArmatureModifierData *amd = reinterpret_cast<ArmatureModifierData *>(md);
    amd->object = arm_obj;
    /* PMX BDEF1/2/4 use linear blend skinning. QDEF needs a per-vertex path and must not
     * globally switch every BDEF4 vertex to Blender's dual-quaternion mode. */
    amd->deformflag |= ARM_DEF_VGROUP;
    amd->deformflag &= ~ARM_DEF_QUATERNION;

    DEG_id_tag_update_ex(ctx.bmain, &mesh_obj->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  }

  DEG_relations_tag_update(ctx.bmain);
}

/** \} */

}  // namespace blender::io::pmx
