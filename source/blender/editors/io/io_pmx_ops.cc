/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editor/io
 */

#ifdef WITH_IO_PMX

#  include "BKE_context.hh"
#  include "BKE_file_handler.hh"
#  include "BKE_report.hh"

#  include "BLI_math_vector_c.hh"
#  include "BLI_path_utils.hh"
#  include "BLI_string.hh"
#  include "BLI_string_utf8.hh"
#  include "BLI_vector.hh"

#  include "DNA_space_types.h"

#  include "ED_armature.hh"
#  include "ED_fileselect.hh"
#  include "ED_outliner.hh"

#  include "RNA_access.hh"
#  include "RNA_define.hh"

#  include "BLT_translation.hh"

#  include "ANIM_bone_collections.hh"

#  include "UI_interface.hh"
#  include "UI_interface_layout.hh"
#  include "UI_resources.hh"

#  include "WM_api.hh"
#  include "WM_types.hh"

#  include "IO_pmx.hh"
#  include "io_pmx_ops.hh"
#  include "io_utils.hh"

#  include "exporter/pmx_export.hh"
#  include "importer/pmx_import_bone_ik.hh"
#  include "importer/pmx_import_bone_append.hh"
#  include "importer/pmx_import_bone_axis.hh"
#  include "importer/pmx_import_pose_snapshot.hh"

#  include <cstring>
#  include <cstdlib>
#  include <string>
#  include <unordered_set>

#  include "BKE_action.hh"

#  include "BKE_armature.hh"
#  include "BKE_constraint.h"
#  include "BKE_idprop.hh"
#  include "BKE_lib_id.hh"
#  include "BLI_path_utils.hh"

#  include "DEG_depsgraph.hh"
#  include "DEG_depsgraph_build.hh"

#  include "BLI_listbase.hh"

#  include "DNA_action_types.h"
#  include "DNA_constraint_types.h"

namespace blender {

static void mmd_organize_bone_collections(Object *ob);

/* [世界的歌] Auto-apply core functions. These extract the application logic
 * from the four manual WM_OT_pmx_apply_* operators so that PMX import can run
 * them in sequence (IK -> Append -> Fixed Axis -> Local Axis) when the user
 * opts in. Each returns the number of bones/constraints applied. They create
 * the same Blender-side approximations (marked mmd_approximate) and are NOT
 * equivalent to MMD native solve. */

static int mmd_auto_apply_ik(Main *bmain, Object *ob, ReportList *reports)
{
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(reports, RPT_ERROR, "Armature has no Pose");
    return 0;
  }
  blender::io::pmx::PMXBoneIKDefinitionSet ik_def;
  if (!blender::io::pmx::read_bone_ik_definition(ob->id, ik_def)) {
    BKE_report(reports, RPT_WARNING, "No PMX IK definition on this armature");
    return 0;
  }
  if (ik_def.ik_bones.empty()) {
    return 0;
  }
  int applied = 0;
  for (const auto &def : ik_def.ik_bones) {
    /* Constraint placement: use links[N-2] (second-to-last link) when available.
     * This places leg IK on the knee regardless of whether the chain has 2 links
     * [knee, thigh] or 3 links [ankle, knee, thigh]. */
    bPoseChannel *pchan = nullptr;
    int chain_count = int(def.links.size());
    if (def.links.size() >= 2) {
      /* links.back() = root bone (e.g. thigh). links[N-2] = constraint bone (e.g. knee). */
      pchan = BKE_pose_channel_find_name(ob->pose,
                                         def.links[def.links.size() - 2].bone_name.c_str());
      /* Control only the constraint bone and its parent (chain=2 for legs/arms). */
      chain_count = 2;
    }
    else if (!def.links.empty()) {
      /* Single-link chain: constraint on the only link (e.g. ankle→toe). */
      pchan = BKE_pose_channel_find_name(ob->pose, def.links[0].bone_name.c_str());
    }
    else {
      pchan = BKE_pose_channel_find_name(ob->pose, def.bone_name.c_str());
    }
    if (pchan == nullptr || chain_count < 1) {
      continue;
    }
    bool already = false;
    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first); con != nullptr;
         con = con->next) {
      if (strcmp(con->name, "MMD_IK_Approx") == 0) {
        already = true;
        break;
      }
    }
    if (already) {
      continue;
    }
    bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, "MMD_IK_Approx", CONSTRAINT_TYPE_KINEMATIC);
    if (con == nullptr) {
      continue;
    }
    bKinematicConstraint *ik = static_cast<bKinematicConstraint *>(con->data);
    ik->tar = ob;
    /* Subtarget: the PMX IK control bone (def.bone_name) is the bone the user
     * moves to drive IK. It is persisted directly from PMX (name_local) and is
     * always imported as a Blender bone, so prefer it over name-pattern guessing.
     *
     * Leg IK:   def.bone_name = "左足ＩＫ",       target bone = "左足首"
     * Toe IK:   def.bone_name = "左つま先ＩＫ",    target bone = "左つま先"
     *
     * The previous name-pattern fallback (root_name + "ＩＫ"/"IK親") only
     * matched leg IK (root = "左足" -> "左足ＩＫ"). For toe IK (root =
     * "左足首", single-link chain) it failed and fell back to
     * def.target_name (the IK target bone itself, a child of the chain),
     * creating a self-referential cycle where the IK target position was
     * driven by the chain bone's own tail — the solver saw the goal as
     * already satisfied and never rotated the ankle, so the foot could not
     * bend independently and only translated with 左足IK親. */
    std::string ik_ctrl;
    if (!def.bone_name.empty() &&
        BKE_pose_channel_find_name(ob->pose, def.bone_name.c_str()) != nullptr)
    {
      ik_ctrl = def.bone_name;
    }
    else if (!def.links.empty()) {
      const std::string &root_name = def.links.back().bone_name;
      const std::string candidates[] = {root_name + "ＩＫ", root_name + "IK親"};
      for (const std::string &cand : candidates) {
        if (BKE_pose_channel_find_name(ob->pose, cand.c_str()) != nullptr) {
          ik_ctrl = cand;
          break;
        }
      }
    }
    if (!ik_ctrl.empty()) {
      STRNCPY_UTF8(ik->subtarget, ik_ctrl.c_str());
    }
    else {
      STRNCPY_UTF8(ik->subtarget, def.target_name.c_str());
    }
    ik->iterations = def.loop_count > 0 ?
                         short(def.loop_count > 1000 ? 1000 : def.loop_count) :
                         short(10);
    /* mmd_tools uses 200 for multi-link chains (legs/arms), 15 for single. */
    if (def.links.size() >= 2 && ik->iterations < 200) {
      ik->iterations = 200;
    }
    else if (def.links.size() < 2 && ik->iterations < 15) {
      ik->iterations = 15;
    }
    ik->rootbone = short(chain_count > 32767 ? 32767 : chain_count);
    ik->type = CONSTRAINT_IK_COPYPOSE;
    /* IK constraints use WORLD space (default) — LOCAL would place the IK
     * target at the subtarget bone's rest-pose origin (model center) instead
     * of its actual armature-space position, causing leg folding. mmd_tools
     * never sets own_space/target_space on IK constraints. */
    con->enforce = 1.0f;

    /* Create LIMIT_ROTATION constraints on IK chain bones from PMX IK link
     * angle limits. This restricts the IK solver from rotating bones beyond
     * their natural range (e.g. the knee can only bend forward/back, not
     * sideways), preventing the 'shattered knee' effect.
     * PMX stores limits in radians in MMD Y-up; swap Y and Z for Blender
     * Z-up. This mirrors mmd_tools' mmd_ik_limit_override constraints. */
    for (const auto &link : def.links) {
      if (!link.limit_angle || link.bone_name == def.target_name) {
        continue;
      }
      bPoseChannel *limit_pchan = BKE_pose_channel_find_name(ob->pose, link.bone_name.c_str());
      if (limit_pchan == nullptr) {
        continue;
      }
      bConstraint *lim_con = BKE_constraint_add_for_pose(
          ob, limit_pchan, "MMD_IK_Limit", CONSTRAINT_TYPE_ROTLIMIT);
      if (lim_con == nullptr) {
        continue;
      }
      bRotLimitConstraint *lim = static_cast<bRotLimitConstraint *>(lim_con->data);
      lim->xmin = -link.limit_max[0];  /* Negate and swap: PMX→Blender X axis flips sign. */
      lim->xmax = -link.limit_min[0];
      lim->ymin = link.limit_min[2];
      lim->ymax = link.limit_max[2];
      lim->zmin = link.limit_min[1];
      lim->zmax = link.limit_max[1];
      lim->flag = eRotLimit_Flags(0);
      /* Only limit axes with meaningful range in the LIMIT_ROTATION constraint.
       * PMX stores Y/Z min=max=0 for knees (no limit defined) — locking them
       * in the post-solve constraint is redundant, so only enable X. */
      if (fabsf(link.limit_max[0] - link.limit_min[0]) > 0.01f) {
        lim->flag |= LIMIT_XROT;
      }
      if (fabsf(link.limit_max[2] - link.limit_min[2]) > 0.01f) {
        lim->flag |= LIMIT_YROT;
      }
      if (fabsf(link.limit_max[1] - link.limit_min[1]) > 0.01f) {
        lim->flag |= LIMIT_ZROT;
      }
      lim_con->ownspace = CONSTRAINT_SPACE_LOCAL;
      lim_con->enforce = 1.0f;
      /* Bone-level IK limits: unconditionally enable all three axes (mirrors
       * mmd_tools core/pmx/importer.py:378-380). iTaSC reads ikflag +
       * limitmin/max DURING solving, not from the LIMIT_ROTATION constraint
       * (post-solve). For knees PMX has Y/Z min=max=0, so enabling Y/Z limit
       * here locks the knee's Y/Z rotation during IK solve — preventing the
       * knee from twisting sideways and causing leg interlacing on VMD frames
       * with large hip rotation (e.g. NXDE frame 1716). Previously we only
       * enabled axes with meaningful range, leaving Y/Z unlocked, which let
       * iTaSC twist the knee and produced a 20° Y-axis rotation difference
       * vs mmd_tools. */
      limit_pchan->ikflag |= BONE_IK_XLIMIT | BONE_IK_YLIMIT | BONE_IK_ZLIMIT;
      limit_pchan->limitmin[0] = lim->xmin;
      limit_pchan->limitmax[0] = lim->xmax;
      limit_pchan->limitmin[1] = lim->ymin;
      limit_pchan->limitmax[1] = lim->ymax;
      limit_pchan->limitmin[2] = lim->zmin;
      limit_pchan->limitmax[2] = lim->zmax;
    }
    IDProperty *pchan_props = pchan->system_properties;
    if (pchan_props == nullptr) {
      pchan_props = blender::bke::idprop::create_group("mmd_ik_approx").release();
      pchan->system_properties = pchan_props;
    }
    IDProperty *old = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_approximate");
    if (old != nullptr) {
      IDP_FreeFromGroup(pchan_props, old);
    }
    IDP_AddToGroup(pchan_props,
                   blender::bke::idprop::create_bool("mmd_approximate", true).release());
    applied++;
  }
  if (applied > 0) {
    BKE_report(reports,
               RPT_INFO,
               "Applied Approximate IK (Blender iTaSC). Not equivalent to MMD CCD. Per-link weights ignored.");
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return applied;
}

static int mmd_auto_apply_append(Main *bmain, Object *ob, ReportList *reports)
{
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(reports, RPT_ERROR, "Armature has no Pose");
    return 0;
  }
  blender::io::pmx::PMXBoneAppendDefinitionSet def;
  if (!blender::io::pmx::read_bone_append_definition(ob->id, def)) {
    BKE_report(reports, RPT_WARNING, "No PMX append-transform definition on this armature");
    return 0;
  }
  if (def.append_bones.empty()) {
    return 0;
  }

  /* ------------------------------------------------------------------
   * Shadow bone creation (mirrors mmd_tools core/bone.py
   * _AT_ShadowBoneCreate). When bone and target_bone are NOT
   * well-aligned (x_axis / y_axis dot < 0.99), reading LOCAL-space
   * rotation directly from target_bone includes bone's own rest-frame
   * orientation offset, doubling the rotation on bones like 左腕捩1/2/3
   * and 左肩调整1/2/3 whose local axes differ from their append parent.
   *
   * The dummy bone (parent=target, same local matrix as bone) plus
   * shadow bone (parent=target.parent, COPY_TRANSFORMS from dummy in
   * POSE space) isolates target_bone's local rotation relative to its
   * own parent — matching mmd_tools' apply_additional_rotation.
   * ------------------------------------------------------------------ */
  struct ShadowReq {
    int def_index;
    std::string dummy_name;
    std::string shadow_name;
  };
  Vector<ShadowReq> shadow_reqs;

  for (int i = 0; i < int(def.append_bones.size()); i++) {
    const auto &ad = def.append_bones[i];
    if (ad.bone_name == ad.parent_name) {
      continue;
    }
    Bone *bone = BKE_armature_find_bone_name(arm, ad.bone_name.c_str());
    Bone *target = BKE_armature_find_bone_name(arm, ad.parent_name.c_str());
    if (bone == nullptr || target == nullptr) {
      continue;
    }
    /* Well-aligned check (mirrors mmd_tools _AT_ShadowBoneCreate.__is_well_aligned):
     * x_axis and y_axis dot > 0.99 means bone and target share the same
     * local orientation — LOCAL-space rotation read from target equals
     * the rotation we actually want, no shadow bone needed. */
    const float dot_x = dot_v3v3(bone->bone_mat[0], target->bone_mat[0]);
    const float dot_y = dot_v3v3(bone->bone_mat[1], target->bone_mat[1]);
    if (dot_x > 0.99f && dot_y > 0.99f) {
      continue;
    }
    shadow_reqs.append({i, "_dummy_" + ad.bone_name, "_shadow_" + ad.bone_name});
  }

  /* Enter edit mode and create dummy + shadow bones in one batch. */
  if (!shadow_reqs.is_empty()) {
    ED_armature_to_edit(arm);
    for (const auto &req : shadow_reqs) {
      const auto &ad = def.append_bones[req.def_index];
      EditBone *bone = ED_armature_ebone_find_name(arm->edbo, ad.bone_name.c_str());
      EditBone *target = ED_armature_ebone_find_name(arm->edbo, ad.parent_name.c_str());
      if (bone == nullptr || target == nullptr) {
        continue;
      }

      /* dummy: parent=target, head=target.head,
       * tail=target.head + (bone.tail - bone.head), roll=bone.roll.
       * This makes dummy's local matrix equal to bone's local matrix. */
      EditBone *dummy = ED_armature_ebone_add(arm, req.dummy_name.c_str());
      dummy->parent = target;
      copy_v3_v3(dummy->head, target->head);
      sub_v3_v3v3(dummy->tail, bone->tail, bone->head);
      add_v3_v3(dummy->tail, target->head);
      dummy->roll = bone->roll;
      dummy->flag |= BONE_NO_DEFORM;

      /* shadow: parent=target.parent, head=dummy.head,
       * tail=dummy.tail, roll=bone.roll. Same local matrix as dummy
       * (and bone), but parented to target's parent. */
      EditBone *shadow = ED_armature_ebone_add(arm, req.shadow_name.c_str());
      shadow->parent = target->parent;
      copy_v3_v3(shadow->head, dummy->head);
      copy_v3_v3(shadow->tail, dummy->tail);
      shadow->roll = bone->roll;
      shadow->flag |= BONE_NO_DEFORM;
    }
    ED_armature_from_edit(bmain, arm);
    ED_armature_edit_free(arm);
    /* Rebuild pose channels so the new shadow/dummy bones are addressable. */
    BKE_pose_ensure(bmain, ob, arm, true);
  }

  /* Add COPY_TRANSFORMS from dummy to shadow in POSE space. This makes
   * shadow's pose matrix = target's pose_matrix @ bone's local matrix,
   * so reading shadow in LOCAL space yields target's rotation relative
   * to target's parent (no bone rest-frame contamination). */
  for (const auto &req : shadow_reqs) {
    bPoseChannel *shadow_pchan = BKE_pose_channel_find_name(ob->pose, req.shadow_name.c_str());
    if (shadow_pchan == nullptr) {
      continue;
    }
    bConstraint *copy_con = BKE_constraint_add_for_pose(
        ob, shadow_pchan, "mmd_tools_at_dummy", CONSTRAINT_TYPE_TRANSLIKE);
    if (copy_con != nullptr) {
      bTransLikeConstraint *translike = static_cast<bTransLikeConstraint *>(copy_con->data);
      translike->tar = ob;
      STRNCPY_UTF8(translike->subtarget, req.dummy_name.c_str());
      copy_con->tarspace = CONSTRAINT_SPACE_POSE;
      copy_con->ownspace = CONSTRAINT_SPACE_POSE;
      copy_con->enforce = 1.0f;
    }
  }

  int applied = 0;
  for (int i = 0; i < int(def.append_bones.size()); i++) {
    const auto &ad = def.append_bones[i];
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    bool already = false;
    if (pchan->system_properties != nullptr) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(pchan->system_properties, "mmd_append_applied", IDP_BOOLEAN);
      if (p != nullptr && IDP_bool_get(p)) {
        already = true;
      }
    }
    if (already) {
      continue;
    }
    /* If a shadow bone was created for this append, use it as subtarget;
     * otherwise fall back to the parent bone directly. */
    std::string subtarget = ad.parent_name;
    for (const auto &req : shadow_reqs) {
      if (req.def_index == i) {
        subtarget = req.shadow_name;
        break;
      }
    }
    auto build_transform = [&](const char *name, const eTransform_ToFrom kind) {
      bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, name, CONSTRAINT_TYPE_TRANSFORM);
      if (con == nullptr) {
        return;
      }
      bTransformConstraint *tcon = static_cast<bTransformConstraint *>(con->data);
      tcon->tar = ob;
      STRNCPY_UTF8(tcon->subtarget, subtarget.c_str());
      tcon->from = kind;
      tcon->to = kind;
      tcon->map[0] = 0;
      tcon->map[1] = 1;
      tcon->map[2] = 2;
      tcon->expo = 0;
      tcon->to_euler_order = CONSTRAINT_EULER_XYZ;
      const float k = ad.ratio;
      if (kind == TRANS_ROTATION) {
        /* Mirror mmd_tools' apply_additional_rotation: negative ratio (cancel
         * rotation, e.g. 肩C) uses ZYX input order; positive uses XYZ.
         * See mmd_tools/core/bone.py apply_additional_rotation. */
        tcon->from_rotation_mode = (k < 0.0f) ? ROT_MODE_ZYX : ROT_MODE_XYZ;
        constexpr float HALF = 3.14159265f;
        for (int i = 0; i < 3; i++) {
          tcon->from_min_rot[i] = -HALF;
          tcon->from_max_rot[i] = HALF;
          tcon->to_min_rot[i] = -HALF * k;
          tcon->to_max_rot[i] = HALF * k;
        }
        tcon->mix_mode_rot = TRANS_MIXROT_AFTER;
      }
      else {
        constexpr float BIG = 1e4f;
        for (int i = 0; i < 3; i++) {
          tcon->from_min[i] = -BIG;
          tcon->from_max[i] = BIG;
          tcon->to_min[i] = -BIG * k;
          tcon->to_max[i] = BIG * k;
        }
        tcon->mix_mode_loc = TRANS_MIXLOC_ADD;
      }
      con->ownspace = CONSTRAINT_SPACE_LOCAL;
      con->tarspace = CONSTRAINT_SPACE_LOCAL;
      con->enforce = 1.0f;
      applied++;
    };
    if (ad.mode & 1) {
      build_transform("MMD_Append_Rotation", TRANS_ROTATION);
    }
    if (ad.mode & 2) {
      build_transform("MMD_Append_Translation", TRANS_LOCATION);
    }
    IDProperty *pchan_props = pchan->system_properties;
    if (pchan_props == nullptr) {
      pchan_props = blender::bke::idprop::create_group("mmd_append").release();
      pchan->system_properties = pchan_props;
    }
    IDProperty *old = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_append_applied");
    if (old != nullptr) {
      IDP_FreeFromGroup(pchan_props, old);
    }
    IDP_AddToGroup(pchan_props, blender::bke::idprop::create_bool("mmd_append_applied", true).release());
  }
  if (applied > 0) {
    BKE_report(reports,
               RPT_INFO,
               "Applied Approximate Append Transform. Negative ratio may introduce visual "
               "artifacts near ±180° rotation limits due to Euler angle discontinuity. Not "
               "equivalent to MMD native implementation. Transform order approximated by bone "
               "index. May not match MMD native order for complex rigs. Conflicts with VMD baked "
               "animation on these bones — mute or bake VMD before use.");
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return applied;
}

static int mmd_auto_apply_fixed_axis(Main *bmain, Object *ob, ReportList *reports)
{
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(reports, RPT_ERROR, "Armature has no Pose");
    return 0;
  }
  blender::io::pmx::PMXBoneAxisDefinitionSet def;
  if (!blender::io::pmx::read_bone_axis_definition(ob->id, def)) {
    BKE_report(reports, RPT_WARNING, "No PMX axis/deform definition on this armature");
    return 0;
  }
  int applied = 0;
  for (const auto &ad : def.bones) {
    if (!ad.has_fixed_axis) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    /* Silent apply: per-bone reports are suppressed in auto-apply mode.
     * A single summary report is emitted below. */
    if (blender::io::pmx::pmx_apply_fixed_axis_to_pchan(ob, pchan, ad, nullptr)) {
      applied++;
    }
  }
  if (applied > 0) {
    BKE_reportf(reports,
               RPT_INFO,
               "Fixed Axis applied: %d bone(s) (native lock for principal axes, "
               "Limit Rotation approximation for arbitrary axes)",
               applied);
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return applied;
}

static int mmd_auto_apply_local_axis(Main *bmain, Object *ob, ReportList *reports)
{
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(reports, RPT_ERROR, "Armature has no Pose");
    return 0;
  }
  blender::io::pmx::PMXBoneAxisDefinitionSet def;
  if (!blender::io::pmx::read_bone_axis_definition(ob->id, def)) {
    BKE_report(reports, RPT_WARNING, "No PMX axis/deform definition on this armature");
    return 0;
  }
  int applied = 0;
  for (const auto &ad : def.bones) {
    if (!ad.has_local_axis) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    if (blender::io::pmx::pmx_apply_local_axis_to_pchan(ob, pchan, ad, nullptr)) {
      applied++;
    }
  }
  if (applied > 0) {
    BKE_reportf(reports,
               RPT_INFO,
               "Local Axis applied: %d bone(s) (Transformation constraint approximation)",
               applied);
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return applied;
}

static wmOperatorStatus wm_pmx_import_exec(bContext *C, wmOperator *op)
{
  PMXImportParams params{};
  params.global_scale = RNA_float_get(op->ptr, "global_scale");
  params.split_by_material = RNA_boolean_get(op->ptr, "split_by_material");
  const bool auto_apply = RNA_boolean_get(op->ptr, "auto_apply_mmd_approximations");

  const auto paths = ed::io::paths_from_operator_properties(op->ptr);
  if (paths.is_empty()) {
    return OPERATOR_CANCELLED;
  }
  STRNCPY(params.filepath, paths[0].c_str());

  WM_cursor_wait(true);
  PMX_import(C, params);
  WM_cursor_wait(false);

  if (auto_apply && params.result_armature != nullptr) {
    Object *arm = params.result_armature;
    Main *bmain_imp = CTX_data_main(C);
    const char *legacy = BLI_getenv("MMD_IK_LEGACY");
    const bool use_legacy = legacy != nullptr && std::strcmp(legacy, "1") == 0;
    const char *v8 = BLI_getenv("MMD_CCD_V8");
    const bool use_v8 = !use_legacy && (v8 == nullptr || std::strcmp(v8, "0") != 0);
    const int a_ik = use_v8 ? 0 : mmd_auto_apply_ik(bmain_imp, arm, op->reports);
    const int a_append = mmd_auto_apply_append(bmain_imp, arm, op->reports);
    const int a_fixed = mmd_auto_apply_fixed_axis(bmain_imp, arm, op->reports);
    const int a_local = mmd_auto_apply_local_axis(bmain_imp, arm, op->reports);
    if (use_v8) {
      BKE_report(op->reports,
                 RPT_INFO,
                 "Skipped Approximate IK because native MMD CCD V8 is enabled by default");
    }
    BKE_reportf(op->reports,
                RPT_INFO,
                "Auto-applied MMD approximations (IK %d, Append %d, Fixed %d, Local %d). "
                "IK targets external control bones (*IK親) where available.",
                a_ik,
                a_append,
                a_fixed,
                a_local);
    mmd_organize_bone_collections(arm);
  }

  Scene *scene = CTX_data_scene(C);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT | ND_OB_ACTIVE, scene);
  return OPERATOR_FINISHED;
}

static void wm_pmx_import_draw(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  PointerRNA *ptr = op->ptr;

  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  layout.prop(ptr, "global_scale", UI_ITEM_NONE, IFACE_("Scale"), ICON_NONE);
  layout.prop(ptr, "split_by_material", UI_ITEM_NONE, IFACE_("Split by Material"), ICON_NONE);
  layout.prop(ptr,
              "auto_apply_mmd_approximations",
              UI_ITEM_NONE,
              IFACE_("Auto-apply MMD Approximations"),
              ICON_NONE);
}

/**
 * Organize armature bones into named collections for cleaner viewport display.
 * Call after auto-apply to separate Main/Deform/IK/Physics/Extras bones.
 */
static void mmd_organize_bone_collections(Object *ob)
{
  bArmature *arm = id_cast<bArmature *>(ob->data);
  if (arm == nullptr) {
    return;
  }
  if (ob->pose == nullptr) {
    BKE_pose_ensure(nullptr, ob, arm, false);
  }
  if (ob->pose == nullptr) {
    return;
  }

  /* Read PMX-persisted definitions for model-agnostic classification. */
  std::unordered_set<std::string> ik_ctrl_bones; /* IK control bones (IK親/ＩＫ). */
  std::unordered_set<std::string> ik_chain_bones; /* Bones that are IK-constrained. */
  std::unordered_set<std::string> append_bones; /* D-bones with append transform. */

  io::pmx::PMXBoneIKDefinitionSet ik_def;
  if (io::pmx::read_bone_ik_definition(ob->id, ik_def)) {
    for (const io::pmx::PMXBoneIKDefinition &def : ik_def.ik_bones) {
      /* The IK bone itself is a control bone (e.g. 左足). */
      ik_ctrl_bones.insert(def.bone_name);
      /* Its target (e.g. 左足ＩＫ) is also a control bone. */
      ik_ctrl_bones.insert(def.target_name);
      /* Chain bones (e.g. 左ひざ) are IK-constrained. */
      for (const io::pmx::PMXBoneIKLink &link : def.links) {
        ik_chain_bones.insert(link.bone_name);
      }
    }
  }

  io::pmx::PMXBoneAppendDefinitionSet ap_def;
  if (io::pmx::read_bone_append_definition(ob->id, ap_def)) {
    for (const io::pmx::PMXBoneAppendDefinition &def : ap_def.append_bones) {
      append_bones.insert(def.bone_name);
    }
  }

  /* Create bone collections. */
  BoneCollection *main_bcoll = ANIM_armature_bonecoll_new(arm, "Main");
  BoneCollection *deform_bcoll = ANIM_armature_bonecoll_new(arm, "Deform");
  BoneCollection *ik_bcoll = ANIM_armature_bonecoll_new(arm, "IK Controls");
  BoneCollection *physics_bcoll = ANIM_armature_bonecoll_new(arm, "Physics");
  BoneCollection *extra_bcoll = ANIM_armature_bonecoll_new(arm, "Extras");

  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(ob->pose->chanbase.first);
       pchan;
       pchan = pchan->next)
  {
    Bone *bone = pchan->bone_get(*ob);
    if (bone == nullptr) {
      continue;
    }
    const std::string name(bone->name);

    /* Physics: widely-recognised physics-driven name patterns. */
    if (STRPREFIX(bone->name, "_shadow_") || STRPREFIX(bone->name, "_dummy_") ||
        STRPREFIX(bone->name, "Bone_Piao") || STRPREFIX(bone->name, "Bone_Hair") ||
        strstr(bone->name, "スカート") != nullptr || strstr(bone->name, "physics") != nullptr ||
        strstr(bone->name, "cloth") != nullptr)
    {
      ANIM_armature_bonecoll_assign(physics_bcoll, bone);
      continue;
    }

    /* Extra: Bip001-style twist helpers and weapon props. */
    if (STRPREFIX(bone->name, "Bip001") || STRPREFIX(bone->name, "Bone_Other") ||
        STRPREFIX(bone->name, "Bone_Constraint") || strstr(bone->name, "WeaponProp") != nullptr)
    {
      ANIM_armature_bonecoll_assign(extra_bcoll, bone);
      continue;
    }

    /* IK Controls: defined by PMX IK definition (control or chain bones). */
    if (ik_ctrl_bones.count(name) > 0 || ik_chain_bones.count(name) > 0) {
      ANIM_armature_bonecoll_assign(ik_bcoll, bone);
      continue;
    }

    /* Deform: D-bones defined by PMX append definition. */
    if (append_bones.count(name) > 0) {
      ANIM_armature_bonecoll_assign(deform_bcoll, bone);
      continue;
    }

    /* Deform fallback: name ends with D / D.L / D.R. */
    const size_t nlen = name.size();
    if (nlen >= 2 && name.compare(nlen - 2, 2, ".D") == 0) {
      ANIM_armature_bonecoll_assign(deform_bcoll, bone);
      continue;
    }
    if (nlen >= 1 && name[nlen - 1] == 'D') {
      ANIM_armature_bonecoll_assign(deform_bcoll, bone);
      continue;
    }

    /* Everything else → Main. */
    ANIM_armature_bonecoll_assign(main_bcoll, bone);
  }

  /* Hide non-essential collections by default. */
  ANIM_bonecoll_hide(arm, deform_bcoll);
  ANIM_bonecoll_hide(arm, physics_bcoll);
  ANIM_bonecoll_hide(arm, extra_bcoll);

  ANIM_armature_bonecoll_active_set(arm, main_bcoll);
  ANIM_armature_runtime_refresh(arm);
}

/* [世界的歌] D1 / Q4: Manual editor operator that creates Blender IK constraints
 * (named MMD_IK_Approx, marked mmd_approximate) to *approximate* MMD CCD IK.
 * This is strictly opt-in and must never run during PMX import (red line 145).
 * The real MMD CCD solver is rebuilt in the E phase and consumes the persisted
 * mmd_pmx_bone_ik_definition + per-bone mmd_ik_toggle instead. */
static bool wm_pmx_apply_ik_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE && ob->data != nullptr;
}

static wmOperatorStatus wm_pmx_apply_ik_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr || ob->type != OB_ARMATURE || ob->data == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Active object must be an MMD PMX Armature");
    return OPERATOR_CANCELLED;
  }
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Armature has no Pose");
    return OPERATOR_CANCELLED;
  }

  blender::io::pmx::PMXBoneIKDefinitionSet ik_def;
  if (!blender::io::pmx::read_bone_ik_definition(ob->id, ik_def)) {
    BKE_report(op->reports,
               RPT_WARNING,
               "No PMX IK definition on this armature; import a PMX model first");
    return OPERATOR_CANCELLED;
  }
  if (ik_def.ik_bones.empty()) {
    BKE_report(op->reports, RPT_INFO, "PMX model has no IK bones; nothing to approximate");
    return OPERATOR_FINISHED;
  }

  int applied = 0;
  for (const blender::io::pmx::PMXBoneIKDefinition &def : ik_def.ik_bones) {
    /* Constraint on links[N-2] (second-to-last link) when available. */
    bPoseChannel *pchan = nullptr;
    int chain_count = int(def.links.size());
    if (def.links.size() >= 2) {
      pchan = BKE_pose_channel_find_name(ob->pose,
                                         def.links[def.links.size() - 2].bone_name.c_str());
      chain_count = 2;
    }
    else if (!def.links.empty()) {
      pchan = BKE_pose_channel_find_name(ob->pose, def.links[0].bone_name.c_str());
    }
    else {
      pchan = BKE_pose_channel_find_name(ob->pose, def.bone_name.c_str());
    }
    if (pchan == nullptr || chain_count < 1) { continue; }
    bool already = false;
    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first); con != nullptr;
         con = con->next) {
      if (strcmp(con->name, "MMD_IK_Approx") == 0) { already = true; break; }
    }
    if (already) { continue; }
    bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, "MMD_IK_Approx", CONSTRAINT_TYPE_KINEMATIC);
    if (con == nullptr) { continue; }
    bKinematicConstraint *ik = static_cast<bKinematicConstraint *>(con->data);
    ik->tar = ob;
    /* Subtarget: find external IK control bone. */
    std::string ik_ctrl;
    if (!def.links.empty()) {
      const std::string &root_name = def.links.back().bone_name;
      const std::string candidates[] = {root_name + "ＩＫ", root_name + "IK親"};
      for (const std::string &cand : candidates) {
        if (BKE_pose_channel_find_name(ob->pose, cand.c_str()) != nullptr) {
          ik_ctrl = cand;
          break;
        }
      }
    }
    if (!ik_ctrl.empty()) {
      STRNCPY_UTF8(ik->subtarget, ik_ctrl.c_str());
    }
    else {
      STRNCPY_UTF8(ik->subtarget, def.target_name.c_str());
    }
    ik->iterations = def.loop_count > 0 ?
                         short(def.loop_count > 1000 ? 1000 : def.loop_count) :
                         short(10);
    /* mmd_tools uses 200 for multi-link chains (legs/arms), 15 for single. */
    if (def.links.size() >= 2 && ik->iterations < 200) {
      ik->iterations = 200;
    }
    else if (def.links.size() < 2 && ik->iterations < 15) {
      ik->iterations = 15;
    }
    ik->rootbone = short(chain_count > 32767 ? 32767 : chain_count);
    ik->type = CONSTRAINT_IK_COPYPOSE;
    con->enforce = 1.0f;

    /* Create LIMIT_ROTATION constraints on IK chain bones from PMX IK link
     * angle limits. This restricts the IK solver from rotating bones beyond
     * their natural range (e.g. the knee can only bend forward/back, not
     * sideways), preventing the 'shattered knee' effect.
     * PMX stores limits in radians in MMD Y-up; swap Y and Z for Blender
     * Z-up. This mirrors mmd_tools' mmd_ik_limit_override constraints. */
    for (const auto &link : def.links) {
      if (!link.limit_angle || link.bone_name == def.target_name) {
        continue;
      }
      bPoseChannel *limit_pchan = BKE_pose_channel_find_name(ob->pose, link.bone_name.c_str());
      if (limit_pchan == nullptr) {
        continue;
      }
      bConstraint *lim_con = BKE_constraint_add_for_pose(
          ob, limit_pchan, "MMD_IK_Limit", CONSTRAINT_TYPE_ROTLIMIT);
      if (lim_con == nullptr) {
        continue;
      }
      bRotLimitConstraint *lim = static_cast<bRotLimitConstraint *>(lim_con->data);
      lim->xmin = -link.limit_max[0];  /* Negate and swap: PMX→Blender X axis flips sign. */
      lim->xmax = -link.limit_min[0];
      lim->ymin = link.limit_min[2];
      lim->ymax = link.limit_max[2];
      lim->zmin = link.limit_min[1];
      lim->zmax = link.limit_max[1];
      lim->flag = eRotLimit_Flags(0);
      /* Only limit axes with meaningful range in the LIMIT_ROTATION constraint.
       * PMX stores Y/Z min=max=0 for knees (no limit defined) — locking them
       * in the post-solve constraint is redundant, so only enable X. */
      if (fabsf(link.limit_max[0] - link.limit_min[0]) > 0.01f) {
        lim->flag |= LIMIT_XROT;
      }
      if (fabsf(link.limit_max[2] - link.limit_min[2]) > 0.01f) {
        lim->flag |= LIMIT_YROT;
      }
      if (fabsf(link.limit_max[1] - link.limit_min[1]) > 0.01f) {
        lim->flag |= LIMIT_ZROT;
      }
      lim_con->ownspace = CONSTRAINT_SPACE_LOCAL;
      lim_con->enforce = 1.0f;
      /* Bone-level IK limits: unconditionally enable all three axes (mirrors
       * mmd_tools core/pmx/importer.py:378-380). iTaSC reads ikflag +
       * limitmin/max DURING solving, not from the LIMIT_ROTATION constraint
       * (post-solve). For knees PMX has Y/Z min=max=0, so enabling Y/Z limit
       * here locks the knee's Y/Z rotation during IK solve — preventing the
       * knee from twisting sideways and causing leg interlacing on VMD frames
       * with large hip rotation (e.g. NXDE frame 1716). Previously we only
       * enabled axes with meaningful range, leaving Y/Z unlocked, which let
       * iTaSC twist the knee and produced a 20° Y-axis rotation difference
       * vs mmd_tools. */
      limit_pchan->ikflag |= BONE_IK_XLIMIT | BONE_IK_YLIMIT | BONE_IK_ZLIMIT;
      limit_pchan->limitmin[0] = lim->xmin;
      limit_pchan->limitmax[0] = lim->xmax;
      limit_pchan->limitmin[1] = lim->ymin;
      limit_pchan->limitmax[1] = lim->ymax;
      limit_pchan->limitmin[2] = lim->zmin;
      limit_pchan->limitmax[2] = lim->zmax;
    }

    /* Mark the pose bone so the approximate nature is queryable (red line 145). */
    IDProperty *pchan_props = pchan->system_properties;
    if (pchan_props == nullptr) {
      pchan_props = blender::bke::idprop::create_group("mmd_ik_approx").release();
      pchan->system_properties = pchan_props;
    }
    IDProperty *old = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_approximate");
    if (old != nullptr) {
      IDP_FreeFromGroup(pchan_props, old);
    }
    IDP_AddToGroup(pchan_props,
                   blender::bke::idprop::create_bool("mmd_approximate", true).release());
    applied++;
  }

  if (applied == 0) {
    BKE_report(op->reports,
               RPT_INFO,
               "Approximate IK already present on all IK bones (MMD_IK_Approx)");
  }
  else {
    /* Q4 required warning: approximate IK is NOT MMD CCD. */
    BKE_report(op->reports,
               RPT_INFO,
               "Applied Approximate IK (Blender iTaSC). Not equivalent to MMD CCD. Per-link weights ignored.");
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return OPERATOR_FINISHED;
}

/* [世界的歌] D2 / Q4: Manual editor operator that creates Blender Transformation
 * constraints (MMD_Append_Rotation / MMD_Append_Translation) to *approximate*
 * MMD append transform (追加変換). Strictly opt-in; must never run during PMX
 * import (red line D2-a). The real native append solver is rebuilt in the E phase.
 * Ratio sign (incl. negative "cancel") is carried by the to_min/to_max range;
 * influence (enforce) stays 1.0. */
static bool wm_pmx_apply_append_transform_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE && ob->data != nullptr;
}

static wmOperatorStatus wm_pmx_apply_append_transform_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr || ob->type != OB_ARMATURE || ob->data == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Active object must be an MMD PMX Armature");
    return OPERATOR_CANCELLED;
  }
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Armature has no Pose");
    return OPERATOR_CANCELLED;
  }

  blender::io::pmx::PMXBoneAppendDefinitionSet def;
  if (!blender::io::pmx::read_bone_append_definition(ob->id, def)) {
    BKE_report(op->reports,
               RPT_WARNING,
               "No PMX append-transform definition on this armature; import a PMX model first");
    return OPERATOR_CANCELLED;
  }
  if (def.append_bones.empty()) {
    BKE_report(op->reports, RPT_INFO, "PMX model has no append-transform bones; nothing to apply");
    return OPERATOR_FINISHED;
  }

  /* Q4 order guard: if an IK definition exists but has not been applied yet
   * (no MMD_IK_Approx on this pchan), warn about reverse ordering. */
  blender::io::pmx::PMXBoneIKDefinitionSet ik_def;
  const bool has_ik = blender::io::pmx::read_bone_ik_definition(ob->id, ik_def) &&
                      !ik_def.ik_bones.empty();

  int applied = 0;
  for (const blender::io::pmx::PMXBoneAppendDefinition &ad : def.append_bones) {
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    /* Idempotency: skip bones already flagged as applied. */
    bool already = false;
    if (pchan->system_properties != nullptr) {
      IDProperty *p = IDP_GetPropertyTypeFromGroup(
          pchan->system_properties, "mmd_append_applied", IDP_BOOLEAN);
      if (p != nullptr && IDP_bool_get(p)) {
        already = true;
      }
    }
    if (already) {
      continue;
    }

    auto build_transform = [&](const char *name, const eTransform_ToFrom kind) {
      bConstraint *con = BKE_constraint_add_for_pose(ob, pchan, name, CONSTRAINT_TYPE_TRANSFORM);
      if (con == nullptr) {
        return;
      }
      bTransformConstraint *tcon = static_cast<bTransformConstraint *>(con->data);
      tcon->tar = ob;
      STRNCPY_UTF8(tcon->subtarget, ad.parent_name.c_str());
      tcon->from = kind;
      tcon->to = kind;
      tcon->map[0] = 0;
      tcon->map[1] = 1;
      tcon->map[2] = 2;
      tcon->expo = 0;
      tcon->to_euler_order = CONSTRAINT_EULER_XYZ;
      /* ratio sign + magnitude carried by the to range; enforce stays 1.0.
       * Symmetric range through origin: owner = target * ratio. */
      const float k = ad.ratio;
      if (kind == TRANS_ROTATION) {
        constexpr float HALF = 3.14159265f; /* ±π */
        for (int i = 0; i < 3; i++) {
          tcon->from_min_rot[i] = -HALF;
          tcon->from_max_rot[i] = HALF;
          tcon->to_min_rot[i] = -HALF * k;
          tcon->to_max_rot[i] = HALF * k;
        }
        tcon->mix_mode_rot = TRANS_MIXROT_AFTER;
      }
      else {
        constexpr float BIG = 1e4f;
        for (int i = 0; i < 3; i++) {
          tcon->from_min[i] = -BIG;
          tcon->from_max[i] = BIG;
          tcon->to_min[i] = -BIG * k;
          tcon->to_max[i] = BIG * k;
        }
        tcon->mix_mode_loc = TRANS_MIXLOC_ADD;
      }
      con->ownspace = CONSTRAINT_SPACE_LOCAL;
      con->tarspace = CONSTRAINT_SPACE_LOCAL;
      con->enforce = 1.0f;
      applied++;
    };

    if (ad.mode & 1) {
      build_transform("MMD_Append_Rotation", TRANS_ROTATION);
    }
    if (ad.mode & 2) {
      build_transform("MMD_Append_Translation", TRANS_LOCATION);
    }

    /* Mark the pose bone so the approximate nature is queryable (red line D2-a). */
    IDProperty *pchan_props = pchan->system_properties;
    if (pchan_props == nullptr) {
      pchan_props = blender::bke::idprop::create_group("mmd_append").release();
      pchan->system_properties = pchan_props;
    }
    IDProperty *old = IDP_GetPropertyFromGroup_null(pchan_props, "mmd_append_applied");
    if (old != nullptr) {
      IDP_FreeFromGroup(pchan_props, old);
    }
    IDP_AddToGroup(pchan_props,
                   blender::bke::idprop::create_bool("mmd_append_applied", true).release());

    /* Q4 reverse-order warning. */
    if (has_ik) {
      bool has_ik_con = false;
      for (bConstraint *c = static_cast<bConstraint *>(pchan->constraints.first); c != nullptr;
           c = c->next) {
        if (strcmp(c->name, "MMD_IK_Approx") == 0) {
          has_ik_con = true;
          break;
        }
      }
      if (!has_ik_con) {
        BKE_report(op->reports,
                   RPT_INFO,
                   "Append transform applied before IK. The IK result will not affect the append "
                   "calculation. Recommended order: Apply IK first, then Append.");
      }
    }
  }

  if (applied == 0) {
    BKE_report(op->reports,
               RPT_INFO,
               "Approximate append transform already present on all append bones");
  }
  else {
    /* Required three-part warning (Q2 negative ratio / Q5 transform order / VMD conflict). */
    BKE_report(op->reports,
               RPT_INFO,
               "Applied Approximate Append Transform. Negative ratio may introduce visual "
               "artifacts near ±180° rotation limits due to Euler angle discontinuity. Not "
               "equivalent to MMD native implementation. Transform order approximated by bone "
               "index. May not match MMD native order for complex rigs. Conflicts with VMD baked "
               "animation on these bones — mute or bake VMD before use.");
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return OPERATOR_FINISHED;
}

void WM_OT_pmx_apply_append_transform(wmOperatorType *ot)
{
  ot->name = "Apply Approximate Append Transform";
  ot->description =
      "Create Blender Transformation constraints (MMD_Append_*) that approximate MMD append "
      "transform. Manual only; not equivalent to MMD native implementation";
  ot->idname = "WM_OT_pmx_apply_append_transform";
  ot->exec = wm_pmx_apply_append_transform_exec;
  ot->poll = wm_pmx_apply_append_transform_poll;
  ot->flag = OPTYPE_UNDO;
}

/* [世界的歌] D3 / Q1+Q2: Manual editor operator that applies MMD FIXED_AXIS.
 * Principal axes (±X/±Y/±Z) are applied as a native `protectflag` lock
 * (exact, no constraint). Arbitrary axes fall back to a Limit Rotation
 * constraint (`MMD_Fixed_Axis_Approx`, marked `mmd_approximate`) as an
 * approximation. Strictly opt-in; must never run during PMX import (red line
 * D3-a). The real native fixed-axis solve is rebuilt in the E phase. */
static bool wm_pmx_apply_fixed_axis_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE && ob->data != nullptr;
}

static wmOperatorStatus wm_pmx_apply_fixed_axis_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr || ob->type != OB_ARMATURE || ob->data == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Active object must be an MMD PMX Armature");
    return OPERATOR_CANCELLED;
  }
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Armature has no Pose");
    return OPERATOR_CANCELLED;
  }

  blender::io::pmx::PMXBoneAxisDefinitionSet def;
  if (!blender::io::pmx::read_bone_axis_definition(ob->id, def)) {
    BKE_report(op->reports,
               RPT_WARNING,
               "No PMX axis/deform definition on this armature; import a PMX model first");
    return OPERATOR_CANCELLED;
  }

  int applied = 0;
  int exact = 0;
  for (const blender::io::pmx::PMXBoneAxisDefinition &ad : def.bones) {
    if (!ad.has_fixed_axis) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    if (blender::io::pmx::pmx_apply_fixed_axis_to_pchan(ob, pchan, ad, op->reports)) {
      applied++;
      if (blender::io::pmx::pmx_fixed_axis_principal_index(ad.fixed_axis) >= 0) {
        exact++;
      }
    }
  }

  if (applied == 0) {
    BKE_report(op->reports, RPT_INFO, "Fixed Axis already applied on all fixed-axis bones");
  }
  else {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Fixed Axis applied: %d bone(s), %d exact (native lock), %d approximate (constraint)",
                applied,
                exact,
                applied - exact);
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return OPERATOR_FINISHED;
}

/* [世界的歌] D3 / Q3: Manual editor operator that approximates MMD LOCAL_AXIS
 * with a Transformation constraint (`MMD_Local_Axis_Approx`, marked
 * `mmd_approximate`). It must NOT modify the bone matrix (red line D3-b) — a
 * single Transformation constraint cannot perform true basis conjugation, so
 * this is an explicit, user-controllable approximation. Strictly opt-in. */
static bool wm_pmx_apply_local_axis_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE && ob->data != nullptr;
}

static wmOperatorStatus wm_pmx_apply_local_axis_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr || ob->type != OB_ARMATURE || ob->data == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Active object must be an MMD PMX Armature");
    return OPERATOR_CANCELLED;
  }
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Armature has no Pose");
    return OPERATOR_CANCELLED;
  }

  blender::io::pmx::PMXBoneAxisDefinitionSet def;
  if (!blender::io::pmx::read_bone_axis_definition(ob->id, def)) {
    BKE_report(op->reports,
               RPT_WARNING,
               "No PMX axis/deform definition on this armature; import a PMX model first");
    return OPERATOR_CANCELLED;
  }

  int applied = 0;
  for (const blender::io::pmx::PMXBoneAxisDefinition &ad : def.bones) {
    if (!ad.has_local_axis) {
      continue;
    }
    bPoseChannel *pchan = BKE_pose_channel_find_name(ob->pose, ad.bone_name.c_str());
    if (pchan == nullptr) {
      continue;
    }
    if (blender::io::pmx::pmx_apply_local_axis_to_pchan(ob, pchan, ad, op->reports)) {
      applied++;
    }
  }

  if (applied == 0) {
    BKE_report(op->reports, RPT_INFO, "Local Axis already applied on all local-axis bones");
  }
  else {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Local Axis applied: %d bone(s) (approximate Transformation constraint)",
                applied);
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return OPERATOR_FINISHED;
}

void WM_OT_pmx_apply_fixed_axis(wmOperatorType *ot)
{
  ot->name = "Apply Fixed Axis";
  ot->description =
      "Apply MMD FIXED_AXIS: native lock_rotation for principal axes, approximate Limit Rotation "
      "constraint otherwise. Manual only; not equivalent to MMD native implementation";
  ot->idname = "WM_OT_pmx_apply_fixed_axis";
  ot->exec = wm_pmx_apply_fixed_axis_exec;
  ot->poll = wm_pmx_apply_fixed_axis_poll;
  ot->flag = OPTYPE_UNDO;
}

void WM_OT_pmx_apply_local_axis(wmOperatorType *ot)
{
  ot->name = "Apply Local Axis";
  ot->description = "Create a Blender Transformation constraint (MMD_Local_Axis_Approx) that "
                    "approximates MMD LOCAL_AXIS. Manual only; not equivalent to MMD native "
                    "implementation";
  ot->idname = "WM_OT_pmx_apply_local_axis";
  ot->exec = wm_pmx_apply_local_axis_exec;
  ot->poll = wm_pmx_apply_local_axis_poll;
  ot->flag = OPTYPE_UNDO;
}

/* [世界的歌] D4 / POSE_DONE: Manual editor operator that captures the final
 * solved Pose into a deterministic `mmd_pose_snapshot` + sets the `mmd_pose_done`
 * logical gate. Strictly opt-in: import and VMD playback never capture
 * automatically (red line D4-a). The capture is pure sampling (red line D4-b). */
static bool wm_pmx_capture_pose_snapshot_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  return ob != nullptr && ob->type == OB_ARMATURE && ob->data != nullptr;
}

static wmOperatorStatus wm_pmx_capture_pose_snapshot_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *ob = CTX_data_active_object(C);
  if (ob == nullptr || ob->type != OB_ARMATURE || ob->data == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Active object must be an MMD PMX Armature");
    return OPERATOR_CANCELLED;
  }
  bArmature *arm = id_cast<bArmature *>(ob->data);
  BKE_pose_ensure(bmain, ob, arm, false);
  if (ob->pose == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Armature has no Pose");
    return OPERATOR_CANCELLED;
  }

  blender::io::pmx::mmd_capture_pose_snapshot(ob, op->reports);

  DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
  DEG_relations_tag_update(bmain);
  return OPERATOR_FINISHED;
}

void WM_OT_pmx_capture_pose_snapshot(wmOperatorType *ot)
{
  ot->name = "Capture Pose Snapshot";
  ot->description =
      "Capture the final solved Pose as a deterministic MMD POSE_DONE snapshot (mmd_pose_snapshot + "
      "mmd_pose_done). Manual only; not triggered on import or VMD playback";
  ot->idname = "WM_OT_pmx_capture_pose_snapshot";
  ot->exec = wm_pmx_capture_pose_snapshot_exec;
  ot->poll = wm_pmx_capture_pose_snapshot_poll;
  ot->flag = OPTYPE_UNDO;
}

void WM_OT_pmx_apply_ik(wmOperatorType *ot)
{
  ot->name = "Apply Approximate IK";
  ot->description = "Create Blender IK constraints (MMD_IK_Approx) that approximate MMD CCD IK. Manual only; not equivalent to MMD CCD";
  ot->idname = "WM_OT_pmx_apply_ik";
  ot->exec = wm_pmx_apply_ik_exec;
  ot->poll = wm_pmx_apply_ik_poll;
  ot->flag = OPTYPE_UNDO;
}

void WM_OT_pmx_import(wmOperatorType *ot)
{
  ot->name = "Import PMX";
  ot->description = "Import an MMD PMX model file";
  ot->idname = "WM_OT_pmx_import";

  ot->invoke = ed::io::filesel_drop_import_invoke;
  ot->exec = wm_pmx_import_exec;
  ot->poll = WM_operator_winactive;
  ot->ui = wm_pmx_import_draw;
  ot->flag = OPTYPE_UNDO | OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);

  RNA_def_float(ot->srna,
                "global_scale",
                0.08f,
                1e-6f,
                1e6f,
                "Scale",
                "",
                0.001f,
                1000.0f);

  RNA_def_boolean(ot->srna,
                  "split_by_material",
                  true,
                  "Split by Material",
                  "Create a separate mesh object for each material");

  RNA_def_boolean(ot->srna,
                  "auto_apply_mmd_approximations",
                  true,
                  "Auto-apply MMD Approximations",
                  "After import, automatically create Blender-side approximations for IK, append "
                  "transform, fixed axis and local axis. These are NOT equivalent to MMD native "
                  "solve; they are auto-suspended when a VMD is imported");

  auto *prop = RNA_def_string(ot->srna, "filter_glob", "*.pmx", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

static wmOperatorStatus wm_pmx_export_invoke(bContext *C,
                                            wmOperator *op,
                                            const wmEvent * /*event*/)
{
  ED_fileselect_ensure_default_filepath(C, op, ".pmx");
  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus wm_pmx_export_exec(bContext *C, wmOperator *op)
{
  /* The model root is the Collection carrying `mmd_pmx_source_data`, not the
   * active object: export reads retained metadata from the Collection and live
   * geometry from the meshes under it. */
  bool ambiguous = false;
  Collection *model_root = io::pmx::find_pmx_model_collection(
      CTX_data_main(C), CTX_data_active_object(C), ambiguous);
  if (model_root == nullptr) {
    if (ambiguous) {
      BKE_report(op->reports,
                 RPT_ERROR,
                 "Several imported PMX models are present; select an object belonging to the one "
                 "to export");
    }
    else {
      BKE_report(op->reports,
                 RPT_ERROR,
                 "No imported PMX model found. PMX export needs the source data that PMX import "
                 "retains on the model collection");
    }
    return OPERATOR_CANCELLED;
  }

  io::pmx::PMXExportOptions options;
  RNA_string_get(op->ptr, "filepath", options.filepath);

  io::pmx::PMXExportReport report;
  if (!io::pmx::export_pmx_model(*model_root, options, op->reports, report)) {
    return OPERATOR_CANCELLED;
  }

  BKE_reportf(op->reports,
              RPT_INFO,
              "PMX export complete: %d vertices, %d faces, %d materials, %d textures (%d copied), "
              "%d bones, %d morphs (%d vertex morph offsets, %d impulse offsets), %d display "
              "frames, %d rigid bodies, %d joints (from %d mesh object(s))",
              report.vertex_count,
              report.face_count,
              report.material_count,
              report.texture_count,
              report.copied_texture_files,
              report.bone_count,
              report.morph_count,
              report.vertex_morph_offset_count,
              report.impulse_morph_offset_count,
              report.display_frame_count,
              report.rigid_body_count,
              report.joint_count,
              report.mesh_object_count);

  /* Surface every place the live scene could not be represented exactly. These
   * are counted during export precisely so they do not pass unnoticed; leaving
   * them unreported would make a lossy export look identical to a clean one. */
  if (report.duplicate_vertices != 0 || report.divergent_duplicates != 0 ||
      report.missing_source_vertices != 0 || report.invalid_vertex_indices != 0 ||
      report.truncated_weights != 0 || report.unmapped_vertex_groups != 0 ||
      report.material_face_count_changes != 0 || report.unresolved_shape_keys != 0 ||
      report.shape_key_size_mismatches != 0 || report.missing_texture_files != 0)
  {
    BKE_reportf(op->reports,
                RPT_WARNING,
                "PMX export compromises: %d duplicate vertices (%d divergent), %d missing source "
                "vertices, %d invalid vertex indices, %d truncated weights, %d unmapped vertex "
                "groups, %d material face-count changes, %d unresolved shape keys, %d shape key "
                "size mismatches, %d missing texture files",
                report.duplicate_vertices,
                report.divergent_duplicates,
                report.missing_source_vertices,
                report.invalid_vertex_indices,
                report.truncated_weights,
                report.unmapped_vertex_groups,
                report.material_face_count_changes,
                report.unresolved_shape_keys,
                report.shape_key_size_mismatches,
                report.missing_texture_files);
  }
  return OPERATOR_FINISHED;
}

static bool wm_pmx_export_check(bContext * /*C*/, wmOperator *op)
{
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (BLI_path_extension_check(filepath, ".pmx")) {
    return false;
  }
  BLI_path_extension_ensure(filepath, FILE_MAX, ".pmx");
  RNA_string_set(op->ptr, "filepath", filepath);
  return true;
}

void WM_OT_pmx_export(wmOperatorType *ot)
{
  ot->name = "Export PMX";
  ot->description = "Export an imported MMD PMX model back to a .pmx file";
  ot->idname = "WM_OT_pmx_export";

  ot->invoke = wm_pmx_export_invoke;
  ot->exec = wm_pmx_export_exec;
  ot->poll = WM_operator_winactive;
  ot->check = wm_pmx_export_check;
  ot->flag = OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_SAVE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);

  auto *prop = RNA_def_string(ot->srna, "filter_glob", "*.pmx", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

namespace ed::io {

void pmx_file_handler_add()
{
  auto fh = std::make_unique<bke::FileHandlerType>();
  STRNCPY_UTF8(fh->idname, "IO_FH_pmx");
  STRNCPY_UTF8(fh->import_operator, "WM_OT_pmx_import");
  STRNCPY_UTF8(fh->export_operator, "WM_OT_pmx_export");
  STRNCPY_UTF8(fh->label, "PMX");
  STRNCPY_UTF8(fh->file_extensions_str, ".pmx");
  fh->poll_drop = poll_file_object_drop;
  bke::file_handler_add(std::move(fh));
}

}  // namespace ed::io

}  // namespace blender

#endif /* WITH_IO_PMX */
