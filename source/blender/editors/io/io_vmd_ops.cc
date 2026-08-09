/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editor/io
 */

#include "io_vmd_ops.hh"

#include "BKE_anim_data.hh"
#include "BKE_armature.hh"
#include "BKE_context.hh"
#include "BKE_file_handler.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"

#include "mmd_ccd_ik.hh"

#include <cmath>
#include <cstring>
#include <set>
#include <string>

#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_path_utils.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_constraint_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "ED_fileselect.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "vmd_import.hh"
#include "vmd_export.hh"
#include "io_utils.hh"

#include "importer/pmx_import_bone_ik.hh"
#include "ANIM_action.hh"

namespace blender {
namespace {

bool vmd_target_poll(bContext *C)
{
  return WM_operator_winactive(C);
}

Object *vmd_morph_controller_from_operator(Main *bmain, wmOperator *op)
{
  ID *controller_id = WM_operator_properties_id_lookup_from_name_or_session_uid(
      bmain, op->ptr, ID_OB);
  return controller_id ? id_cast<Object *>(controller_id) : nullptr;
}

/* Auto-detect the PMXMorphController mesh that belongs to the target armature.
 * In a PMX import both the armature and the morph controller are parented to
 * the same model root object (see pmx_import_armature.cc and
 * pmx_import_morph_controller.cc), so we walk up from the armature to its parent
 * and scan that parent's children for a mesh named "PMXMorphController". A
 * scene-wide fallback covers single-model scenes. Returns nullptr when no such
 * object exists. */
static Object *vmd_find_morph_controller(Main *bmain, Object *target)
{
  if (target == nullptr) {
    return nullptr;
  }

  /* Anchors: the armature itself, and its parent (the model root). The morph
   * controller is a sibling of the armature under the model root, but we also
   * check the armature as anchor in case the hierarchy differs. */
  Object *anchors[2] = {target, target->parent};
  for (int i = 0; i < 2; i++) {
    Object *anchor = anchors[i];
    if (anchor == nullptr) {
      continue;
    }
    for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
         ob = static_cast<Object *>(ob->id.next))
    {
      if (ob->parent != anchor || ob->type != OB_MESH) {
        continue;
      }
      /* Allow the ".001" style suffix Blender adds for duplicate models. */
      if (std::strncmp(ob->id.name + 2, "PMXMorphController", 18) == 0) {
        return ob;
      }
    }
  }

  /* Fallback: any PMXMorphController mesh in the scene (single-model case). */
  for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
       ob = static_cast<Object *>(ob->id.next))
  {
    if (ob->type == OB_MESH &&
        std::strncmp(ob->id.name + 2, "PMXMorphController", 18) == 0)
    {
      return ob;
    }
  }
  return nullptr;
}

/* Base items for the dynamic "Target Armature" enum. The full list (including
 * every armature in the scene) is produced by vmd_target_armature_itemf. */
static const EnumPropertyItem vmd_target_base_items[] = {
    {0, "ACTIVE", 0, "Active / Auto", "Use the active armature, or the first armature found in the scene"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem *vmd_target_armature_itemf(bContext *C,
                                                         PointerRNA * /*ptr*/,
                                                         PropertyRNA * /*prop*/,
                                                         bool *r_free)
{
  if (C == nullptr) {
    /* Needed for docs and i18n tools. */
    return vmd_target_base_items;
  }

  EnumPropertyItem *item = nullptr;
  int totitem = 0;
  EnumPropertyItem tmp = {0};

  tmp.identifier = "ACTIVE";
  tmp.name = "Active / Auto";
  tmp.description = "Use the active armature, or the first armature found in the scene";
  tmp.value = 0;
  RNA_enum_item_add(&item, &totitem, &tmp);

  Main *bmain = CTX_data_main(C);
  int ordinal = 1;
  for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
       ob = static_cast<Object *>(ob->id.next))
  {
    if (ob->type != OB_ARMATURE) {
      continue;
    }
    /* Identifier reuses the (persistent) object name; resolution is by value,
     * so non-ASCII / spaced MMD names are fine as an opaque key. */
    tmp.identifier = ob->id.name + 2;
    tmp.name = ob->id.name + 2;
    tmp.description = "Armature object";
    tmp.value = ordinal++;
    RNA_enum_item_add(&item, &totitem, &tmp);
  }

  RNA_enum_item_end(&item, &totitem);
  *r_free = true;
  return item;
}

/* Resolve the armature the VMD motion is applied onto. The "Target Armature"
 * enum is used (explicit pick, or ACTIVE -> active / first armature). The
 * WM_operator_properties_id_lookup session_uid is reserved for the morph
 * controller and is NOT consumed here. Returns nullptr (with a report) when no
 * armature exists. */
/* [世界的歌] When a VMD is imported, the motion file usually bakes MMD's
 * IK / append-transform / axis solve into per-bone keyframes. The Blender-side
 * approximations created by PMX auto-apply (or by the manual Apply operators)
 * would fight those baked curves, producing broken motion. So we suspend every
 * MMD_* approximation IK constraint (enforce = 0) after the VMD animation is
 * written. They can be re-enabled from the constraint panel or via the Apply
 * operators for manual posing.
 *
 * However, some VMD files are "mixed-type": certain IK chains (e.g. NXDE's
 * knee) have NO FK keyframes on chain bones — the knee only bends via IK
 * solving. Blindly suspending all IK constraints leaves such chains frozen
 * (the knee stays straight, the leg only translates with the upper body).
 *
 * To support both pure-FK VMDs (suspend all IK) and mixed-type VMDs (keep IK
 * for chains lacking FK coverage), we inspect the just-built action's F-Curves
 * per IK chain: if EVERY chain-link bone has its own rotation F-Curve, the
 * chain is pure-FK-baked → suspend; otherwise keep the IK constraint active so
 * the E-phase CCD solver can bend the chain. */

/** Check whether any F-Curve in the action drives a *meaningful* rotation
 *  channel of the given bone. Matches data paths of the form:
 *    pose.bones["<bone_name>"].rotation_quaternion
 *    pose.bones["<bone_name>"].rotation_euler
 *    pose.bones["<bone_name>"].rotation_axis_angle
 *  The bone_name comes from PMX name_local and never contains '"' or ']'.
 *
 *  "Meaningful" means the channel has at least 2 keyframes whose values
 *  actually differ. VMD exporters frequently emit a single identity-quaternion
 *  keyframe (or several identical ones) for bones that are meant to be driven
 *  by IK solving rather than by FK curves. Treating such placeholder curves as
 *  "no FK rotation" is what lets mixed-type VMDs keep their IK chains active so
 *  the E-phase CCD solver can bend the knee/ankle. Without this check every
 *  chain-link bone with a placeholder curve would be misclassified as
 *  pure-FK-baked, and the IK constraint would be wrongly suspended — freezing
 *  the knee straight while the IK target bone keeps translating. */
static bool bone_has_rotation_fcurve(const bAction &action,
                                     const AnimData &ob_adt,
                                     const std::string &bone_name)
{
  /* Build the prefix we expect to see at the start of the rna_path:
   *   pose.bones["<bone_name>"].rotation
   * Checking this single prefix is enough — rotation_quaternion / _euler /
   * _axis_angle all share it and all qualify as "the bone has FK rotation". */
  const std::string prefix = "pose.bones[\"" + bone_name + "\"].rotation";

  /* Use the animrig C++ wrapper to traverse the slotted action. VMD imports
   * produce a single layer/strip/channelbag, but we walk all of them for
   * safety. The slot handle is read from the armature's animdata. */
  const animrig::Action &anim = action.wrap();
  const int slot_handle = ob_adt.slot_handle ? ob_adt.slot_handle :
                                                anim.slot(0)->handle;
  for (const animrig::Layer *layer : anim.layers()) {
    for (const animrig::Strip *strip : layer->strips()) {
      const animrig::Channelbag *cbag = strip->data<animrig::StripKeyframeData>(anim)
                                            .channelbag_for_slot(slot_handle);
      if (cbag == nullptr) {
        continue;
      }
      for (const FCurve *fc : cbag->fcurves()) {
        if (fc->rna_path().is_empty()) {
          continue;
        }
        if (strncmp(fc->rna_path().c_str(), prefix.c_str(), prefix.size()) != 0) {
          continue;
        }
        /* A single keyframe (or several identical ones) is a placeholder,
         * not a real FK-baked channel. Require >= 2 keyframes. */
        if (fc->totvert < 2) {
          continue;
        }
        /* And require at least one keyframe whose value differs from the
         * first — a flat constant curve is also just a placeholder. */
        const float first_val = fc->bezt[0].vec[1][1];
        for (int i = 1; i < fc->totvert; i++) {
          if (std::fabs(fc->bezt[i].vec[1][1] - first_val) > 1e-5f) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

/** Check whether the action contains any `mmd_ik_toggle` F-Curve.
 *
 *  apply_vmd_ik_toggle() creates per-frame `pose.bones["<name>"].mmd_ik_toggle`
 *  F-Curves when the VMD file ships a property (IK on/off) track. The E-phase
 *  CCD solver reads these F-Curves at eval time to decide per-frame whether to
 *  solve each IK chain.
 *
 *  When such F-Curves exist, we keep native CCD enabled on EVERY IK control
 *  bone and let the per-frame toggle gate the solver. When they don't exist
 *  (pure-FK VMD with no property track), we fall back to the mixed-chain
 *  heuristic below. */
static bool action_has_ik_toggle_fcurves(const bAction &action, const AnimData &ob_adt)
{
  const animrig::Action &anim = action.wrap();
  const int slot_handle = ob_adt.slot_handle ? ob_adt.slot_handle :
                                                anim.slot(0)->handle;
  for (const animrig::Layer *layer : anim.layers()) {
    for (const animrig::Strip *strip : layer->strips()) {
      const animrig::Channelbag *cbag = strip->data<animrig::StripKeyframeData>(anim)
                                            .channelbag_for_slot(slot_handle);
      if (cbag == nullptr) {
        continue;
      }
      for (const FCurve *fc : cbag->fcurves()) {
        if (fc->rna_path().is_empty()) {
          continue;
        }
        if (strstr(fc->rna_path().c_str(), ".mmd_ik_toggle") != nullptr) {
          return true;
        }
      }
    }
  }
  return false;
}

static int vmd_suspend_mmd_approx_constraints(Main *bmain, Object *ob)
{
  if (ob->pose == nullptr) {
    return 0;
  }

  /* Load PMX IK definitions so we can inspect each IK chain's link bones. */
  io::pmx::PMXBoneIKDefinitionSet ik_def;
  const bool has_ik_def = io::pmx::read_bone_ik_definition(ob->id, ik_def);

  /* Detect whether the VMD shipped a property (IK toggle) track.
   * If it did, the E-phase CCD solver will read the per-frame toggle value
   * from the `mmd_ik_toggle` F-Curves and decide itself whether to solve each
   * chain. In that case we keep native IK enabled on every IK control bone and
   * skip the mixed-chain heuristic entirely.
   *
   * If it did NOT (pure-FK VMD with no property track), we fall back to the
   * mixed-chain heuristic: chains whose links all have FK rotation curves are
   * treated as FK-baked and their native IK is disabled; chains missing FK
   * coverage keep native IK so CCD can bend them. */
  bool has_vmd_ik_toggle = false;
  if (has_ik_def && ob->adt && ob->adt->action) {
    has_vmd_ik_toggle = action_has_ik_toggle_fcurves(*ob->adt->action, *ob->adt);
  }

  /* Set of link-bone names belonging to IK chains that LACK full FK rotation
   * coverage. For multi-link chains (e.g. leg IK: [knee, thigh]) ALL links
   * must have FK rotation to qualify as pure-FK; a single missing link forces
   * the whole chain to stay on IK. */
  std::set<std::string> mixed_chain_link_bones;
  /* Set of IK control bone names (def.bone_name) for mixed-type chains.
   * These keep native CCD IK enabled so the E-phase solver can bend them. */
  std::set<std::string> mixed_ik_bone_names;
  if (!has_vmd_ik_toggle && has_ik_def && ob->adt && ob->adt->action) {
    bAction &action = *ob->adt->action;
    AnimData &ob_adt = *ob->adt;
    for (const io::pmx::PMXBoneIKDefinition &def : ik_def.ik_bones) {
      bool any_link_missing_fk = false;
      for (const io::pmx::PMXBoneIKLink &link : def.links) {
        if (!bone_has_rotation_fcurve(action, ob_adt, link.bone_name)) {
          any_link_missing_fk = true;
          break;
        }
      }
      if (any_link_missing_fk) {
        for (const io::pmx::PMXBoneIKLink &link : def.links) {
          mixed_chain_link_bones.insert(link.bone_name);
        }
        mixed_ik_bone_names.insert(def.bone_name);
      }
    }
  }

  /* Toggle native CCD IK per IK control bone.
   *
   * When the VMD ships an IK toggle track, keep native IK enabled on every IK
   * control bone — the E-phase CCD solver will read the per-frame toggle value
   * from the `mmd_ik_toggle` F-Curve and skip solving when the VMD says IK off.
   * This mirrors mmd_tools, which lets the VMD property track drive IK on/off
   * per frame.
   *
   * Otherwise (pure-FK VMD with no property track) use the mixed-chain
   * heuristic: disable native IK on pure-FK chains so CCD leaves them alone,
   * keep it on mixed-type chains so CCD can bend them. */
  if (has_ik_def) {
    bArmature *arm = id_cast<bArmature *>(ob->data);
    if (arm) {
      for (const io::pmx::PMXBoneIKDefinition &def : ik_def.ik_bones) {
        const bool is_mixed = has_vmd_ik_toggle ||
                              mixed_ik_bone_names.count(def.bone_name) > 0;
        Bone *ik_bone = BKE_armature_find_bone_name(arm, def.bone_name.c_str());
        if (ik_bone) {
          mmd::mmd_native_ik_set_enabled(*ik_bone, is_mixed);
        }
      }
    }
  }

  int suspended = 0;
  int kept = 0;
  for (bPoseChannel *pchan = static_cast<bPoseChannel *>(ob->pose->chanbase.first); pchan != nullptr;
       pchan = pchan->next) {
    for (bConstraint *con = static_cast<bConstraint *>(pchan->constraints.first); con != nullptr;
         con = con->next) {
      /* MMD_Local_Axis_Approx: self-referential TRANSFORM (ADD mode) that
       * ADDs a bone's own local-space rotation to itself. It is needed for
       * real-time posing, but VMD keyframes are already baked in the correct
       * local-axis frame, so ADDing again double-counts the rotation and
       * breaks the pose (e.g. NXDE frame 108 arms twist behind the head).
       * mmd_tools does not create any equivalent constraint on arm bones.
       *
       * Unconditionally suspend this constraint when a VMD is driving the
       * pose — it must NOT be kept active even when the VMD ships an IK
       * toggle track (the IK toggle only governs MMD_IK_Approx/Limit, not
       * local-axis approximation). */
      if (strcmp(con->name, "MMD_Local_Axis_Approx") == 0) {
        if (con->enforce != 0.0f) {
          con->enforce = 0.0f;
          suspended++;
        }
        continue;
      }

      /* MMD_Fixed_Axis_Approx: LIMIT_ROTATION constraint that restricts a
       * bone's rotation to its fixed axis (used by 捩/twist bones like
       * 左腕捩, 左手捩). VMD keyframes are already baked in the correct
       * fixed-axis frame, so applying the limit again double-restricts the
       * rotation and breaks the pose (e.g. NXDE frame 108 左ひじ twists
       * behind the head because 左腕捩's LIMIT_ROTATION clips the baked
       * rotation, propagating wrong parent transform down the arm chain).
       * mmd_tools uses bone.lock_rotation instead of a constraint, which
       * does not conflict with VMD keyframes.
       *
       * Unconditionally suspend when a VMD is driving the pose. D bones use
       * MMD_Append_Rotation (not MMD_Fixed_Axis_Approx), so the D bone
       * chain is unaffected. */
      if (strcmp(con->name, "MMD_Fixed_Axis_Approx") == 0) {
        if (con->enforce != 0.0f) {
          con->enforce = 0.0f;
          suspended++;
        }
        continue;
      }

      /* Suspend IK-related approximations (MMD_IK_Approx / MMD_IK_Limit).
       *
       * Keep MMD_Append_Rotation active — the D bone chain needs it to
       * follow source bones even when VMD fcurves drive those source
       * bones directly. */
      if (strcmp(con->name, "MMD_IK_Approx") != 0 &&
          strcmp(con->name, "MMD_IK_Limit") != 0)
      {
        continue;
      }
      /* When the VMD ships an IK toggle track, keep ALL IK constraints active.
       * The MMD_IK_Approx constraint (Blender native iTaSC IK) will override
       * FK rotations when IK is on, and yield to FK when IK is off — the
       * per-frame toggle is driven by the MMD_IK_Approx influence F-Curve
       * created by apply_vmd_ik_toggle() in vmd_import.cc. This mirrors
       * mmd_tools, which keyframes the IK constraint influence from the
       * VMD IK toggle property track. */
      if (has_vmd_ik_toggle) {
        if (con->enforce == 0.0f) {
          con->enforce = 1.0f;
        }
        kept++;
        continue;
      }
      /* Mixed-type chain: this link bone belongs to an IK chain that lacks FK
       * rotation coverage. Keep the constraint active so CCD can solve. */
      if (mixed_chain_link_bones.count(pchan->name) > 0) {
        kept++;
        continue;
      }
      if (con->enforce != 0.0f) {
        con->enforce = 0.0f;
        suspended++;
      }
    }
  }
  if (suspended > 0 || kept > 0) {
    DEG_id_tag_update_ex(bmain, &ob->id, ID_RECALC_GEOMETRY | ID_RECALC_TRANSFORM);
    DEG_relations_tag_update(bmain);
  }
  return suspended;
}

static Object *vmd_resolve_target(bContext *C, wmOperator *op, Main *bmain)
{
  int target_value = 0;
  PropertyRNA *target_prop = RNA_struct_find_property(op->ptr, "target");
  if (target_prop != nullptr) {
    target_value = RNA_property_enum_get(op->ptr, target_prop);
  }

  if (target_value <= 0) {
    Object *active = CTX_data_active_object(C);
    if (active != nullptr && active->type == OB_ARMATURE) {
      return active;
    }
    for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
         ob = static_cast<Object *>(ob->id.next))
    {
      if (ob->type == OB_ARMATURE) {
        BKE_reportf(op->reports,
                    RPT_INFO,
                    "No active armature; using '%s' as the VMD target",
                    ob->id.name + 2);
        return ob;
      }
    }
    return nullptr;
  }

  /* Resolve by ordinal among armatures (stable across dropdown rebuilds). */
  int ordinal = 1;
  for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
       ob = static_cast<Object *>(ob->id.next))
  {
    if (ob->type != OB_ARMATURE) {
      continue;
    }
    if (ordinal == target_value) {
      return ob;
    }
    ordinal++;
  }
  return nullptr;
}

wmOperatorStatus wm_vmd_import_exec(bContext *C, wmOperator *op)
{
  const auto paths = ed::io::paths_from_operator_properties(op->ptr);
  if (paths.is_empty()) {
    return OPERATOR_CANCELLED;
  }

  Main *bmain = CTX_data_main(C);

  Object *target = vmd_resolve_target(C, op, bmain);
  if (target == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "VMD import requires an Armature object in the scene");
    return OPERATOR_CANCELLED;
  }

  const bool import_morphs = RNA_boolean_get(op->ptr, "import_morphs");
  Object *morph_controller = vmd_morph_controller_from_operator(bmain, op);
  if (morph_controller == nullptr && target != nullptr) {
    morph_controller = vmd_find_morph_controller(bmain, target);
  }
  if (import_morphs && morph_controller != nullptr) {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Using '%s' as the VMD morph controller",
                morph_controller->id.name + 2);
  }
  if (import_morphs && morph_controller == nullptr) {
    BKE_report(op->reports,
               RPT_ERROR,
               "VMD Morph import requires an explicit PMXMorphController object");
    return OPERATOR_CANCELLED;
  }

  io::vmd::VMDImportOptions options;
  options.frame_offset = RNA_int_get(op->ptr, "frame_offset");
  options.replace_existing_action = RNA_boolean_get(op->ptr, "replace_existing_action");
  options.use_vmd_bezier_interpolation = RNA_boolean_get(op->ptr, "use_vmd_bezier_interpolation");
  if (options.use_vmd_bezier_interpolation) {
    /* C3: VMD-native bezier interpolation implies non-linear bone tracks. */
    options.use_linear_interpolation = false;
  }

  io::vmd::VMDImportReport result;
  const bool success = import_morphs ? io::vmd::import_vmd_action_with_morphs(
                                           bmain,
                                           *target,
                                           *morph_controller,
                                           paths[0],
                                           options,
                                           op->reports,
                                           result) :
                                       io::vmd::import_vmd_action(
                                           bmain, *target, paths[0], options, op->reports, result);
  if (!success) {
    return OPERATOR_CANCELLED;
  }

  /* Suspend MMD approximate constraints: VMD bakes its own solve, so the
   * Blender-side approximations must yield to avoid fighting the baked curves. */
  const int suspended = vmd_suspend_mmd_approx_constraints(bmain, target);
  if (suspended > 0) {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Suspended %d MMD IK constraints. "
                "Append/Fixed/Local kept active for D bone tracking. "
                "Re-enable via the constraint panel or Apply operators for "
                "manual posing.",
                suspended);
  }

  if (AnimData *anim_data = BKE_animdata_from_id(&target->id)) {
    DEG_id_tag_update_ex(bmain, &target->id, ID_RECALC_ANIMATION);
    if (anim_data->action != nullptr) {
      DEG_id_tag_update_ex(bmain, &anim_data->action->id, ID_RECALC_ANIMATION_NO_FLUSH);
    }
  }
  if (import_morphs) {
    DEG_id_tag_update_ex(bmain,
                         &morph_controller->id,
                         ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    Mesh *controller_mesh = id_cast<Mesh *>(morph_controller->data);
    if (controller_mesh != nullptr) {
      DEG_id_tag_update_ex(bmain,
                           &controller_mesh->id,
                           ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    }
    Key *controller_key = controller_mesh ? controller_mesh->key : nullptr;
    if (controller_key != nullptr) {
      DEG_id_tag_update_ex(bmain,
                           &controller_key->id,
                           ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
      if (AnimData *anim_data = BKE_animdata_from_id(&controller_key->id)) {
        if (anim_data->action != nullptr) {
          DEG_id_tag_update_ex(
              bmain, &anim_data->action->id, ID_RECALC_ANIMATION_NO_FLUSH);
        }
      }
    }
  }
  DEG_relations_tag_update(bmain);
  if (Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C)) {
    ED_update_for_newframe(bmain, depsgraph);
  }

  Scene *scene = CTX_data_scene(C);
  /* Update scene frame range to match the imported VMD data. */
  {
    int vmd_start = result.action.first_frame;
    int vmd_end = result.action.last_frame;
    if (!result.morph_action.skipped) {
      if (result.morph_action.first_frame >= 0) {
        vmd_start = vmd_start < 0 ? result.morph_action.first_frame :
                                    (result.morph_action.first_frame < vmd_start ?
                                         result.morph_action.first_frame :
                                         vmd_start);
      }
      if (result.morph_action.last_frame >= 0) {
        vmd_end = vmd_end < 0 ? result.morph_action.last_frame :
                                (result.morph_action.last_frame > vmd_end ?
                                     result.morph_action.last_frame :
                                     vmd_end);
      }
    }
    if (vmd_start >= 0 && vmd_end >= 0 && vmd_end > vmd_start) {
      scene->r.sfra = vmd_start;
      scene->r.efra = vmd_end;
      BKE_reportf(op->reports,
                  RPT_INFO,
                  "Scene frame range set to match VMD: %d–%d",
                  vmd_start,
                  vmd_end);
    }
  }
  WM_event_add_notifier(C, NC_ANIMATION | ND_NLA_ACTCHANGE, nullptr);
  WM_event_add_notifier(C, NC_ANIMATION | ND_ANIMCHAN | NA_EDITED, nullptr);
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
  return OPERATOR_FINISHED;
}

wmOperatorStatus wm_vmd_camera_import_exec(bContext *C, wmOperator *op)
{
  const auto paths = ed::io::paths_from_operator_properties(op->ptr);
  if (paths.is_empty()) {
    return OPERATOR_CANCELLED;
  }

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  LayerCollection *active_collection = BKE_layer_collection_get_active_editable(view_layer);
  Collection *target_collection = active_collection ? active_collection->collection :
                                                       scene->master_collection;
  if (target_collection == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "VMD camera import requires an editable collection");
    return OPERATOR_CANCELLED;
  }

  io::vmd::VMDImportOptions options;
  options.frame_offset = RNA_int_get(op->ptr, "frame_offset");
  options.coordinate_scale = RNA_float_get(op->ptr, "coordinate_scale");
  options.replace_existing_action = RNA_boolean_get(op->ptr, "replace_existing_action");
  options.use_vmd_bezier_interpolation = RNA_boolean_get(op->ptr,
                                                         "use_vmd_bezier_interpolation");
  options.use_linear_interpolation = !options.use_vmd_bezier_interpolation;

  Object *target_camera = nullptr;
  Object *active_object = CTX_data_active_object(C);
  if (active_object != nullptr && active_object->type == OB_CAMERA) {
    target_camera = active_object;
  }
  else if (scene->camera != nullptr && scene->camera->type == OB_CAMERA) {
    target_camera = scene->camera;
  }
  if (target_camera != nullptr) {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Using existing Camera '%s' for VMD camera import",
                target_camera->id.name + 2);
  }

  io::vmd::VMDImportReport result;
  if (!io::vmd::import_vmd_camera(
          bmain, *target_collection, paths[0], options, op->reports, result, target_camera))
  {
    return OPERATOR_CANCELLED;
  }

  Object *imported_camera = nullptr;
  for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
       ob = static_cast<Object *>(ob->id.next))
  {
    if (result.camera_action.target_camera_name == std::string(ob->id.name + 2)) {
      imported_camera = ob;
      break;
    }
  }
  if (imported_camera != nullptr) {
    scene->camera = imported_camera;
    DEG_id_tag_update_ex(bmain, &imported_camera->id, ID_RECALC_ANIMATION | ID_RECALC_TRANSFORM);
    if (imported_camera->data != nullptr) {
      DEG_id_tag_update_ex(
          bmain, imported_camera->data, ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    }
  }

  if (result.camera_action.first_frame >= 0 && result.camera_action.last_frame >= 0 &&
      result.camera_action.last_frame > result.camera_action.first_frame)
  {
    scene->r.sfra = result.camera_action.first_frame;
    scene->r.efra = result.camera_action.last_frame;
    BKE_reportf(op->reports,
                RPT_INFO,
                "Scene frame range set to match VMD camera: %d-%d",
                scene->r.sfra,
                scene->r.efra);
  }

  DEG_relations_tag_update(bmain);
  if (Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C)) {
    ED_update_for_newframe(bmain, depsgraph);
  }
  WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
  WM_event_add_notifier(C, NC_ANIMATION | ND_NLA_ACTCHANGE, nullptr);
  BKE_reportf(op->reports,
              RPT_INFO,
              "VMD camera import complete: %d keyframes, target '%s', camera '%s'",
              result.camera_action.keyframe_count / 10,
              result.camera_action.target_empty_name.c_str(),
              result.camera_action.target_camera_name.c_str());
  return OPERATOR_FINISHED;
}

void wm_vmd_import_draw(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "target", UI_ITEM_NONE, "Target Armature", ICON_NONE);
  layout.prop(op->ptr, "frame_offset", UI_ITEM_NONE, "Frame Offset", ICON_NONE);
  layout.prop(op->ptr,
              "replace_existing_action",
              UI_ITEM_NONE,
              "Replace Existing Action",
              ICON_NONE);
  layout.prop(op->ptr, "import_morphs", UI_ITEM_NONE, "Import Morphs", ICON_NONE);
  layout.prop(op->ptr,
              "use_vmd_bezier_interpolation",
              UI_ITEM_NONE,
              "VMD Native Bezier",
              ICON_NONE);
}

void wm_vmd_camera_import_draw(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "frame_offset", UI_ITEM_NONE, "Frame Offset", ICON_NONE);
  layout.prop(op->ptr, "coordinate_scale", UI_ITEM_NONE, "Coordinate Scale", ICON_NONE);
  layout.prop(op->ptr,
              "replace_existing_action",
              UI_ITEM_NONE,
              "Replace Existing Action",
              ICON_NONE);
  layout.prop(op->ptr,
              "use_vmd_bezier_interpolation",
              UI_ITEM_NONE,
              "VMD Native Bezier",
              ICON_NONE);
}

wmOperatorStatus wm_vmd_export_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  Scene *scene = CTX_data_scene(C);
  if (!RNA_struct_property_is_set(op->ptr, "frame_start")) {
    RNA_int_set(op->ptr, "frame_start", scene->r.sfra);
  }
  if (!RNA_struct_property_is_set(op->ptr, "frame_end")) {
    RNA_int_set(op->ptr, "frame_end", scene->r.efra);
  }
  ED_fileselect_ensure_default_filepath(C, op, ".vmd");
  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

wmOperatorStatus wm_vmd_export_exec(bContext *C, wmOperator *op)
{
  Object *armature = CTX_data_active_object(C);
  if (armature == nullptr || armature->type != OB_ARMATURE) {
    BKE_report(op->reports, RPT_ERROR, "VMD export requires the active object to be an Armature");
    return OPERATOR_CANCELLED;
  }
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  io::vmd::VMDExportOptions options;
  options.frame_start = RNA_int_get(op->ptr, "frame_start");
  options.frame_end = RNA_int_get(op->ptr, "frame_end");
  options.coordinate_scale = RNA_float_get(op->ptr, "coordinate_scale");
  char model_name[256];
  RNA_string_get(op->ptr, "model_name", model_name);
  options.model_name = model_name;
  if (RNA_boolean_get(op->ptr, "export_morphs")) {
    options.morph_controller = vmd_find_morph_controller(CTX_data_main(C), armature);
  }
  io::vmd::VMDExportReport report;
  if (!io::vmd::export_vmd_action(*armature, filepath, options, op->reports, report)) {
    return OPERATOR_CANCELLED;
  }
  BKE_reportf(op->reports,
              RPT_INFO,
              "VMD export complete: %d bones, %d bone frames, %d morphs, %d morph frames",
              report.bone_count,
              report.bone_frame_count,
              report.morph_count,
              report.morph_frame_count);
  return OPERATOR_FINISHED;
}

void wm_vmd_export_draw(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "model_name", UI_ITEM_NONE, "Model Name", ICON_NONE);
  layout.prop(op->ptr, "frame_start", UI_ITEM_NONE, "Start Frame", ICON_NONE);
  layout.prop(op->ptr, "frame_end", UI_ITEM_NONE, "End Frame", ICON_NONE);
  layout.prop(op->ptr, "coordinate_scale", UI_ITEM_NONE, "Coordinate Scale", ICON_NONE);
  layout.prop(op->ptr, "export_morphs", UI_ITEM_NONE, "Export Morphs", ICON_NONE);
}

bool wm_vmd_export_check(bContext * /*C*/, wmOperator *op)
{
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (BLI_path_extension_check(filepath, ".vmd")) {
    return false;
  }
  BLI_path_extension_ensure(filepath, FILE_MAX, ".vmd");
  RNA_string_set(op->ptr, "filepath", filepath);
  return true;
}

/* Resolve the Camera whose Action is exported. Selecting either half of a VMD camera rig is
 * accepted because the importer leaves the Empty active, and the Scene camera covers the common
 * case of exporting the shot that is currently being rendered. */
static Object *vmd_resolve_export_camera(bContext *C, Main *bmain)
{
  Object *active = CTX_data_active_object(C);
  if (active != nullptr) {
    if (active->type == OB_CAMERA) {
      return active;
    }
    if (active->type == OB_EMPTY) {
      for (Object *ob = static_cast<Object *>(bmain->objects.first); ob != nullptr;
           ob = static_cast<Object *>(ob->id.next))
      {
        if (ob->parent == active && ob->type == OB_CAMERA) {
          return ob;
        }
      }
    }
  }
  Scene *scene = CTX_data_scene(C);
  if (scene != nullptr && scene->camera != nullptr && scene->camera->type == OB_CAMERA) {
    return scene->camera;
  }
  return nullptr;
}

wmOperatorStatus wm_vmd_camera_export_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Object *camera = vmd_resolve_export_camera(C, bmain);
  if (camera == nullptr) {
    BKE_report(op->reports,
               RPT_ERROR,
               "VMD camera export requires an active Camera, a VMD camera rig Empty, or a Scene "
               "camera");
    return OPERATOR_CANCELLED;
  }
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  io::vmd::VMDCameraExportOptions options;
  options.frame_start = RNA_int_get(op->ptr, "frame_start");
  options.frame_end = RNA_int_get(op->ptr, "frame_end");
  options.coordinate_scale = RNA_float_get(op->ptr, "coordinate_scale");
  char model_name[256];
  RNA_string_get(op->ptr, "model_name", model_name);
  options.model_name = model_name;

  io::vmd::VMDCameraExportReport report;
  if (!io::vmd::export_vmd_camera(*camera, filepath, options, op->reports, report)) {
    return OPERATOR_CANCELLED;
  }
  BKE_reportf(op->reports,
              RPT_INFO,
              "VMD camera export complete: %d frames from '%s' (%s)",
              report.camera_frame_count,
              camera->id.name + 2,
              report.used_camera_rig ? "camera rig" : "standalone camera");
  return OPERATOR_FINISHED;
}

void wm_vmd_camera_export_draw(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "model_name", UI_ITEM_NONE, "Model Name", ICON_NONE);
  layout.prop(op->ptr, "frame_start", UI_ITEM_NONE, "Start Frame", ICON_NONE);
  layout.prop(op->ptr, "frame_end", UI_ITEM_NONE, "End Frame", ICON_NONE);
  layout.prop(op->ptr, "coordinate_scale", UI_ITEM_NONE, "Coordinate Scale", ICON_NONE);
}

}  // namespace

void WM_OT_vmd_import(wmOperatorType *ot)
{
  ot->name = "Import VMD";
  ot->description = "Import VMD bone animation onto a target Armature";
  ot->idname = "WM_OT_vmd_import";
  ot->invoke = ed::io::filesel_drop_import_invoke;
  ot->exec = wm_vmd_import_exec;
  ot->poll = vmd_target_poll;
  ot->ui = wm_vmd_import_draw;
  ot->flag = OPTYPE_UNDO | OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);

  PropertyRNA *target_prop = RNA_def_enum(ot->srna,
                                          "target",
                                          vmd_target_base_items,
                                          0,
                                          "Target Armature",
                                          "Armature to apply the VMD motion onto");
  RNA_def_property_enum_funcs_runtime(
      target_prop, nullptr, nullptr, vmd_target_armature_itemf, nullptr, nullptr);

  RNA_def_int(ot->srna,
              "frame_offset",
              30,
              -1000000,
              1000000,
              "Frame Offset",
              "Add this value to every imported VMD frame",
              -1000,
              1000);
  RNA_def_boolean(ot->srna,
                  "replace_existing_action",
                  true,
                  "Replace Existing Action",
                  "Replace the active Armature's existing Action");
  RNA_def_boolean(ot->srna,
                  "import_morphs",
                  true,
                  "Import Morphs",
                  "Import Vertex Morph animation into an explicit PMX Morph Controller");
  RNA_def_boolean(ot->srna,
                  "use_vmd_bezier_interpolation",
                  true,
                  "VMD Native Bezier",
                  "Use VMD-embedded bezier interpolation for bone tracks instead of linear");
  WM_operator_properties_id_lookup(ot, false);

  PropertyRNA *prop = RNA_def_string(ot->srna, "filter_glob", "*.vmd", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

void WM_OT_vmd_camera_import(wmOperatorType *ot)
{
  ot->name = "Import VMD Camera";
  ot->description = "Import VMD camera animation into a native Empty and Camera rig";
  ot->idname = "WM_OT_vmd_camera_import";
  ot->invoke = ed::io::filesel_drop_import_invoke;
  ot->exec = wm_vmd_camera_import_exec;
  ot->poll = vmd_target_poll;
  ot->ui = wm_vmd_camera_import_draw;
  ot->flag = OPTYPE_UNDO | OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);

  RNA_def_int(ot->srna,
              "frame_offset",
              30,
              -1000000,
              1000000,
              "Frame Offset",
              "Add this value to every imported VMD camera frame",
              -1000,
              1000);
  RNA_def_float(ot->srna,
                "coordinate_scale",
                0.08f,
                0.000001f,
                1000.0f,
                "Coordinate Scale",
                "Blender units per MMD coordinate unit",
                0.001f,
                1.0f);
  RNA_def_boolean(ot->srna,
                  "replace_existing_action",
                  true,
                  "Replace Existing Action",
                  "Replace actions on the imported camera rig when applicable");
  RNA_def_boolean(ot->srna,
                  "use_vmd_bezier_interpolation",
                  true,
                  "VMD Native Bezier",
                  "Use VMD-embedded Bezier interpolation for camera tracks instead of linear");
  PropertyRNA *prop = RNA_def_string(ot->srna, "filter_glob", "*.vmd", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

void WM_OT_vmd_export(wmOperatorType *ot)
{
  ot->name = "Export VMD";
  ot->description = "Export the active Armature Action as VMD bone motion";
  ot->idname = "WM_OT_vmd_export";
  ot->invoke = wm_vmd_export_invoke;
  ot->exec = wm_vmd_export_exec;
  ot->poll = vmd_target_poll;
  ot->ui = wm_vmd_export_draw;
  ot->check = wm_vmd_export_check;
  ot->flag = OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_SAVE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  RNA_def_string(ot->srna,
                 "model_name",
                 "Model",
                 255,
                 "Model Name",
                 "VMD model name stored in the fixed header");
  RNA_def_int(ot->srna,
              "frame_start",
              0,
              0,
              MAXFRAME,
              "Start Frame",
              "First Action frame to export; it becomes VMD frame zero",
              0,
              MAXFRAME);
  RNA_def_int(ot->srna,
              "frame_end",
              250,
              0,
              MAXFRAME,
              "End Frame",
              "Last Action frame to export",
              0,
              MAXFRAME);
  RNA_def_float(ot->srna,
                "coordinate_scale",
                0.08f,
                0.000001f,
                1000.0f,
                "Coordinate Scale",
                "Blender units per MMD coordinate unit",
                0.001f,
                1.0f);
  RNA_def_boolean(ot->srna,
                  "export_morphs",
                  true,
                  "Export Morphs",
                  "Export the same model's PMXMorphController Shape Key Action when available");
  PropertyRNA *prop = RNA_def_string(ot->srna, "filter_glob", "*.vmd", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

void WM_OT_vmd_camera_export(wmOperatorType *ot)
{
  ot->name = "Export VMD Camera";
  ot->description = "Export a Camera Action as VMD camera motion";
  ot->idname = "WM_OT_vmd_camera_export";
  ot->invoke = wm_vmd_export_invoke;
  ot->exec = wm_vmd_camera_export_exec;
  ot->poll = vmd_target_poll;
  ot->ui = wm_vmd_camera_export_draw;
  ot->check = wm_vmd_export_check;
  ot->flag = OPTYPE_PRESET;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_SAVE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  RNA_def_string(ot->srna,
                 "model_name",
                 io::vmd::VMD_CAMERA_MODEL_NAME,
                 255,
                 "Model Name",
                 "VMD header name; MikuMikuDance expects the camera/lighting name here");
  RNA_def_int(ot->srna,
              "frame_start",
              0,
              0,
              MAXFRAME,
              "Start Frame",
              "First Action frame to export; it becomes VMD frame zero",
              0,
              MAXFRAME);
  RNA_def_int(ot->srna,
              "frame_end",
              250,
              0,
              MAXFRAME,
              "End Frame",
              "Last Action frame to export",
              0,
              MAXFRAME);
  RNA_def_float(ot->srna,
                "coordinate_scale",
                0.08f,
                0.000001f,
                1000.0f,
                "Coordinate Scale",
                "Blender units per MMD coordinate unit",
                0.001f,
                1.0f);
  PropertyRNA *prop = RNA_def_string(ot->srna, "filter_glob", "*.vmd", 0, "Extension Filter", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

namespace ed::io {

void vmd_file_handler_add()
{
  auto fh = std::make_unique<bke::FileHandlerType>();
  STRNCPY_UTF8(fh->idname, "IO_FH_vmd");
  STRNCPY_UTF8(fh->import_operator, "WM_OT_vmd_import");
  STRNCPY_UTF8(fh->export_operator, "WM_OT_vmd_export");
  STRNCPY_UTF8(fh->label, "VMD");
  STRNCPY_UTF8(fh->file_extensions_str, ".vmd");
  fh->poll_drop = poll_file_object_drop;
  bke::file_handler_add(std::move(fh));
}

}  // namespace ed::io

}  // namespace blender
