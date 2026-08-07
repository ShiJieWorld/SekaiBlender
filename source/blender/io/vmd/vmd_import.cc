/* SPDX-FileCopyrightText: 2026 MuAnChen
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup io_vmd
 */

#include "vmd_import.hh"

#include "ANIM_action.hh"
#include "ANIM_armature_iter.hh"
#include "ANIM_fcurve.hh"
#include "ANIM_keyframing.hh"
#include "BKE_anim_data.hh"
#include "BKE_armature.hh"
#include "BKE_fcurve.hh"
#include "BKE_idprop.hh"
#include "BKE_key.hh"
#include "BKE_lib_id.hh"
#include "BKE_report.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "BLI_path_utils.hh"
#include "BLI_string.hh"

#include "DNA_action_types.h"
#include "DNA_armature_types.h"
#include "DNA_anim_types.h"
#include "DNA_constraint_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

/* [世界的歌] C2-2E: PMX importer persists Group/Vertex channel metadata on the
 * controller object under this property name. The VMD importer reads the Group
 * raw channel names from it so mapped tracks can be annotated by morph type. */
constexpr char kMorphDefinitionProperty[] = "mmd_pmx_morph_definition";

namespace blender::io::vmd {
namespace {

void add_error(VMDImportReport &result, ReportList *reports, const std::string &message)
{
  result.errors.push_back(message);
  BKE_report(reports, RPT_ERROR, message.c_str());
}

void add_warning(VMDImportReport &result, ReportList *reports, const std::string &message)
{
  result.warnings.push_back(message);
  BKE_report(reports, RPT_WARNING, message.c_str());
}

/* [世界的歌] R4: A VMD morph import can run against a valid Mesh+Key controller
 * even when no PMX import has persisted the morph definition. In that case the
 * Group/Vertex morph-type distinction metadata is unavailable; this is a
 * capability limitation, not a failure, so it is reported as a soft warning. */
bool has_morph_definition(const Object &controller)
{
  const IDProperty *properties = controller.id.system_properties;
  if (properties == nullptr) {
    return false;
  }
  const IDProperty *definition = IDP_GetPropertyFromGroup_null(properties,
                                                               kMorphDefinitionProperty);
  return definition != nullptr && definition->type == IDP_GROUP;
}

std::vector<std::string> collect_bone_names(const bArmature &armature)
{
  std::vector<std::string> names;
  names.reserve(size_t(BKE_armature_bonelist_count(&armature.bonebase)));
  animrig::ANIM_armature_foreach_bone(&armature.bonebase,
                                      [&](const Bone *bone) { names.emplace_back(bone->name); });
  return names;
}

void append_read_report(VMDImportReport &result, ReportList *reports)
{
  for (const std::string &warning : result.read.warnings) {
    result.warnings.push_back(warning);
    BKE_report(reports, RPT_INFO, warning.c_str());
  }
  for (const std::string &error : result.read.errors) {
    result.errors.push_back(error);
    BKE_report(reports, RPT_ERROR, error.c_str());
  }
}

void append_mapping_report(VMDImportReport &result, ReportList *reports)
{
  for (const VMDMappingIssue &issue : result.mapping.issues) {
    if (issue.severity == VMDMappingIssue::Severity::Error) {
      result.errors.push_back(issue.path + ": " + issue.message);
      BKE_report(reports, RPT_ERROR, result.errors.back().c_str());
    }
    else if (issue.severity == VMDMappingIssue::Severity::Warning) {
      result.warnings.push_back(issue.path + ": " + issue.message);
      BKE_report(reports, RPT_INFO, result.warnings.back().c_str());
    }
  }
}

std::string action_stem_from_path(const std::string &filepath)
{
  const char *basename = BLI_path_basename(filepath.c_str());
  const char *extension = BLI_path_extension_or_end(basename);
  std::string stem(basename, extension - basename);
  if (stem.empty()) {
    stem = "VMD Motion";
  }
  return stem;
}

std::string action_name_from_path(const std::string &filepath)
{
  return action_stem_from_path(filepath) + " | VMD";
}

std::string morph_action_name_from_path(const std::string &filepath)
{
  return action_stem_from_path(filepath) + " | VMD Morph";
}

void append_morph_mapping_report(VMDImportReport &result, ReportList *reports)
{
  for (const VMDMappingIssue &issue : result.morph_mapping.issues) {
    const std::string message = "VMD morph " + issue.path + ": " + issue.message;
    if (issue.severity == VMDMappingIssue::Severity::Error) {
      result.errors.push_back(message);
      BKE_report(reports, RPT_ERROR, result.errors.back().c_str());
    }
    else if (issue.severity == VMDMappingIssue::Severity::Warning) {
      result.warnings.push_back(message);
      BKE_report(reports, RPT_INFO, result.warnings.back().c_str());
    }
  }
}

void append_morph_action_report(VMDImportReport &result)
{
  result.warnings.insert(
      result.warnings.end(), result.morph_action.warnings.begin(), result.morph_action.warnings.end());
  result.errors.insert(
      result.errors.end(), result.morph_action.errors.begin(), result.morph_action.errors.end());
}

bool validate_morph_controller(const Object &controller, ReportList *reports, VMDImportReport &result)
{
  if (controller.type != OB_MESH || controller.data == nullptr) {
    add_error(result, reports, "VMD morph target must be a Mesh PMXMorphController object");
    return false;
  }
  const Mesh *mesh = id_cast<const Mesh *>(controller.data);
  if (mesh == nullptr || mesh->key == nullptr) {
    add_error(result, reports, "VMD morph target Mesh has no Shape Key data");
    return false;
  }
  if (mesh->key->block.first == nullptr || mesh->key->refkey == nullptr) {
    add_error(result, reports, "VMD morph target Key has no Basis Shape Key");
    return false;
  }

  std::vector<std::string> names;
  for (const KeyBlock *key_block = static_cast<const KeyBlock *>(mesh->key->block.first);
       key_block != nullptr;
       key_block = key_block->next)
  {
    if (key_block == mesh->key->refkey) {
      continue;
    }
    if (key_block->name[0] == '\0') {
      add_error(result, reports, "VMD morph target contains an empty Shape Key name");
      return false;
    }
    const std::string name(key_block->name);
    if (std::find(names.begin(), names.end(), name) != names.end()) {
      add_error(result, reports, "VMD morph target contains duplicate Shape Key name: " + name);
      return false;
    }
    names.push_back(name);
  }
  if (names.empty()) {
    add_error(result, reports, "VMD morph target Key contains no Morph Shape Keys");
    return false;
  }
  return true;
}

std::vector<std::string> collect_morph_names(const Object &controller)
{
  const Mesh *mesh = id_cast<const Mesh *>(controller.data);
  const Key *key = mesh->key;
  std::vector<std::string> names;
  for (const KeyBlock *key_block = static_cast<const KeyBlock *>(key->block.first);
       key_block != nullptr;
       key_block = key_block->next)
  {
    if (key_block != key->refkey) {
      names.emplace_back(key_block->name);
    }
  }
  return names;
}

/* [世界的歌] C2-2E / R1-VMD: Collect Group Morph raw channel metadata from the
 * controller's persisted mmd_pmx_morph_definition. PMX encodes Group Morph as
 * morph_type 0; only channels flagged controller_channel are real raw channels
 * that can be driven by a VMD morph track. The second set records Group raw
 * channels whose definition marks vertex_output == false (R1-VMD). */
struct GroupMorphChannelInfo {
  std::unordered_set<std::string> group_channel_names;
  std::unordered_set<std::string> group_no_vertex_output_names;
};

GroupMorphChannelInfo read_group_morph_channel_names(Object &controller)
{
  GroupMorphChannelInfo info;
  IDProperty *properties = IDP_ID_system_properties_ensure(&controller.id);
  if (properties == nullptr) {
    return info;
  }
  const IDProperty *definition = IDP_GetPropertyFromGroup_null(properties,
                                                              kMorphDefinitionProperty);
  if (definition == nullptr || definition->type != IDP_GROUP) {
    return info;
  }
  const IDProperty *channels = IDP_GetPropertyFromGroup_null(definition, "channels");
  if (channels == nullptr || channels->type != IDP_IDPARRAY) {
    return info;
  }
  for (int i = 0; i < channels->len; i++) {
    const IDProperty *channel = IDP_GetIndexArray(const_cast<IDProperty *>(channels), i);
    if (channel == nullptr || channel->type != IDP_GROUP) {
      continue;
    }
    const IDProperty *type_prop = IDP_GetPropertyFromGroup_null(channel, "morph_type");
    const IDProperty *name_prop = IDP_GetPropertyFromGroup_null(channel, "controller_key_name");
    const IDProperty *channel_prop = IDP_GetPropertyFromGroup_null(channel, "controller_channel");
    const IDProperty *vertex_prop = IDP_GetPropertyFromGroup_null(channel, "vertex_output");
    if (type_prop == nullptr || type_prop->type != IDP_INT || name_prop == nullptr ||
        name_prop->type != IDP_STRING)
    {
      continue;
    }
    if (channel_prop != nullptr && channel_prop->type == IDP_INT &&
        channel_prop->data.val == 0)
    {
      /* controller_channel is false: not a raw channel */
      continue;
    }
    /* PMX MorphType::Group == 0 */
    if (type_prop->data.val == 0) {
      const char *name = static_cast<const char *>(name_prop->data.pointer);
      info.group_channel_names.insert(name);
      if (vertex_prop != nullptr && vertex_prop->type == IDP_INT && vertex_prop->data.val == 0) {
        info.group_no_vertex_output_names.insert(name);
      }
    }
  }
  return info;
}

/* [世界的歌] C2-2E / R1-VMD: Annotate mapped morph tracks whose target is a
 * Group raw channel, and further flag those with no direct vertex output. This
 * is a pure in-memory decoration of the mapping report; it does not change
 * which tracks are written, only how the result is reported. */
void annotate_group_morph_targets(VMDMorphMappingReport &report, Object &controller)
{
  const GroupMorphChannelInfo info = read_group_morph_channel_names(controller);
  if (info.group_channel_names.empty()) {
    return;
  }
  for (VMDMappedMorphTrack &track : report.mapped_tracks) {
    if (info.group_channel_names.count(track.target_morph_name) > 0) {
      track.is_group_target = true;
      if (info.group_no_vertex_output_names.count(track.target_morph_name) > 0) {
        track.is_group_no_vertex_output = true;
      }
    }
  }
}

bool same_model_context(const Object &armature,
                        const Object &controller,
                        ReportList *reports,
                        VMDImportReport &result)
{
  if (&armature == &controller) {
    add_error(result, reports, "VMD Armature and morph Controller must be different objects");
    return false;
  }
  if (armature.parent != nullptr && controller.parent != nullptr &&
      armature.parent != controller.parent)
  {
    add_error(result, reports, "VMD Armature and morph Controller must share the same PMX root");
    return false;
  }
  return true;
}

bool any_existing_action(const Object &armature, const Key &key)
{
  const AnimData *armature_anim_data = BKE_animdata_from_id(&armature.id);
  const AnimData *key_anim_data = BKE_animdata_from_id(&key.id);
  return (armature_anim_data != nullptr && armature_anim_data->action != nullptr) ||
         (key_anim_data != nullptr && key_anim_data->action != nullptr);
}

void append_action_report(VMDImportReport &result)
{
  result.warnings.insert(result.warnings.end(),
                         result.action.warnings.begin(),
                         result.action.warnings.end());
  result.errors.insert(result.errors.end(), result.action.errors.begin(), result.action.errors.end());
}

void append_camera_action_report(VMDImportReport &result)
{
  result.warnings.insert(result.warnings.end(),
                         result.camera_action.warnings.begin(),
                         result.camera_action.warnings.end());
  result.errors.insert(result.errors.end(),
                       result.camera_action.errors.begin(),
                       result.camera_action.errors.end());
}

/* [世界的歌] D1: Drives each PMX IK bone's `mmd_ik_toggle` channel from the VMD
 * property keyframes' ik_states. mmd_ik_toggle == true means the IK chain is
 * enabled (MMD default); false mutes it.
 *
 * This function creates per-frame F-Curves on the armature action:
 *  1. pose.bones["<ik_bone>"].mmd_ik_toggle  — records the raw VMD toggle state
 *  2. pose.bones["<constraint_bone>"].constraints["MMD_IK_Approx"].influence
 *     — drives iTaSC's IK/FK switch per frame (toggle=1 → IK on, toggle=0 → FK)
 *
 * Without #2, the MMD_IK_Approx constraint influence stays at 1.0 (set by
 * vmd_suspend_mmd_approx_constraints) and IK always overrides FK, causing
 * broken poses on VMD frames that disable IK (e.g. NXDE frame 607 arm).
 * This mirrors mmd_tools, which keyframes the IK constraint influence from
 * the VMD IK toggle property track.
 *
 * VMD property keyframes are full snapshots: each keyframe lists the on/off
 * state of every IK bone at that frame. We expand them into per-bone F-Curves
 * (one keyframe per property frame per IK bone). Bones not mentioned in a
 * given snapshot keep their previous value (held by constant interpolation). */
void apply_vmd_ik_toggle(Object &target_armature,
                         const VMDModel &model,
                         const int frame_offset,
                         ReportList *reports)
{
  bArmature *armature = id_cast<bArmature *>(target_armature.data);
  if (armature == nullptr) {
    return;
  }

  /* Collect IK definitions from the PMX-persisted IK definition.
   * Each definition maps the IK control bone (def.bone_name, which carries
   * the VMD IK toggle) to the constraint-bearing bone (def.links[N-2],
   * where MMD_IK_Approx lives). We need both to drive the constraint's
   * influence F-Curve from the VMD toggle track. */
  struct IKDef {
    std::string ik_bone_name;
    std::string constraint_bone_name;
  };
  std::vector<IKDef> ik_defs;
  const IDProperty *props = target_armature.id.system_properties;
  if (props != nullptr) {
    const IDProperty *definition = IDP_GetPropertyFromGroup_null(props,
                                                                  "mmd_pmx_bone_ik_definition");
    if (definition != nullptr && definition->type == IDP_GROUP) {
      const IDProperty *ik_bones = IDP_GetPropertyFromGroup_null(definition, "ik_bones");
      if (ik_bones != nullptr && ik_bones->type == IDP_IDPARRAY) {
        for (int i = 0; i < ik_bones->len; i++) {
          const IDProperty *item = IDP_GetIndexArray(const_cast<IDProperty *>(ik_bones), i);
          if (item == nullptr || item->type != IDP_GROUP) {
            continue;
          }
          IKDef def;
          const IDProperty *name_prop = IDP_GetPropertyFromGroup_null(item, "name");
          if (name_prop != nullptr && name_prop->type == IDP_STRING) {
            def.ik_bone_name = IDP_string_get(name_prop);
          }
          /* Resolve the constraint-bearing bone from IK chain links.
           * MMD_IK_Approx is created on links[N-2] (e.g. knee for leg IK),
           * or links[0] for single-link chains (e.g. toe IK). */
          const IDProperty *links_prop = IDP_GetPropertyFromGroup_null(item, "links");
          if (links_prop != nullptr && links_prop->type == IDP_IDPARRAY &&
              links_prop->len > 0)
          {
            int idx = links_prop->len >= 2 ? links_prop->len - 2 : 0;
            const IDProperty *link_item = IDP_GetIndexArray(
                const_cast<IDProperty *>(links_prop), idx);
            if (link_item != nullptr && link_item->type == IDP_GROUP) {
              const IDProperty *bone_prop = IDP_GetPropertyFromGroup_null(link_item, "bone");
              if (bone_prop != nullptr && bone_prop->type == IDP_STRING) {
                def.constraint_bone_name = IDP_string_get(bone_prop);
              }
            }
          }
          if (!def.ik_bone_name.empty()) {
            ik_defs.push_back(std::move(def));
          }
        }
      }
    }
  }
  if (ik_defs.empty()) {
    return;
  }

  /* Require an action to write F-Curves into. The action is created by
   * build_vmd_action() which runs before this function. */
  AnimData *adt = BKE_animdata_from_id(&target_armature.id);
  if (adt == nullptr || adt->action == nullptr) {
    BKE_report(reports,
               RPT_WARNING,
               "VMD IK toggle: armature has no Action; skipping IK toggle F-Curves");
    return;
  }
  bAction *action = adt->action;
  const int slot_handle = adt->slot_handle;

  /* Resolve the action's channelbag for the armature's slot. */
  animrig::Action &anim = action->wrap();
  animrig::Channelbag *channelbag = nullptr;
  for (animrig::Layer *layer : anim.layers()) {
    for (animrig::Strip *strip : layer->strips()) {
      channelbag = strip->data<animrig::StripKeyframeData>(anim).channelbag_for_slot(
          slot_handle);
      if (channelbag != nullptr) {
        break;
      }
    }
    if (channelbag != nullptr) {
      break;
    }
  }
  if (channelbag == nullptr) {
    BKE_report(reports,
               RPT_WARNING,
               "VMD IK toggle: no channelbag for armature slot; skipping IK toggle F-Curves");
    return;
  }

  /* Build per-bone timelines from VMD property keyframes.
   * Each property keyframe is a full snapshot; we record the on/off state
   * for every IK bone at that frame. */

  int recorded = 0;
  for (const IKDef &ik_def : ik_defs) {
    const std::string &bone_name = ik_def.ik_bone_name;
    /* Track the toggle state across frames; default true (MMD default). */
    bool current_state = true;

    /* Collect keyframes for this bone. */
    struct IKToggleKey {
      int frame;
      bool enabled;
    };
    std::vector<IKToggleKey> keys;

    for (const VMDPropertyKeyframe &pk : model.property_keyframes) {
      /* Check if this snapshot mentions our bone. */
      bool found = false;
      for (const VMDPropertyIKState &state : pk.ik_states) {
        if (state.bone_name == bone_name) {
          current_state = state.enabled;
          found = true;
          break;
        }
      }
      /* Only record frames where the bone is mentioned OR where the state
       * would visibly change. To keep it simple and correct, record every
       * property keyframe with the current (possibly held) state. */
      const int64_t shifted_frame = int64_t(pk.frame) + int64_t(frame_offset);
      if (shifted_frame < std::numeric_limits<int>::min() ||
          shifted_frame > std::numeric_limits<int>::max())
      {
        BKE_report(reports,
                   RPT_WARNING,
                   "VMD IK toggle: property frame plus offset is outside the supported range");
        continue;
      }
      keys.push_back({int(shifted_frame), current_state});
    }

    if (keys.empty()) {
      /* No property keyframes at all: keep default (true) on the bone. */
      Bone *bone = BKE_armature_find_bone_name(armature, bone_name.c_str());
      if (bone != nullptr) {
        IDProperty *bone_props = bone->system_properties;
        if (bone_props == nullptr) {
          bone_props = blender::bke::idprop::create_group("mmd_ik").release();
          bone->system_properties = bone_props;
        }
        IDProperty *old = IDP_GetPropertyFromGroup_null(bone_props, "mmd_ik_toggle");
        if (old != nullptr) {
          IDP_FreeFromGroup(bone_props, old);
        }
        IDP_AddToGroup(bone_props,
                       blender::bke::idprop::create_bool("mmd_ik_toggle", true).release());
      }
      continue;
    }

    /* Create F-Curve: pose.bones["<bone_name>"].mmd_ik_toggle */
    char escaped_name[128] = {};
    BLI_str_escape(escaped_name, bone_name.c_str(), sizeof(escaped_name));
    const std::string rna_path = std::string("pose.bones[\"") + escaped_name +
                                 "\"].mmd_ik_toggle";

    animrig::FCurveDescriptor descriptor;
    descriptor.rna_path = rna_path;
    descriptor.array_index = 0;
    descriptor.prop_type = PROP_FLOAT;
    descriptor.prop_subtype = PROP_NONE;
    FCurve &fcurve = channelbag->fcurve_ensure(nullptr, descriptor);

    for (const IKToggleKey &key : keys) {
      const float value = key.enabled ? 1.0f : 0.0f;
      const animrig::KeyframeSettings settings = {
          BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, BEZT_IPO_CONST};
      animrig::insert_vert_fcurve(
          &fcurve, {float(key.frame), value}, settings, INSERTKEY_FAST);
    }
    BKE_fcurve_handles_recalc(fcurve);

    /* Create influence F-Curve on the IK constraint so iTaSC switches IK/FK
     * per frame. Without this, influence stays at 1.0 and IK always overrides
     * FK — broken poses on VMD frames that disable IK.
     *
     * The constraint may be named "MMD_IK_Approx" (our auto-apply), "IK"
     * (mmd_tools default or manual add), or something else. Resolve the actual
     * IK constraint name on the constraint-bearing bone so the F-Curve RNA
     * path matches. Prefer "MMD_IK_Approx" if present, else the first IK
     * constraint of type CONSTRAINT_TYPE_KINEMATIC. */
    if (!ik_def.constraint_bone_name.empty()) {
      bPoseChannel *cpchan = BKE_pose_channel_find_name(target_armature.pose,
                                                         ik_def.constraint_bone_name.c_str());
      std::string ik_con_name;
      if (cpchan != nullptr) {
        /* First pass: look for our auto-apply marker name. */
        for (bConstraint *con = static_cast<bConstraint *>(cpchan->constraints.first);
             con != nullptr;
             con = con->next)
        {
          if (con->type == CONSTRAINT_TYPE_KINEMATIC &&
              strcmp(con->name, "MMD_IK_Approx") == 0)
          {
            ik_con_name = con->name;
            break;
          }
        }
        /* Fallback: first IK constraint of any name. */
        if (ik_con_name.empty()) {
          for (bConstraint *con = static_cast<bConstraint *>(cpchan->constraints.first);
               con != nullptr;
               con = con->next)
          {
            if (con->type == CONSTRAINT_TYPE_KINEMATIC) {
              ik_con_name = con->name;
              break;
            }
          }
        }
      }
      if (!ik_con_name.empty()) {
        char escaped_cname[128] = {};
        BLI_str_escape(escaped_cname,
                       ik_def.constraint_bone_name.c_str(),
                       sizeof(escaped_cname));
        char escaped_con[128] = {};
        BLI_str_escape(escaped_con, ik_con_name.c_str(), sizeof(escaped_con));
        const std::string inf_path = std::string("pose.bones[\"") + escaped_cname +
                                     "\"].constraints[\"" + escaped_con + "\"].influence";

        animrig::FCurveDescriptor inf_desc;
        inf_desc.rna_path = inf_path;
        inf_desc.array_index = 0;
        inf_desc.prop_type = PROP_FLOAT;
        inf_desc.prop_subtype = PROP_NONE;
        FCurve &inf_fcurve = channelbag->fcurve_ensure(nullptr, inf_desc);

        for (const IKToggleKey &key : keys) {
          const float value = key.enabled ? 1.0f : 0.0f;
          const animrig::KeyframeSettings settings = {
              BEZT_KEYTYPE_KEYFRAME, HD_AUTO_ANIM, BEZT_IPO_CONST};
          animrig::insert_vert_fcurve(
              &inf_fcurve, {float(key.frame), value}, settings, INSERTKEY_FAST);
        }
        BKE_fcurve_handles_recalc(inf_fcurve);
      }
    }

    /* Also store the first-frame state as a static fallback on the Bone's
     * system_properties (used by the CCD solver before depsgraph evaluation
     * and for models without VMD animation). */
    Bone *bone = BKE_armature_find_bone_name(armature, bone_name.c_str());
    if (bone != nullptr) {
      IDProperty *bone_props = bone->system_properties;
      if (bone_props == nullptr) {
        bone_props = blender::bke::idprop::create_group("mmd_ik").release();
        bone->system_properties = bone_props;
      }
      IDProperty *old = IDP_GetPropertyFromGroup_null(bone_props, "mmd_ik_toggle");
      if (old != nullptr) {
        IDP_FreeFromGroup(bone_props, old);
      }
      IDP_AddToGroup(bone_props,
                     blender::bke::idprop::create_bool("mmd_ik_toggle", keys.front().enabled)
                         .release());
    }
    recorded++;
  }

  if (recorded > 0 && reports != nullptr) {
    BKE_reportf(reports,
                RPT_INFO,
                "VMD IK toggle F-Curves created on %d IK bone(s) (mmd_ik_toggle + "
                "MMD_IK_Approx influence; drives iTaSC IK/FK switch per frame)",
                recorded);
  }
}

}  // namespace

bool import_vmd_action(Main *bmain,
                       Object &target_armature,
                       const std::string &filepath,
                       const VMDImportOptions &options,
                       ReportList *reports,
                       VMDImportReport &r_result)
{
  r_result = {};

  if (target_armature.type != OB_ARMATURE || !target_armature.data) {
    add_error(r_result, reports, "VMD import target must be an Armature object");
    return false;
  }

  bArmature *armature = id_cast<bArmature *>(target_armature.data);
  if (!armature) {
    add_error(r_result, reports, "VMD import target has invalid Armature data");
    return false;
  }

  try {
    VMDModel model = read_vmd(filepath, &r_result.read);
    append_read_report(r_result, reports);

    if (!r_result.read.ok()) {
      return false;
    }

    BKE_pose_ensure(bmain, &target_armature, armature, false);
    const std::vector<std::string> target_bone_names = collect_bone_names(*armature);
    r_result.mapping = map_bone_tracks(model, target_bone_names);
    append_mapping_report(r_result, reports);

    if (!r_result.mapping.mapping_valid) {
      if (r_result.errors.empty()) {
        add_error(r_result, reports, "VMD bone tracks cannot be mapped to the target Armature");
      }
      return false;
    }

    VMDActionOptions action_options;
    action_options.frame_offset = options.frame_offset;
    action_options.replace_existing_action = options.replace_existing_action;
    action_options.coordinate_scale = options.coordinate_scale;
    action_options.use_linear_interpolation = options.use_linear_interpolation;
    action_options.use_vmd_bezier_interpolation = options.use_vmd_bezier_interpolation;

    const std::string action_name = action_name_from_path(filepath);
    if (!build_vmd_action(bmain,
                          target_armature,
                          model,
                          r_result.mapping,
                          action_name,
                          action_options,
                          reports,
                          r_result.action))
    {
      r_result.errors.insert(
          r_result.errors.end(), r_result.action.errors.begin(), r_result.action.errors.end());
      r_result.warnings.insert(
          r_result.warnings.end(), r_result.action.warnings.begin(), r_result.action.warnings.end());
      return false;
    }

    apply_vmd_ik_toggle(target_armature, model, options.frame_offset, reports);

    r_result.success = true;
    return true;
  }
  catch (const VMDReaderError &error) {
    add_error(r_result, reports, error.what());
    return false;
  }
}

bool import_vmd_action_with_morphs(Main *bmain,
                                   Object &target_armature,
                                   Object &target_morph_controller,
                                   const std::string &filepath,
                                   const VMDImportOptions &options,
                                   ReportList *reports,
                                   VMDImportReport &r_result)
{
  r_result = {};

  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD import requires a valid Main database");
    return false;
  }
  if (target_armature.type != OB_ARMATURE || target_armature.data == nullptr) {
    add_error(r_result, reports, "VMD import target must be an Armature object");
    return false;
  }
  bArmature *armature = id_cast<bArmature *>(target_armature.data);
  if (armature == nullptr) {
    add_error(r_result, reports, "VMD import target has invalid Armature data");
    return false;
  }
  if (!same_model_context(target_armature, target_morph_controller, reports, r_result) ||
      !validate_morph_controller(target_morph_controller, reports, r_result))
  {
    return false;
  }

  Mesh *controller_mesh = id_cast<Mesh *>(target_morph_controller.data);
  Key *controller_key = controller_mesh->key;
  if (!options.replace_existing_action && any_existing_action(target_armature, *controller_key)) {
    add_error(r_result,
              reports,
              "VMD targets already have an Action; refusing to replace Armature or Morph Action");
    return false;
  }

  try {
    VMDModel model = read_vmd(filepath, &r_result.read);
    append_read_report(r_result, reports);
    if (!r_result.read.ok()) {
      return false;
    }

    BKE_pose_ensure(bmain, &target_armature, armature, false);
    const std::vector<std::string> bone_names = collect_bone_names(*armature);
    const std::vector<std::string> morph_names = collect_morph_names(target_morph_controller);
    r_result.mapping = map_bone_tracks(model, bone_names);
    r_result.morph_mapping = map_morph_tracks(model, morph_names);
    /* [世界的歌] R4: If the PMX importer never persisted the morph definition on
     * this controller, the Group/Vertex morph-type distinction is unavailable.
     * This is a soft capability warning, not a failure: Vertex Morphs import
     * normally, and Group raw channels are imported by name; their eventual
     * vertex effect is produced by the C2-2D Driver expansion (set up during
     * PMX import) rather than by this Action. */
    if (!has_morph_definition(target_morph_controller)) {
      add_warning(r_result,
                  reports,
                  "PMX morph definition (" + std::string(kMorphDefinitionProperty) +
                      ") is missing on the morph controller; Group/Vertex morph types cannot be "
                      "distinguished. Vertex Morphs import normally; Group Morph raw channels are "
                      "imported by name, and their vertex effect is produced by the C2-2D Driver "
                      "expansion set up during PMX import");
    }
    /* [世界的歌] C2-2E: mark which mapped morph tracks drive a Group raw
     * channel so the result report can distinguish Group from Vertex morphs. */
    annotate_group_morph_targets(r_result.morph_mapping, target_morph_controller);
    append_mapping_report(r_result, reports);
    append_morph_mapping_report(r_result, reports);

    if (!r_result.mapping.mapping_valid) {
      if (r_result.errors.empty()) {
        add_error(r_result, reports, "VMD bone tracks cannot be mapped to the target Armature");
      }
      return false;
    }
    if (!r_result.morph_mapping.mapping_valid) {
      if (r_result.errors.empty()) {
        add_error(r_result, reports, "VMD morph tracks cannot be mapped to the target Controller");
      }
      return false;
    }

    VMDActionOptions action_options;
    action_options.frame_offset = options.frame_offset;
    action_options.replace_existing_action = options.replace_existing_action;
    action_options.coordinate_scale = options.coordinate_scale;
    action_options.use_linear_interpolation = options.use_linear_interpolation;
    action_options.use_vmd_bezier_interpolation = options.use_vmd_bezier_interpolation;

    if (!build_vmd_action(bmain,
                          target_armature,
                          model,
                          r_result.mapping,
                          action_name_from_path(filepath),
                          action_options,
                          reports,
                          r_result.action))
    {
      append_action_report(r_result);
      return false;
    }

    apply_vmd_ik_toggle(target_armature, model, options.frame_offset, reports);

    if (r_result.morph_mapping.mapped_track_count == 0) {
      r_result.morph_action.skipped = true;
      r_result.morph_action.missing_track_count = r_result.morph_mapping.missing_track_count;
      r_result.morph_action.warnings.push_back(
          "VMD contains no Morph tracks matching the target Controller; Morph Action skipped");
      append_morph_action_report(r_result);
    }
    else {
      VMDMorphActionOptions morph_options;
      morph_options.frame_offset = options.frame_offset;
      morph_options.replace_existing_action = options.replace_existing_action;
      /* VMD morph (face) tracks carry no bezier control points, so they are
       * inherently linear in MMD. Never inherit the bone bezier flag, which
       * would otherwise leak a non-VMD-accurate "bezier" path into morphs and
       * trigger the C2-1C accuracy warning. */
      morph_options.use_linear_interpolation = true;
      if (!build_vmd_morph_action(bmain,
                                  *controller_key,
                                  model,
                                  r_result.morph_mapping,
                                  morph_action_name_from_path(filepath),
                                  morph_options,
                                  reports,
                                  r_result.morph_action))
      {
        append_morph_action_report(r_result);
        return false;
      }
    }

    DEG_id_tag_update_ex(bmain,
                         &target_armature.id,
                         ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    DEG_id_tag_update_ex(bmain,
                         &target_morph_controller.id,
                         ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    DEG_id_tag_update_ex(bmain,
                         &controller_key->id,
                         ID_RECALC_ANIMATION | ID_RECALC_GEOMETRY);
    if (AnimData *armature_anim_data = BKE_animdata_from_id(&target_armature.id)) {
      if (armature_anim_data->action != nullptr) {
        DEG_id_tag_update_ex(bmain,
                             &armature_anim_data->action->id,
                             ID_RECALC_ANIMATION_NO_FLUSH);
      }
    }
    if (AnimData *morph_anim_data = BKE_animdata_from_id(&controller_key->id)) {
      if (morph_anim_data->action != nullptr) {
        DEG_id_tag_update_ex(bmain,
                             &morph_anim_data->action->id,
                             ID_RECALC_ANIMATION_NO_FLUSH);
      }
    }

    r_result.success = true;
    return true;
  }
  catch (const VMDReaderError &error) {
    add_error(r_result, reports, error.what());
    return false;
  }
}

bool import_vmd_camera(Main *bmain,
                       Collection &target_collection,
                       const std::string &filepath,
                       const VMDImportOptions &options,
                       ReportList *reports,
                       VMDImportReport &r_result,
                       Object *requested_camera)
{
  r_result = {};
  if (bmain == nullptr) {
    add_error(r_result, reports, "VMD camera import requires a valid Main database");
    return false;
  }

  try {
    VMDModel model = read_vmd(filepath, &r_result.read);
    append_read_report(r_result, reports);
    if (!r_result.read.ok()) {
      return false;
    }
    if (model.camera_keyframes.empty()) {
      add_error(r_result, reports, "VMD file contains no camera keyframes");
      return false;
    }

    if (!model.bone_keyframes.empty() || !model.morph_keyframes.empty() ||
        !model.property_keyframes.empty())
    {
      add_warning(r_result,
                  reports,
                  "VMD camera import ignores bone, morph, and property/IK keyframes");
    }
    if (model.light_frame_count != 0 || model.shadow_frame_count != 0) {
      add_warning(r_result,
                  reports,
                  "VMD camera import ignores light and self-shadow keyframes");
    }

    const std::string action_name = action_name_from_path(filepath);
    const std::string rig_name = action_stem_from_path(filepath) + " | MMD Camera";
    Object *target_empty = nullptr;
    Object *camera_object = nullptr;
    if (!create_vmd_camera_rig(bmain,
                               target_collection,
                               rig_name,
                               options.coordinate_scale,
                               target_empty,
                               camera_object,
                               reports,
                               r_result.camera_action,
                               requested_camera))
    {
      append_camera_action_report(r_result);
      return false;
    }

    VMDCameraActionOptions camera_options;
    camera_options.frame_offset = options.frame_offset;
    camera_options.replace_existing_action = options.replace_existing_action;
    camera_options.coordinate_scale = options.coordinate_scale;
    camera_options.use_linear_interpolation = options.use_linear_interpolation;
    camera_options.use_vmd_bezier_interpolation = options.use_vmd_bezier_interpolation;
    if (!build_vmd_camera_action(bmain,
                                 *target_empty,
                                 *camera_object,
                                 model,
                                 action_name,
                                 camera_options,
                                 reports,
                                 r_result.camera_action))
    {
      append_camera_action_report(r_result);
      if (requested_camera == nullptr) {
        BKE_id_free(bmain, &camera_object->id);
      }
      BKE_id_free(bmain, &target_empty->id);
      return false;
    }
    append_camera_action_report(r_result);
    r_result.success = true;
    return true;
  }
  catch (const VMDReaderError &error) {
    add_error(r_result, reports, error.what());
    return false;
  }
}

}  // namespace blender::io::vmd
