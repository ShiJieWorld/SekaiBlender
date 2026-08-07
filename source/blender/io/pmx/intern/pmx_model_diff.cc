#include "pmx_model_diff.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>

namespace {

/* Accumulates issues for one section, capping how many are recorded while still
 * counting all of them. A section that is wholly wrong then reads as one number
 * instead of tens of thousands of lines. */
class SectionDiff {
 public:
  SectionDiff(PMXModelDiffReport &report, const PMXModelDiffOptions &options, const char *name)
      : report_(report), options_(options)
  {
    PMXModelDiffSection section;
    section.name = name;
    report_.sections.push_back(section);
    index_ = int(report_.sections.size()) - 1;
  }

  void add(const std::string &path, const std::string &message)
  {
    PMXModelDiffSection &section = report_.sections[index_];
    section.issue_count++;
    report_.total_issues++;
    if (section.reported_count >= options_.max_issues_per_section) {
      return;
    }
    section.reported_count++;
    report_.issues.push_back({path, message});
  }

 private:
  PMXModelDiffReport &report_;
  const PMXModelDiffOptions &options_;
  int index_ = 0;
};

/* Paths are only materialized when a difference is found. Building them
 * unconditionally would allocate several strings per vertex. */
std::string path_of(const char *section, const int index, const char *field)
{
  char buffer[256];
  if (index < 0) {
    std::snprintf(buffer, sizeof(buffer), "%s.%s", section, field);
  }
  else {
    std::snprintf(buffer, sizeof(buffer), "%s[%d].%s", section, index, field);
  }
  return buffer;
}

std::string nested_path_of(
    const char *section, const int index, const char *sub, const int sub_index, const char *field)
{
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), "%s[%d].%s[%d].%s", section, index, sub, sub_index, field);
  return buffer;
}

std::string format_float(const float value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9g", double(value));
  return buffer;
}

std::string format_vector(const float *values, const int count)
{
  std::string result = "(";
  for (int i = 0; i < count; i++) {
    if (i != 0) {
      result += ", ";
    }
    result += format_float(values[i]);
  }
  result += ")";
  return result;
}

bool floats_close(const float a, const float b, const float tolerance)
{
  if (!std::isfinite(a) || !std::isfinite(b)) {
    /* PMXReader rejects non-finite values, so reaching this means one side did
     * not come from the reader. Never treat it as equal. */
    return false;
  }
  if (tolerance <= 0.0f) {
    return a == b;
  }
  return std::fabs(a - b) <= tolerance;
}

void cmp_int(SectionDiff &diff,
             const int expected,
             const int actual,
             const char *section,
             const int index,
             const char *field)
{
  if (expected == actual) {
    return;
  }
  diff.add(path_of(section, index, field),
           "expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}

void cmp_float(SectionDiff &diff,
               const float tolerance,
               const float expected,
               const float actual,
               const char *section,
               const int index,
               const char *field)
{
  if (floats_close(expected, actual, tolerance)) {
    return;
  }
  diff.add(path_of(section, index, field),
           "expected " + format_float(expected) + ", got " + format_float(actual));
}

void cmp_string(SectionDiff &diff,
                const std::string &expected,
                const std::string &actual,
                const char *section,
                const int index,
                const char *field)
{
  if (expected == actual) {
    return;
  }
  diff.add(path_of(section, index, field), "expected '" + expected + "', got '" + actual + "'");
}

/* One issue per vector rather than per component: a wrong axis mapping differs
 * in several components at once and is clearer read as a whole vector. */
void cmp_vec(SectionDiff &diff,
             const float tolerance,
             const float *expected,
             const float *actual,
             const int count,
             const char *section,
             const int index,
             const char *field)
{
  for (int i = 0; i < count; i++) {
    if (!floats_close(expected[i], actual[i], tolerance)) {
      diff.add(path_of(section, index, field),
               "expected " + format_vector(expected, count) + ", got " +
                   format_vector(actual, count));
      return;
    }
  }
}

void cmp_nested_vec(SectionDiff &diff,
                    const float tolerance,
                    const float *expected,
                    const float *actual,
                    const int count,
                    const char *section,
                    const int index,
                    const char *sub,
                    const int sub_index,
                    const char *field)
{
  for (int i = 0; i < count; i++) {
    if (!floats_close(expected[i], actual[i], tolerance)) {
      diff.add(nested_path_of(section, index, sub, sub_index, field),
               "expected " + format_vector(expected, count) + ", got " +
                   format_vector(actual, count));
      return;
    }
  }
}

/**
 * Report a container size mismatch and return the number of items that can
 * still be compared.
 *
 * The prefix is compared even when counts differ, so a count error does not
 * hide the field errors behind it.
 */
size_t cmp_count(SectionDiff &diff,
                 const size_t expected,
                 const size_t actual,
                 const char *section,
                 const char *field)
{
  if (expected != actual) {
    diff.add(path_of(section, -1, field),
             "expected " + std::to_string(expected) + ", got " + std::to_string(actual));
  }
  return expected < actual ? expected : actual;
}

/**
 * Collapse a vertex's weight slots to a bone -> total-weight map.
 *
 * Slot-wise comparison is wrong here, because two PMX weight arrays can describe
 * the identical deformation without agreeing slot for slot. Both differences are
 * properties of the source format, measured on a real model by
 * `PMXRoundTripTest.reports_skinning_slots_blender_cannot_represent`:
 *
 *   - A zero-weight slot still carries a bone index. Import drops the slot
 *     (`pmx_import_weights.cc` skips `weight <= 0`), so the index is not
 *     recoverable on export. A zero-weight bone deforms nothing, so this is
 *     inert. Measured: 14180 of 58242 vertices.
 *
 *   - The same bone may occupy more than one weighted slot. A Blender vertex
 *     group is a bone -> weight mapping and cannot hold a duplicate, so import
 *     accumulates (`dw->weight += weight`). Applying one bone twice at w1 and w2
 *     is identical to applying it once at w1+w2. Measured: 1755 vertices, 2265
 *     slots collapsed.
 *
 * Summing per bone and dropping negligible weights makes both invisible while
 * still catching what matters: a weight moved to the wrong bone, a lost
 * influence, or a changed magnitude.
 */
std::map<int, float> summed_influences(const PMXVertex &vertex, const float negligible)
{
  std::map<int, float> totals;
  const size_t count = vertex.bone_indices.size() < vertex.bone_weights.size() ?
                           vertex.bone_indices.size() :
                           vertex.bone_weights.size();
  for (size_t i = 0; i < count; i++) {
    const float weight = vertex.bone_weights[i];
    if (!(std::fabs(weight) > negligible)) {
      continue;
    }
    totals[vertex.bone_indices[i]] += weight;
  }
  return totals;
}

/** One Vertex Morph offset, summed per vertex. */
struct SummedOffset {
  float value[3] = {0.0f, 0.0f, 0.0f};
};

/**
 * Collapse a Vertex Morph's offsets to a vertex -> total-offset map.
 *
 * Element-wise comparison is wrong here for the same reason it is wrong for
 * skinning: two offset lists can describe the identical deformation without
 * agreeing element for element. Measured on a real model by
 * `PMXRoundTripTest.reports_vertex_morph_offsets_blender_cannot_represent`:
 *
 *   - PMX imposes no order on a Morph's offsets, and 10 of 119 Morphs in the
 *     sample are not ascending by vertex index. A Shape Key is indexed by
 *     vertex and cannot carry the file's order, so export emits ascending.
 *     Comparing element-wise would report those 10 Morphs as wholly different
 *     when every offset is in fact present and correct.
 *
 *   - The same vertex may appear more than once in one Morph. Import
 *     accumulates (`pmx_import_morph.cc` does `data[index] += offset`), so
 *     export recovers one entry carrying the sum. Displacing a vertex twice is
 *     identical to displacing it once by the total. Measured 0 in the sample,
 *     but it is legal PMX and summing costs nothing.
 *
 * Offsets whose total is negligible are dropped. An explicitly zero offset is
 * legal PMX and indistinguishable from a vertex the Morph does not touch, so
 * requiring it to round-trip would be requiring export to recover information
 * the Shape Key cannot hold. Using the comparison tolerance as the threshold is
 * what makes this self-consistent: an offset this small compares equal to zero
 * anyway, so keeping it could only ever produce a difference the comparator
 * would immediately call insignificant.
 *
 * What this still catches: an offset on the wrong vertex, a lost offset, a
 * changed magnitude, and a wholly empty section -- every expected entry is then
 * reported absent.
 */
std::map<int, SummedOffset> summed_vertex_offsets(const PMXMorph &morph, const float negligible)
{
  std::map<int, SummedOffset> totals;
  for (const PMXVertexMorphOffset &offset : morph.vertex_offsets) {
    SummedOffset &total = totals[offset.vertex_index];
    for (int i = 0; i < 3; i++) {
      total.value[i] += offset.offset[i];
    }
  }
  /* Sum first, then drop: two offsets on one vertex can cancel, and the
   * cancelled result is what import would have stored. */
  std::map<int, SummedOffset>::iterator it = totals.begin();
  while (it != totals.end()) {
    const float *value = it->second.value;
    const bool significant = std::fabs(value[0]) > negligible ||
                             std::fabs(value[1]) > negligible ||
                             std::fabs(value[2]) > negligible;
    if (significant) {
      ++it;
    }
    else {
      it = totals.erase(it);
    }
  }
  return totals;
}

/* --- Sections --------------------------------------------------------------- */

void diff_header(PMXModelDiffReport &report,
                 const PMXModelDiffOptions &options,
                 const PMXModel &expected,
                 const PMXModel &actual)
{
  SectionDiff diff(report, options, "header");
  cmp_float(diff, 0.0f, expected.header.version, actual.header.version, "header", -1, "version");
  cmp_int(diff, expected.header.encoding, actual.header.encoding, "header", -1, "encoding");
  cmp_int(diff, expected.header.add_uv_cnt, actual.header.add_uv_cnt, "header", -1, "add_uv_cnt");

  /* `signature` and `header_size` are format constants, and the index sizes are
   * deliberately not compared: the writer recomputes the narrowest width that
   * fits, which can legitimately differ from whatever the source file used
   * without changing a single value the model holds. */
}

void diff_model_info(PMXModelDiffReport &report,
                     const PMXModelDiffOptions &options,
                     const PMXModel &expected,
                     const PMXModel &actual)
{
  SectionDiff diff(report, options, "model_info");
  cmp_string(diff, expected.name_local, actual.name_local, "model_info", -1, "name_local");
  cmp_string(
      diff, expected.name_universal, actual.name_universal, "model_info", -1, "name_universal");
  cmp_string(
      diff, expected.comment_local, actual.comment_local, "model_info", -1, "comment_local");
  cmp_string(diff,
             expected.comment_universal,
             actual.comment_universal,
             "model_info",
             -1,
             "comment_universal");
}

void diff_vertices(PMXModelDiffReport &report,
                   const PMXModelDiffOptions &options,
                   const PMXModel &expected,
                   const PMXModel &actual)
{
  SectionDiff diff(report, options, "vertices");
  const size_t count = cmp_count(
      diff, expected.vertices.size(), actual.vertices.size(), "vertices", "count");
  const int add_uv = expected.header.add_uv_cnt < actual.header.add_uv_cnt ?
                         expected.header.add_uv_cnt :
                         actual.header.add_uv_cnt;

  for (size_t i = 0; i < count; i++) {
    const PMXVertex &a = expected.vertices[i];
    const PMXVertex &b = actual.vertices[i];
    const int index = int(i);

    cmp_vec(diff, options.geometry_tolerance, a.pos, b.pos, 3, "vertices", index, "pos");
    cmp_vec(diff, options.normal_tolerance, a.normal, b.normal, 3, "vertices", index, "normal");
    cmp_vec(diff, options.unit_tolerance, a.uv, b.uv, 2, "vertices", index, "uv");
    for (int set = 0; set < add_uv; set++) {
      char field[32];
      std::snprintf(field, sizeof(field), "additional_uv[%d]", set);
      cmp_vec(diff,
              options.unit_tolerance,
              a.additional_uv[set].data(),
              b.additional_uv[set].data(),
              4,
              "vertices",
              index,
              field);
    }

    cmp_int(diff,
            int(a.weight_type),
            int(b.weight_type),
            "vertices",
            index,
            "weight_type");
    /* PMXReader normalizes both arrays to the length implied by `weight_type`,
     * so a matching type guarantees matching lengths. Comparing them under a
     * mismatched type would only produce noise on top of the real difference. */
    if (a.weight_type == b.weight_type) {
      const std::map<int, float> expected_influences = summed_influences(a,
                                                                        options.unit_tolerance);
      const std::map<int, float> actual_influences = summed_influences(b, options.unit_tolerance);
      for (const std::pair<const int, float> &entry : expected_influences) {
        const std::map<int, float>::const_iterator found = actual_influences.find(entry.first);
        if (found == actual_influences.end()) {
          diff.add(path_of("vertices", index, "influences"),
                   "bone " + std::to_string(entry.first) + " carries weight " +
                       format_float(entry.second) + " but is absent");
          continue;
        }
        if (!floats_close(entry.second, found->second, options.unit_tolerance)) {
          diff.add(path_of("vertices", index, "influences"),
                   "bone " + std::to_string(entry.first) + ": expected weight " +
                       format_float(entry.second) + ", got " + format_float(found->second));
        }
      }
      for (const std::pair<const int, float> &entry : actual_influences) {
        if (expected_influences.find(entry.first) == expected_influences.end()) {
          diff.add(path_of("vertices", index, "influences"),
                   "bone " + std::to_string(entry.first) + " gained weight " +
                       format_float(entry.second));
        }
      }
    }

    if (a.weight_type == BoneWeightType::SDEF && b.weight_type == BoneWeightType::SDEF) {
      cmp_vec(diff, options.geometry_tolerance, a.sdef_c, b.sdef_c, 3, "vertices", index, "sdef_c");
      cmp_vec(
          diff, options.geometry_tolerance, a.sdef_r0, b.sdef_r0, 3, "vertices", index, "sdef_r0");
      cmp_vec(
          diff, options.geometry_tolerance, a.sdef_r1, b.sdef_r1, 3, "vertices", index, "sdef_r1");
    }

    cmp_float(diff,
              options.unit_tolerance,
              a.edge_factor,
              b.edge_factor,
              "vertices",
              index,
              "edge_factor");
  }
}

void diff_faces(PMXModelDiffReport &report,
                const PMXModelDiffOptions &options,
                const PMXModel &expected,
                const PMXModel &actual)
{
  SectionDiff diff(report, options, "faces");
  const size_t count = cmp_count(
      diff, expected.face_indices.size(), actual.face_indices.size(), "faces", "count");
  for (size_t i = 0; i < count; i++) {
    if (expected.face_indices[i] != actual.face_indices[i]) {
      diff.add(path_of("faces", int(i), "vertex_index"),
               "expected " + std::to_string(expected.face_indices[i]) + ", got " +
                   std::to_string(actual.face_indices[i]));
    }
  }
}

void diff_textures(PMXModelDiffReport &report,
                   const PMXModelDiffOptions &options,
                   const PMXModel &expected,
                   const PMXModel &actual)
{
  SectionDiff diff(report, options, "textures");
  const size_t count = cmp_count(
      diff, expected.textures.size(), actual.textures.size(), "textures", "count");
  for (size_t i = 0; i < count; i++) {
    cmp_string(
        diff, expected.textures[i].path, actual.textures[i].path, "textures", int(i), "path");
  }
}

void diff_materials(PMXModelDiffReport &report,
                    const PMXModelDiffOptions &options,
                    const PMXModel &expected,
                    const PMXModel &actual)
{
  SectionDiff diff(report, options, "materials");
  const size_t count = cmp_count(
      diff, expected.materials.size(), actual.materials.size(), "materials", "count");
  const float tol = options.metadata_tolerance;

  for (size_t i = 0; i < count; i++) {
    const PMXMaterial &a = expected.materials[i];
    const PMXMaterial &b = actual.materials[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "materials", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "materials", index, "name_universal");
    cmp_vec(diff, tol, a.diffuse, b.diffuse, 4, "materials", index, "diffuse");
    cmp_vec(diff, tol, a.specular, b.specular, 3, "materials", index, "specular");
    cmp_float(diff, tol, a.specular_power, b.specular_power, "materials", index, "specular_power");
    cmp_vec(diff, tol, a.ambient, b.ambient, 3, "materials", index, "ambient");
    cmp_int(diff, a.flag, b.flag, "materials", index, "flag");
    cmp_vec(diff, tol, a.edge_color, b.edge_color, 4, "materials", index, "edge_color");
    cmp_float(diff, tol, a.edge_size, b.edge_size, "materials", index, "edge_size");
    cmp_int(diff, a.texture_idx, b.texture_idx, "materials", index, "texture_idx");
    cmp_int(diff,
            a.sphere_texture_idx,
            b.sphere_texture_idx,
            "materials",
            index,
            "sphere_texture_idx");
    cmp_int(diff, int(a.sphere_mode), int(b.sphere_mode), "materials", index, "sphere_mode");
    cmp_int(diff, a.toon_flag, b.toon_flag, "materials", index, "toon_flag");
    cmp_int(diff, a.toon_texture_idx, b.toon_texture_idx, "materials", index, "toon_texture_idx");
    cmp_int(diff,
            a.toon_internal_value,
            b.toon_internal_value,
            "materials",
            index,
            "toon_internal_value");
    cmp_string(diff, a.memo, b.memo, "materials", index, "memo");
    cmp_int(diff,
            a.face_vertex_count,
            b.face_vertex_count,
            "materials",
            index,
            "face_vertex_count");
  }
}

void diff_bones(PMXModelDiffReport &report,
                const PMXModelDiffOptions &options,
                const PMXModel &expected,
                const PMXModel &actual)
{
  SectionDiff diff(report, options, "bones");
  const size_t count = cmp_count(diff, expected.bones.size(), actual.bones.size(), "bones", "count");

  for (size_t i = 0; i < count; i++) {
    const PMXBone &a = expected.bones[i];
    const PMXBone &b = actual.bones[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "bones", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "bones", index, "name_universal");
    /* The bone head is a scaled round-trip, unlike the rest of this section. */
    cmp_vec(diff, options.geometry_tolerance, a.pos, b.pos, 3, "bones", index, "pos");
    cmp_int(diff, a.parent_index, b.parent_index, "bones", index, "parent_index");
    cmp_int(diff, a.transform_order, b.transform_order, "bones", index, "transform_order");
    cmp_int(diff, a.flag, b.flag, "bones", index, "flag");
    cmp_int(diff, a.tail_pos_bone, b.tail_pos_bone, "bones", index, "tail_pos_bone");
    cmp_vec(diff,
            options.geometry_tolerance,
            a.tail_pos_offset,
            b.tail_pos_offset,
            3,
            "bones",
            index,
            "tail_pos_offset");
    cmp_int(diff,
            a.inherit_parent_index,
            b.inherit_parent_index,
            "bones",
            index,
            "inherit_parent_index");
    cmp_float(diff,
              options.metadata_tolerance,
              a.inherit_parent_ratio,
              b.inherit_parent_ratio,
              "bones",
              index,
              "inherit_parent_ratio");
    cmp_vec(diff,
            options.unit_tolerance,
            a.fixed_axis,
            b.fixed_axis,
            3,
            "bones",
            index,
            "fixed_axis");
    cmp_vec(diff, options.unit_tolerance, a.local_x, b.local_x, 3, "bones", index, "local_x");
    cmp_vec(diff, options.unit_tolerance, a.local_z, b.local_z, 3, "bones", index, "local_z");
    cmp_int(diff, a.ik_target_index, b.ik_target_index, "bones", index, "ik_target_index");
    cmp_int(diff, a.ik_loop_count, b.ik_loop_count, "bones", index, "ik_loop_count");
    cmp_float(diff,
              options.unit_tolerance,
              a.ik_angle_limit,
              b.ik_angle_limit,
              "bones",
              index,
              "ik_angle_limit");
    cmp_int(diff,
            a.external_parent_index,
            b.external_parent_index,
            "bones",
            index,
            "external_parent_index");

    const size_t link_count = cmp_count(
        diff, a.ik_links.size(), b.ik_links.size(), "bones", "ik_links.count");
    for (size_t j = 0; j < link_count; j++) {
      const PMXIKLink &la = a.ik_links[j];
      const PMXIKLink &lb = b.ik_links[j];
      if (la.bone_index != lb.bone_index) {
        diff.add(nested_path_of("bones", index, "ik_links", int(j), "bone_index"),
                 "expected " + std::to_string(la.bone_index) + ", got " +
                     std::to_string(lb.bone_index));
      }
      if (la.limit_angle != lb.limit_angle) {
        diff.add(nested_path_of("bones", index, "ik_links", int(j), "limit_angle"),
                 std::string("expected ") + (la.limit_angle ? "true" : "false") + ", got " +
                     (lb.limit_angle ? "true" : "false"));
      }
      if (la.limit_angle && lb.limit_angle) {
        cmp_nested_vec(diff,
                       options.unit_tolerance,
                       la.limit_min,
                       lb.limit_min,
                       3,
                       "bones",
                       index,
                       "ik_links",
                       int(j),
                       "limit_min");
        cmp_nested_vec(diff,
                       options.unit_tolerance,
                       la.limit_max,
                       lb.limit_max,
                       3,
                       "bones",
                       index,
                       "ik_links",
                       int(j),
                       "limit_max");
      }
    }
  }
}

void diff_morphs(PMXModelDiffReport &report,
                 const PMXModelDiffOptions &options,
                 const PMXModel &expected,
                 const PMXModel &actual)
{
  SectionDiff diff(report, options, "morphs");
  const size_t count = cmp_count(
      diff, expected.morphs.size(), actual.morphs.size(), "morphs", "count");
  const float meta = options.metadata_tolerance;

  for (size_t i = 0; i < count; i++) {
    const PMXMorph &a = expected.morphs[i];
    const PMXMorph &b = actual.morphs[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "morphs", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "morphs", index, "name_universal");
    cmp_int(diff, a.panel, b.panel, "morphs", index, "panel");
    cmp_int(diff, int(a.type), int(b.type), "morphs", index, "type");
    if (a.type != b.type) {
      /* PMXReader fills only the offset vector matching the type, so comparing
       * offsets across different types reports emptiness, not a real
       * difference. */
      continue;
    }

    const size_t group_count = cmp_count(
        diff, a.group_offsets.size(), b.group_offsets.size(), "morphs", "group_offsets.count");
    for (size_t j = 0; j < group_count; j++) {
      if (a.group_offsets[j].morph_index != b.group_offsets[j].morph_index) {
        diff.add(nested_path_of("morphs", index, "group_offsets", int(j), "morph_index"),
                 "expected " + std::to_string(a.group_offsets[j].morph_index) + ", got " +
                     std::to_string(b.group_offsets[j].morph_index));
      }
      if (!floats_close(a.group_offsets[j].influence, b.group_offsets[j].influence, meta)) {
        diff.add(nested_path_of("morphs", index, "group_offsets", int(j), "influence"),
                 "expected " + format_float(a.group_offsets[j].influence) + ", got " +
                     format_float(b.group_offsets[j].influence));
      }
    }

    /* Vertex Morph offsets are compared as a vertex -> total-offset map rather
     * than element-wise, for the reasons `summed_vertex_offsets` documents:
     * source order is arbitrary and export cannot reproduce it.
     *
     * The raw count is deliberately not compared. It is not the count that
     * proves the offsets survived -- an absent entry is reported by the map
     * comparison below, so a wholly empty section produces one issue per lost
     * offset rather than a single count line. An earlier version compared
     * neither the count nor the contents and hid the total loss of 119 Morphs'
     * worth of offsets; comparing the map is what actually closes that hole,
     * and it does so without failing on the 10 Morphs whose source order simply
     * differs. */
    const std::map<int, SummedOffset> expected_offsets = summed_vertex_offsets(
        a, options.geometry_tolerance);
    const std::map<int, SummedOffset> actual_offsets = summed_vertex_offsets(
        b, options.geometry_tolerance);
    for (const std::pair<const int, SummedOffset> &entry : expected_offsets) {
      const std::map<int, SummedOffset>::const_iterator found = actual_offsets.find(entry.first);
      if (found == actual_offsets.end()) {
        diff.add(path_of("morphs", index, "vertex_offsets"),
                 "vertex " + std::to_string(entry.first) + " expected offset " +
                     format_vector(entry.second.value, 3) + " but is absent");
        continue;
      }
      for (int i = 0; i < 3; i++) {
        if (!floats_close(entry.second.value[i], found->second.value[i], options.geometry_tolerance))
        {
          diff.add(path_of("morphs", index, "vertex_offsets"),
                   "vertex " + std::to_string(entry.first) + ": expected offset " +
                       format_vector(entry.second.value, 3) + ", got " +
                       format_vector(found->second.value, 3));
          break;
        }
      }
    }
    for (const std::pair<const int, SummedOffset> &entry : actual_offsets) {
      if (expected_offsets.find(entry.first) == expected_offsets.end()) {
        diff.add(path_of("morphs", index, "vertex_offsets"),
                 "vertex " + std::to_string(entry.first) + " gained offset " +
                     format_vector(entry.second.value, 3));
      }
    }

    const size_t bone_count = cmp_count(
        diff, a.bone_offsets.size(), b.bone_offsets.size(), "morphs", "bone_offsets.count");
    for (size_t j = 0; j < bone_count; j++) {
      if (a.bone_offsets[j].bone_index != b.bone_offsets[j].bone_index) {
        diff.add(nested_path_of("morphs", index, "bone_offsets", int(j), "bone_index"),
                 "expected " + std::to_string(a.bone_offsets[j].bone_index) + ", got " +
                     std::to_string(b.bone_offsets[j].bone_index));
      }
      cmp_nested_vec(diff,
                     options.geometry_tolerance,
                     a.bone_offsets[j].pos,
                     b.bone_offsets[j].pos,
                     3,
                     "morphs",
                     index,
                     "bone_offsets",
                     int(j),
                     "pos");
      cmp_nested_vec(diff,
                     options.unit_tolerance,
                     a.bone_offsets[j].rot,
                     b.bone_offsets[j].rot,
                     4,
                     "morphs",
                     index,
                     "bone_offsets",
                     int(j),
                     "rot");
    }

    const size_t uv_count = cmp_count(
        diff, a.uv_offsets.size(), b.uv_offsets.size(), "morphs", "uv_offsets.count");
    for (size_t j = 0; j < uv_count; j++) {
      if (a.uv_offsets[j].vertex_index != b.uv_offsets[j].vertex_index) {
        diff.add(nested_path_of("morphs", index, "uv_offsets", int(j), "vertex_index"),
                 "expected " + std::to_string(a.uv_offsets[j].vertex_index) + ", got " +
                     std::to_string(b.uv_offsets[j].vertex_index));
        break;
      }
      cmp_nested_vec(diff,
                     options.unit_tolerance,
                     a.uv_offsets[j].offset,
                     b.uv_offsets[j].offset,
                     4,
                     "morphs",
                     index,
                     "uv_offsets",
                     int(j),
                     "offset");
    }

    const size_t material_count = cmp_count(diff,
                                            a.material_offsets.size(),
                                            b.material_offsets.size(),
                                            "morphs",
                                            "material_offsets.count");
    for (size_t j = 0; j < material_count; j++) {
      const PMXMaterialMorphOffset &ma = a.material_offsets[j];
      const PMXMaterialMorphOffset &mb = b.material_offsets[j];
      if (ma.material_index != mb.material_index) {
        diff.add(nested_path_of("morphs", index, "material_offsets", int(j), "material_index"),
                 "expected " + std::to_string(ma.material_index) + ", got " +
                     std::to_string(mb.material_index));
      }
      if (ma.calc_mode != mb.calc_mode) {
        diff.add(nested_path_of("morphs", index, "material_offsets", int(j), "calc_mode"),
                 "expected " + std::to_string(ma.calc_mode) + ", got " +
                     std::to_string(mb.calc_mode));
      }
      const int sub = int(j);
      cmp_nested_vec(
          diff, meta, ma.diffuse, mb.diffuse, 4, "morphs", index, "material_offsets", sub, "diffuse");
      cmp_nested_vec(diff,
                     meta,
                     ma.specular,
                     mb.specular,
                     3,
                     "morphs",
                     index,
                     "material_offsets",
                     sub,
                     "specular");
      if (!floats_close(ma.specular_power, mb.specular_power, meta)) {
        diff.add(nested_path_of("morphs", index, "material_offsets", sub, "specular_power"),
                 "expected " + format_float(ma.specular_power) + ", got " +
                     format_float(mb.specular_power));
      }
      cmp_nested_vec(
          diff, meta, ma.ambient, mb.ambient, 3, "morphs", index, "material_offsets", sub, "ambient");
      cmp_nested_vec(diff,
                     meta,
                     ma.edge_color,
                     mb.edge_color,
                     4,
                     "morphs",
                     index,
                     "material_offsets",
                     sub,
                     "edge_color");
      if (!floats_close(ma.edge_size, mb.edge_size, meta)) {
        diff.add(nested_path_of("morphs", index, "material_offsets", sub, "edge_size"),
                 "expected " + format_float(ma.edge_size) + ", got " + format_float(mb.edge_size));
      }
      cmp_nested_vec(diff,
                     meta,
                     ma.texture_factor,
                     mb.texture_factor,
                     4,
                     "morphs",
                     index,
                     "material_offsets",
                     sub,
                     "texture_factor");
      cmp_nested_vec(diff,
                     meta,
                     ma.sphere_texture_factor,
                     mb.sphere_texture_factor,
                     4,
                     "morphs",
                     index,
                     "material_offsets",
                     sub,
                     "sphere_texture_factor");
      cmp_nested_vec(diff,
                     meta,
                     ma.toon_texture_factor,
                     mb.toon_texture_factor,
                     4,
                     "morphs",
                     index,
                     "material_offsets",
                     sub,
                     "toon_texture_factor");
    }

    const size_t impulse_count = cmp_count(diff,
                                           a.impulse_offsets.size(),
                                           b.impulse_offsets.size(),
                                           "morphs",
                                           "impulse_offsets.count");
    for (size_t j = 0; j < impulse_count; j++) {
      const PMXImpulseMorphOffset &ia = a.impulse_offsets[j];
      const PMXImpulseMorphOffset &ib = b.impulse_offsets[j];
      if (ia.rigid_index != ib.rigid_index) {
        diff.add(nested_path_of("morphs", index, "impulse_offsets", int(j), "rigid_index"),
                 "expected " + std::to_string(ia.rigid_index) + ", got " +
                     std::to_string(ib.rigid_index));
      }
      if (ia.local_flag != ib.local_flag) {
        diff.add(nested_path_of("morphs", index, "impulse_offsets", int(j), "local_flag"),
                 "expected " + std::to_string(ia.local_flag) + ", got " +
                     std::to_string(ib.local_flag));
      }
      cmp_nested_vec(diff,
                     options.geometry_tolerance,
                     ia.velocity,
                     ib.velocity,
                     3,
                     "morphs",
                     index,
                     "impulse_offsets",
                     int(j),
                     "velocity");
      cmp_nested_vec(diff,
                     options.geometry_tolerance,
                     ia.torque,
                     ib.torque,
                     3,
                     "morphs",
                     index,
                     "impulse_offsets",
                     int(j),
                     "torque");
    }
  }
}

void diff_display_frames(PMXModelDiffReport &report,
                         const PMXModelDiffOptions &options,
                         const PMXModel &expected,
                         const PMXModel &actual)
{
  SectionDiff diff(report, options, "display_frames");
  const size_t count = cmp_count(diff,
                                 expected.display_frames.size(),
                                 actual.display_frames.size(),
                                 "display_frames",
                                 "count");

  for (size_t i = 0; i < count; i++) {
    const PMXDisplayFrame &a = expected.display_frames[i];
    const PMXDisplayFrame &b = actual.display_frames[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "display_frames", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "display_frames", index, "name_universal");
    cmp_int(diff, a.flag, b.flag, "display_frames", index, "flag");

    const size_t item_count = cmp_count(
        diff, a.items.size(), b.items.size(), "display_frames", "items.count");
    for (size_t j = 0; j < item_count; j++) {
      if (a.items[j].type != b.items[j].type || a.items[j].index != b.items[j].index) {
        diff.add(nested_path_of("display_frames", index, "items", int(j), "entry"),
                 "expected (type " + std::to_string(a.items[j].type) + ", index " +
                     std::to_string(a.items[j].index) + "), got (type " +
                     std::to_string(b.items[j].type) + ", index " +
                     std::to_string(b.items[j].index) + ")");
      }
    }
  }
}

void diff_rigid_bodies(PMXModelDiffReport &report,
                       const PMXModelDiffOptions &options,
                       const PMXModel &expected,
                       const PMXModel &actual)
{
  SectionDiff diff(report, options, "rigid_bodies");
  const size_t count = cmp_count(
      diff, expected.rigid_bodies.size(), actual.rigid_bodies.size(), "rigid_bodies", "count");

  for (size_t i = 0; i < count; i++) {
    const PMXRigidBody &a = expected.rigid_bodies[i];
    const PMXRigidBody &b = actual.rigid_bodies[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "rigid_bodies", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "rigid_bodies", index, "name_universal");
    cmp_int(diff, a.bone_index, b.bone_index, "rigid_bodies", index, "bone_index");
    cmp_int(diff, a.collision_group, b.collision_group, "rigid_bodies", index, "collision_group");
    cmp_int(diff,
            a.no_collision_group,
            b.no_collision_group,
            "rigid_bodies",
            index,
            "no_collision_group");
    cmp_int(diff, a.shape_type, b.shape_type, "rigid_bodies", index, "shape_type");
    /* Sizes and positions are scaled; rotation and scalar parameters are
     * restored without a scale operation. */
    cmp_vec(diff,
            options.geometry_tolerance,
            a.shape_size,
            b.shape_size,
            3,
            "rigid_bodies",
            index,
            "shape_size");
    cmp_vec(diff, options.geometry_tolerance, a.pos, b.pos, 3, "rigid_bodies", index, "pos");
    cmp_vec(diff, options.unit_tolerance, a.rot, b.rot, 3, "rigid_bodies", index, "rot");
    cmp_float(diff, options.unit_tolerance, a.mass, b.mass, "rigid_bodies", index, "mass");
    cmp_float(diff,
              options.unit_tolerance,
              a.linear_damping,
              b.linear_damping,
              "rigid_bodies",
              index,
              "linear_damping");
    cmp_float(diff,
              options.unit_tolerance,
              a.angular_damping,
              b.angular_damping,
              "rigid_bodies",
              index,
              "angular_damping");
    cmp_float(diff,
              options.unit_tolerance,
              a.restitution,
              b.restitution,
              "rigid_bodies",
              index,
              "restitution");
    cmp_float(
        diff, options.unit_tolerance, a.friction, b.friction, "rigid_bodies", index, "friction");
    cmp_int(diff, a.physics_type, b.physics_type, "rigid_bodies", index, "physics_type");
  }
}

void diff_joints(PMXModelDiffReport &report,
                 const PMXModelDiffOptions &options,
                 const PMXModel &expected,
                 const PMXModel &actual)
{
  SectionDiff diff(report, options, "joints");
  const size_t count = cmp_count(
      diff, expected.joints.size(), actual.joints.size(), "joints", "count");

  for (size_t i = 0; i < count; i++) {
    const PMXJoint &a = expected.joints[i];
    const PMXJoint &b = actual.joints[i];
    const int index = int(i);

    cmp_string(diff, a.name_local, b.name_local, "joints", index, "name_local");
    cmp_string(diff, a.name_universal, b.name_universal, "joints", index, "name_universal");
    cmp_int(diff, a.type, b.type, "joints", index, "type");
    cmp_int(diff, a.rigid_a_index, b.rigid_a_index, "joints", index, "rigid_a_index");
    cmp_int(diff, a.rigid_b_index, b.rigid_b_index, "joints", index, "rigid_b_index");
    cmp_vec(diff, options.geometry_tolerance, a.pos, b.pos, 3, "joints", index, "pos");
    cmp_vec(diff, options.unit_tolerance, a.rot, b.rot, 3, "joints", index, "rot");
    cmp_vec(diff,
            options.geometry_tolerance,
            a.translation_limit_min,
            b.translation_limit_min,
            3,
            "joints",
            index,
            "translation_limit_min");
    cmp_vec(diff,
            options.geometry_tolerance,
            a.translation_limit_max,
            b.translation_limit_max,
            3,
            "joints",
            index,
            "translation_limit_max");
    cmp_vec(diff,
            options.unit_tolerance,
            a.rotation_limit_min,
            b.rotation_limit_min,
            3,
            "joints",
            index,
            "rotation_limit_min");
    cmp_vec(diff,
            options.unit_tolerance,
            a.rotation_limit_max,
            b.rotation_limit_max,
            3,
            "joints",
            index,
            "rotation_limit_max");
    cmp_vec(diff,
            options.unit_tolerance,
            a.spring_translation,
            b.spring_translation,
            3,
            "joints",
            index,
            "spring_translation");
    cmp_vec(diff,
            options.unit_tolerance,
            a.spring_rotation,
            b.spring_rotation,
            3,
            "joints",
            index,
            "spring_rotation");
  }
}

}  // namespace

std::string PMXModelDiffReport::to_string() const
{
  if (equal()) {
    return "";
  }
  std::ostringstream out;
  out << "PMX model diff: " << total_issues << " difference(s)\n";
  out << "  sections with differences:\n";
  for (const PMXModelDiffSection &section : sections) {
    if (section.issue_count == 0) {
      continue;
    }
    out << "    " << section.name << ": " << section.issue_count;
    if (section.issue_count > section.reported_count) {
      out << " (showing " << section.reported_count << ")";
    }
    out << "\n";
  }
  for (const PMXModelDiffIssue &issue : issues) {
    out << "  " << issue.path << ": " << issue.message << "\n";
  }
  return out.str();
}

PMXModelDiffReport diff_pmx_models(const PMXModel &expected,
                                   const PMXModel &actual,
                                   const PMXModelDiffOptions &options)
{
  PMXModelDiffReport report;
  diff_header(report, options, expected, actual);
  diff_model_info(report, options, expected, actual);
  diff_vertices(report, options, expected, actual);
  diff_faces(report, options, expected, actual);
  diff_textures(report, options, expected, actual);
  diff_materials(report, options, expected, actual);
  diff_bones(report, options, expected, actual);
  diff_morphs(report, options, expected, actual);
  diff_display_frames(report, options, expected, actual);
  diff_rigid_bodies(report, options, expected, actual);
  diff_joints(report, options, expected, actual);
  return report;
}
