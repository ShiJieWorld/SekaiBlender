#ifndef PMX_WRITER_H
#define PMX_WRITER_H

#include "pmx_types.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

class PMXWriterError : public std::runtime_error {
public:
    explicit PMXWriterError(const std::string& msg) : std::runtime_error(msg) {}
};

// Memory-based binary writer - mirrors MemReader in pmx_reader.h so the two
// stay byte-for-byte symmetric.
class MemWriter {
public:
    MemWriter() = default;

    size_t size() const { return buf_.size(); }
    const std::vector<uint8_t>& buffer() const { return buf_; }
    std::vector<uint8_t> take_buffer() { return std::move(buf_); }

    void reserve(size_t bytes) { buf_.reserve(bytes); }

    void write_byte(uint8_t v) { buf_.push_back(v); }

    void write_signed_byte(int8_t v) { write_byte((uint8_t)v); }

    void write_short(int16_t v) { write_unsigned_short((uint16_t)v); }

    void write_unsigned_short(uint16_t v) {
        buf_.push_back((uint8_t)(v & 0xFF));
        buf_.push_back((uint8_t)((v >> 8) & 0xFF));
    }

    void write_int(int32_t v) { write_unsigned_int((uint32_t)v); }

    void write_unsigned_int(uint32_t v) {
        buf_.push_back((uint8_t)(v & 0xFF));
        buf_.push_back((uint8_t)((v >> 8) & 0xFF));
        buf_.push_back((uint8_t)((v >> 16) & 0xFF));
        buf_.push_back((uint8_t)((v >> 24) & 0xFF));
    }

    /* [世界的歌] The reader rejects non-finite floats, so the writer must never
     * emit them either. Otherwise a written file cannot be read back. */
    void write_float(float v, const char *field = "float") {
        if (!std::isfinite(v)) {
            throw PMXWriterError("Non-finite floating-point value for " + std::string(field));
        }
        uint32_t raw;
        memcpy(&raw, &v, sizeof(raw));
        write_unsigned_int(raw);
    }

    void write_float3(const float v[3], const char *field = "float3") {
        write_float(v[0], field);
        write_float(v[1], field);
        write_float(v[2], field);
    }

    void write_float4(const float v[4], const char *field = "float4") {
        write_float(v[0], field);
        write_float(v[1], field);
        write_float(v[2], field);
        write_float(v[3], field);
    }

    void write_bytes(const void *data, size_t count) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        buf_.insert(buf_.end(), p, p + count);
    }

private:
    std::vector<uint8_t> buf_;
};

struct PMXWriteOptions {
    /**
     * Recompute `header` index sizes / signature / header_size from the actual
     * section counts instead of trusting the values already in `model.header`.
     *
     * This must stay enabled for models that were not produced by PMXReader
     * (e.g. authored in Blender), because a default-constructed PMXHeader has
     * zeroed index sizes which are not a valid PMX encoding.
     *
     * When disabled, the existing header is validated and used verbatim, which
     * is what an exact byte-for-byte round-trip of a parsed file needs.
     */
    bool recompute_index_sizes = true;
};

class PMXWriter {
public:
    /** Serialize `model` to `filepath`. Throws PMXWriterError on failure. */
    static void write(const PMXModel& model,
                      const std::string& filepath,
                      const PMXWriteOptions& options = {});

    /** Serialize `model` into a memory buffer. Throws PMXWriterError on failure. */
    static std::vector<uint8_t> write_to_memory(const PMXModel& model,
                                                const PMXWriteOptions& options = {});

    /**
     * Smallest PMX index size (1, 2 or 4 bytes) able to address `count` items.
     *
     * `is_signed` selects the signed index ranges PMX uses for every index kind
     * except vertex indices, which are read unsigned.
     */
    static uint8_t minimum_index_size(size_t count, bool is_signed);

private:
    PMXWriter(const PMXModel& model, const PMXWriteOptions& options);

    void write_internal();

    /** Produce the header actually emitted, honoring PMXWriteOptions. */
    PMXHeader resolve_header() const;

    // Index helpers
    void write_count(size_t count, const char *field);
    void write_index(int value, uint8_t size, bool is_signed, const char *field);
    void write_vertex_index(int value, const char *field);
    void write_bone_index(int value, const char *field);
    void write_texture_index(int value, const char *field);
    void write_material_index(int value, const char *field);
    void write_morph_index(int value, const char *field);
    void write_rigid_index(int value, const char *field);

    void write_string(const std::string& value, const char *field);

    // Section writers
    void write_header();
    void write_model_info();
    void write_vertices();
    void write_faces();
    void write_textures();
    void write_materials();
    void write_bones();
    void write_morphs();
    void write_display_frames();
    void write_rigid_bodies();
    void write_joints();

    MemWriter w_;
    const PMXModel& model_;
    PMXWriteOptions options_;
    PMXHeader header_{};
};

#endif // PMX_WRITER_H
