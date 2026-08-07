#ifndef PMX_READER_H
#define PMX_READER_H

#include "pmx_types.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <stdexcept>
#include <vector>
#include <cstdint>
#include <cmath>

class PMXReaderError : public std::runtime_error {
public:
    explicit PMXReaderError(const std::string& msg) : std::runtime_error(msg) {}
};

// Memory-based binary reader - no file I/O issues
class MemReader {
public:
    MemReader(const uint8_t* data, size_t size)
        : data_(data), size_(size), pos_(0) {}

    size_t pos() const { return pos_; }
    size_t size() const { return size_; }
    const uint8_t* data() const { return data_; }

    /* [世界的歌] Keep all binary bounds checks overflow-safe. */
    bool has_remaining(size_t bytes) const { return pos_ <= size_ && bytes <= size_ - pos_; }

    void require_remaining(size_t bytes, const char *field = "data") const {
        if (!has_remaining(bytes)) {
            throw PMXReaderError("Unexpected EOF reading " + std::string(field));
        }
    }

    uint8_t read_byte() {
        require_remaining(1);
        return data_[pos_++];
    }

    int8_t read_signed_byte() { return (int8_t)read_byte(); }

    int16_t read_short() {
        require_remaining(2);
        int16_t v = (int16_t)(data_[pos_] | (data_[pos_+1] << 8));
        pos_ += 2;
        return v;
    }

    uint16_t read_unsigned_short() {
        require_remaining(2);
        uint16_t v = (uint16_t)(data_[pos_] | (data_[pos_+1] << 8));
        pos_ += 2;
        return v;
    }

    int32_t read_int() {
        require_remaining(4);
        int32_t v = (int32_t)(data_[pos_] | (data_[pos_+1] << 8) |
                              (data_[pos_+2] << 16) | (data_[pos_+3] << 24));
        pos_ += 4;
        return v;
    }

    uint32_t read_unsigned_int() {
        require_remaining(4);
        uint32_t v = (uint32_t)(data_[pos_] | (data_[pos_+1] << 8) |
                                (data_[pos_+2] << 16) | (data_[pos_+3] << 24));
        pos_ += 4;
        return v;
    }

    float read_float(const char *field = "float") {
        const size_t offset = pos_;
        uint32_t raw = read_unsigned_int();
        float f;
        memcpy(&f, &raw, sizeof(f));
        if (!std::isfinite(f)) {
            throw PMXReaderError("Non-finite floating-point value for " + std::string(field) +
                                 " at offset " + std::to_string(offset));
        }
        return f;
    }

    void read_float3(float out[3]) {
        out[0] = read_float();
        out[1] = read_float();
        out[2] = read_float();
    }

    void read_float4(float out[4]) {
        out[0] = read_float();
        out[1] = read_float();
        out[2] = read_float();
        out[3] = read_float();
    }

    void skip(size_t n) {
        require_remaining(n);
        pos_ += n;
    }

    void seek(size_t p) {
        if (p > size_) throw PMXReaderError("Seek past end");
        pos_ = p;
    }

private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_;
};

class PMXReader {
public:
    static PMXModel read(const std::string& filepath);

    /**
     * Parse a PMX model straight out of memory.
     *
     * `read()` is a thin file wrapper around this; exposing it lets a writer
     * round-trip be verified without touching the filesystem. `filepath` is only
     * used for diagnostics.
     */
    static PMXModel read_from_memory(const uint8_t* data,
                                     size_t size,
                                     const std::string& filepath = "<memory>");

private:
    PMXReader(const uint8_t* data, size_t size, const std::string& filepath);
    ~PMXReader() = default;

    PMXModel read_internal();
    void read_section(const char *section_name, void (PMXReader::*reader)());

    // Index helpers
    int read_count(const char *field, size_t minimum_bytes_per_item, int hard_limit);
    int read_index(uint8_t size, bool is_signed);
    void validate_index_value(int value,
                              size_t count,
                              bool allow_minus_one,
                              const char *field) const;
    void validate_references();
    int read_vertex_index();
    int read_bone_index();
    int read_texture_index();
    int read_material_index();
    int read_morph_index();
    int read_rigid_index();

    std::string read_string();

    // Section readers
    void read_header();
    void read_model_info();
    void read_vertices();
    void read_faces();
    void read_textures();
    void read_materials();
    void read_bones();
    void read_morphs();
    void read_display_frames();
    void read_rigid_bodies();
    void read_joints();

    MemReader r_;
    std::string filepath_;
    PMXModel model_;
};

#endif // PMX_READER_H
