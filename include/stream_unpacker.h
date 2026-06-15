#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include "memory_mapped_file.h"

namespace crdm {

#pragma pack(push, 1)
struct FileHeader {
    char magic[8];
    uint32_t version;
    uint32_t num_channels;
    uint32_t sample_rate_khz;
    uint64_t num_samples;
    uint64_t timestamp_resolution_ns;
    uint8_t reserved[28];
};

struct SampleFrame {
    uint64_t timestamp_ns;
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 64, "FileHeader must be 64 bytes");

class StreamUnpacker {
public:
    struct ChannelData {
        uint64_t timestamp_ns;
        std::vector<float> voltages;
    };

    StreamUnpacker() = default;

    bool init(const MemoryMappedFile& mmf);

    [[nodiscard]] uint32_t num_channels() const noexcept { return num_channels_; }
    [[nodiscard]] uint64_t num_samples() const noexcept { return num_samples_; }
    [[nodiscard]] uint32_t sample_rate_khz() const noexcept { return sample_rate_khz_; }

    class Iterator {
    public:
        Iterator(const uint8_t* ptr, uint32_t num_channels, uint64_t frame_idx)
            : ptr_(ptr), num_channels_(num_channels), frame_idx_(frame_idx) {}

        Iterator& operator++() {
            ptr_ += sizeof(uint64_t) + num_channels_ * sizeof(int16_t);
            ++frame_idx_;
            return *this;
        }

        bool operator!=(const Iterator& other) const { return ptr_ != other.ptr_; }

        void read(uint64_t& ts, float* values) const {
            const uint8_t* p = ptr_;
            std::memcpy(&ts, p, sizeof(uint64_t));
            p += sizeof(uint64_t);
            for (uint32_t i = 0; i < num_channels_; ++i) {
                int16_t raw;
                std::memcpy(&raw, p, sizeof(int16_t));
                p += sizeof(int16_t);
                values[i] = static_cast<float>(raw) * 3.051850947599719e-05f;
            }
        }

        uint64_t frame_index() const { return frame_idx_; }

    private:
        const uint8_t* ptr_;
        uint32_t num_channels_;
        uint64_t frame_idx_;
    };

    Iterator begin() const { return Iterator(data_start_, num_channels_, 0); }
    Iterator end() const { return Iterator(data_end_, num_channels_, num_samples_); }

    size_t frame_stride() const { return sizeof(uint64_t) + num_channels_ * sizeof(int16_t); }

private:
    uint32_t num_channels_ = 0;
    uint64_t num_samples_ = 0;
    uint32_t sample_rate_khz_ = 0;
    const uint8_t* data_start_ = nullptr;
    const uint8_t* data_end_ = nullptr;
};

} // namespace crdm
