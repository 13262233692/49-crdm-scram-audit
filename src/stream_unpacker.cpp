#include "stream_unpacker.h"
#include <cstring>

namespace crdm {

bool StreamUnpacker::init(const MemoryMappedFile& mmf) {
    stats_ = FrameStats{};
    num_channels_ = 0;
    declared_samples_ = 0;
    safe_num_samples_ = 0;
    sample_rate_khz_ = 0;
    frame_stride_ = 0;
    data_start_ = nullptr;
    data_end_ = nullptr;

    if (!mmf.is_open() || mmf.size() < sizeof(FileHeader)) {
        return false;
    }

    const FileHeader* header = reinterpret_cast<const FileHeader*>(mmf.data());

    if (std::memcmp(header->magic, "CRDMAE01", 8) != 0) {
        return false;
    }

    if (header->num_channels == 0 || header->num_channels > 1024) {
        return false;
    }

    if (header->sample_rate_khz == 0 || header->sample_rate_khz > 100000) {
        return false;
    }

    num_channels_ = header->num_channels;
    declared_samples_ = header->num_samples;
    sample_rate_khz_ = header->sample_rate_khz;
    frame_stride_ = sizeof(uint64_t) + num_channels_ * sizeof(int16_t);

    const uint64_t remaining_bytes = mmf.size() - sizeof(FileHeader);
    const uint64_t max_possible_samples = remaining_bytes / frame_stride_;

    safe_num_samples_ = (declared_samples_ < max_possible_samples)
                        ? declared_samples_
                        : max_possible_samples;

    data_start_ = mmf.data() + sizeof(FileHeader);
    data_end_ = data_start_ + safe_num_samples_ * frame_stride_;

    return true;
}

} // namespace crdm
