#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <random>

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
#pragma pack(pop)

int main() {
    const char* filename = "test_crdm_data.bin";
    const uint32_t num_channels = 4;
    const uint64_t num_samples = 32768;
    const uint32_t sample_rate_khz = 500;

    FileHeader hdr{};
    std::memcpy(hdr.magic, "CRDMAE01", 8);
    hdr.version = 1;
    hdr.num_channels = num_channels;
    hdr.sample_rate_khz = sample_rate_khz;
    hdr.num_samples = num_samples;
    hdr.timestamp_resolution_ns = 1;

    FILE* f = std::fopen(filename, "wb");
    if (!f) {
        std::fprintf(stderr, "Cannot create %s\n", filename);
        return 1;
    }

    std::fwrite(&hdr, sizeof(hdr), 1, f);

    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, 0.1f);

    const double dt_ns = 1e9 / (sample_rate_khz * 1000.0);
    uint64_t base_ts = 1718409600000000000ULL;

    for (uint64_t s = 0; s < num_samples; ++s) {
        uint64_t ts = base_ts + static_cast<uint64_t>(s * dt_ns);
        std::fwrite(&ts, sizeof(uint64_t), 1, f);

        double t = static_cast<double>(s) / (sample_rate_khz * 1000.0);

        for (uint32_t ch = 0; ch < num_channels; ++ch) {
            double freq_hz = 50000.0 + ch * 75000.0;
            double v = 0.5 * std::sin(2.0 * M_PI * freq_hz * t) + noise(rng);
            if (s > num_samples * 0.6 && s < num_samples * 0.7) {
                double burst_freq = 200000.0 + ch * 50000.0;
                v += 1.5 * std::sin(2.0 * M_PI * burst_freq * t) *
                     std::exp(-8.0 * (static_cast<double>(s) / num_samples - 0.65) *
                              (static_cast<double>(s) / num_samples - 0.65));
            }
            int16_t raw = static_cast<int16_t>(std::max(-1.0, std::min(1.0, v)) * 32767.0);
            std::fwrite(&raw, sizeof(int16_t), 1, f);
        }
    }

    std::fclose(f);
    std::printf("Generated test file: %s (%u channels, %llu samples)\n",
                filename, num_channels,
                static_cast<unsigned long long>(num_samples));
    return 0;
}
