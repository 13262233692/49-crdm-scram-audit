#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
#include "wavelet_packet.h"

namespace crdm {

class EnergyBandAnalyzer {
public:
    static constexpr int NUM_SUBBANDS = WaveletPacketDecomposer::NUM_SUBBANDS;

    struct BandEnergy {
        double energy[NUM_SUBBANDS];
        double total_energy;
        double ratio[NUM_SUBBANDS];
    };

    EnergyBandAnalyzer() = default;

    void init(size_t num_channels);

    void process_channel_block(uint32_t channel_idx,
                               const std::array<std::vector<float>, NUM_SUBBANDS>& subbands);

    void finalize_frame(uint64_t frame_idx, uint64_t timestamp_ns);

    void reset();

    const std::vector<BandEnergy>& per_frame_energy() const { return per_frame_; }
    const std::vector<uint64_t>& timestamps() const { return timestamps_; }
    const std::array<double, NUM_SUBBANDS>& global_band_energy() const { return global_band_energy_; }
    double global_total_energy() const { return global_total_energy_; }

    void compute_global_ratios(double ratios[NUM_SUBBANDS]) const;

private:
    size_t num_channels_ = 0;
    std::array<double, NUM_SUBBANDS> channel_accumulator_;
    uint32_t channels_processed_ = 0;

    std::vector<BandEnergy> per_frame_;
    std::vector<uint64_t> timestamps_;

    std::array<double, NUM_SUBBANDS> global_band_energy_{};
    double global_total_energy_ = 0.0;
};

} // namespace crdm
