#include "energy_analyzer.h"
#include <cstring>
#include <cmath>

namespace crdm {

void EnergyBandAnalyzer::init(size_t num_channels) {
    num_channels_ = num_channels;
    per_frame_.reserve(1024);
    timestamps_.reserve(1024);
    reset();
}

void EnergyBandAnalyzer::reset() {
    channels_processed_ = 0;
    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        channel_accumulator_[i] = 0.0;
        global_band_energy_[i] = 0.0;
    }
    global_total_energy_ = 0.0;
    per_frame_.clear();
    timestamps_.clear();
}

void EnergyBandAnalyzer::process_channel_block(
    uint32_t /*channel_idx*/,
    const std::array<std::vector<float>, NUM_SUBBANDS>& subbands)
{
    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        double e = 0.0;
        const size_t n = subbands[i].size();
        const float* __restrict data = subbands[i].data();
        for (size_t j = 0; j < n; ++j) {
            const double v = static_cast<double>(data[j]);
            e += v * v;
        }
        channel_accumulator_[i] += e;
    }
    ++channels_processed_;
}

void EnergyBandAnalyzer::finalize_frame(uint64_t frame_idx, uint64_t timestamp_ns) {
    if (channels_processed_ < num_channels_) {
        return;
    }

    BandEnergy be{};
    double total = 0.0;
    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        be.energy[i] = channel_accumulator_[i];
        total += channel_accumulator_[i];
    }
    be.total_energy = total;

    if (total > 0.0) {
        const double inv_total = 1.0 / total;
        for (int i = 0; i < NUM_SUBBANDS; ++i) {
            be.ratio[i] = be.energy[i] * inv_total;
        }
    }

    per_frame_.push_back(be);
    timestamps_.push_back(timestamp_ns);

    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        global_band_energy_[i] += be.energy[i];
        global_total_energy_ += be.energy[i];
    }

    channels_processed_ = 0;
    for (int i = 0; i < NUM_SUBBANDS; ++i) {
        channel_accumulator_[i] = 0.0;
    }
}

void EnergyBandAnalyzer::compute_global_ratios(double ratios[NUM_SUBBANDS]) const {
    if (global_total_energy_ > 0.0) {
        const double inv = 1.0 / global_total_energy_;
        for (int i = 0; i < NUM_SUBBANDS; ++i) {
            ratios[i] = global_band_energy_[i] * inv;
        }
    } else {
        for (int i = 0; i < NUM_SUBBANDS; ++i) {
            ratios[i] = 0.0;
        }
    }
}

} // namespace crdm
