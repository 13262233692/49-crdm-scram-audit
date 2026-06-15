#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
#include <string>
#include <sstream>
#include <iomanip>

namespace crdm {

class FatigueAnalyzer {
public:
    static constexpr int NUM_SUBBANDS = 8;
    static constexpr int CRACK_EROSION_BAND = 2;
    static constexpr double CRITICAL_BAND_RATIO = 0.12;
    static constexpr double MINER_FAILURE_THRESHOLD = 1.0;
    static constexpr double STEEL_DENSITY_G_CM3 = 7.85;
    static constexpr double BEARING_SURFACE_AREA_CM2 = 12.6;
    static constexpr double SENSING_VOLUME_DEPTH_UM = 450.0;
    static constexpr double FATIGUE_LIMIT_STRESS_MPA = 280.0;
    static constexpr double DAMAGE_COEFFICIENT_K = 2.75e-9;

    struct ShearFatigueGridPoint {
        double stress_mpa;
        double cycles_to_failure_n;
    };

    struct DamageEvent {
        uint64_t start_timestamp_ns;
        uint64_t duration_ns;
        double peak_band_ratio;
        double energy_integral;
        double stress_mpa;
        double miner_fraction;
        double mass_loss_ug;
        double metal_loss_um;
    };

    struct FatigueResult {
        double total_band2_energy = 0.0;
        double total_energy = 0.0;
        double avg_band2_ratio = 0.0;
        double peak_band2_ratio = 0.0;

        uint64_t high_stress_duration_ns = 0;
        uint64_t total_event_count = 0;
        uint64_t critical_event_count = 0;

        double palmgren_miner_sum_D = 0.0;
        double remaining_life_fraction = 0.0;
        double predicted_remaining_scrams = 0.0;

        double cumulative_mass_loss_mg = 0.0;
        double cumulative_metal_loss_um = 0.0;
        double current_mass_loss_rate_mg_h = 0.0;

        std::vector<DamageEvent> events;
        std::vector<double> history_band2_ratio;
        std::vector<double> history_miner_D;
        std::vector<double> history_mass_loss;
    };

    FatigueAnalyzer();

    void init(uint32_t sample_rate_khz, size_t num_history_slots = 64);

    void reset();

    void process_block(const double global_energy[8],
                       const double block_band_ratio[8],
                       double block_total_energy,
                       uint64_t block_start_ts_ns,
                       uint64_t block_duration_ns,
                       uint64_t block_index);

    const FatigueResult& result() const { return result_; }

    std::string render_ascii_spindle_report(size_t bar_width = 50) const;

    std::string render_miner_trend(size_t width = 64, size_t height = 14) const;

private:
    static double estimate_stress_mpa(double band_energy, double duration_sec) noexcept;
    static double lookup_cycles_to_failure(double stress_mpa) noexcept;
    static double est_mass_loss_ug(double stress_mpa, double n_cycles) noexcept;

    static const std::array<ShearFatigueGridPoint, 10> s_n_grid_;

    FatigueResult result_;
    uint32_t sample_rate_khz_ = 500;
    size_t history_capacity_ = 64;
    double running_band2_accum_ = 0.0;
    double running_total_accum_ = 0.0;
    uint64_t event_open_start_ns_ = 0;
    double event_peak_ratio_ = 0.0;
    double event_energy_ = 0.0;
    bool in_critical_event_ = false;
};

} // namespace crdm
