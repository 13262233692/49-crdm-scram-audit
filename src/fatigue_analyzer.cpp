#include "fatigue_analyzer.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace crdm {

const std::array<FatigueAnalyzer::ShearFatigueGridPoint, 10>
FatigueAnalyzer::s_n_grid_ = {{
    { 700.0,      1000.0 },
    { 550.0,     10000.0 },
    { 450.0,    100000.0 },
    { 380.0,    500000.0 },
    { 340.0,   1000000.0 },
    { 310.0,   5000000.0 },
    { 295.0,  10000000.0 },
    { 285.0,  50000000.0 },
    { 280.0, 100000000.0 },
    { 280.0, 10000000000.0 }
}};

FatigueAnalyzer::FatigueAnalyzer() = default;

void FatigueAnalyzer::init(uint32_t sample_rate_khz, size_t num_history_slots) {
    sample_rate_khz_ = sample_rate_khz;
    history_capacity_ = num_history_slots;
    reset();
}

void FatigueAnalyzer::reset() {
    result_ = FatigueResult{};
    result_.remaining_life_fraction = 1.0;
    result_.predicted_remaining_scrams = 10000.0;
    running_band2_accum_ = 0.0;
    running_total_accum_ = 0.0;
    event_open_start_ns_ = 0;
    event_peak_ratio_ = 0.0;
    event_energy_ = 0.0;
    in_critical_event_ = false;
    result_.events.reserve(256);
    result_.history_band2_ratio.reserve(history_capacity_);
    result_.history_miner_D.reserve(history_capacity_);
    result_.history_mass_loss.reserve(history_capacity_);
}

double FatigueAnalyzer::estimate_stress_mpa(double band_energy, double duration_sec) noexcept {
    if (duration_sec <= 0.0) return 0.0;
    const double power = band_energy / duration_sec;
    const double stress_rel = std::pow(power * DAMAGE_COEFFICIENT_K, 0.5);
    return 20.0 + 680.0 * stress_rel / (1.0 + stress_rel);
}

double FatigueAnalyzer::lookup_cycles_to_failure(double stress_mpa) noexcept {
    if (stress_mpa >= s_n_grid_[0].stress_mpa) {
        const double ratio = stress_mpa / s_n_grid_[0].stress_mpa;
        const double p = std::pow(ratio, 4.0);
        return std::max(50.0, s_n_grid_[0].cycles_to_failure_n / p);
    }
    if (stress_mpa <= s_n_grid_.back().stress_mpa) {
        return s_n_grid_.back().cycles_to_failure_n;
    }
    for (size_t i = 1; i < s_n_grid_.size(); ++i) {
        if (stress_mpa >= s_n_grid_[i].stress_mpa) {
            const ShearFatigueGridPoint& hi = s_n_grid_[i-1];
            const ShearFatigueGridPoint& lo = s_n_grid_[i];
            const double span = hi.stress_mpa - lo.stress_mpa;
            const double alpha = span > 0 ? (stress_mpa - lo.stress_mpa) / span : 0.0;
            const double log_n = (1.0 - alpha) * std::log10(lo.cycles_to_failure_n)
                               + alpha * std::log10(hi.cycles_to_failure_n);
            return std::pow(10.0, log_n);
        }
    }
    return s_n_grid_.back().cycles_to_failure_n;
}

double FatigueAnalyzer::est_mass_loss_ug(double stress_mpa, double n_cycles) noexcept {
    const double delta_stress = std::max(0.0, stress_mpa - FATIGUE_LIMIT_STRESS_MPA);
    const double wear_factor = 1.0 - std::exp(-delta_stress / 95.0);
    const double s_ratio = stress_mpa / 300.0;
    return 0.000012 * n_cycles * wear_factor * wear_factor * std::pow(s_ratio, 2.3);
}

void FatigueAnalyzer::process_block(
    const double* /*global_energy*/,
    const double* block_band_ratio,
    double block_total_energy,
    uint64_t block_start_ts_ns,
    uint64_t block_duration_ns,
    uint64_t /*block_index*/)
{
    const double band2_ratio = block_band_ratio[CRACK_EROSION_BAND];
    const double band2_energy = block_total_energy * band2_ratio;
    const double duration_sec = static_cast<double>(block_duration_ns) * 1e-9;

    result_.total_energy += block_total_energy;
    result_.total_band2_energy += band2_energy;
    result_.peak_band2_ratio = std::max(result_.peak_band2_ratio, band2_ratio);

    running_band2_accum_ += band2_energy;
    running_total_accum_ += block_total_energy;

    const bool above_threshold = band2_ratio >= CRITICAL_BAND_RATIO;

    if (above_threshold && !in_critical_event_) {
        in_critical_event_ = true;
        event_open_start_ns_ = block_start_ts_ns;
        event_peak_ratio_ = band2_ratio;
        event_energy_ = band2_energy;
    } else if (above_threshold && in_critical_event_) {
        event_peak_ratio_ = std::max(event_peak_ratio_, band2_ratio);
        event_energy_ += band2_energy;
    }

    if (in_critical_event_ && !above_threshold) {
        DamageEvent ev{};
        ev.start_timestamp_ns = event_open_start_ns_;
        ev.duration_ns = block_start_ts_ns - event_open_start_ns_ + block_duration_ns;
        ev.peak_band_ratio = event_peak_ratio_;
        ev.energy_integral = event_energy_;
        const double ev_sec = std::max(1e-6, static_cast<double>(ev.duration_ns) * 1e-9);
        ev.stress_mpa = estimate_stress_mpa(ev.energy_integral, ev_sec);
        const double n_cycles = ev_sec * static_cast<double>(sample_rate_khz_) * 1000.0 * 0.01;
        const double N_f = lookup_cycles_to_failure(ev.stress_mpa);
        ev.miner_fraction = n_cycles / std::max(N_f, 1.0);
        ev.mass_loss_ug = est_mass_loss_ug(ev.stress_mpa, n_cycles);
        const double volume_cm3 = ev.mass_loss_ug * 1e-6 / STEEL_DENSITY_G_CM3;
        const double area_cm2 = BEARING_SURFACE_AREA_CM2;
        ev.metal_loss_um = (volume_cm3 * 1e12) / std::max(area_cm2 * 1e8, 1.0) * 1000.0
                         * (SENSING_VOLUME_DEPTH_UM / 500.0);

        result_.palmgren_miner_sum_D += ev.miner_fraction;
        result_.cumulative_mass_loss_mg += ev.mass_loss_ug * 0.001;
        result_.cumulative_metal_loss_um += ev.metal_loss_um;
        result_.high_stress_duration_ns += ev.duration_ns;
        result_.total_event_count++;
        if (ev.stress_mpa > 400.0) result_.critical_event_count++;
        result_.events.push_back(ev);

        in_critical_event_ = false;
        event_open_start_ns_ = 0;
        event_peak_ratio_ = 0.0;
        event_energy_ = 0.0;
    }

    result_.history_band2_ratio.push_back(band2_ratio);
    result_.history_miner_D.push_back(result_.palmgren_miner_sum_D);
    result_.history_mass_loss.push_back(result_.cumulative_mass_loss_mg);

    if (result_.palmgren_miner_sum_D < MINER_FAILURE_THRESHOLD) {
        result_.remaining_life_fraction = 1.0 - result_.palmgren_miner_sum_D;
    } else {
        const double excess = result_.palmgren_miner_sum_D - MINER_FAILURE_THRESHOLD;
        result_.remaining_life_fraction = std::max(0.0, 1.0 / (1.0 + excess));
    }

    const size_t histsize = result_.history_band2_ratio.size();
    if (histsize > history_capacity_) {
        const size_t erase_n = histsize - history_capacity_;
        result_.history_band2_ratio.erase(result_.history_band2_ratio.begin(),
                                          result_.history_band2_ratio.begin() + erase_n);
        result_.history_miner_D.erase(result_.history_miner_D.begin(),
                                      result_.history_miner_D.begin() + erase_n);
        result_.history_mass_loss.erase(result_.history_mass_loss.begin(),
                                        result_.history_mass_loss.begin() + erase_n);
    }

    if (result_.total_energy > 0) {
        result_.avg_band2_ratio = result_.total_band2_energy / result_.total_energy;
    }

    uint64_t ref_ts = block_start_ts_ns;
    if (!result_.events.empty()) {
        ref_ts = result_.events.front().start_timestamp_ns;
    }
    const double total_observed_ns = static_cast<double>(
        (block_start_ts_ns + block_duration_ns) - ref_ts);
    const double total_observed_sec = total_observed_ns * 1e-9;
    if (total_observed_sec > 3600.0) {
        result_.current_mass_loss_rate_mg_h = result_.cumulative_mass_loss_mg * 3600.0 / total_observed_sec;
    } else {
        const double denom = std::max(total_observed_sec, 60.0);
        result_.current_mass_loss_rate_mg_h = result_.cumulative_mass_loss_mg * 3600.0 / denom;
    }

    if (result_.total_event_count > 0) {
        const double avg_D_per_event = result_.palmgren_miner_sum_D /
            static_cast<double>(result_.total_event_count);
        if (avg_D_per_event > 0) {
            result_.predicted_remaining_scrams = result_.remaining_life_fraction / avg_D_per_event;
        }
    }
}

static const char* get_health_label(double remaining) {
    if (remaining > 0.85) return "EXCELLENT (A+)";
    if (remaining > 0.65) return "GOOD (A)";
    if (remaining > 0.40) return "WATCHLIST (B)";
    if (remaining > 0.20) return "INSPECTION REQUIRED (C)";
    if (remaining > 0.05) return "CRITICAL - PLANNED OUTAGE (D)";
    return "RED LINE - SCRAM IMMINENT (F)";
}

static const char* get_health_marker(double remaining) {
    if (remaining > 0.85) return "";
    if (remaining > 0.65) return "";
    if (remaining > 0.40) return "";
    if (remaining > 0.20) return "!";
    if (remaining > 0.05) return "!!";
    return "!!!";
}

std::string FatigueAnalyzer::render_ascii_spindle_report(size_t bar_width) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    const double life = result_.remaining_life_fraction;
    const double miner = result_.palmgren_miner_sum_D;
    const size_t life_fill = static_cast<size_t>(std::max(0.0, life * static_cast<double>(bar_width)));

    const char* HL = "+";
    const char* VL = "|";
    const char* HB = "-";
    const char* CR = "+";
    const char* CL = "+";
    const char* BR = "+";
    const char* BL = "+";
    const char* VBar = "|";
    const char* HBar = "=";

    (void)HL; (void)VL; (void)HB; (void)CR; (void)CL; (void)BR; (void)BL; (void)VBar; (void)HBar;

    oss << "\n";
    oss << "  +==================================================================================+\n";
    oss << "  |  /_\\ SPINDLE / BEARING FATIGUE - RESIDUAL LIFE PROJECTION  /_\\                |\n";
    oss << "  |  ###   [PHENOMENOLOGICAL PALMGREN-MINER LINEAR DAMAGE]    ###                |\n";
    oss << "  +==================================================================================+\n";
    oss << "  |                                                                                  |\n";

    oss << "  |  #### REMAINING LIFE FRACTION  ####                                             |\n";
    oss << "  |  [";
    for (size_t i = 0; i < bar_width; ++i) oss << (i < life_fill ? "#" : ".");
    oss << "]  |\n";

    char line[512];
    const char* marker = get_health_marker(life);
    const char* label  = get_health_label(life);
    std::snprintf(line, sizeof(line), "  |   LIFE = %6.2f%%   |  Sum D = %7.4f   (Miner D<1.0 limit = %s) %s %s\n",
        life * 100.0, miner, miner < 1.0 ? "SAFE" : "EXCEEDED", marker, label);
    oss << line;
    for (size_t i = std::strlen(line); i < 84; ++i) oss << " ";
    oss << "|\n";
    oss << "  |                                                                                  |\n";

    oss << "  |  #### CRACK / EROSION BAND (120~180 kHz = Sub-band #2)  ####                       |\n";
    oss << "  |  +------------------+-------------------------------------------------+         |\n";

    const double max_ratio = std::max(result_.peak_band2_ratio, CRITICAL_BAND_RATIO * 2.5);
    const int rw = 38;
    const size_t avg_f = static_cast<size_t>(result_.avg_band2_ratio / max_ratio * rw);
    const size_t pk_f = static_cast<size_t>(result_.peak_band2_ratio / max_ratio * rw);
    const size_t thr_f = static_cast<size_t>(CRITICAL_BAND_RATIO / max_ratio * rw);

    std::string avg_bar(rw, ' '), peak_bar(rw, ' ');
    for (int i = 0; i < rw; ++i) {
        if (static_cast<size_t>(i) == thr_f - 1 && thr_f > 0) {
            avg_bar[i] = '>'; peak_bar[i] = '>';
        } else if (static_cast<size_t>(i) < avg_f) avg_bar[i] = '=';
        if (static_cast<size_t>(i) < pk_f) peak_bar[i] = '#';
    }

    std::snprintf(line, sizeof(line),
        "  |  | Avg Ratio   %5.2f%% | %s | THRESHOLD %5.2f%%\n",
        result_.avg_band2_ratio * 100.0, avg_bar.c_str(), 100.0 * CRITICAL_BAND_RATIO);
    oss << line;
    for (size_t i = std::strlen(line); i < 84; ++i) oss << " ";
    oss << "|\n";

    std::snprintf(line, sizeof(line),
        "  |  | Peak Ratio  %5.2f%% | %s |\n",
        result_.peak_band2_ratio * 100.0, peak_bar.c_str());
    oss << line;
    for (size_t i = std::strlen(line); i < 84; ++i) oss << " ";
    oss << "|\n";

    oss << "  |  +------------------+-------------------------------------------------+         |\n";
    oss << "  |                                                                                  |\n";

    oss << "  |  #### MINERAL MASS / MICRO-GEOMETRY EROSION  ####                                |\n";
    oss << "  |  +----------------------------------------------------------------+             |\n";

    std::snprintf(line, sizeof(line),
        "  |  | Cumulative Metal Loss (micro-erosion)     : %9.4f micron depth    |\n",
        result_.cumulative_metal_loss_um);
    oss << line;

    std::snprintf(line, sizeof(line),
        "  |  | Cumulative Mass (Fe-Cr alloy wear)        : %9.4f mg            |\n",
        result_.cumulative_mass_loss_mg);
    oss << line;

    std::snprintf(line, sizeof(line),
        "  |  | Projected Erosion Rate                    : %9.4f mg/hour        |\n",
        result_.current_mass_loss_rate_mg_h);
    oss << line;

    std::snprintf(line, sizeof(line),
        "  |  | Total Crack-Event High-Stress Duration    : %9.3f sec            |\n",
        static_cast<double>(result_.high_stress_duration_ns) * 1e-9);
    oss << line;

    oss << "  |  +----------------------------------------------------------------+             |\n";
    oss << "  |                                                                                  |\n";

    oss << "  |  #### S-N ANTI-SHEAR FATIGUE GRID PROJECTION  ####                               |\n";
    std::snprintf(line, sizeof(line),
        "  |  Fatigue Events: %llu total,  %llu critical (>400 MPa)\n",
        static_cast<unsigned long long>(result_.total_event_count),
        static_cast<unsigned long long>(result_.critical_event_count));
    oss << line;
    for (size_t i = std::strlen(line); i < 84; ++i) oss << " ";
    oss << "|\n";

    std::snprintf(line, sizeof(line),
        "  |  Predicted remaining SCRAM cycles: %.0f permissible discharges\n",
        std::floor(result_.predicted_remaining_scrams + 0.5));
    oss << line;
    for (size_t i = std::strlen(line); i < 84; ++i) oss << " ";
    oss << "|\n";

    oss << "  |                                                                                  |\n";
    oss << "  +==================================================================================+\n";

    return oss.str();
}

std::string FatigueAnalyzer::render_miner_trend(size_t width, size_t height) const {
    std::ostringstream oss;
    const std::vector<double>& hist = result_.history_miner_D;
    if (hist.empty()) {
        oss << "\n  [No Miner-D history available for trend chart]\n";
        return oss.str();
    }

    const size_t N = hist.size();
    double max_D = MINER_FAILURE_THRESHOLD;
    for (size_t i = 0; i < N; ++i) {
        if (hist[i] > max_D) max_D = hist[i];
    }
    max_D *= 1.15;

    std::vector<std::string> canvas(height);
    for (size_t r = 0; r < height; ++r) canvas[r].assign(width, ' ');

    const size_t thr_row = height - 1 -
        static_cast<size_t>(MINER_FAILURE_THRESHOLD / max_D * static_cast<double>(height - 1));
    for (size_t x = 0; x < width; ++x) {
        if (thr_row < height) canvas[thr_row][x] = '_';
    }

    const double x_scale = static_cast<double>(width - 2) /
        static_cast<double>(std::max<size_t>(1, N - 1));
    for (size_t i = 0; i < N; ++i) {
        const size_t x = 1 + static_cast<size_t>(static_cast<double>(i) * x_scale);
        const double frac = hist[i] / max_D;
        const size_t y_base = height - 1 -
            static_cast<size_t>(frac * static_cast<double>(height - 1));
        const size_t y = y_base < height ? y_base : height - 1;
        if (x < width) {
            const bool overflow = hist[i] >= MINER_FAILURE_THRESHOLD;
            canvas[y][x] = overflow ? '*' : '@';
            if (overflow) {
                for (size_t yy = y + 1; yy < height; ++yy) {
                    if (canvas[yy][x] == ' ') canvas[yy][x] = 'x';
                }
            }
        }
    }

    oss << "\n";
    oss << "  +" << std::string(width, '=') << "+\n";

    char title[256];
    std::snprintf(title, sizeof(title),
        "  PALMGREN-MINER DAMAGE EVOLUTION  Sum(n/N) vs TIME BLOCKS (%zu points)",
        hist.size());
    std::string tstr = title;
    while (tstr.size() < width) tstr += " ";
    if (tstr.size() > width) tstr.resize(width);
    oss << "  | " << tstr << " |\n";
    oss << "  +" << std::string(width, '=') << "+\n";

    for (size_t r = 0; r < height; ++r) {
        const double row_val = max_D * (1.0 - static_cast<double>(r) / static_cast<double>(height - 1));
        char ylabel[32];
        if (r == thr_row) {
            std::snprintf(ylabel, sizeof(ylabel), "D=1.00__");
        } else {
            std::snprintf(ylabel, sizeof(ylabel), "%6.2f ", row_val);
        }
        oss << "  | " << ylabel << canvas[r] << " |\n";
    }
    oss << "  +" << std::string(width, '=') << "+\n";

    oss << "    block#1";
    size_t axis_w = 0;
    const size_t tick_n = 8;
    for (size_t t = 1; t < tick_n; ++t) {
        const size_t xstep = (width - tick_n) / tick_n;
        for (size_t k = 0; k < xstep; ++k) { oss << " "; axis_w++; }
        oss << "+";
    }
    char last_tick[64];
    std::snprintf(last_tick, sizeof(last_tick), " block#%zu\n", N);
    oss << last_tick;

    return oss.str();
}

} // namespace crdm
