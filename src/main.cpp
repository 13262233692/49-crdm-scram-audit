#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <thread>

#include "memory_mapped_file.h"
#include "stream_unpacker.h"
#include "wavelet_packet.h"
#include "energy_analyzer.h"
#include "raii_utils.h"

using namespace crdm;

static void print_usage(const char* prog) {
    std::fprintf(stderr,
        "CRDM Scram AE Audit Tool - High-throughput Acoustic Emission Analysis\n"
        "                        (RAII Resource-Safe Edition)\n"
        "\n"
        "Usage: %s [options] <input_binary_file> [input_file2 ...]\n"
        "       %s --batch <filelist.txt>\n"
        "\n"
        "Options:\n"
        "  -b <size>       Block size in samples per channel (default: 16384)\n"
        "  -o <dir>        Output directory for reports (default: stdout, per-file)\n"
        "  --batch <file>  Read newline-separated list of input files\n"
        "  --maxbad <N>    Abort file after N malformed frames (default: 10000)\n"
        "  -v              Verbose mode\n"
        "  -h              Show this help\n"
        "\n"
        "Input file format: CRDMAE01 binary format (64-byte header + frame stream)\n",
        prog, prog);
}

struct Options {
    std::vector<std::string> input_files;
    std::string output_dir;
    size_t block_size = 16384;
    uint64_t max_bad_frames = 10000;
    bool verbose = false;
};

static std::vector<std::string> read_file_list(const std::string& list_path) {
    std::vector<std::string> result;
    RAIIFile lf(list_path.c_str(), "r");
    if (!lf) return result;
    char line[4096];
    while (std::fgets(line, sizeof(line), lf)) {
        size_t n = std::strlen(line);
        while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r' || line[n-1] == ' ')) {
            line[--n] = '\0';
        }
        if (n > 0 && line[0] != '#') {
            result.emplace_back(line);
        }
    }
    return result;
}

static bool parse_args(int argc, char** argv, Options& opts) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            return false;
        } else if (std::strcmp(argv[i], "-v") == 0) {
            opts.verbose = true;
        } else if (std::strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            opts.block_size = static_cast<size_t>(std::atoll(argv[++i]));
            if (opts.block_size < 1024) opts.block_size = 1024;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            opts.output_dir = argv[++i];
        } else if (std::strcmp(argv[i], "--maxbad") == 0 && i + 1 < argc) {
            opts.max_bad_frames = static_cast<uint64_t>(std::atoll(argv[++i]));
        } else if (std::strcmp(argv[i], "--batch") == 0 && i + 1 < argc) {
            auto files = read_file_list(argv[++i]);
            opts.input_files.insert(opts.input_files.end(), files.begin(), files.end());
        } else if (argv[i][0] != '-') {
            opts.input_files.emplace_back(argv[i]);
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return false;
        }
    }
    return !opts.input_files.empty();
}

struct PerFileResult {
    std::string filename;
    bool success = false;
    std::string error_msg;
    uint64_t total_samples = 0;
    uint64_t processed_samples = 0;
    uint64_t bad_frames = 0;
    double elapsed_sec = 0.0;
    double global_ratios[8] = {0};
};

static std::string make_output_path(const std::string& out_dir, const std::string& in_file) {
    std::string base = in_file;
    auto slash = base.find_last_of("/\\");
    if (slash != std::string::npos) base = base.substr(slash + 1);
    auto dot = base.rfind('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    if (!out_dir.empty()) {
        return out_dir + "/" + base + "_report.txt";
    }
    return base + "_report.txt";
}

static PerFileResult process_single_file(const std::string& filename, const Options& opts) {
    PerFileResult result;
    result.filename = filename;

    auto t0 = std::chrono::high_resolution_clock::now();
    uint64_t processed = 0;

    RAIIFile out;
    if (!opts.output_dir.empty()) {
        std::string out_path = make_output_path(opts.output_dir, filename);
        out = RAIIFile(out_path.c_str(), "w");
        if (!out) {
            result.error_msg = "Cannot open output file: " + out_path;
            return result;
        }
    }
    FILE* out_fp = out ? out.get() : stdout;

    MemoryMappedFile mmf;
    if (!mmf.open(filename.c_str())) {
        result.error_msg = "Cannot open or memory-map file";
        return result;
    }
    auto mmf_guard = make_scope_guard([&mmf]() noexcept { mmf.close(); });

    StreamUnpacker unpacker;
    if (!unpacker.init(mmf)) {
        result.error_msg = "Invalid or corrupted CRDMAE01 binary header";
        return result;
    }

    const uint32_t num_ch = unpacker.num_channels();
    const uint64_t total_samples = unpacker.num_samples();
    const uint32_t sr = unpacker.sample_rate_khz();
    result.total_samples = total_samples;

    if (opts.verbose) {
        std::fprintf(stderr, "[INFO] %s: channels=%u, samples=%llu, rate=%u kHz, declared=%llu\n",
            filename.c_str(), num_ch,
            static_cast<unsigned long long>(total_samples), sr,
            static_cast<unsigned long long>(unpacker.declared_samples()));
    }

    WaveletPacketDecomposer wpd;
    wpd.reset(opts.block_size);

    EnergyBandAnalyzer analyzer;
    analyzer.init(num_ch);

    std::vector<float> channel_buf(opts.block_size * num_ch);
    std::vector<float> frame_values(num_ch);
    std::array<std::vector<float>, WaveletPacketDecomposer::NUM_SUBBANDS> subbands;

    size_t block_fill = 0;
    uint64_t block_start_ts = 0;
    uint64_t block_idx = 0;
    uint64_t frame_count = 0;
    uint64_t bad_seen = 0;
    bool aborted = false;

    auto it = unpacker.begin();
    auto end_it = unpacker.end();

    for (; it != end_it; ++it) {
        uint64_t ts = 0;
        bool read_ok = it.read(ts, frame_values.data());

        if (!read_ok) {
            ++bad_seen;
            if (bad_seen >= opts.max_bad_frames) {
                result.error_msg = "Malformed frame limit (" + std::to_string(opts.max_bad_frames) + ") exceeded, aborting file";
                aborted = true;
                break;
            }
            continue;
        }

        if (it.is_bad()) ++bad_seen;

        if (block_fill == 0) block_start_ts = ts;
        for (uint32_t ch = 0; ch < num_ch; ++ch) {
            channel_buf[block_fill * num_ch + ch] = frame_values[ch];
        }
        ++block_fill;
        ++frame_count;

        if (block_fill >= opts.block_size || frame_count >= total_samples) {
            for (uint32_t ch = 0; ch < num_ch; ++ch) {
                std::vector<float> ch_samples(block_fill);
                for (size_t s = 0; s < block_fill; ++s) {
                    ch_samples[s] = channel_buf[s * num_ch + ch];
                }
                wpd.decompose_channel(ch_samples.data(), block_fill, subbands);
                analyzer.process_channel_block(ch, subbands);
            }
            analyzer.finalize_frame(block_idx, block_start_ts);
            ++block_idx;
            processed += block_fill;
            block_fill = 0;

            if (opts.verbose && (block_idx & 0x3F) == 0) {
                std::fprintf(stderr, "\r[INFO] %s: %llu / %llu (%.1f%%) bad=%llu",
                    filename.c_str(),
                    static_cast<unsigned long long>(processed),
                    static_cast<unsigned long long>(total_samples),
                    total_samples > 0 ? 100.0 * processed / total_samples : 0.0,
                    static_cast<unsigned long long>(unpacker.stats().malformed_ts_frames +
                                                    unpacker.stats().malformed_bounds_frames +
                                                    unpacker.stats().malformed_value_frames));
                std::fflush(stderr);
            }
        }
    }

    if (opts.verbose) std::fprintf(stderr, "\n");

    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    result.elapsed_sec = elapsed;
    result.processed_samples = processed;
    result.bad_frames = unpacker.stats().malformed_ts_frames +
                        unpacker.stats().malformed_bounds_frames +
                        unpacker.stats().malformed_value_frames;

    analyzer.compute_global_ratios(result.global_ratios);

    std::fprintf(out_fp,
        "================================================================\n"
        "  CRDM Scram Acoustic Emission - Energy Band Audit Report\n"
        "  (RAII Resource-Safe Build)\n"
        "================================================================\n"
        "  File:              %s\n"
        "  Channels:          %u\n"
        "  Declared samples:  %llu\n"
        "  Safe samples:      %llu\n"
        "  Sample rate:       %u kHz\n"
        "  Blocks analyzed:   %llu\n"
        "  Processing time:   %.3f seconds\n"
        "  Throughput:        %.2f MSamples/sec\n"
        "----------------------------------------------------------------\n"
        "  Frame Integrity:\n"
        "    Total frames scanned:  %llu\n"
        "    Good frames:           %llu\n"
        "    Bad timestamp frames:  %llu\n"
        "    Bad bounds frames:     %llu\n"
        "    Bad value frames:      %llu\n"
        "    Aborted:               %s\n",
        filename.c_str(),
        num_ch,
        static_cast<unsigned long long>(unpacker.declared_samples()),
        static_cast<unsigned long long>(total_samples),
        sr,
        static_cast<unsigned long long>(block_idx),
        elapsed,
        (processed / 1e6) / std::max(elapsed, 1e-9),
        static_cast<unsigned long long>(unpacker.stats().total_frames),
        static_cast<unsigned long long>(unpacker.stats().good_frames),
        static_cast<unsigned long long>(unpacker.stats().malformed_ts_frames),
        static_cast<unsigned long long>(unpacker.stats().malformed_bounds_frames),
        static_cast<unsigned long long>(unpacker.stats().malformed_value_frames),
        aborted ? "YES (bad-frame limit)" : "NO");

    std::fprintf(out_fp,
        "\n  GLOBAL ENERGY BAND RATIO MATRIX (3-level WPD, db4)\n"
        "  -------------------------------------------------\n\n");

    const char* band_labels[8] = {
        "Band 0 (0-62.5 kHz)   ",
        "Band 1 (62.5-125 kHz) ",
        "Band 2 (125-187.5 kHz)",
        "Band 3 (187.5-250 kHz)",
        "Band 4 (250-312.5 kHz)",
        "Band 5 (312.5-375 kHz)",
        "Band 6 (375-437.5 kHz)",
        "Band 7 (437.5-500 kHz)"
    };

    std::fprintf(out_fp, "  Sub-band              Energy            Ratio      Histogram\n");
    std::fprintf(out_fp, "  --------              ------            -----      ---------\n");
    for (int i = 0; i < 8; ++i) {
        const int bar_len = static_cast<int>(result.global_ratios[i] * 60.0);
        std::string bar(bar_len, '#');
        std::fprintf(out_fp, "  %s  %14.6e   %8.5f%%   |%s\n",
            band_labels[i],
            analyzer.global_band_energy()[i],
            result.global_ratios[i] * 100.0,
            bar.c_str());
    }
    std::fprintf(out_fp, "  ------------------------------------------------------------\n");
    std::fprintf(out_fp, "  TOTAL                 %14.6e  100.00000%%\n",
        analyzer.global_total_energy());

    std::fprintf(out_fp,
        "================================================================\n"
        "  Audit summary: %s\n"
        "================================================================\n\n",
        unpacker.has_errors() ? "Frames with anomalies detected (see counts above)"
                              : "Clean scan - no frame anomalies detected");

    result.success = true;
    return result;
}

int main(int argc, char** argv) {
    Options opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    std::vector<PerFileResult> all_results;
    all_results.reserve(opts.input_files.size());

    size_t ok_count = 0;
    size_t fail_count = 0;
    double total_elapsed = 0.0;
    uint64_t total_processed = 0;

    auto global_t0 = std::chrono::high_resolution_clock::now();

    for (size_t fidx = 0; fidx < opts.input_files.size(); ++fidx) {
        const auto& fn = opts.input_files[fidx];
        if (opts.verbose) {
            std::fprintf(stderr, "\n===== [%zu/%zu] Processing: %s =====\n",
                fidx + 1, opts.input_files.size(), fn.c_str());
        }

        PerFileResult r = process_single_file(fn, opts);
        total_elapsed += r.elapsed_sec;
        total_processed += r.processed_samples;

        if (r.success) {
            ++ok_count;
            if (opts.verbose) {
                std::fprintf(stderr, "[OK]   %s: %.2f MS/s, %llu bad frames\n",
                    fn.c_str(),
                    (r.processed_samples / 1e6) / std::max(r.elapsed_sec, 1e-9),
                    static_cast<unsigned long long>(r.bad_frames));
            }
        } else {
            ++fail_count;
            std::fprintf(stderr, "[FAIL] %s: %s\n", fn.c_str(), r.error_msg.c_str());
        }

        all_results.push_back(std::move(r));

        std::this_thread::yield();
    }

    auto global_t1 = std::chrono::high_resolution_clock::now();
    double global_elapsed = std::chrono::duration<double>(global_t1 - global_t0).count();

    std::fprintf(stderr,
        "\n================================================================\n"
        "  BATCH PROCESSING SUMMARY\n"
        "================================================================\n"
        "  Total files submitted:   %zu\n"
        "  Successfully processed:  %zu\n"
        "  Failed / skipped:        %zu\n"
        "  Total samples handled:   %llu\n"
        "  Cumulative CPU time:     %.3f sec\n"
        "  Wall-clock time:         %.3f sec\n"
        "  Aggregate throughput:    %.2f MSamples/sec\n"
        "================================================================\n",
        opts.input_files.size(),
        ok_count,
        fail_count,
        static_cast<unsigned long long>(total_processed),
        total_elapsed,
        global_elapsed,
        (total_processed / 1e6) / std::max(global_elapsed, 1e-9));

    return fail_count > 0 ? 4 : 0;
}
