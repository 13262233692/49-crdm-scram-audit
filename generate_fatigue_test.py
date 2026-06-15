import struct
import math

filename = "test_fatigue_damage.bin"
num_channels = 8
num_samples = 131072
sample_rate_khz = 500

magic = b"CRDMAE01"
version = 1
ts_res = 1
reserved = b"\x00" * 28
header = struct.pack("<8sIIIQQ28s",
    magic, version, num_channels, sample_rate_khz,
    num_samples, ts_res, reserved)

with open(filename, "wb") as f:
    f.write(header)

    dt_ns = int(1e9 / (sample_rate_khz * 1000))
    base_ts = 1718409600000000000

    import random
    random.seed(7)

    for s in range(num_samples):
        ts = base_ts + s * dt_ns
        f.write(struct.pack("<Q", ts))

        t = s / (sample_rate_khz * 1000.0)
        frame_frac = s / num_samples

        vals = []
        for ch in range(num_channels):
            v = random.gauss(0, 0.04)
            v += 0.12 * math.sin(2 * math.pi * (30000 + ch * 12000) * t)

            if 0.20 < frame_frac < 0.24:
                burst_freq = 120000 + ch * 7500
                env = math.exp(-50 * ((frame_frac - 0.22) ** 2))
                v += 1.3 * math.sin(2 * math.pi * burst_freq * t) * env

            if 0.50 < frame_frac < 0.58:
                burst_freq = 150000 + ch * 5000
                env = math.exp(-10 * ((frame_frac - 0.54) ** 2))
                v += 1.8 * math.sin(2 * math.pi * burst_freq * t) * env

            if 0.78 < frame_frac < 0.81:
                burst_freq = 175000 + ch * 3000
                env = math.exp(-90 * ((frame_frac - 0.795) ** 2))
                v += 0.9 * math.sin(2 * math.pi * burst_freq * t) * env

            if 0.92 < frame_frac < 0.99:
                burst_freq = 160000
                env = frame_frac > 0.965
                v += 2.3 * math.sin(2 * math.pi * burst_freq * t) * (0.6 + 0.4 * env)

            v = max(-1.0, min(1.0, v))
            raw = int(v * 32767.0)
            f.write(struct.pack("<h", raw))

print(f"Generated fatigue test file: {filename}")
print(f"  Channels: {num_channels}, Samples: {num_samples}")
print(f"  Damage bursts: 4 zones (22%, 54%, 79.5%, 95% time)")
print(f"  Expected Band2 (125-187.5 kHz) ratio: HIGH during bursts")
