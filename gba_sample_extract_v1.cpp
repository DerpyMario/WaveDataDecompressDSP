// gba_sample_extract.cpp
//
// Extracts the raw "Direct Sound" PCM instrument/voice samples embedded in
// a Game Boy Advance ROM: the digitized waveforms used by the GBA's
// built-in "Sappy" / MusicPlayer2000 (m4a) sound engine, and by the many
// engines derived from or compatible with it. Output is uncompressed PCM,
// resampled up from the GBA's native rates (5734-42048 Hz) to a standard
// 44.1kHz/16-bit .wav by default -- pass --native for the exact original
// 8-bit bytes at the ROM's native rate instead.
//
// This tool deliberately does NOT try to decode or sequence the "song"
// data (the tracker-style note events people usually mean when they say
// "sappy music"). It only looks for the self-contained 16-byte sample
// headers documented for the engine's raw sample sub-format and dumps the
// plain audio that follows each one to a .wav file. That's the distinction
// behind "non-sappy audio ... normal samples": the raw waveform data
// itself, not the sequenced/compressed music that references it.
//
// Sample header layout (all fields little-endian), 16 bytes total,
// immediately followed by the sample data:
//
//   offset 0x0  u8[3]   must be 0x00 0x00 0x00
//   offset 0x3  u8      0x00 = one-shot, 0x40 = looped
//   offset 0x4  u32     pitch = sample_rate_hz * 1024
//   offset 0x8  u32     loop_start_raw   (true loop start = value + 1)
//   offset 0xC  u32     length_raw       (true length     = value + 1)
//   offset 0x10 s8[length]   signed 8-bit PCM sample data
//
// There is no directory telling us where these headers live, so this is
// a heuristic scan: every candidate is checked against several sanity
// rules (see isPlausibleSample) before being accepted. All the thresholds
// are exposed as CLI flags so you can loosen/tighten them per ROM.
//
// Build:  g++ -O2 -std=c++17 -o gba_sample_extract gba_sample_extract.cpp
// Usage:  ./gba_sample_extract rom.gba -o out_dir [options]

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>

namespace stdfs = std::filesystem;

// ---------------------------------------------------------------------
// little-endian helpers (explicit, so this is correct regardless of the
// host machine's native endianness)
// ---------------------------------------------------------------------

static uint32_t readU32LE(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void writeU32LE(std::ostream& out, uint32_t v) {
    uint8_t b[4] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
                     (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF) };
    out.write(reinterpret_cast<char*>(b), 4);
}

static void writeU16LE(std::ostream& out, uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
    out.write(reinterpret_cast<char*>(b), 2);
}

// ---------------------------------------------------------------------
// options
// ---------------------------------------------------------------------

struct Options {
    std::string romPath;
    std::string outDir  = "extracted_samples";
    uint32_t minHz       = 4000;
    uint32_t maxHz       = 50000;
    uint32_t minLen      = 4;
    uint32_t maxLen      = 2'000'000;
    uint32_t step        = 4;
    bool biasConvert     = true;   // source bytes are signed 8-bit (standard GBA convention)
    bool verbose         = false;
    bool strictRate      = true;   // require one of the engine's fixed mixing rates (see below)
    bool nativeMode      = false;  // skip resampling; emit raw 8-bit PCM at the ROM's native rate
    uint32_t targetRate  = 44100;  // output sample rate when not in native mode
    uint32_t outBits     = 16;     // output bit depth when not in native mode (8 or 16)
};

// The engine's DirectSound mixer only ever runs at one of these 12 fixed
// rates (this is a hardware/engine constraint, not a per-sample choice),
// so a genuine sample header's pitch field must equal sample_rate*1024 for
// one of these exactly. Requiring an exact match is a much stronger filter
// than a plausible-range check and is the default; pass --any-rate to fall
// back to a loose range+alignment check instead (useful for custom/variant
// engines that might not follow this table).
struct RateEntry { uint32_t pitch; uint32_t hz; };
static const RateEntry kFixedRates[] = {
    {0x00599800, 5734},  {0x007b3000, 7884},  {0x00a44000, 10512}, {0x00d10c00, 13379},
    {0x00f66000, 15768}, {0x011bb400, 18157}, {0x01488000, 21024}, {0x01a21800, 26758},
    {0x01ecc000, 31536}, {0x02376800, 36314}, {0x02732400, 40137}, {0x02910000, 42048},
};

static void printUsage(const char* argv0) {
    std::cerr <<
        "Usage: " << argv0 << " <rom.gba> [options]\n"
        "\n"
        "By default, output is resampled to 44.1kHz / 16-bit uncompressed PCM\n"
        "(the GBA's native rates -- 5734 to 42048 Hz -- are far below what most\n"
        "audio software/hardware expects). Pass --native to instead dump the\n"
        "exact original 8-bit bytes at the ROM's native rate, unmodified.\n"
        "\n"
        "Options:\n"
        "  -o, --outdir <dir>   Output directory (default: extracted_samples)\n"
        "      --rate <hz>      Output sample rate, resampled from native   (default 44100)\n"
        "      --bits <8|16>    Output bit depth                            (default 16)\n"
        "      --native         Skip resampling: raw 8-bit PCM at native rate, unmodified\n"
        "      --min-hz <n>     Minimum plausible sample rate   (default 4000)\n"
        "      --max-hz <n>     Maximum plausible sample rate   (default 50000)\n"
        "      --min-len <n>    Minimum sample length in bytes  (default 4)\n"
        "      --max-len <n>    Maximum sample length in bytes  (default 4000000)\n"
        "      --step <n>       Scan alignment step in bytes    (default 4)\n"
        "      --no-bias        Source is already unsigned 8-bit (skip sign flip on read)\n"
        "      --any-rate       Accept any plausible rate (--min-hz/--max-hz), not just\n"
        "                       the engine's 12 fixed mixing rates (default: exact match only)\n"
        "  -v, --verbose        Print every candidate as it's found\n"
        "  -h, --help           Show this help\n";
}

static bool parseArgs(int argc, char** argv, Options& opt) {
    std::vector<std::string> positional;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << name << "\n"; std::exit(1); }
            return argv[++i];
        };
        if (a == "-h" || a == "--help")       { printUsage(argv[0]); std::exit(0); }
        else if (a == "-o" || a == "--outdir") opt.outDir = need("--outdir");
        else if (a == "--rate")                opt.targetRate = std::stoul(need("--rate"));
        else if (a == "--bits")                opt.outBits = std::stoul(need("--bits"));
        else if (a == "--native")              opt.nativeMode = true;
        else if (a == "--min-hz")              opt.minHz  = std::stoul(need("--min-hz"));
        else if (a == "--max-hz")              opt.maxHz  = std::stoul(need("--max-hz"));
        else if (a == "--min-len")             opt.minLen = std::stoul(need("--min-len"));
        else if (a == "--max-len")             opt.maxLen = std::stoul(need("--max-len"));
        else if (a == "--step")                opt.step   = std::stoul(need("--step"));
        else if (a == "--no-bias")             opt.biasConvert = false;
        else if (a == "--any-rate")             opt.strictRate = false;
        else if (a == "-v" || a == "--verbose") opt.verbose = true;
        else if (!a.empty() && a[0] == '-')    { std::cerr << "Unknown option: " << a << "\n"; return false; }
        else positional.push_back(a);
    }
    if (positional.empty()) return false;
    opt.romPath = positional[0];
    return true;
}

// ---------------------------------------------------------------------
// ROM loading + GBA cartridge header (just for a friendly identification
// printout -- not required for the sample scan itself)
// ---------------------------------------------------------------------

static std::vector<uint8_t> readWholeFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open file: " + path);
    f.seekg(0, std::ios::end);
    std::streamoff size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> data(static_cast<size_t>(size));
    if (size > 0) f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

struct GbaHeaderInfo {
    std::string title, gameCode, makerCode;
    bool checksumOk = false;
    bool present = false;
};

static GbaHeaderInfo readGbaHeader(const std::vector<uint8_t>& rom) {
    GbaHeaderInfo h;
    if (rom.size() < 0xC0) return h;
    h.present = true;
    h.title = std::string(reinterpret_cast<const char*>(&rom[0xA0]), 12);
    while (!h.title.empty() && h.title.back() == '\0') h.title.pop_back();
    h.gameCode  = std::string(reinterpret_cast<const char*>(&rom[0xAC]), 4);
    h.makerCode = std::string(reinterpret_cast<const char*>(&rom[0xB0]), 2);
    int32_t sum = 0;
    for (size_t i = 0xA0; i <= 0xBC; i++) sum -= rom[i];
    uint8_t calc = (uint8_t)(sum - 0x19);
    h.checksumOk = (calc == rom[0xBD]);
    return h;
}

// ---------------------------------------------------------------------
// sample header scan
// ---------------------------------------------------------------------

struct FoundSample {
    size_t   offset;
    uint32_t sampleRate;
    uint32_t length;     // true length, in bytes/samples
    uint32_t loopStart;  // true loop start (only meaningful if looped)
    bool     looped;
};

static bool isPlausibleSample(const std::vector<uint8_t>& rom, size_t off,
                               const Options& opt, FoundSample& out) {
    if (off + 16 > rom.size()) return false;
    const uint8_t* p = &rom[off];

    uint32_t flags = readU32LE(p);
    if (flags != 0x00000000u && flags != 0x40000000u) return false;
    bool looped = (flags == 0x40000000u);

    uint32_t rawPitch = readU32LE(p + 4);
    if (rawPitch == 0) return false;

    uint32_t hz = 0;
    if (opt.strictRate) {
        bool matched = false;
        for (const auto& r : kFixedRates) {
            if (r.pitch == rawPitch) { hz = r.hz; matched = true; break; }
        }
        if (!matched) return false;
    } else {
        if (rawPitch % 1024 != 0) return false;   // real pitches are exact multiples of 1024
        hz = rawPitch / 1024;
        if (hz < opt.minHz || hz > opt.maxHz) return false;
    }

    uint32_t rawLoopStart = readU32LE(p + 8);
    uint32_t rawLength    = readU32LE(p + 12);

    uint64_t length = (uint64_t)rawLength + 1;      // header stores length-1
    if (length < opt.minLen || length > opt.maxLen) return false;

    uint64_t loopStart = (uint64_t)rawLoopStart + 1; // header stores loopStart-1
    if (looped && loopStart > length) return false;

    if (off + 16 + length > rom.size()) return false;

    out.offset     = off;
    out.sampleRate = hz;
    out.length     = (uint32_t)length;
    out.loopStart  = looped ? (uint32_t)loopStart : 0;
    out.looped     = looped;
    return true;
}

// ---------------------------------------------------------------------
// WAV writer (mono, 8-bit PCM; adds a 'smpl' loop chunk for looped samples)
// ---------------------------------------------------------------------

static void writeWav(const std::string& path, const uint8_t* pcm, uint32_t length,
                      uint32_t sampleRate, bool biasConvert, bool looped, uint32_t loopStart) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot create " + path);

    const uint32_t smplDataSize = 36 + 24;               // fixed fields + one loop struct
    const uint32_t fmtChunk     = 8 + 16;
    const uint32_t smplChunk    = looped ? (8 + smplDataSize) : 0;
    const uint32_t dataChunk    = 8 + length;
    const uint32_t riffSize     = 4 + fmtChunk + smplChunk + dataChunk;

    out.write("RIFF", 4); writeU32LE(out, riffSize); out.write("WAVE", 4);

    out.write("fmt ", 4); writeU32LE(out, 16);
    writeU16LE(out, 1);            // PCM
    writeU16LE(out, 1);            // mono
    writeU32LE(out, sampleRate);
    writeU32LE(out, sampleRate);   // byte rate (mono, 8-bit -> == sample rate)
    writeU16LE(out, 1);            // block align
    writeU16LE(out, 8);            // bits per sample

    if (looped) {
        out.write("smpl", 4); writeU32LE(out, smplDataSize);
        writeU32LE(out, 0);                                  // manufacturer
        writeU32LE(out, 0);                                  // product
        writeU32LE(out, (uint32_t)(1000000000.0 / sampleRate)); // sample period (ns)
        writeU32LE(out, 60);                                 // MIDI unity note
        writeU32LE(out, 0);                                  // MIDI pitch fraction
        writeU32LE(out, 0);                                  // SMPTE format
        writeU32LE(out, 0);                                  // SMPTE offset
        writeU32LE(out, 1);                                  // num sample loops
        writeU32LE(out, 0);                                  // sampler data size
        writeU32LE(out, 0);                                  // cue point ID
        writeU32LE(out, 0);                                  // loop type (0 = forward)
        writeU32LE(out, loopStart);                          // loop start
        writeU32LE(out, length > 0 ? length - 1 : 0);         // loop end
        writeU32LE(out, 0);                                  // fraction
        writeU32LE(out, 0);                                  // play count (0 = infinite)
    }

    out.write("data", 4); writeU32LE(out, length);
    if (biasConvert) {
        std::vector<uint8_t> buf(length);
        for (uint32_t i = 0; i < length; i++) buf[i] = (uint8_t)((int8_t)pcm[i] + 128);
        out.write(reinterpret_cast<const char*>(buf.data()), length);
    } else {
        out.write(reinterpret_cast<const char*>(pcm), length);
    }
}

// ---------------------------------------------------------------------
// resampling to a standard, higher output rate (used unless --native)
// ---------------------------------------------------------------------

// Interprets one raw source byte as a centered (silence = 0) 16-bit value.
static int16_t sourceByteToCentered16(uint8_t raw, bool biasConvert) {
    if (biasConvert) return (int16_t)((int8_t)raw) * 256;   // source is signed 8-bit (standard GBA convention)
    return (int16_t)(((int)raw) - 128) * 256;               // source is already unsigned 8-bit
}

// Linear-interpolation resampler. The source material is already
// low-bandwidth 8-bit game audio (native rates top out at 42048 Hz) being
// resampled *up*, so linear interpolation is sufficient here without
// pulling in a full windowed-sinc filter.
static std::vector<int16_t> resampleLinear(const std::vector<int16_t>& src, double srcRate, double dstRate) {
    if (src.empty() || srcRate <= 0 || dstRate <= 0 || srcRate == dstRate) return src;
    size_t N = src.size();
    size_t M = (size_t)std::llround((double)N * dstRate / srcRate);
    if (M == 0) M = 1;
    std::vector<int16_t> out(M);
    double step = srcRate / dstRate;
    for (size_t j = 0; j < M; j++) {
        double pos = (double)j * step;
        size_t i0 = (size_t)pos;
        double frac = pos - (double)i0;
        int16_t s0 = src[std::min(i0, N - 1)];
        int16_t s1 = src[std::min(i0 + 1, N - 1)];
        out[j] = (int16_t)std::lround(s0 + frac * (double)(s1 - s0));
    }
    return out;
}

// Writes mono, uncompressed PCM at `outBits` (8 or 16), after resampling
// from the sample's native rate up to `targetRate`. Loop points (given in
// native-rate sample units) are rescaled to match.
static uint32_t writeWavResampled(const std::string& path, const std::vector<int16_t>& centeredNative,
                                uint32_t nativeRate, uint32_t targetRate, uint32_t outBits,
                                bool looped, uint32_t loopStartNative) {
    std::vector<int16_t> centered = resampleLinear(centeredNative, nativeRate, targetRate);
    uint32_t n = (uint32_t)centered.size();

    uint32_t loopStart = 0;
    if (looped && nativeRate > 0) {
        loopStart = (uint32_t)std::llround((double)loopStartNative * targetRate / nativeRate);
        if (n > 0 && loopStart >= n) loopStart = n - 1;
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot create " + path);

    uint16_t bytesPerSample = (outBits == 8) ? 1 : 2;
    uint32_t byteRate    = targetRate * bytesPerSample;
    uint16_t blockAlign  = bytesPerSample;
    uint32_t dataSize    = n * bytesPerSample;

    const uint32_t smplDataSize = 36 + 24;
    const uint32_t fmtChunk     = 8 + 16;
    const uint32_t smplChunk    = looped ? (8 + smplDataSize) : 0;
    const uint32_t dataChunk    = 8 + dataSize;
    const uint32_t riffSize     = 4 + fmtChunk + smplChunk + dataChunk;

    out.write("RIFF", 4); writeU32LE(out, riffSize); out.write("WAVE", 4);
    out.write("fmt ", 4); writeU32LE(out, 16);
    writeU16LE(out, 1);            // PCM
    writeU16LE(out, 1);            // mono
    writeU32LE(out, targetRate);
    writeU32LE(out, byteRate);
    writeU16LE(out, blockAlign);
    writeU16LE(out, (uint16_t)outBits);

    if (looped) {
        out.write("smpl", 4); writeU32LE(out, smplDataSize);
        writeU32LE(out, 0);                                       // manufacturer
        writeU32LE(out, 0);                                       // product
        writeU32LE(out, (uint32_t)(1000000000.0 / targetRate));   // sample period (ns)
        writeU32LE(out, 60);                                      // MIDI unity note
        writeU32LE(out, 0);                                       // MIDI pitch fraction
        writeU32LE(out, 0);                                       // SMPTE format
        writeU32LE(out, 0);                                       // SMPTE offset
        writeU32LE(out, 1);                                       // num sample loops
        writeU32LE(out, 0);                                       // sampler data size
        writeU32LE(out, 0);                                       // cue point ID
        writeU32LE(out, 0);                                       // loop type (0 = forward)
        writeU32LE(out, loopStart);                               // loop start
        writeU32LE(out, n > 0 ? n - 1 : 0);                       // loop end
        writeU32LE(out, 0);                                       // fraction
        writeU32LE(out, 0);                                       // play count (0 = infinite)
    }

    out.write("data", 4); writeU32LE(out, dataSize);
    std::vector<uint8_t> buf(dataSize);
    if (outBits == 16) {
        for (uint32_t i = 0; i < n; i++) {
            int16_t v = centered[i];
            buf[i * 2]     = (uint8_t)(v & 0xFF);
            buf[i * 2 + 1] = (uint8_t)((v >> 8) & 0xFF);
        }
    } else {
        for (uint32_t i = 0; i < n; i++) {
            int v = centered[i] / 256 + 128;
            v = std::max(0, std::min(255, v));
            buf[i] = (uint8_t)v;
        }
    }
    out.write(reinterpret_cast<const char*>(buf.data()), dataSize);
    return n;
}

// ---------------------------------------------------------------------
// main
// ---------------------------------------------------------------------

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) { printUsage(argv[0]); return 1; }
    if (opt.step == 0) opt.step = 4;
    if (opt.outBits != 8 && opt.outBits != 16) {
        std::cerr << "--bits must be 8 or 16\n";
        return 1;
    }

    try {
        std::vector<uint8_t> rom = readWholeFile(opt.romPath);
        std::cout << "Loaded ROM: " << opt.romPath << " (" << rom.size() << " bytes)\n";

        GbaHeaderInfo hdr = readGbaHeader(rom);
        if (hdr.present) {
            std::cout << "  Internal title : " << hdr.title << "\n"
                      << "  Game code      : " << hdr.gameCode << "\n"
                      << "  Maker code     : " << hdr.makerCode << "\n"
                      << "  Header checksum: " << (hdr.checksumOk ? "OK" : "MISMATCH") << "\n\n";
        }

        stdfs::create_directories(opt.outDir);

        std::vector<FoundSample> found;
        size_t i = 0;
        while (i + 16 <= rom.size()) {
            FoundSample cand;
            if (isPlausibleSample(rom, i, opt, cand)) {
                found.push_back(cand);
                if (opt.verbose) {
                    std::cout << "  [0x" << std::hex << cand.offset << std::dec << "] "
                              << cand.sampleRate << " Hz, " << cand.length << " bytes"
                              << (cand.looped ? (", looped @" + std::to_string(cand.loopStart)) : ", one-shot")
                              << "\n";
                }
                // Jump past this sample's data so we don't rescan inside real
                // PCM data (which could coincidentally look like a header)
                // and so the scan stays fast.
                size_t next = cand.offset + 16 + cand.length;
                size_t rem = next % opt.step;
                i = rem ? next + (opt.step - rem) : next;
            } else {
                i += opt.step;
            }
        }

        std::cout << "\nFound " << found.size() << " candidate sample(s).\n";
        if (opt.nativeMode) {
            std::cout << "Mode: native (raw 8-bit PCM at each sample's original rate)\n";
        } else {
            std::cout << "Mode: uncompressed PCM" << opt.outBits << ", resampled to " << opt.targetRate << " Hz\n";
        }

        std::ofstream manifest(opt.outDir + "/manifest.csv");
        manifest << "index,rom_offset_hex,native_rate_hz,output_rate_hz,output_bits,length_bytes,looped,loop_start,filename\n";

        uint64_t totalNativeBytes = 0;
        uint64_t totalOutputBytes = 0;
        for (size_t idx = 0; idx < found.size(); idx++) {
            const auto& s = found[idx];
            uint32_t outRate = opt.nativeMode ? s.sampleRate : opt.targetRate;
            uint32_t outBits = opt.nativeMode ? 8 : opt.outBits;

            std::ostringstream name;
            name << "sample_" << std::setw(4) << std::setfill('0') << idx << std::setfill(' ')
                 << "_0x" << std::hex << s.offset << std::dec
                 << "_" << outRate << "Hz" << (s.looped ? "_loop" : "") << ".wav";
            std::string filename = name.str();
            std::string fullPath = opt.outDir + "/" + filename;

            if (opt.nativeMode) {
                writeWav(fullPath, &rom[s.offset + 16], s.length, s.sampleRate,
                         opt.biasConvert, s.looped, s.loopStart);
                totalOutputBytes += s.length;
            } else {
                std::vector<int16_t> centered(s.length);
                for (uint32_t k = 0; k < s.length; k++)
                    centered[k] = sourceByteToCentered16(rom[s.offset + 16 + k], opt.biasConvert);
                uint32_t outSamples = writeWavResampled(fullPath, centered, s.sampleRate, opt.targetRate,
                                                          opt.outBits, s.looped, s.loopStart);
                totalOutputBytes += (uint64_t)outSamples * (opt.outBits == 8 ? 1 : 2);
            }

            manifest << idx << ",0x" << std::hex << s.offset << std::dec << ","
                     << s.sampleRate << "," << outRate << "," << outBits << ","
                     << s.length << "," << (s.looped ? 1 : 0) << ","
                     << s.loopStart << "," << filename << "\n";
            totalNativeBytes += s.length;
        }
        manifest.close();

        std::cout << "Extracted " << found.size() << " sample(s): " << totalNativeBytes
                  << " native bytes -> " << totalOutputBytes << " output bytes (~"
                  << (totalOutputBytes / 1024) << " KiB) -> \"" << opt.outDir << "\"\n"
                  << "Manifest: " << opt.outDir << "/manifest.csv\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
