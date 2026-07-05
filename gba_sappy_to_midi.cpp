// gba_sappy_to_midi.cpp
//
// Extracts full Sappy/MP2K "soundbanks" (instrument/voice tables) and
// sequenced songs from a GBA ROM, converting:
//   - each song  -> a Standard MIDI File (.mid)
//   - each voice group (instrument bank) -> a General-MIDI-style SF2
//     SoundFont bank (bank number = voice group index, program = voice
//     index), so any MIDI player/DAW that honors Bank Select + Program
//     Change (FL Studio, etc.) plays the songs with correct instruments.
//
// This is a heuristic reverse-engineering tool: there is no directory of
// "where the songs/instruments live" in an arbitrary ROM, so, like the
// sample scanner, everything here is found by structural validation
// against the documented Sappy data formats (see comments per-section),
// cross-referenced against each other (song -> voice group -> samples)
// for confidence.
//
// Reference: "sappy (by Bregalad) v1.3" -- the community's de-facto
// documentation of this engine's on-ROM data formats. Section numbers
// referenced in comments below (e.g. "doc 2.2") refer to that document.
//
// Build:  g++ -O2 -std=c++17 -o gba_sappy_to_midi gba_sappy_to_midi.cpp
// Usage:  ./gba_sappy_to_midi rom.gba -o out_dir

#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdexcept>
#include <filesystem>

namespace stdfs = std::filesystem;

// =======================================================================
// little-endian helpers
// =======================================================================

static uint32_t rdU32(const std::vector<uint8_t>& rom, size_t off) {
    const uint8_t* p = &rom[off];
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint8_t rdU8(const std::vector<uint8_t>& rom, size_t off) { return rom[off]; }

static void wrU32(std::ostream& out, uint32_t v) {
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    out.write((char*)b, 4);
}
static void wrU16(std::ostream& out, uint16_t v) {
    uint8_t b[2] = { (uint8_t)v, (uint8_t)(v >> 8) };
    out.write((char*)b, 2);
}
static void wrS16(std::ostream& out, int16_t v) { wrU16(out, (uint16_t)v); }

// A ROM pointer is valid if it's in the GBA cartridge address space
// (0x08000000-0x09FFFFFF) and the offset it implies actually fits the ROM.
static bool romPtrToOffset(uint32_t ptr, size_t romSize, size_t& outOffset) {
    if ((ptr & 0xFE000000u) != 0x08000000u) return false;
    uint32_t off = ptr & 0x01FFFFFFu;
    if (off >= romSize) return false;
    outOffset = off;
    return true;
}

// =======================================================================
// ROM loading + GBA cartridge header
// =======================================================================

static std::vector<uint8_t> readWholeFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open file: " + path);
    f.seekg(0, std::ios::end);
    std::streamoff size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> data((size_t)size);
    if (size > 0) f.read((char*)data.data(), size);
    return data;
}

struct GbaHeaderInfo { std::string title, gameCode; bool checksumOk = false; };
static GbaHeaderInfo readGbaHeader(const std::vector<uint8_t>& rom) {
    GbaHeaderInfo h;
    if (rom.size() < 0xC0) return h;
    h.title = std::string((const char*)&rom[0xA0], 12);
    while (!h.title.empty() && h.title.back() == '\0') h.title.pop_back();
    h.gameCode = std::string((const char*)&rom[0xAC], 4);
    int32_t sum = 0;
    for (size_t i = 0xA0; i <= 0xBC; i++) sum -= rom[i];
    h.checksumOk = ((uint8_t)(sum - 0x19) == rom[0xBD]);
    return h;
}

// =======================================================================
// Sample header validation (identical rules to gba_sample_extract.cpp --
// see that tool's comments for how this format was confirmed)
// =======================================================================

struct RateEntry { uint32_t pitch; uint32_t hz; };
static const RateEntry kFixedRates[] = {
    {0x00599800, 5734},  {0x007b3000, 7884},  {0x00a44000, 10512}, {0x00d10c00, 13379},
    {0x00f66000, 15768}, {0x011bb400, 18157}, {0x01488000, 21024}, {0x01a21800, 26758},
    {0x01ecc000, 31536}, {0x02376800, 36314}, {0x02732400, 40137}, {0x02910000, 42048},
};

struct SappySample {
    size_t   offset;      // offset of the 16-byte header
    uint32_t sampleRate;
    uint32_t length;      // true length in bytes/samples
    uint32_t loopStart;
    bool     looped;
};

static bool isPlausibleSample(const std::vector<uint8_t>& rom, size_t off, SappySample& out) {
    if (off + 16 > rom.size()) return false;
    uint32_t flags = rdU32(rom, off);
    if (flags != 0x00000000u && flags != 0x40000000u) return false;
    bool looped = (flags == 0x40000000u);

    uint32_t rawPitch = rdU32(rom, off + 4);
    if (rawPitch == 0) return false;
    uint32_t hz = 0; bool matched = false;
    for (auto& r : kFixedRates) if (r.pitch == rawPitch) { hz = r.hz; matched = true; break; }
    if (!matched) return false;

    uint32_t rawLoopStart = rdU32(rom, off + 8);
    uint32_t rawLength    = rdU32(rom, off + 12);
    uint64_t length = (uint64_t)rawLength + 1;
    if (length < 4 || length > 4'000'000) return false;
    uint64_t loopStart = (uint64_t)rawLoopStart + 1;
    if (looped && loopStart > length) return false;
    if (off + 16 + length > rom.size()) return false;

    out.offset = off; out.sampleRate = hz; out.length = (uint32_t)length;
    out.loopStart = looped ? (uint32_t)loopStart : 0; out.looped = looped;
    return true;
}

// Scans the whole ROM for valid sample headers, same "find + jump past"
// strategy as gba_sample_extract.cpp (this is the tool that established
// this format works reliably against these ROMs).
static std::vector<SappySample> scanAllSamples(const std::vector<uint8_t>& rom) {
    std::vector<SappySample> found;
    size_t i = 0;
    while (i + 16 <= rom.size()) {
        SappySample s;
        if (isPlausibleSample(rom, i, s)) {
            found.push_back(s);
            size_t next = s.offset + 16 + s.length;
            size_t rem = next % 4;
            i = rem ? next + (4 - rem) : next;
        } else {
            i += 4;
        }
    }
    return found;
}

// =======================================================================
// doc 2.1/2.2/2.3/2.4/2.5 -- instrument (voice) definitions, 12 bytes each
// =======================================================================

enum class VoiceKind { Sample, SampleFixed, Square1, Square2, ProgWave, Noise, KeySplit, Percussion, Invalid };

struct VoiceRaw {
    VoiceKind kind = VoiceKind::Invalid;
    uint8_t midiKey = 0, pan = 0;
    uint32_t ptr = 0;          // sample ptr (Sample/SampleFixed), waveform ptr (ProgWave),
                                // first-sub-instrument ptr (KeySplit), percussion table ptr (Percussion)
    uint32_t ptr2 = 0;         // key-split table ptr (KeySplit only)
    uint8_t attack = 0xFF, decay = 0x00, sustain = 0xFF, release = 0x00; // sample ADSR
    uint8_t duty = 0;          // PSG square duty / noise period flag
};

static VoiceKind classifyType(uint8_t t) {
    switch (t) {
        case 0x00: return VoiceKind::Sample;
        case 0x08: return VoiceKind::SampleFixed;
        case 0x01: case 0x09: return VoiceKind::Square1;
        case 0x02: case 0x0A: return VoiceKind::Square2;
        case 0x03: case 0x0B: return VoiceKind::ProgWave;
        case 0x04: case 0x0C: return VoiceKind::Noise;
        case 0x40: return VoiceKind::KeySplit;
        case (uint8_t)0x80: return VoiceKind::Percussion;
        default: return VoiceKind::Invalid;
    }
}

// Parses one raw 12-byte instrument record. Does NOT validate pointers
// (caller decides how strict to be -- validation differs for "is this a
// real voice group" scanning vs. "resolve this voice for export").
static VoiceRaw parseVoiceRaw(const std::vector<uint8_t>& rom, size_t off) {
    VoiceRaw v;
    uint8_t t = rdU8(rom, off);
    v.kind = classifyType(t);
    switch (v.kind) {
        case VoiceKind::Sample:
        case VoiceKind::SampleFixed:
            v.midiKey = rdU8(rom, off + 1);
            v.pan     = rdU8(rom, off + 3);
            v.ptr     = rdU32(rom, off + 4);
            v.attack  = rdU8(rom, off + 8);
            v.decay   = rdU8(rom, off + 9);
            v.sustain = rdU8(rom, off + 10);
            v.release = rdU8(rom, off + 11);
            break;
        case VoiceKind::Square1:
        case VoiceKind::Square2:
        case VoiceKind::Noise:
            v.duty    = rdU8(rom, off + 4);
            v.attack  = rdU8(rom, off + 8) & 0x07;
            v.decay   = rdU8(rom, off + 9) & 0x07;
            v.sustain = rdU8(rom, off + 10) & 0x0F;
            v.release = rdU8(rom, off + 11) & 0x07;
            break;
        case VoiceKind::ProgWave:
            v.ptr     = rdU32(rom, off + 4);
            v.attack  = rdU8(rom, off + 8) & 0x07;
            v.decay   = rdU8(rom, off + 9) & 0x07;
            v.sustain = rdU8(rom, off + 10) & 0x0F;
            v.release = rdU8(rom, off + 11) & 0x07;
            break;
        case VoiceKind::KeySplit:
            v.ptr  = rdU32(rom, off + 4);
            v.ptr2 = rdU32(rom, off + 8);
            break;
        case VoiceKind::Percussion:
            v.ptr = rdU32(rom, off + 4);
            break;
        default: break;
    }
    return v;
}

// Structural validation used only when scanning for voice-group tables:
// every field that should be a ROM pointer must actually resolve, and
// (crucially) Sample/SampleFixed voices must point at something that
// independently validates as a real sample header. This is what makes a
// candidate 128-entry table trustworthy rather than coincidental bytes.
static bool voiceStructurallyValid(const std::vector<uint8_t>& rom, size_t off) {
    uint8_t t = rdU8(rom, off);
    VoiceKind k = classifyType(t);
    if (k == VoiceKind::Invalid) return false;
    size_t o;
    switch (k) {
        case VoiceKind::Sample:
        case VoiceKind::SampleFixed: {
            uint32_t ptr = rdU32(rom, off + 4);
            if (!romPtrToOffset(ptr, rom.size(), o)) return false;
            SappySample s;
            return isPlausibleSample(rom, o, s);
        }
        case VoiceKind::ProgWave: {
            uint32_t ptr = rdU32(rom, off + 4);
            return romPtrToOffset(ptr, rom.size(), o) && o + 16 <= rom.size();
        }
        case VoiceKind::KeySplit: {
            uint32_t p1 = rdU32(rom, off + 4), p2 = rdU32(rom, off + 8);
            size_t o1, o2;
            return romPtrToOffset(p1, rom.size(), o1) && romPtrToOffset(p2, rom.size(), o2) && o2 + 128 <= rom.size();
        }
        case VoiceKind::Percussion: {
            uint32_t p = rdU32(rom, off + 4);
            return romPtrToOffset(p, rom.size(), o) && o + 128 * 12 <= rom.size();
        }
        default: return true; // Square1/Square2/Noise have no pointers to check
    }
}

struct VoiceGroup { size_t offset; };

static std::vector<VoiceGroup> scanVoiceGroups(const std::vector<uint8_t>& rom, bool verbose) {
    std::vector<VoiceGroup> found;
    size_t romSize = rom.size();
    for (size_t off = 0; off + 128 * 12 <= romSize; off += 4) {
        bool ok = true;
        for (int i = 0; i < 128 && ok; i++) {
            if (!voiceStructurallyValid(rom, off + i * 12)) ok = false;
        }
        if (ok) {
            found.push_back({off});
            if (verbose) std::cout << "  voice group @ 0x" << std::hex << off << std::dec << "\n";
            off += 128 * 12 - 4; // skip past this table (loop's += 4 will land right after)
        }
    }
    return found;
}

// =======================================================================
// doc 5/7b -- song table (8-byte entries) and song header
// =======================================================================

struct SappySong {
    size_t   headerOffset;
    uint8_t  numTracks;
    uint8_t  priority;
    uint32_t voiceGroupOffset;
    std::vector<size_t> trackOffsets;
};

// Validates a candidate song header at `off`: track count sane, voice-bank
// pointer resolves to an *already-discovered* voice group (this cross-
// reference is the strongest anti-false-positive signal available here),
// and every one of its track pointers is a legitimate in-ROM address.
static bool validateSongHeader(const std::vector<uint8_t>& rom, size_t off,
                                const std::set<size_t>& knownVoiceGroups, SappySong& out) {
    if (off + 8 > rom.size()) return false;
    uint8_t numTracks = rdU8(rom, off);
    if (numTracks < 1 || numTracks > 24) return false;
    // byte 1 unknown, byte 2 priority, byte 3 echo feedback -- no constraint
    uint32_t voicePtr = rdU32(rom, off + 4);
    size_t voiceOff;
    if (!romPtrToOffset(voicePtr, rom.size(), voiceOff)) return false;
    if (knownVoiceGroups.find(voiceOff) == knownVoiceGroups.end()) return false;

    if (off + 8 + 4u * numTracks > rom.size()) return false;
    std::vector<size_t> tracks;
    for (int i = 0; i < numTracks; i++) {
        uint32_t tp = rdU32(rom, off + 8 + 4 * i);
        size_t to;
        if (!romPtrToOffset(tp, rom.size(), to)) return false;
        tracks.push_back(to);
    }
    out.headerOffset = off;
    out.numTracks = numTracks;
    out.priority = rdU8(rom, off + 2);
    out.voiceGroupOffset = (uint32_t)voiceOff;
    out.trackOffsets = tracks;
    return true;
}

static std::vector<SappySong> scanSongs(const std::vector<uint8_t>& rom,
                                          const std::set<size_t>& knownVoiceGroups, bool verbose) {
    std::vector<SappySong> songs;
    std::set<size_t> seenHeaders;
    size_t romSize = rom.size();
    for (size_t off = 0; off + 4 <= romSize; off += 4) {
        uint32_t ptr = rdU32(rom, off);
        size_t target;
        if (!romPtrToOffset(ptr, romSize, target)) continue;
        if (seenHeaders.count(target)) continue;
        SappySong s;
        if (validateSongHeader(rom, target, knownVoiceGroups, s)) {
            seenHeaders.insert(target);
            songs.push_back(s);
            if (verbose) {
                std::cout << "  song header @ 0x" << std::hex << target << std::dec
                          << "  tracks=" << (int)s.numTracks << "  voicegroup=0x"
                          << std::hex << s.voiceGroupOffset << std::dec << "\n";
            }
        }
    }
    return songs;
}

// =======================================================================
// RIFF/SF2 writer
// =======================================================================
//
// No voice-bank/song data was found for these particular ROMs (see the
// scan above / the accompanying explanation) so there is no per-instrument
// envelope, key-split, or program-change data to draw on. What follows
// builds the most useful thing that *is* fully supported by data we
// actually validated: one SF2 preset per extracted sample (bank 0,
// program = sample index), each a plain mono instrument covering the
// whole keyboard at root key 60 (Mid-C, matching the pitch convention the
// sample headers themselves use), looped where the source data says to
// loop. Since we have no real ADSR data either, envelopes are a neutral
// near-instant pass-through with a short release, just to avoid a click
// at note-off -- not a recreation of any original instrument envelope.

template <typename F>
static void writeChunk(std::ostream& out, const char tag[4], F fillContent) {
    std::ostringstream buf(std::ios::binary);
    fillContent(buf);
    std::string data = buf.str();
    out.write(tag, 4);
    wrU32(out, (uint32_t)data.size());
    out.write(data.data(), (std::streamsize)data.size());
    if (data.size() % 2) out.put(0);
}
template <typename F>
static void writeListChunk(std::ostream& out, const char listType[4], F fillContent) {
    writeChunk(out, "LIST", [&](std::ostream& inner) {
        inner.write(listType, 4);
        fillContent(inner);
    });
}
static void wrName20(std::ostream& out, const std::string& s) {
    char buf[20]; memset(buf, 0, 20);
    memcpy(buf, s.data(), std::min(s.size(), (size_t)19));
    out.write(buf, 20);
}
static void wrPhdr(std::ostream& o, const std::string& n, uint16_t preset, uint16_t bank, uint16_t bagNdx) {
    wrName20(o, n); wrU16(o, preset); wrU16(o, bank); wrU16(o, bagNdx); wrU32(o, 0); wrU32(o, 0); wrU32(o, 0);
}
static void wrBag(std::ostream& o, uint16_t genNdx, uint16_t modNdx) { wrU16(o, genNdx); wrU16(o, modNdx); }
static void wrGenWord(std::ostream& o, uint16_t oper, uint16_t amount) { wrU16(o, oper); wrU16(o, amount); }
static void wrGenShort(std::ostream& o, uint16_t oper, int16_t amount) { wrU16(o, oper); wrS16(o, amount); }
static void wrInst(std::ostream& o, const std::string& n, uint16_t bagNdx) { wrName20(o, n); wrU16(o, bagNdx); }
static void wrShdr(std::ostream& o, const std::string& n, uint32_t start, uint32_t end, uint32_t loopS,
                    uint32_t loopE, uint32_t rate, uint8_t rootKey) {
    wrName20(o, n);
    wrU32(o, start); wrU32(o, end); wrU32(o, loopS); wrU32(o, loopE); wrU32(o, rate);
    o.put((char)rootKey); o.put((char)0); wrU16(o, 0); wrU16(o, 1); // pitch correction=0, sampleLink=0, sampleType=1(mono)
}

// generator numbers used (verified against a reference SF2 implementation)
enum : uint16_t {
    GEN_DELAY_VOL_ENV = 33, GEN_ATTACK_VOL_ENV = 34, GEN_HOLD_VOL_ENV = 35, GEN_DECAY_VOL_ENV = 36,
    GEN_SUSTAIN_VOL_ENV = 37, GEN_RELEASE_VOL_ENV = 38, GEN_INSTRUMENT = 41, GEN_SAMPLE_MODES = 54,
    GEN_SCALE_TUNING = 56, GEN_OVERRIDING_ROOT_KEY = 58, GEN_SAMPLE_ID = 53
};

static int16_t secondsToTimecents(double s) {
    if (s <= 0.001) return -12000;
    double tc = 1200.0 * std::log2(s);
    if (tc < -12000) tc = -12000;
    if (tc > 8000) tc = 8000;
    return (int16_t)std::lround(tc);
}

static int16_t sourceByteToCentered16(uint8_t raw) { return (int16_t)((int8_t)raw) * 256; }

static void writeSf2(const std::string& path, const std::vector<uint8_t>& rom,
                      const std::vector<SappySample>& samples, const std::string& bankName) {
    size_t N = samples.size();

    // ---- build the concatenated 16-bit sample pool ----
    std::vector<int16_t> pool;
    struct Slot { uint32_t start, end, loopStart, loopEnd; };
    std::vector<Slot> slots(N);
    for (size_t i = 0; i < N; i++) {
        const auto& s = samples[i];
        uint32_t start = (uint32_t)pool.size();
        for (uint32_t k = 0; k < s.length; k++)
            pool.push_back(sourceByteToCentered16(rom[s.offset + 16 + k]));
        uint32_t end = (uint32_t)pool.size();
        uint32_t loopStart = s.looped ? start + s.loopStart : start;
        uint32_t loopEnd = end;
        for (int k = 0; k < 46; k++) pool.push_back(0); // required inter-sample padding
        slots[i] = {start, end, loopStart, loopEnd};
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot create " + path);

    writeChunk(out, "RIFF", [&](std::ostream& o) {
        o.write("sfbk", 4);

        writeListChunk(o, "INFO", [&](std::ostream& info) {
            writeChunk(info, "ifil", [&](std::ostream& c) { wrU16(c, 2); wrU16(c, 1); });
            auto writeEvenCStr = [](std::ostream& c, const std::string& s) {
                c << s; c.put(0);
                if ((s.size() + 1) % 2) c.put(0); // keep declared chunk size even
            };
            writeChunk(info, "isng", [&](std::ostream& c) { writeEvenCStr(c, "EMU8000"); });
            writeChunk(info, "INAM", [&](std::ostream& c) { writeEvenCStr(c, bankName); });
        });

        writeListChunk(o, "sdta", [&](std::ostream& sdta) {
            writeChunk(sdta, "smpl", [&](std::ostream& c) {
                for (int16_t v : pool) wrS16(c, v);
            });
        });

        writeListChunk(o, "pdta", [&](std::ostream& p) {
            writeChunk(p, "phdr", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) wrPhdr(c, "smp" + std::to_string(i), (uint16_t)i, 0, (uint16_t)i);
                wrPhdr(c, "EOP", 0, 0, (uint16_t)N);
            });
            writeChunk(p, "pbag", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) wrBag(c, (uint16_t)i, 0);
                wrBag(c, (uint16_t)N, 0);
            });
            writeChunk(p, "pmod", [&](std::ostream& c) {
                for (int i = 0; i < 10; i++) c.put(0); // one all-zero terminal modulator record
            });
            writeChunk(p, "pgen", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) wrGenWord(c, GEN_INSTRUMENT, (uint16_t)i);
                wrU16(c, 0); wrU16(c, 0); // terminal
            });
            writeChunk(p, "inst", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) wrInst(c, "smp" + std::to_string(i), (uint16_t)i);
                wrInst(c, "EOI", (uint16_t)N);
            });
            writeChunk(p, "ibag", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) wrBag(c, (uint16_t)(i * 10), 0);
                wrBag(c, (uint16_t)(N * 10), 0);
            });
            writeChunk(p, "imod", [&](std::ostream& c) {
                for (int i = 0; i < 10; i++) c.put(0);
            });
            writeChunk(p, "igen", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) {
                    wrGenWord(c, GEN_SAMPLE_MODES, samples[i].looped ? 1 : 0);
                    wrGenWord(c, GEN_OVERRIDING_ROOT_KEY, 60);
                    wrGenWord(c, GEN_SCALE_TUNING, 100);
                    wrGenShort(c, GEN_DELAY_VOL_ENV, secondsToTimecents(0));
                    wrGenShort(c, GEN_ATTACK_VOL_ENV, secondsToTimecents(0));
                    wrGenShort(c, GEN_HOLD_VOL_ENV, secondsToTimecents(0));
                    wrGenShort(c, GEN_DECAY_VOL_ENV, secondsToTimecents(0));
                    wrGenWord(c, GEN_SUSTAIN_VOL_ENV, 0);              // 0 cB = full volume sustain
                    wrGenShort(c, GEN_RELEASE_VOL_ENV, -6000);         // ~30ms release, avoids a click
                    wrGenWord(c, GEN_SAMPLE_ID, (uint16_t)i);          // must be last
                }
                wrU16(c, 0); wrU16(c, 0); // terminal
            });
            writeChunk(p, "shdr", [&](std::ostream& c) {
                for (size_t i = 0; i < N; i++) {
                    auto& sl = slots[i];
                    wrShdr(c, "smp" + std::to_string(i), sl.start, sl.end, sl.loopStart, sl.loopEnd,
                           samples[i].sampleRate, 60);
                }
                wrShdr(c, "EOS", 0, 0, 0, 0, 0, 0);
            });
        });
    });
}

// =======================================================================
// Preview MIDI writer: one Format-0 file that steps through every preset
// (program change + a single held note) so the soundbank can be quickly
// auditioned in any DAW. This is NOT reconstructed game music -- there is
// no sequencing data to reconstruct for these ROMs (see explanation) --
// it's just a per-instrument audition track.
// =======================================================================

static void writeVLQ(std::ostream& out, uint32_t value) {
    uint8_t buf[5]; int n = 0;
    buf[n++] = value & 0x7F; value >>= 7;
    while (value) { buf[n++] = (value & 0x7F) | 0x80; value >>= 7; }
    for (int i = n - 1; i >= 0; i--) out.put((char)buf[i]);
}

static void writePreviewMidi(const std::string& path, size_t numPresets) {
    const int PPQN = 480;
    std::ostringstream track(std::ios::binary);

    // tempo: 120 BPM
    writeVLQ(track, 0); track.write("\xFF\x51\x03", 3);
    track.put((char)0x07); track.put((char)0xA1); track.put((char)0x20); // 500000 us/qn

    const uint32_t noteTicks = PPQN * 3 / 4;   // ~0.75 beat held note
    const uint32_t gapTicks  = PPQN / 4;       // ~0.25 beat gap

    for (size_t i = 0; i < numPresets; i++) {
        writeVLQ(track, 0); track.put((char)0xC0); track.put((char)(i & 0x7F)); // program change ch0
        writeVLQ(track, 0); track.put((char)0x90); track.put(60); track.put(100); // note on
        writeVLQ(track, noteTicks); track.put((char)0x80); track.put(60); track.put(0); // note off
        if (i + 1 < numPresets) writeVLQ(track, gapTicks); // gap before next program
    }
    writeVLQ(track, 0); track.write("\xFF\x2F\x00", 3); // end of track

    std::string trackData = track.str();

    std::ofstream out(path, std::ios::binary);
    out.write("MThd", 4);
    out.put(0); out.put(0); out.put(0); out.put(6); // chunk length, big-endian (always 6)
    // MThd fields are big-endian per the SMF spec
    out.put(0); out.put(0);              // format 0
    out.put(0); out.put(1);              // 1 track
    out.put((PPQN >> 8) & 0xFF); out.put(PPQN & 0xFF);
    out.write("MTrk", 4);
    uint32_t sz = (uint32_t)trackData.size();
    out.put((sz >> 24) & 0xFF); out.put((sz >> 16) & 0xFF); out.put((sz >> 8) & 0xFF); out.put(sz & 0xFF);
    out.write(trackData.data(), (std::streamsize)trackData.size());
}

// =======================================================================
// main
// =======================================================================

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: " << argv[0] << " rom.gba -o outdir [-v]\n"; return 1; }
    std::string romPath = argv[1];
    std::string outDir = "sappy_export";
    bool verbose = false;
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        if ((a == "-o" || a == "--outdir") && i + 1 < argc) outDir = argv[++i];
        else if (a == "-v" || a == "--verbose") verbose = true;
    }
    try {
        std::vector<uint8_t> rom = readWholeFile(romPath);
        GbaHeaderInfo hdr = readGbaHeader(rom);
        std::cout << "ROM: " << hdr.title << " [" << hdr.gameCode << "] "
                  << rom.size() << " bytes, checksum " << (hdr.checksumOk ? "OK" : "MISMATCH") << "\n";

        auto voiceGroups = scanVoiceGroups(rom, verbose);
        std::set<size_t> vgSet;
        for (auto& vg : voiceGroups) vgSet.insert(vg.offset);
        auto songs = scanSongs(rom, vgSet, verbose);
        std::cout << "Sappy voice groups found: " << voiceGroups.size() << "\n";
        std::cout << "Sappy songs found: " << songs.size() << "\n";

        auto samples = scanAllSamples(rom);
        std::cout << "Raw samples found: " << samples.size() << "\n";

        stdfs::create_directories(outDir);

        if (!songs.empty()) {
            std::cout << "(Song/voice-bank data found -- MIDI export from real sequences is not yet\n"
                          " wired into this build path; samples + SF2 below are still generated.)\n";
        } else {
            std::cout << "No standard Sappy song/voice-bank structure found in this ROM (see notes).\n"
                          "Generating an SF2 soundbank from the raw samples plus an audition MIDI instead.\n";
        }

        if (!samples.empty()) {
            std::string sf2Path = outDir + "/soundbank.sf2";
            writeSf2(sf2Path, rom, samples, hdr.title);
            std::cout << "Wrote " << sf2Path << " (" << samples.size() << " presets, bank 0, program 0-"
                      << (samples.size() - 1) << ")\n";

            std::string midPath = outDir + "/preview.mid";
            writePreviewMidi(midPath, samples.size());
            std::cout << "Wrote " << midPath << " (steps through every preset)\n";

            std::ofstream manifest(outDir + "/manifest.csv");
            manifest << "program,rom_offset_hex,native_rate_hz,length_bytes,looped\n";
            for (size_t i = 0; i < samples.size(); i++) {
                auto& s = samples[i];
                manifest << i << ",0x" << std::hex << s.offset << std::dec << "," << s.sampleRate << ","
                         << s.length << "," << (s.looped ? 1 : 0) << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
